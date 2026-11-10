import { quaternionDistanceDeg, quaternionToMatrix } from '../robot/kinematics';
import { planGrindingPath } from '../robot/grinding';
import { GRINDING_LIMITS, validateGrindingResult } from '../robot/grinding-validation';
import type { GrindingRequest, GrindingSuccess } from '../robot/grinding-types';
import type { Pose, Vec3 } from '../robot/types';
import { activeToolTcpPose, pathPointToWorld, robotBaseWorldPose, type FrameGraph } from '../workspace/frame-graph';
import type { ProgramPath } from '../workspace/program';
import { validateSurfacePath, validateSurfaceUvSettings } from '../workspace/surface-path';
import { generateSurfacePath } from './generate';
import { validateSurfaceRequest, validateSurfaceResult } from './generation-validation';
import type { SurfaceRequest, SurfaceSuccess } from './generation-types';
import type { SceneSurface } from './types';
import { evaluateSurfaceAnchor, surfaceWorldMesh, validateSurfacePickReference } from './validation';
import type { SurfaceUvInput } from './SurfaceUvDialog';

export interface SurfacePathPlanInput {
  surface: SceneSurface;
  path: ProgramPath;
  graph: FrameGraph;
  settings: SurfaceUvInput;
  /** Per-target surface normals replace any legacy shared-orientation constraint. */
  request: Omit<GrindingRequest, 'targets' | 'orientation' | 'allowDetours'>;
}
export interface SurfacePathPlan {
  geometry: SurfaceSuccess;
  motion: GrindingSuccess;
  regionRevision: number;
  sourcePathId: string;
  surfaceId: string;
  surfaceRevision: number;
  graphRevision: number;
  modelId: string;
  settings: SurfaceUvInput;
  speedMmPerSecond: number;
}

const distance = (a: readonly number[], b: readonly number[]) => Math.hypot(...a.map((value, index) => value - b[index]));
const toolZ = (pose: Pose): Vec3 => { const matrix = quaternionToMatrix(pose.orientation); return [matrix[2], matrix[6], matrix[10]]; };
const samePose = (a: Pose, b: Pose) => distance(a.position, b.position) <= 1e-4 && quaternionDistanceDeg(a.orientation, b.orientation) <= 1e-4;
const cancelled = (signal?: AbortSignal) => { if (signal?.aborted) throw new DOMException('表面路径生成已取消。', 'AbortError'); };

/** This checks geometry only; program motion guards separately require resolved robot joints. */
export function surfacePathGeometryReason(path: ProgramPath, graph: FrameGraph, surfaces: readonly SceneSurface[]): string | null {
  if (!path.surface) return null;
  try {
    const region = validateSurfacePath(path.surface), surface = surfaces.find(item => item.id === region.surfaceId);
    if (!surface) return '路径关联的工件模型已删除，请重新选择模型表面。';
    if (region.invalidated || surface.revision !== region.surfaceRevision) return '工件模型或路径已改变，表面区域已失效，请重新点选。';
    const sign = region.kind === 'uv' && region.settings?.reverseNormal ? -1 : 1;
    for (const point of region.points) {
      if (point.pick.surfaceId !== surface.id || point.pick.surfaceRevision !== surface.revision) return '表面点来自不同模型或旧的模型位置，请重新点选。';
      validateSurfacePickReference(point.pick, surface);
      const actual = evaluateSurfaceAnchor(surface, point.pick), world = pathPointToWorld(graph, path.id, point.pose);
      if (distance(world.position, actual.position) > 1e-4) return '路径点已离开其模型表面，请重新点选区域。';
      const expected = actual.normal.map(value => value * sign);
      if (distance(toolZ(world), expected) > 1e-5) return '路径点的工具 Z 轴与模型表面法向不一致，请重新生成。';
    }
    return null;
  } catch (cause) { return cause instanceof Error ? cause.message : '表面区域数据无效，请重新点选。'; }
}

/** Upper bound for regular arc-length intervals; short final boundary intervals are intentionally exempt. */
export function surfaceSpacingToleranceMm(settings: Pick<SurfaceUvInput, 'uSpacingMm' | 'vSpacingMm'>): number {
  return Math.max(.05, Math.min(settings.uSpacingMm, settings.vSpacingMm) * .01);
}

/** Geometry and IK are one transaction: no caller receives a partially generated or unverified path. */
export async function planSurfacePath(input: SurfacePathPlanInput, signal?: AbortSignal): Promise<SurfacePathPlan> {
  cancelled(signal);
  // Neither edits nor mutations after the first await may alter this calculation.
  const snapshot = structuredClone(input), { surface, path, graph } = snapshot;
  const settings = { ...validateSurfaceUvSettings(snapshot.settings), maxNormalStepDeg: snapshot.settings.maxNormalStepDeg };
  if (!Number.isFinite(settings.maxNormalStepDeg) || settings.maxNormalStepDeg < .1 || settings.maxNormalStepDeg > 30) throw new Error('相邻姿态角度上限须在 0.1° 至 30° 内。');
  const reason = surfacePathGeometryReason(path, graph, [surface]);
  if (reason) throw new Error(reason);
  const region = path.surface;
  if (!region || region.kind !== 'region') throw new Error('请先在 Path 上点选模型表面区域。');
  if (region.points.length < 4 || region.points.length > 256 || region.controlPointIds.length !== 4) throw new Error('区域需有 4 至 256 个边界点，并按边界顺序标记 4 个角控制点。');
  const controlIndices = region.controlPointIds.map(id => region.points.findIndex(point => point.id === id)) as [number, number, number, number];
  if (new Set(controlIndices).size !== 4 || controlIndices.some((value, index) => value < 0 || index > 0 && value <= controlIndices[index - 1])) throw new Error('4 个控制点必须是沿区域边界依次排列的不同角点。');
  const modelId = snapshot.request.robot.kind === 'abb' ? snapshot.request.robot.modelId : snapshot.request.robot.chain.modelId;
  if (graph.modelId !== modelId) throw new Error('区域与当前机器人型号不一致，请重新打开路径生成。');
  if (!samePose(snapshot.request.basePose, robotBaseWorldPose(graph)) || !samePose(snapshot.request.toolPose, activeToolTcpPose(graph))) throw new Error('机器人基座或工具坐标已改变，请重新生成。');
  const request: SurfaceRequest = validateSurfaceRequest({
    mesh: surfaceWorldMesh(surface),
    boundary: region.points.map(point => ({ triangleIndex: point.pick.triangleIndex, barycentric: [...point.pick.barycentric] })),
    controlIndices, sampling: { mode: 'spacing', u: settings.uSpacingMm, v: settings.vSpacingMm },
    normalSign: settings.reverseNormal ? -1 : 1, maxNormalStepDeg: settings.maxNormalStepDeg, maxPoints: 256,
  });
  cancelled(signal);
  const rawGeometry = await generateSurfacePath(request, signal);
  cancelled(signal);
  const geometry = validateSurfaceResult(rawGeometry, request);
  if (geometry.status !== 'success') throw new Error(geometry.message);
  const spacingTolerance = surfaceSpacingToleranceMm(settings);
  if (geometry.maxRegularSpacingErrorMm > spacingTolerance + 1e-8) throw new Error(`曲面常规段的间距误差为 ${geometry.maxRegularSpacingErrorMm.toFixed(3)} mm，超过允许的 ${spacingTolerance.toFixed(3)} mm。请缩小区域或调整控制点后重新生成。`);
  const motionRequest: GrindingRequest = {
    ...snapshot.request,
    targets: geometry.points.map(point => structuredClone(point.pose)),
    orientation: null,
    allowDetours: false,
  };
  const rawMotion = await planGrindingPath(motionRequest, signal);
  cancelled(signal);
  const motion = validateGrindingResult(rawMotion, motionRequest);
  if (motion.status !== 'success') throw new Error(motion.message);
  // The approach may begin off the surface. Only its final, indexed target is
  // saved; additional transfer waypoints would make the new Path leave the workpiece.
  if (motion.waypoints.length !== geometry.points.length || motion.waypoints.some((point, index) => point.targetIndex !== index || point.kind !== (index === 0 ? 'transfer' : 'process') || !samePose(point.pose, geometry.points[index].pose))) throw new Error('机器人路径没有逐点保留模型表面目标，未创建加工路径。');
  for (let index = 1; index < motion.path.length; index++) {
    if (motion.path[index].joints.some((value, axis) => Math.abs(value - motion.path[index - 1].joints[axis]) > GRINDING_LIMITS.jointStepDeg + 1e-6)) throw new Error('机器人关节在加工路径中发生突变，请调整区域或起始姿态。');
  }
  for (let index = 1; index < motion.waypoints.length; index++) {
    if (motion.waypoints[index].joints.some((value, axis) => Math.abs(value - motion.waypoints[index - 1].joints[axis]) > 30 + 1e-6)) throw new Error('相邻加工点的关节变化超过 30°，请缩小点间距或调整机器人姿态。');
  }
  cancelled(signal);
  return {
    geometry, motion, regionRevision: region.revision, sourcePathId: path.id,
    surfaceId: surface.id, surfaceRevision: surface.revision, graphRevision: graph.revision, modelId,
    settings, speedMmPerSecond: motionRequest.speedMmPerSecond,
  };
}
