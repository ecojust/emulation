import { beforeAll, beforeEach, describe, expect, it, vi } from 'vitest';
import { forwardKinematics, quaternionToMatrix } from '../src/robot/kinematics';
import { getRobotModel } from '../src/robot/model';
import { planGrindingReference } from '../src/robot/grinding-reference';
import type { GrindingRequest, GrindingSuccess } from '../src/robot/grinding-types';
import type { Pose, Vec3 } from '../src/robot/types';
import { composePose, createFrameGraph, identityPose, worldPointToPath } from '../src/workspace/frame-graph';
import type { ProgramPath } from '../src/workspace/program';
import type { SurfacePathData } from '../src/workspace/surface-path';
import { planSurfaceReference } from '../src/surface/reference';
import { evaluateSurfaceAnchor, surfaceWorldMesh } from '../src/surface/validation';
import type { SceneSurface, SurfaceAnchor } from '../src/surface/types';
import type { SurfaceRequest, SurfaceSuccess } from '../src/surface/generation-types';

const clients = vi.hoisted(() => ({ geometry: vi.fn(), motion: vi.fn() }));
vi.mock('../src/surface/generate', () => ({ generateSurfacePath: clients.geometry }));
vi.mock('../src/robot/grinding', () => ({ planGrindingPath: clients.motion }));
import { planSurfacePath, surfacePathGeometryReason, surfaceSpacingToleranceMm, type SurfacePathPlanInput } from '../src/surface/path-planning';

function fixture(): SurfacePathPlanInput {
  const model = getRobotModel('irb1100-4-0.58'), ready = forwardKinematics(model, model.readyJoints).pose;
  const matrix = quaternionToMatrix(ready.orientation), normal = [matrix[2], matrix[6], matrix[10]];
  const surface: SceneSurface = {
    id: 'part', name: '小平面', revision: 3, visible: true, source: { name: 'plane.stl', format: 'stl', unit: 'mm' },
    pose: { position: ready.position.map((value, index) => value + normal[index] * 3) as Vec3, orientation: ready.orientation },
    mesh: { positions: [0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 0, 0, 4, 4, 0, 0, 4, 0], normals: Array.from({ length: 6 }, () => [0, 0, 1]).flat() },
  };
  const graph = { ...createFrameGraph(model.id), revision: 4, paths: { region: { parent: { kind: 'world' as const }, localPose: { position: [13, -7, 5] as Vec3, orientation: [Math.SQRT1_2, 0, 0, Math.SQRT1_2] as Pose['orientation'] } } } };
  const anchors: SurfaceAnchor[] = [{ triangleIndex: 0, barycentric: [1, 0, 0] }, { triangleIndex: 0, barycentric: [0, 1, 0] }, { triangleIndex: 0, barycentric: [0, 0, 1] }, { triangleIndex: 1, barycentric: [0, 0, 1] }];
  const region: SurfacePathData = { kind: 'region', revision: 7, surfaceId: surface.id, surfaceRevision: surface.revision,
    points: anchors.map((anchor, index) => {
      const actual = evaluateSurfaceAnchor(surface, anchor);
      return { id: 's' + index, name: 'S00' + (index + 1), pose: worldPointToPath(graph, 'region', { position: actual.position, orientation: surface.pose.orientation }), pick: { ...anchor, ...actual, surfaceId: surface.id, surfaceRevision: surface.revision } };
    }), controlPointIds: ['s0', 's1', 's2', 's3'] };
  const path: ProgramPath = { id: 'region', name: '表面区域', pointIds: [], surface: region };
  return { surface, graph, path, settings: { uSpacingMm: 2, vSpacingMm: 2, reverseNormal: false, maxNormalStepDeg: 5 },
    request: { robot: { kind: 'abb', modelId: model.id }, basePose: identityPose(), toolPose: identityPose(), startJoints: [...model.readyJoints], toolRadiusMm: 0, clearanceMm: 0, speedMmPerSecond: 50, zones: [] } };
}

let input: SurfacePathPlanInput, geometry: SurfaceSuccess, motion: GrindingSuccess;
function generateFor(value: SurfacePathPlanInput): SurfaceSuccess {
  const request: SurfaceRequest = { mesh: surfaceWorldMesh(value.surface), boundary: value.path.surface!.points.map(point => ({ triangleIndex: point.pick.triangleIndex, barycentric: point.pick.barycentric })), controlIndices: [0, 1, 2, 3], sampling: { mode: 'spacing', u: value.settings.uSpacingMm, v: value.settings.vSpacingMm }, normalSign: value.settings.reverseNormal ? -1 : 1, maxNormalStepDeg: value.settings.maxNormalStepDeg, maxPoints: 256 };
  const generated = planSurfaceReference(request); if (generated.status !== 'success') throw new Error(generated.message); return generated;
}
function planFor(value: SurfacePathPlanInput, generated: SurfaceSuccess): GrindingSuccess {
  const planned = planGrindingReference({ ...value.request, targets: generated.points.map(point => point.pose), allowDetours: false, orientation: null });
  if (planned.status !== 'success') throw new Error(planned.message); return planned;
}
beforeAll(() => { input = fixture(); geometry = generateFor(input); motion = planFor(input, geometry); });
beforeEach(() => {
  clients.geometry.mockReset().mockImplementation(async () => structuredClone(geometry));
  clients.motion.mockReset().mockImplementation(async () => structuredClone(motion));
});

describe('surface region freshness in world and Path frames', () => {
  it('reconstructs original anchors after rotated Path placement without changing their world positions', () => {
    expect(surfacePathGeometryReason(input.path, input.graph, [input.surface])).toBeNull();
    expect(surfacePathGeometryReason({ ...input.path, surface: undefined }, input.graph, [])).toBeNull();
  });
  it('rejects deleted, revised, off-surface and mismatched-normal references', () => {
    expect(surfacePathGeometryReason(input.path, input.graph, [])).toMatch(/删除/);
    expect(surfacePathGeometryReason(input.path, input.graph, [{ ...input.surface, revision: 4 }])).toMatch(/失效/);
    for (const mutate of [
      (value: SurfacePathPlanInput) => { value.path.surface!.points[0].pose.position[0] += 1; },
      (value: SurfacePathPlanInput) => { value.path.surface!.points[0].pick.position[0] += 1; },
      (value: SurfacePathPlanInput) => { value.path.surface!.points[0].pick.normal[0] += .1; },
    ]) {
      const value = structuredClone(input); mutate(value);
      expect(surfacePathGeometryReason(value.path, value.graph, [value.surface])).not.toBeNull();
    }
  });
  it('keeps unsigned mesh provenance while allowing a UV target with reversed tool +Z', () => {
    const value = structuredClone(input), region = value.path.surface!;
    const points = region.points.map(point => {
      const outward = evaluateSurfaceAnchor(value.surface, point.pick);
      const pose = composePose({ position: outward.position, orientation: value.surface.pose.orientation }, { position: [0, 0, 0], orientation: [0, 1, 0, 0] });
      return { ...point, pose: worldPointToPath(value.graph, value.path.id, pose) };
    });
    value.path = { ...value.path, surface: { ...region, kind: 'uv', points, controlPointIds: [], source: { pathId: 'original-region', revision: 7 }, settings: { uSpacingMm: 2, vSpacingMm: 2, reverseNormal: true } } };
    expect(surfacePathGeometryReason(value.path, value.graph, [value.surface])).toBeNull();
    value.path = { ...value.path, surface: { ...value.path.surface!, settings: { uSpacingMm: 2, vSpacingMm: 2, reverseNormal: false } } };
    expect(surfacePathGeometryReason(value.path, value.graph, [value.surface])).toMatch(/Z 轴/);
  });
});

describe('transactional surface geometry and continuous robot planning', () => {
  it('submits world-space anchors, keeps all on-surface targets and allows the initial off-surface approach', async () => {
    const value = structuredClone(input), original = structuredClone(value), controller = new AbortController();
    const result = await planSurfacePath(value, controller.signal);
    expect(value).toEqual(original);
    expect(result).toMatchObject({ sourcePathId: 'region', regionRevision: 7, surfaceId: 'part', surfaceRevision: 3, graphRevision: 4, modelId: input.graph.modelId, settings: input.settings, speedMmPerSecond: 50 });
    expect(result.geometry.points.length).toBe(9);
    expect(clients.geometry).toHaveBeenCalledWith(expect.objectContaining({ mesh: surfaceWorldMesh(input.surface), controlIndices: [0, 1, 2, 3], sampling: { mode: 'spacing', u: 2, v: 2 }, normalSign: 1 }), controller.signal);
    expect(clients.motion).toHaveBeenCalledWith(expect.objectContaining({ targets: geometry.points.map(point => point.pose), orientation: null, allowDetours: false }), controller.signal);
    expect(result.motion.waypoints.map(point => point.targetIndex)).toEqual(geometry.points.map((_, index) => index));
    expect(result.motion.waypoints[0].kind).toBe('transfer');
    expect(result.motion.waypoints.slice(1).every(point => point.kind === 'process')).toBe(true);
    expect(Math.hypot(...result.motion.path[0].pose.position.map((value, index) => value - geometry.points[0].pose.position[index]))).toBeGreaterThan(2.9);
  });
  it('rejects invalid control order and changed base/tool context before submitting work', async () => {
    for (const mutate of [
      (value: SurfacePathPlanInput) => { value.path = { ...value.path, surface: { ...value.path.surface!, controlPointIds: ['s0', 's2', 's1', 's3'] } }; },
      (value: SurfacePathPlanInput) => { value.request.basePose.position[0] += 1; },
      (value: SurfacePathPlanInput) => { value.request.toolPose.position[2] += 1; },
      (value: SurfacePathPlanInput) => { value.surface.revision += 1; },
    ]) {
      const value = structuredClone(input); mutate(value); await expect(planSurfacePath(value)).rejects.toThrow();
    }
    expect(clients.geometry).not.toHaveBeenCalled(); expect(clients.motion).not.toHaveBeenCalled();
  });
  it('does not submit or accept any stage after cancellation', async () => {
    const controller = new AbortController(); controller.abort();
    await expect(planSurfacePath(input, controller.signal)).rejects.toMatchObject({ name: 'AbortError' });
    expect(clients.geometry).not.toHaveBeenCalled();
    const duringGeometry = new AbortController();
    clients.geometry.mockImplementationOnce(async () => { duringGeometry.abort(); return structuredClone(geometry); });
    await expect(planSurfacePath(input, duringGeometry.signal)).rejects.toMatchObject({ name: 'AbortError' });
    expect(clients.motion).not.toHaveBeenCalled();
    const duringMotion = new AbortController();
    clients.motion.mockImplementationOnce(async () => { duringMotion.abort(); return structuredClone(motion); });
    await expect(planSurfacePath(input, duringMotion.signal)).rejects.toMatchObject({ name: 'AbortError' });
  });
  it('rejects geometry failure without starting robot planning or exposing partial data', async () => {
    clients.geometry.mockResolvedValueOnce({ status: 'failure', code: 'hole', message: '区域包含孔洞。' });
    await expect(planSurfacePath(input)).rejects.toThrow('区域包含孔洞'); expect(clients.motion).not.toHaveBeenCalled();
    clients.geometry.mockRejectedValueOnce(new Error('native unavailable'));
    await expect(planSurfacePath(input)).rejects.toThrow('native unavailable'); expect(clients.motion).not.toHaveBeenCalled();
  });
  it('enforces regular surface spacing quality while permitting the shorter boundary remainder', async () => {
    expect(surfaceSpacingToleranceMm({ uSpacingMm: 2, vSpacingMm: 2 })).toBe(.05);
    expect(surfaceSpacingToleranceMm({ uSpacingMm: 20, vSpacingMm: 10 })).toBe(.1);
    // A genuine trapezoid has longer transverse edge intervals than its
    // centerline. Its independently valid report must still fail our quality gate.
    const distorted = structuredClone(input), region = distorted.path.surface!;
    distorted.path = { ...distorted.path, surface: { ...region, points: region.points.map((point, index) => {
      if (index < 2) return point;
      const anchor: SurfaceAnchor = { triangleIndex: 1, barycentric: index === 2 ? [0, .75, .25] : [0, .25, .75] };
      const actual = evaluateSurfaceAnchor(distorted.surface, anchor);
      return { ...point, pose: worldPointToPath(distorted.graph, distorted.path.id, { position: actual.position, orientation: distorted.surface.pose.orientation }), pick: { ...point.pick, ...anchor, ...actual } };
    }) } };
    const uneven = generateFor(distorted);
    expect(uneven.maxRegularSpacingErrorMm).toBeGreaterThan(surfaceSpacingToleranceMm(distorted.settings));
    clients.geometry.mockResolvedValueOnce(uneven);
    await expect(planSurfacePath(distorted)).rejects.toThrow(/超过允许的/); expect(clients.motion).not.toHaveBeenCalled();
    // A 4 mm square at 3 mm pitch has a real 1 mm final interval in each
    // direction. The 2 mm remainder difference is not a regular-pitch error.
    const remainder = structuredClone(input); remainder.settings = { ...remainder.settings, uSpacingMm: 3, vSpacingMm: 3 };
    const shortBoundary = generateFor(remainder);
    expect(shortBoundary.maxRegularSpacingErrorMm).toBeLessThan(1e-8);
    expect(shortBoundary.maxSpacingErrorMm).toBeCloseTo(2, 8);
    clients.geometry.mockResolvedValueOnce(shortBoundary);
    clients.motion.mockResolvedValueOnce(planFor(remainder, shortBoundary));
    expect((await planSurfacePath(remainder)).geometry.maxSpacingErrorMm).toBeCloseTo(2, 8);
  });
  it('independently rejects forged off-surface geometry and incorrect normal alignment', async () => {
    for (const mutate of [
      (value: SurfaceSuccess) => { value.points[1].pose.position[0] += 1; },
      (value: SurfaceSuccess) => { value.points[1].pose.orientation = [1, 0, 0, 0]; },
      (value: SurfaceSuccess) => { value.points[1].normal = [1, 0, 0]; },
    ]) {
      const value = structuredClone(geometry); mutate(value); clients.geometry.mockResolvedValueOnce(value);
      await expect(planSurfacePath(input)).rejects.toThrow();
    }
    expect(clients.motion).not.toHaveBeenCalled();
  });
  it('rejects missing, reordered or extra robot waypoints and large actual joint jumps', async () => {
    for (const mutate of [
      (value: GrindingSuccess) => { value.waypoints.pop(); },
      (value: GrindingSuccess) => { value.waypoints[1].targetIndex = 2; },
      (value: GrindingSuccess) => { value.waypoints.splice(1, 0, { ...value.waypoints[1], targetIndex: null, kind: 'transfer' }); },
      (value: GrindingSuccess) => { value.path[1].joints[0] += 35; },
    ]) {
      const value = structuredClone(motion); mutate(value); clients.motion.mockResolvedValueOnce(value);
      await expect(planSurfacePath(input)).rejects.toThrow();
    }
  });
  it('returns no partial plan when IK cannot preserve all surface poses', async () => {
    clients.motion.mockResolvedValueOnce({ status: 'failure', code: 'unreachable', message: '第 3 个目标不可达。', failedTarget: 2 });
    await expect(planSurfacePath(input)).rejects.toThrow('第 3 个目标不可达');
  });
  it('takes an immutable request snapshot before asynchronous clients run', async () => {
    const value = structuredClone(input);
    clients.geometry.mockImplementationOnce(async () => {
      value.settings = { ...value.settings, uSpacingMm: 50 }; value.request.speedMmPerSecond = 500;
      return structuredClone(geometry);
    });
    const result = await planSurfacePath(value);
    expect(result.settings.uSpacingMm).toBe(2); expect(result.speedMmPerSecond).toBe(50);
    expect((clients.motion.mock.calls[0][0] as GrindingRequest).speedMmPerSecond).toBe(50);
  });
});
