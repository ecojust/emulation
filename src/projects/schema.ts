import catalog from '../catalog/catalog.json';
import { validateSourceProject, type SourceProject } from '../programming/documents';
import { composePose, normalizePose, pathPointToWorld, pathWorldPose, referenceWorldPose, robotBaseWorldPose, toolTcpWorldPose, toolWorldPose, type Attachment, type FrameGraph, type FrameParent } from '../workspace/frame-graph';
import { getProgramProfile } from '../workspace/program-templates';
import type { RobotProgram } from '../workspace/program';
import { createDefaultDangerZone, type DangerZoneDraft, type WorkspaceView } from '../workspace/useWorkspace';
import type { JointAngles, Pose, Vec3 } from '../robot/types';

export const PROJECT_LIMITS = { projects: 128, bytes: 64 * 1024 * 1024, thumbnailBytes: 1024 * 1024, points: 4096, paths: 256, coordinateMm: 1_000_000, cameraUnits: 1000 } as const;
export interface ProjectCamera { position: Vec3; target: Vec3 }
export interface ProjectModelScene { robotPresent: boolean; joints: JointAngles; graph: FrameGraph; program: RobotProgram }
export interface ProjectScene {
  activeModelId: string;
  models: Record<string, ProjectModelScene>;
  zones: DangerZoneDraft[];
  activeView: WorkspaceView;
  leftCollapsed: boolean;
  view: 'iso' | 'front' | 'top';
  showFrames: boolean;
  loop: boolean;
  speed: number;
  camera: ProjectCamera | null;
}
export interface SimulationProject {
  schemaVersion: 1;
  id: string;
  name: string;
  createdAt: string;
  updatedAt: string;
  revision: number;
  scene: ProjectScene;
  sources: Record<string, SourceProject>;
  /** Captured renderer pixels, never a stock model illustration. */
  thumbnail: string | null;
}
export const PROJECT_MODELS = catalog.models.map(({ id, name, brand }) => ({ id, name, brand }));
const knownModels = new Set(PROJECT_MODELS.map(model => model.id));
const record = (value: unknown): value is Record<string, any> => Boolean(value) && typeof value === 'object' && !Array.isArray(value);
function requireValue(condition: unknown, message: string): asserts condition { if (!condition) throw new Error('项目数据无效：' + message); }
function identifier(value: unknown): asserts value is string { requireValue(typeof value === 'string' && value.length > 0 && value.length <= 128 && !['__proto__', 'constructor', 'prototype'].includes(value), '对象标识'); }
function revision(value: unknown): asserts value is number { requireValue(Number.isSafeInteger(value) && Number(value) >= 0, '版本编号'); }
function tuple(value: unknown, size: number, bound = Number.MAX_VALUE): asserts value is number[] { requireValue(Array.isArray(value) && value.length === size && Array.from(value).every(n => typeof n === 'number' && Number.isFinite(n) && Math.abs(n) <= bound), '坐标或关节值'); }
function boundedPose(value: unknown): Pose {
  requireValue(record(value), '位姿'); tuple(value.position, 3, PROJECT_LIMITS.coordinateMm); tuple(value.orientation, 4);
  return normalizePose(value as unknown as Pose);
}
function modelId(value: unknown): asserts value is string { requireValue(typeof value === 'string' && knownModels.has(value), '机器人型号'); }
export function projectName(value: unknown): string {
  if (typeof value !== 'string' || !value.trim() || [...value.trim()].length > 80 || /[\x00-\x1f\x7f]/.test(value)) throw new Error('项目名称须为1至80个字符，不能包含控制字符。');
  return value.trim();
}
function attachment(raw: unknown, frameIds: Set<string>, tool = false): Attachment {
  requireValue(record(raw) && record(raw.parent), '挂接结构');
  const p = raw.parent;
  requireValue(p.kind === 'world' || p.kind === 'frame' && frameIds.has(p.id) && !tool || p.kind === 'flange' && tool, '挂接父级');
  const parent: FrameParent = p.kind === 'frame' ? { kind: 'frame', id: p.id } : { kind: p.kind };
  return { parent, localPose: boundedPose(raw.localPose) };
}
function validateProgram(raw: unknown, id: string): RobotProgram {
  requireValue(record(raw) && raw.modelId === id, '路径型号'); revision(raw.revision);
  requireValue(Array.isArray(raw.paths) && raw.paths.length <= PROJECT_LIMITS.paths && Array.isArray(raw.points) && raw.points.length <= PROJECT_LIMITS.points, '路径数量');
  revision(raw.nextPointNumber); revision(raw.nextPathNumber); requireValue(raw.nextPointNumber > 0 && raw.nextPathNumber > 0, '路径计数');
  const ids = new Set<string>();
  for (const object of [...raw.paths, ...raw.points]) { requireValue(record(object), '路径对象'); identifier(object.id); requireValue(!ids.has(object.id), '重复路径标识'); ids.add(object.id); }
  const refs = new Set<string>(), points = new Map(raw.points.map((point: any) => [point.id, point]));
  for (const path of raw.paths) {
    projectName(path.name); requireValue(Array.isArray(path.pointIds), '路径点列表');
    if (path.planning !== undefined) requireValue(path.planning?.kind === 'grinding' && Number.isFinite(path.planning.clearanceMm) && path.planning.clearanceMm >= 0 && path.planning.clearanceMm <= 1000, '打磨设置');
    for (const pointId of path.pointIds) { const point = points.get(pointId) as any; requireValue(point && point.pathId === path.id && !refs.has(pointId), '路径点引用'); refs.add(pointId); }
  }
  requireValue(refs.size === points.size, '孤立路径点');
  for (const point of raw.points) {
    requireValue(point.kind === 'MoveL', '运动类型'); revision(point.revision); projectName(point.pointName);
    tuple(point.joints, 6, 36000); boundedPose(point.pose);
    requireValue(Math.abs(Math.hypot(...point.pose.orientation) - 1) <= 1e-6, '目标方向四元数');
    requireValue(Number.isFinite(point.speedMmPerSecond) && point.speedMmPerSecond >= 1 && point.speedMmPerSecond <= 2000, '运动速度');
    requireValue(point.motionRole === undefined || ['process', 'transfer'].includes(point.motionRole), '运动用途');
    if (getProgramProfile(id).brand === 'ABB') {
      tuple(point.configuration, 4); requireValue(point.configuration.every(Number.isSafeInteger) && point.configuration[3] >= 0 && point.configuration[3] <= 7, 'ABB配置');
    } else requireValue(point.configuration === null, '其他品牌不可使用ABB配置');
  }
  return structuredClone(raw) as unknown as RobotProgram;
}
function validateModel(raw: unknown, id: string): ProjectModelScene {
  requireValue(record(raw) && typeof raw.robotPresent === 'boolean', '机器人状态'); tuple(raw.joints, 6, 36000);
  const program = validateProgram(raw.program, id), g = raw.graph;
  requireValue(record(g) && g.modelId === id, '坐标图型号'); revision(g.revision);
  requireValue(Array.isArray(g.frames) && g.frames.length <= 64 && Array.isArray(g.tools) && g.tools.length <= 32 && record(g.paths), '坐标图结构');
  const ids = new Set<string>(), frameIds = new Set<string>();
  for (const frame of g.frames) { requireValue(record(frame), '坐标系'); identifier(frame.id); requireValue(!ids.has(frame.id), '重复坐标对象'); ids.add(frame.id); frameIds.add(frame.id); }
  const frames = g.frames.map((frame: any) => ({ id: frame.id, name: projectName(frame.name), ...attachment(frame, frameIds) }));
  const tools = Array.from(g.tools).map((tool: any) => {
    requireValue(record(tool), '工具'); identifier(tool.id); requireValue(!ids.has(tool.id), '重复工具'); ids.add(tool.id);
    requireValue(Number.isFinite(tool.radiusMm) && tool.radiusMm > 0 && tool.radiusMm <= 10000, '工具包络');
    const mount = attachment(tool, frameIds, true); requireValue(raw.robotPresent || mount.parent.kind !== 'flange', '已删除机器人仍有挂载工具');
    return { id: tool.id, name: projectName(tool.name), ...mount, tcpPose: boundedPose(tool.tcpPose), radiusMm: tool.radiusMm };
  });
  const paths: Record<string, Attachment> = {};
  for (const [pathId, value] of Object.entries(g.paths)) { identifier(pathId); requireValue(program.paths.some(path => path.id === pathId) && !ids.has(pathId), '路径坐标引用'); paths[pathId] = attachment(value, frameIds); }
  requireValue(g.activeToolId === null || tools.some((tool: any) => tool.id === g.activeToolId && tool.parent.kind === 'flange'), '活动工具');
  const graph: FrameGraph = { modelId: id, revision: g.revision, frames, tools, paths, robotBase: attachment(g.robotBase, frameIds), activeToolId: g.activeToolId };
  for (const frame of frames) boundedPose(referenceWorldPose(graph, frame.id));
  boundedPose(robotBaseWorldPose(graph));
  for (const path of program.paths) boundedPose(pathWorldPose(graph, path.id));
  for (const point of program.points) boundedPose(pathPointToWorld(graph, point.pathId, point.pose));
  for (const tool of tools) {
    boundedPose(composePose(tool.localPose, tool.tcpPose));
    if (tool.parent.kind === 'world') { boundedPose(toolWorldPose(graph, tool.id)); boundedPose(toolTcpWorldPose(graph, tool.id)); }
  }
  return { robotPresent: raw.robotPresent, joints: [...raw.joints] as JointAngles, graph, program };
}
/** Run after the intended model's FK is available and before mounting an editor. */
export function validateProjectModelWorld(model: ProjectModelScene, flangeInBase: Pose): void {
  boundedPose(flangeInBase);
  if (model.robotPresent) boundedPose(composePose(robotBaseWorldPose(model.graph), flangeInBase));
  for (const tool of model.graph.tools) {
    boundedPose(toolWorldPose(model.graph, tool.id, flangeInBase));
    boundedPose(toolTcpWorldPose(model.graph, tool.id, flangeInBase));
  }
}
export function validateProjectScene(raw: unknown): ProjectScene {
  requireValue(record(raw) && record(raw.models), '场景'); modelId(raw.activeModelId);
  const models: Record<string, ProjectModelScene> = {};
  for (const [id, value] of Object.entries(raw.models)) { modelId(id); models[id] = validateModel(value, id); }
  requireValue(Array.isArray(raw.zones) && raw.zones.length <= 64, '禁入区列表');
  const zoneIds = new Set<string>();
  const zones = Array.from(raw.zones).map((zone: any) => {
    requireValue(record(zone), '禁入区'); identifier(zone.id); requireValue(!zoneIds.has(zone.id), '重复禁入区'); zoneIds.add(zone.id);
    requireValue(typeof zone.name === 'string' && zone.name.length <= 128 && typeof zone.enabled === 'boolean', '禁入区属性');
    for (const values of [zone.center, zone.size]) requireValue(Array.isArray(values) && values.length === 3 && Array.from(values).every(n => typeof n === 'string' && n.length <= 64), '禁入区输入');
    return structuredClone(zone) as DangerZoneDraft; // Preserve invalid numeric drafts; they stay unchecked in the workspace.
  });
  requireValue(['scene', 'program'].includes(raw.activeView) && ['iso', 'front', 'top'].includes(raw.view), '视图');
  for (const value of [raw.leftCollapsed, raw.showFrames, raw.loop]) requireValue(typeof value === 'boolean', '显示设置');
  requireValue(Number.isFinite(raw.speed) && raw.speed >= 1 && raw.speed <= 2000, '记录速度');
  if (raw.camera !== null) { requireValue(record(raw.camera), '相机'); tuple(raw.camera.position, 3, PROJECT_LIMITS.cameraUnits); tuple(raw.camera.target, 3, PROJECT_LIMITS.cameraUnits); requireValue(Math.hypot(...raw.camera.position.map((n: number, i: number) => n - raw.camera.target[i])) > 1e-7, '相机距离'); }
  return { activeModelId: raw.activeModelId, models, zones, activeView: raw.activeView, leftCollapsed: raw.leftCollapsed, view: raw.view, showFrames: raw.showFrames, loop: raw.loop, speed: raw.speed, camera: raw.camera ? structuredClone(raw.camera) as ProjectCamera : null };
}
export function validateThumbnail(value: unknown): asserts value is string {
  requireValue(typeof value === 'string' && value.length <= PROJECT_LIMITS.thumbnailBytes && /^data:image\/(?:png|jpeg);base64,[A-Za-z0-9+/]+={0,2}$/.test(value), '封面图格式或大小');
}
export function validateSimulationProject(raw: unknown): SimulationProject {
  requireValue(record(raw) && raw.schemaVersion === 1, '项目版本'); identifier(raw.id); const name = projectName(raw.name); revision(raw.revision);
  for (const time of [raw.createdAt, raw.updatedAt]) requireValue(typeof time === 'string' && Number.isFinite(Date.parse(time)), '保存时间');
  requireValue(record(raw.sources), '模块源码');
  const sources: Record<string, SourceProject> = {};
  for (const [id, source] of Object.entries(raw.sources)) { modelId(id); sources[id] = validateSourceProject(source, id); }
  if (raw.thumbnail !== null) validateThumbnail(raw.thumbnail);
  const result: SimulationProject = { schemaVersion: 1, id: raw.id, name, createdAt: raw.createdAt, updatedAt: raw.updatedAt, revision: raw.revision, scene: validateProjectScene(raw.scene), sources, thumbnail: raw.thumbnail };
  requireValue(new TextEncoder().encode(JSON.stringify(result)).byteLength <= PROJECT_LIMITS.bytes, '项目超过64 MiB');
  return result;
}
export function createSimulationProject(name: string, robotId: string, id: string = crypto.randomUUID(), now = new Date().toISOString()): SimulationProject {
  modelId(robotId);
  return validateSimulationProject({ schemaVersion: 1, id, name: projectName(name), createdAt: now, updatedAt: now, revision: 0, thumbnail: null, sources: {},
    scene: { activeModelId: robotId, models: {}, zones: [createDefaultDangerZone()], activeView: 'scene', leftCollapsed: false, view: 'iso', showFrames: false, loop: false, speed: 100, camera: null } });
}

export interface ProjectSummary {
  id: string;
  name: string;
  activeModelId: string;
  robotPresent: boolean;
  thumbnail: string | null;
  updatedAt: string;
  revision: number;
}
/** Small atomic index entry; the homepage never reads module text or full scenes. */
export function projectSummary(project: SimulationProject): ProjectSummary {
  return { id: project.id, name: project.name, activeModelId: project.scene.activeModelId,
    robotPresent: project.scene.models[project.scene.activeModelId]?.robotPresent ?? true,
    thumbnail: project.thumbnail, updatedAt: project.updatedAt, revision: project.revision };
}
export function validateProjectSummary(raw: unknown): ProjectSummary {
  requireValue(record(raw), '项目索引'); identifier(raw.id); modelId(raw.activeModelId); revision(raw.revision);
  requireValue(typeof raw.robotPresent === 'boolean' && typeof raw.updatedAt === 'string' && Number.isFinite(Date.parse(raw.updatedAt)), '项目索引属性');
  if (raw.thumbnail !== null) validateThumbnail(raw.thumbnail);
  return { id: raw.id, name: projectName(raw.name), activeModelId: raw.activeModelId, robotPresent: raw.robotPresent, thumbnail: raw.thumbnail, updatedAt: raw.updatedAt, revision: raw.revision };
}
