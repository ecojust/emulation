import { matrixToQuaternion, normalizeQuaternion, quaternionToMatrix } from '../robot/kinematics';
import type { Mat4, Pose, Quaternion, Vec3 } from '../robot/types';

/** Brand-neutral scene transforms. Public units are mm and scalar-first quaternions. */
export type FrameParent = { readonly kind: 'world' } | { readonly kind: 'frame'; readonly id: string } | { readonly kind: 'flange' };
export interface Attachment { readonly parent: FrameParent; readonly localPose: Pose }
export interface ReferenceFrame extends Attachment { readonly id: string; readonly name: string }
export interface ToolFrame extends Attachment {
  readonly id: string;
  readonly name: string;
  /** Mounting origin -> TCP; distinct from the parent's mounting transform. */
  readonly tcpPose: Pose;
  /** Capsule envelope from mounting origin to TCP, including spherical end caps. */
  readonly radiusMm: number;
}
export interface FrameGraph {
  readonly modelId: string;
  readonly revision: number;
  readonly frames: readonly ReferenceFrame[];
  readonly robotBase: Attachment;
  readonly tools: readonly ToolFrame[];
  readonly paths: Readonly<Record<string, Attachment>>;
  readonly activeToolId: string | null;
}
export type FrameObjectRef = { kind: 'robot' } | { kind: 'frame' | 'tool' | 'path'; id: string };
export interface FrameGraphChange<T> { graph: FrameGraph; value: T }
export type FrameGraphStore = Readonly<Record<string, FrameGraph>>;
export const FRAME_GRAPH_LIMITS = Object.freeze({ frames: 64, tools: 32, toolRadiusMm: 10000 });
export const WORLD_PARENT: FrameParent = Object.freeze({ kind: 'world' });
export const FLANGE_PARENT: FrameParent = Object.freeze({ kind: 'flange' });

function tuple(value: unknown, size: number, label: string): number[] {
  if (!Array.isArray(value) || value.length !== size || Array.from(value).some(n => typeof n !== 'number' || !Number.isFinite(n))) throw new Error(`${label}需要${size}个有限数值。`);
  return value;
}
export function identityPose(): Pose { return { position: [0, 0, 0], orientation: [1, 0, 0, 0] }; }
export function normalizePose(pose: Pose): Pose {
  if (!pose) throw new Error('需要有效的位姿。');
  return { position: [...tuple(pose.position, 3, '位置')] as Vec3, orientation: normalizeQuaternion(tuple(pose.orientation, 4, '四元数') as Quaternion) };
}
function frozenPose(pose: Pose): Pose {
  const result = normalizePose(pose);
  Object.freeze(result.position); Object.freeze(result.orientation); return Object.freeze(result);
}
export function poseMatrix(pose: Pose): Mat4 {
  const value = normalizePose(pose), matrix = quaternionToMatrix(value.orientation);
  [matrix[3], matrix[7], matrix[11]] = value.position;
  return matrix;
}
export function poseFromMatrix(matrix: Mat4): Pose {
  tuple(matrix, 16, '刚体矩阵');
  if (Math.abs(matrix[12]) + Math.abs(matrix[13]) + Math.abs(matrix[14]) + Math.abs(matrix[15] - 1) > 1e-8) throw new Error('矩阵必须为刚体齐次变换。');
  const columns = [[matrix[0], matrix[4], matrix[8]], [matrix[1], matrix[5], matrix[9]], [matrix[2], matrix[6], matrix[10]]];
  const dot = (a: number[], b: number[]) => a.reduce((sum, n, i) => sum + n * b[i], 0);
  const determinant = matrix[0] * (matrix[5] * matrix[10] - matrix[6] * matrix[9]) - matrix[1] * (matrix[4] * matrix[10] - matrix[6] * matrix[8]) + matrix[2] * (matrix[4] * matrix[9] - matrix[5] * matrix[8]);
  if (columns.some(c => Math.abs(dot(c, c) - 1) > 1e-7) || Math.abs(dot(columns[0], columns[1])) > 1e-7 || Math.abs(dot(columns[1], columns[2])) > 1e-7 || Math.abs(dot(columns[0], columns[2])) > 1e-7 || Math.abs(determinant - 1) > 1e-7) throw new Error('坐标系旋转不能含缩放、剪切或镜像。');
  return { position: [matrix[3], matrix[7], matrix[11]], orientation: matrixToQuaternion(matrix) };
}
function rotate(q: Quaternion, p: Vec3): Vec3 {
  const m = quaternionToMatrix(q);
  return [m[0] * p[0] + m[1] * p[1] + m[2] * p[2], m[4] * p[0] + m[5] * p[1] + m[6] * p[2], m[8] * p[0] + m[9] * p[1] + m[10] * p[2]];
}
export function composePose(parentPose: Pose, localPose: Pose): Pose {
  const a = normalizePose(parentPose), b = normalizePose(localPose), rotated = rotate(a.orientation, b.position);
  const [w, x, y, z] = a.orientation, [v, i, j, k] = b.orientation;
  return normalizePose({ position: a.position.map((n, index) => n + rotated[index]) as Vec3,
    orientation: [w * v - x * i - y * j - z * k, w * i + x * v + y * k - z * j, w * j - x * k + y * v + z * i, w * k + x * j - y * i + z * v] });
}
export function inversePose(pose: Pose): Pose {
  const { position, orientation: [w, x, y, z] } = normalizePose(pose), inverse: Quaternion = [w, -x, -y, -z];
  return normalizePose({ position: rotate(inverse, position.map(n => -n) as Vec3), orientation: inverse });
}
/** Resolve child in parent's coordinates, not a translation-only subtraction. */
export function relativePose(parentWorld: Pose, childWorld: Pose): Pose { return composePose(inversePose(parentWorld), childWorld); }
export function poseEulerZYX(pose: Pose): Vec3 {
  const [w, x, y, z] = normalizePose(pose).orientation;
  const sine = Math.max(-1, Math.min(1, 2 * (w * y - z * x))), pitch = Math.asin(sine), locked = Math.abs(sine) > 1 - 1e-10;
  const yaw = locked ? 2 * Math.atan2(z, w) : Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  const roll = locked ? 0 : Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  return [yaw, pitch, roll].map(n => n * 180 / Math.PI) as Vec3;
}
function identifier(id: string): string {
  if (typeof id !== 'string' || !id.trim() || id.length > 128 || ['__proto__', 'constructor', 'prototype'].includes(id)) throw new Error('对象标识必须为有效的非空字符串。');
  return id;
}
function nameValue(name: string): string {
  if (typeof name !== 'string' || !name.trim() || [...name.trim()].length > 80) throw new Error('对象名称须为1至80个字符。');
  return name.trim();
}
function frozenParent(parent: FrameParent): FrameParent {
  if (!parent || !['world', 'frame', 'flange'].includes(parent.kind)) throw new Error('父对象类型无效。');
  return parent.kind === 'frame' ? Object.freeze({ kind: 'frame', id: identifier(parent.id) }) : parent.kind === 'world' ? WORLD_PARENT : FLANGE_PARENT;
}
function attachment(parent: FrameParent, localPose: Pose): Attachment { return Object.freeze({ parent: frozenParent(parent), localPose: frozenPose(localPose) }); }
function revision(graph: FrameGraph, patch: Partial<Omit<FrameGraph, 'modelId' | 'revision'>>): FrameGraph { return Object.freeze({ ...graph, ...patch, revision: graph.revision + 1 }); }
function existingParent(graph: FrameGraph, parent: FrameParent, flangeAllowed = false): void {
  frozenParent(parent);
  if (parent.kind === 'flange' && !flangeAllowed) throw new Error('该对象不能挂接到机器人法兰。');
  if (parent.kind === 'frame' && !graph.frames.some(frame => frame.id === parent.id)) throw new Error('参考坐标系已不存在。');
}
function newId(graph: FrameGraph, id: string): string {
  identifier(id);
  if (graph.frames.some(frame => frame.id === id) || graph.tools.some(tool => tool.id === id) || Object.hasOwn(graph.paths, id)) throw new Error('对象标识已被使用。');
  return id;
}
export function createFrameGraph(modelId: string): FrameGraph {
  identifier(modelId);
  return Object.freeze({ modelId, revision: 0, frames: Object.freeze([]), robotBase: attachment(WORLD_PARENT, identityPose()), tools: Object.freeze([]), paths: Object.freeze({}), activeToolId: null });
}
export function replaceFrameGraph(store: FrameGraphStore, graph: FrameGraph): FrameGraphStore { return Object.freeze({ ...store, [identifier(graph.modelId)]: graph }); }
export function referenceWorldPose(graph: FrameGraph, id: string): Pose {
  const visited = new Set<string>();
  function visit(key: string): Pose {
    if (visited.has(key)) throw new Error('参考坐标系不能形成循环层级。');
    visited.add(key);
    const frame = graph.frames.find(value => value.id === key);
    if (!frame) throw new Error('参考坐标系已不存在。');
    if (frame.parent.kind === 'flange') throw new Error('参考坐标系不能挂接到法兰。');
    return composePose(frame.parent.kind === 'world' ? identityPose() : visit(frame.parent.id), frame.localPose);
  }
  return visit(id);
}
export function parentWorldPose(graph: FrameGraph, parent: FrameParent, flangeInBase?: Pose): Pose {
  existingParent(graph, parent, true);
  if (parent.kind === 'world') return identityPose();
  if (parent.kind === 'frame') return referenceWorldPose(graph, parent.id);
  if (!flangeInBase) throw new Error('工具挂接需要当前机器人的有效法兰位姿。');
  return composePose(robotBaseWorldPose(graph), flangeInBase);
}
export function robotBaseWorldPose(graph: FrameGraph): Pose {
  existingParent(graph, graph.robotBase.parent);
  return composePose(parentWorldPose(graph, graph.robotBase.parent), graph.robotBase.localPose);
}
export function pathAttachment(graph: FrameGraph, pathId: string): Attachment {
  identifier(pathId);
  return Object.hasOwn(graph.paths, pathId) ? graph.paths[pathId] : attachment(WORLD_PARENT, identityPose());
}
export function pathWorldPose(graph: FrameGraph, pathId: string): Pose {
  const path = pathAttachment(graph, pathId);
  existingParent(graph, path.parent);
  return composePose(parentWorldPose(graph, path.parent), path.localPose);
}
export function pathPointToWorld(graph: FrameGraph, pathId: string, localPoint: Pose): Pose { return composePose(pathWorldPose(graph, pathId), localPoint); }
export function worldPointToPath(graph: FrameGraph, pathId: string, worldPoint: Pose): Pose { return relativePose(pathWorldPose(graph, pathId), worldPoint); }
function toolById(graph: FrameGraph, id: string): ToolFrame {
  const tool = graph.tools.find(value => value.id === id);
  if (!tool) throw new Error('工具已不存在。');
  return tool;
}
export function toolWorldPose(graph: FrameGraph, id: string, flangeInBase?: Pose): Pose {
  const tool = toolById(graph, id);
  return composePose(parentWorldPose(graph, tool.parent, flangeInBase), tool.localPose);
}
export function toolTcpWorldPose(graph: FrameGraph, id: string, flangeInBase?: Pose): Pose { return composePose(toolWorldPose(graph, id, flangeInBase), toolById(graph, id).tcpPose); }
/** Flange -> selected tool TCP. No attached selected tool means the flange itself. */
export function activeToolTcpPose(graph: FrameGraph): Pose {
  if (graph.activeToolId === null) return identityPose();
  const tool = toolById(graph, graph.activeToolId);
  if (tool.parent.kind !== 'flange') throw new Error('当前工具尚未挂接到机器人法兰。');
  return composePose(tool.localPose, tool.tcpPose);
}
export function baseFlangeToWorldTcp(graph: FrameGraph, flangeInBase: Pose): Pose { return composePose(composePose(robotBaseWorldPose(graph), flangeInBase), activeToolTcpPose(graph)); }
export function worldTcpToBaseFlange(graph: FrameGraph, worldTcp: Pose): Pose { return composePose(relativePose(robotBaseWorldPose(graph), worldTcp), inversePose(activeToolTcpPose(graph))); }
export function addReferenceFrame(graph: FrameGraph, options: { id?: string; name?: string; parent?: FrameParent; localPose?: Pose } = {}): FrameGraphChange<ReferenceFrame> {
  if (graph.frames.length >= FRAME_GRAPH_LIMITS.frames) throw new Error('参考坐标系数量已达上限。');
  const parent = options.parent ?? WORLD_PARENT; existingParent(graph, parent);
  const value: ReferenceFrame = Object.freeze({ id: newId(graph, options.id ?? crypto.randomUUID()), name: nameValue(options.name ?? `参考坐标系 ${graph.frames.length + 1}`), ...attachment(parent, options.localPose ?? identityPose()) });
  return { graph: revision(graph, { frames: Object.freeze([...graph.frames, value]) }), value };
}
function radius(value: number): number {
  if (!Number.isFinite(value) || value <= 0 || value > FRAME_GRAPH_LIMITS.toolRadiusMm) throw new Error('工具包络半径必须大于0且不超过10000 mm。');
  return value;
}
export function addToolFrame(graph: FrameGraph, options: { id?: string; name?: string; localPose?: Pose; tcpPose?: Pose; radiusMm?: number } = {}): FrameGraphChange<ToolFrame> {
  if (graph.tools.length >= FRAME_GRAPH_LIMITS.tools) throw new Error('工具数量已达上限。');
  const value: ToolFrame = Object.freeze({ id: newId(graph, options.id ?? crypto.randomUUID()), name: nameValue(options.name ?? `工具 ${graph.tools.length + 1}`), ...attachment(WORLD_PARENT, options.localPose ?? identityPose()), tcpPose: frozenPose(options.tcpPose ?? { position: [0, 0, 100], orientation: [1, 0, 0, 0] }), radiusMm: radius(options.radiusMm ?? 10) });
  return { graph: revision(graph, { tools: Object.freeze([...graph.tools, value]) }), value };
}
/** Reparenting preserves both world position and world orientation, including a tool's TCP. */
export function reparentFrameObject(graph: FrameGraph, object: FrameObjectRef, parent: FrameParent, flangeInBase?: Pose): FrameGraph {
  existingParent(graph, parent, object.kind === 'tool');
  if (object.kind === 'tool' && parent.kind === 'frame') throw new Error('工具可放在世界坐标系或挂接到机器人法兰。');
  if (object.kind === 'frame') {
    const current = graph.frames.find(frame => frame.id === object.id);
    if (!current) throw new Error('参考坐标系已不存在。');
    let ancestor = parent;
    const visited = new Set<string>();
    while (ancestor.kind === 'frame') {
      if (ancestor.id === object.id || visited.has(ancestor.id)) throw new Error('不能将坐标系挂接到自身或其子节点。');
      visited.add(ancestor.id);
      const ancestorId = ancestor.id, ancestorFrame = graph.frames.find(frame => frame.id === ancestorId);
      if (!ancestorFrame) throw new Error('参考坐标系已不存在。');
      ancestor = ancestorFrame.parent;
    }
    const world = referenceWorldPose(graph, object.id), next = Object.freeze({ ...current, ...attachment(parent, relativePose(parentWorldPose(graph, parent), world)) });
    return revision(graph, { frames: Object.freeze(graph.frames.map(frame => frame.id === object.id ? next : frame)) });
  }
  if (object.kind === 'robot') return revision(graph, { robotBase: attachment(parent, relativePose(parentWorldPose(graph, parent), robotBaseWorldPose(graph))) });
  if (object.kind === 'path') {
    if (graph.frames.some(frame => frame.id === object.id) || graph.tools.some(tool => tool.id === object.id)) throw new Error('路径标识与其他对象重复。');
    const next = attachment(parent, relativePose(parentWorldPose(graph, parent), pathWorldPose(graph, object.id)));
    return revision(graph, { paths: Object.freeze({ ...graph.paths, [identifier(object.id)]: next }) });
  }
  const tool = toolById(graph, object.id), world = toolWorldPose(graph, object.id, flangeInBase);
  const next: ToolFrame = Object.freeze({ ...tool, ...attachment(parent, relativePose(parentWorldPose(graph, parent, flangeInBase), world)) });
  return revision(graph, { tools: Object.freeze(graph.tools.map(value => value.id === object.id ? next : value)), activeToolId: parent.kind === 'flange' ? object.id : graph.activeToolId === object.id ? null : graph.activeToolId });
}
export function editReferenceFrame(graph: FrameGraph, id: string, patch: { name?: string; localPose?: Pose }): FrameGraph {
  if (!graph.frames.some(frame => frame.id === id)) throw new Error('参考坐标系已不存在。');
  return revision(graph, { frames: Object.freeze(graph.frames.map(frame => frame.id === id ? Object.freeze({ ...frame, name: patch.name === undefined ? frame.name : nameValue(patch.name), localPose: patch.localPose === undefined ? frame.localPose : frozenPose(patch.localPose) }) : frame)) });
}
export function editRobotBase(graph: FrameGraph, localPose: Pose): FrameGraph { return revision(graph, { robotBase: attachment(graph.robotBase.parent, localPose) }); }
export function editPathAttachment(graph: FrameGraph, pathId: string, localPose: Pose): FrameGraph {
  if (graph.frames.some(frame => frame.id === pathId) || graph.tools.some(tool => tool.id === pathId)) throw new Error('路径标识与其他对象重复。');
  const current = pathAttachment(graph, pathId);
  return revision(graph, { paths: Object.freeze({ ...graph.paths, [pathId]: attachment(current.parent, localPose) }) });
}
export function removePathAttachment(graph: FrameGraph, pathId: string): FrameGraph {
  if (!Object.hasOwn(graph.paths, pathId)) return graph;
  const paths = { ...graph.paths }; delete paths[pathId];
  return revision(graph, { paths: Object.freeze(paths) });
}
export function editToolFrame(graph: FrameGraph, id: string, patch: { name?: string; localPose?: Pose; tcpPose?: Pose; radiusMm?: number }): FrameGraph {
  const tool = toolById(graph, id);
  const next = Object.freeze({ ...tool, name: patch.name === undefined ? tool.name : nameValue(patch.name), localPose: patch.localPose === undefined ? tool.localPose : frozenPose(patch.localPose), tcpPose: patch.tcpPose === undefined ? tool.tcpPose : frozenPose(patch.tcpPose), radiusMm: patch.radiusMm === undefined ? tool.radiusMm : radius(patch.radiusMm) });
  return revision(graph, { tools: Object.freeze(graph.tools.map(value => value.id === id ? next : value)) });
}
export function selectActiveTool(graph: FrameGraph, id: string | null): FrameGraph {
  if (id !== null && toolById(graph, id).parent.kind !== 'flange') throw new Error('请先将工具挂接到机器人法兰。');
  return graph.activeToolId === id ? graph : revision(graph, { activeToolId: id });
}
export function removeToolFrame(graph: FrameGraph, id: string): FrameGraph {
  if (!graph.tools.some(tool => tool.id === id)) return graph;
  return revision(graph, { tools: Object.freeze(graph.tools.filter(tool => tool.id !== id)), activeToolId: graph.activeToolId === id ? null : graph.activeToolId });
}
/** Removing a frame detaches its direct children to world without moving them. */
export function removeReferenceFrame(graph: FrameGraph, id: string): FrameGraph {
  if (!graph.frames.some(frame => frame.id === id)) return graph;
  let next = graph;
  for (const child of graph.frames.filter(frame => frame.parent.kind === 'frame' && frame.parent.id === id)) next = reparentFrameObject(next, { kind: 'frame', id: child.id }, WORLD_PARENT);
  if (next.robotBase.parent.kind === 'frame' && next.robotBase.parent.id === id) next = reparentFrameObject(next, { kind: 'robot' }, WORLD_PARENT);
  for (const [pathId, path] of Object.entries(next.paths)) if (path.parent.kind === 'frame' && path.parent.id === id) next = reparentFrameObject(next, { kind: 'path', id: pathId }, WORLD_PARENT);
  return revision(next, { frames: Object.freeze(next.frames.filter(frame => frame.id !== id)) });
}

/** World capsule endpoints use the tool mounting origin, not the robot flange. */
export function toolEnvelopeWorld(graph: FrameGraph, id: string, flangeInBase?: Pose): { startMm: Vec3; endMm: Vec3; radiusMm: number } {
  const tool = toolById(graph, id);
  return { startMm: toolWorldPose(graph, id, flangeInBase).position, endMm: toolTcpWorldPose(graph, id, flangeInBase).position, radiusMm: tool.radiusMm };
}
