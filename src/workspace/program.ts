import type { JointAngles, Pose, RobConf } from '../robot/types';
import { getProgramProfile, PROGRAM_MODEL_IDS } from './program-templates';
import { SURFACE_PATH_LIMITS, validateSurfacePath, validateSurfacePathSample, validateSurfaceUvSettings, type SurfacePathData, type SurfacePathPoint, type SurfacePathSample, type SurfaceUvSettings } from './surface-path';

export interface ProgramSnapshot {
  pose: Pose;
  joints: JointAngles;
  configuration: RobConf | null;
  speedMmPerSecond: number;
  /** Motion role: transfer moves are not surface-processing strokes. */
  motionRole?: 'process' | 'transfer';
}
export interface ProgramPoint extends ProgramSnapshot {
  readonly id: string;
  readonly pointName: string;
  readonly kind: 'MoveL';
  readonly pathId: string;
  readonly revision: number;
}
export interface PathPlanningSettings { readonly kind: 'grinding'; readonly clearanceMm: number }
export interface ProgramPath {
  readonly id: string;
  readonly name: string;
  /** Point references in this Path’s capture order. */
  readonly pointIds: readonly string[];
  readonly planning?: PathPlanningSettings;
  readonly surface?: SurfacePathData;
}
export interface RobotProgram {
  readonly modelId: string;
  readonly revision: number;
  readonly points: readonly ProgramPoint[];
  readonly paths: readonly ProgramPath[];
  readonly nextPointNumber: number;
  readonly nextPathNumber: number;
}
export type ProgramStore = Readonly<Record<string, RobotProgram>>;
export interface ProgramChange<T> { program: RobotProgram; value: T }

function checkModelId(modelId: string): void {
  getProgramProfile(modelId);
}
export function createRobotProgram(modelId: string): RobotProgram {
  checkModelId(modelId);
  return Object.freeze({ modelId, revision: 0, points: Object.freeze([]), paths: Object.freeze([]), nextPointNumber: 1, nextPathNumber: 1 });
}
export function createProgramStore(): ProgramStore {
  return Object.freeze(Object.fromEntries(PROGRAM_MODEL_IDS.map(modelId => [modelId, createRobotProgram(modelId)])));
}
export function replaceRobotProgram(store: ProgramStore, program: RobotProgram): ProgramStore {
  checkModelId(program.modelId);
  if (store[program.modelId] === program) return store;
  return Object.freeze({ ...store, [program.modelId]: program });
}

function finiteTuple(value: unknown, length: number): value is number[] {
  return Array.isArray(value) && value.length === length && Array.from(value).every(item => typeof item === 'number' && Number.isFinite(item));
}
function samePose(a: Pose, b: Pose, tolerance: number): boolean {
  return Boolean(a && b && finiteTuple(b.position, 3) && finiteTuple(b.orientation, 4)
    && a.position.every((value, index) => Math.abs(value - b.position[index]) <= tolerance)
    && a.orientation.every((value, index) => Math.abs(value - b.orientation[index]) <= tolerance));
}
function samePhysicalPose(a: Pose, b: Pose, tolerance: number): boolean {
  return samePose(a, b, tolerance) || samePose(a, { ...b, orientation: b.orientation.map(value => -value) as Pose['orientation'] }, tolerance);
}
/** Callers check FK/limits; only ABB snapshots carry its controller-specific robconf. */
function cloneSnapshot(modelId: string, snapshot: ProgramSnapshot): ProgramSnapshot {
  const abb = getProgramProfile(modelId).brand === 'ABB';
  if (!snapshot || !snapshot.pose || !finiteTuple(snapshot.pose.position, 3) || !finiteTuple(snapshot.pose.orientation, 4)
    || Math.abs(Math.hypot(...snapshot.pose.orientation) - 1) > 1e-6
    || !finiteTuple(snapshot.joints, 6)
    || !Number.isFinite(snapshot.speedMmPerSecond) || snapshot.speedMmPerSecond < 1 || snapshot.speedMmPerSecond > 2000) {
    throw new Error('记录程序点需要有效的姿态、六轴关节角和速度。');
  }
  if (abb && (!finiteTuple(snapshot.configuration, 4) || !snapshot.configuration.every(Number.isSafeInteger)
    || snapshot.configuration[3] < 0 || snapshot.configuration[3] > 7)) throw new Error('记录 ABB MoveL 需要有效的 robconf；奇异或未求解姿态不能作为有效指令。');
  if (!abb && snapshot.configuration !== null) throw new Error('非 ABB 程序点不能携带 ABB robconf。');
  // Runtime freezing prevents a consumer from mutating saved snapshots via nested tuples.
  const pose: Pose = { position: [...snapshot.pose.position], orientation: [...snapshot.pose.orientation] };
  Object.freeze(pose.position); Object.freeze(pose.orientation); Object.freeze(pose);
  const joints: JointAngles = [...snapshot.joints]; Object.freeze(joints);
  const configuration: RobConf | null = snapshot.configuration ? [...snapshot.configuration] : null;
  if (configuration) Object.freeze(configuration);
  if (snapshot.motionRole !== undefined && !['process', 'transfer'].includes(snapshot.motionRole)) throw new Error('路径点的运动用途无效。');
  return { pose, joints, configuration, speedMmPerSecond: snapshot.speedMmPerSecond, ...(snapshot.motionRole ? { motionRole: snapshot.motionRole } : {}) };
}
function checkNewId(program: RobotProgram, id: string): void {
  if (!id.trim() || id.length > 128 || ['__proto__', 'constructor', 'prototype'].includes(id) || program.points.some(point => point.id === id) || program.paths.some(path => path.id === id || path.surface?.points.some(point => point.id === id))) throw new Error('程序对象标识必须非空且唯一。');
}
function update(program: RobotProgram, patch: Partial<Omit<RobotProgram, 'modelId' | 'revision'>>): RobotProgram {
  return Object.freeze({ ...program, ...patch, revision: program.revision + 1 });
}
function pathObject(id: string, name: string, pointIds: readonly string[], planning?: PathPlanningSettings, surface?: SurfacePathData): ProgramPath {
  return Object.freeze({ id, name, pointIds: Object.freeze([...pointIds]), ...(planning ? { planning: Object.freeze({ ...planning }) } : {}), ...(surface ? { surface } : {}) });
}
function pathName(name: string): string {
  const trimmed = name.trim();
  if (!trimmed || Array.from(trimmed).length > 80) throw new Error('Path 名称须为 1 至 80 个字符。');
  return trimmed;
}

/** Create an independent container before recording any robot poses. */
export function createProgramPath(program: RobotProgram, name?: string, id: string = crypto.randomUUID()): ProgramChange<ProgramPath> {
  checkNewId(program, id);
  const path = pathObject(id, pathName(name ?? `Path ${program.nextPathNumber}`), []);
  return { program: update(program, { paths: Object.freeze([...program.paths, path]), nextPathNumber: program.nextPathNumber + 1 }), value: path };
}

/** Append a frozen pose snapshot and its owning Path reference in one commit. */
export function captureProgramPointToPath(program: RobotProgram, pathId: string, snapshot: ProgramSnapshot, id: string = crypto.randomUUID()): ProgramChange<ProgramPoint> {
  const path = program.paths.find(item => item.id === pathId);
  if (!path) throw new Error('Path 已不存在，请重新选择。');
  if (path.surface) throw new Error('表面区域及UV路径不能直接记录机器人姿态；请使用普通Path示教，或完成表面路径求解。');
  checkNewId(program, id);
  if (!Number.isSafeInteger(program.nextPointNumber) || program.nextPointNumber < 1 || program.nextPointNumber > 99999) throw new Error('程序点编号已超出 P001 至 P99999 范围。');
  const point: ProgramPoint = Object.freeze({ ...cloneSnapshot(program.modelId, snapshot), id, pointName: `P${String(program.nextPointNumber).padStart(3, '0')}`, kind: 'MoveL', pathId, revision: 0 });
  const paths = program.paths.map(item => item.id === pathId ? pathObject(item.id, item.name, [...item.pointIds, point.id], item.planning) : item);
  return { program: update(program, { points: Object.freeze([...program.points, point]), paths: Object.freeze(paths), nextPointNumber: program.nextPointNumber + 1 }), value: point };
}

/** Resolve an explicit Path order without borrowing points from another Path. */
export function resolveProgramPathPoints(path: ProgramPath, points: readonly ProgramPoint[]): readonly ProgramPoint[] {
  if (new Set(path.pointIds).size !== path.pointIds.length) throw new Error('Path 包含重复的目标点引用。');
  const byId = new Map(points.map(point => [point.id, point]));
  if (byId.size !== points.length) throw new Error('目标点标识重复。');
  return Object.freeze(path.pointIds.map(id => {
    const point = byId.get(id);
    if (!point || point.pathId !== path.id) throw new Error('Path 包含已删除或不属于此 Path 的目标点。');
    return point;
  }));
}

export function getProgramPathPoints(program: RobotProgram, pathId: string): readonly ProgramPoint[] {
  const path = program.paths.find(item => item.id === pathId);
  if (!path) throw new Error('Path 已不存在。');
  return resolveProgramPathPoints(path, program.points);
}

export function updateProgramPoint(program: RobotProgram, id: string, snapshot: ProgramSnapshot): ProgramChange<ProgramPoint> {
  const previous = program.points.find(point => point.id === id);
  if (!previous) throw new Error('目标点已不存在。');
  const owner = program.paths.find(path => path.id === previous.pathId);
  if (owner?.surface?.kind === 'uv' && (!samePose(previous.pose, snapshot.pose, 0) || snapshot.motionRole === 'transfer')) throw new Error('表面路径目标的位置和法向姿态必须保留；只能更新同一目标的候选关节配置。');
  const point: ProgramPoint = Object.freeze({ ...previous, ...cloneSnapshot(program.modelId, snapshot), revision: previous.revision + 1 });
  return { program: update(program, { points: Object.freeze(program.points.map(current => current.id === id ? point : current)) }), value: point };
}

export function removeProgramPoint(program: RobotProgram, id: string): RobotProgram {
  if (!program.points.some(point => point.id === id)) return program;
  const owner = program.paths.find(path => path.pointIds.includes(id));
  if (owner?.surface?.kind === 'uv') {
    const sample = owner.surface.points[owner.pointIds.indexOf(id)];
    if (sample) return removeSurfacePathPoint(program, owner.id, sample.id);
  }
  const paths = program.paths.map(path => path.pointIds.includes(id)
    ? pathObject(path.id, path.name, path.pointIds.filter(pointId => pointId !== id), path.planning, path.surface) : path);
  return update(program, { points: Object.freeze(program.points.filter(point => point.id !== id)), paths: Object.freeze(paths) });
}

/** Path points belong to their container; deleting it must not leave orphan poses. */
export function removeProgramPath(program: RobotProgram, id: string): RobotProgram {
  if (!program.paths.some(path => path.id === id)) return program;
  const cleared = invalidateSurfaceDependants(program, id);
  return update(cleared, { paths: Object.freeze(cleared.paths.filter(path => path.id !== id)),
    points: Object.freeze(cleared.points.filter(point => point.pathId !== id)) });
}

export function renameProgramPath(program: RobotProgram, id: string, name: string): RobotProgram {
  if (!program.paths.some(path => path.id === id)) throw new Error('Path 已不存在。');
  const nextName = pathName(name);
  if (program.paths.find(path => path.id === id)!.name === nextName) return program;
  return update(program, { paths: Object.freeze(program.paths.map(path => path.id === id ? pathObject(path.id, nextName, path.pointIds, path.planning, path.surface) : path)) });
}

/** Validate the entire replacement before committing; failed planning never partly rewrites a Path. */
export function replaceProgramPathPoints(program: RobotProgram, pathId: string, snapshots: readonly ProgramSnapshot[], planning?: PathPlanningSettings): ProgramChange<readonly ProgramPoint[]> {
  const path = program.paths.find(item => item.id === pathId);
  if (!path) throw new Error('路径已不存在。');
  if (path.surface) throw new Error('表面路径必须通过表面目标专用求解保存，不能覆盖原始表面位姿。');
  if (!snapshots.length || snapshots.length > 256) throw new Error('路径需包含1至256个目标点。');
  if (planning && (planning.kind !== 'grinding' || !Number.isFinite(planning.clearanceMm) || planning.clearanceMm < 0 || planning.clearanceMm > 1000)) throw new Error('路径规划间距无效。');
  snapshots.forEach(snapshot => cloneSnapshot(program.modelId, snapshot));
  let changed = update(program, { points: Object.freeze(program.points.filter(point => point.pathId !== pathId)),
    paths: Object.freeze(program.paths.map(item => item.id === pathId ? pathObject(item.id, item.name, [], planning) : item)) });
  const points: ProgramPoint[] = [];
  for (const snapshot of snapshots) { const next = captureProgramPointToPath(changed, pathId, snapshot); changed = next.program; points.push(next.value); }
  return { program: changed, value: Object.freeze(points) };
}

export interface SurfacePathContext {
  readonly paths?: readonly ProgramPath[];
  /** Null means the referenced model has been deleted. Omitted means checked elsewhere. */
  readonly surfaceRevision?: number | null;
}
/** Read-only Path lookups remain available for inspectors; actions must use this gate. */
export function surfacePathDisabledReason(path: ProgramPath, context: SurfacePathContext = {}): string | null {
  const surface = path.surface;
  if (!surface) return null;
  if (surface.invalidated || context.surfaceRevision === null || context.surfaceRevision !== undefined && context.surfaceRevision !== surface.surfaceRevision) return '表面模型或区域已改变，请重新点选或生成路径。';
  if (surface.kind === 'region') return '这是点选区域，请先生成UV加工路径并求解后再运行或转换程序。';
  if (context.paths) {
    const source = context.paths.find(item => item.id === surface.source?.pathId)?.surface;
    if (!source || source.kind !== 'region' || source.invalidated || source.controlPointIds.length !== 4 || source.revision !== surface.source?.revision
      || source.surfaceId !== surface.surfaceId || source.surfaceRevision !== surface.surfaceRevision) return '源区域已改变或删除，请重新生成UV加工路径。';
  }
  if (surface.resolvedRevision !== surface.revision || path.pointIds.length !== surface.points.length) return 'UV路径尚未完成连续逆解，请先求解整条路径。';
  return null;
}
export function assertProgramPathReady(path: ProgramPath, context?: SurfacePathContext): void {
  const reason = surfacePathDisabledReason(path, context); if (reason) throw new Error(reason);
}

/** Structural save-time checks; missing models/sources are retained as stale geometry by the project layer. */
export function assertSurfacePathProgram(path: ProgramPath, points: readonly ProgramPoint[], allPaths: readonly ProgramPath[] = [path]): void {
  if (!path.surface) return;
  const surface = validateSurfacePath(path.surface);
  const ids = new Set<string>([...allPaths.map(item => item.id), ...points.map(point => point.id)]);
  for (const other of allPaths) for (const point of other.surface?.points ?? []) {
    if (ids.has(point.id)) throw new Error('表面点与场景路径或程序点标识重复。'); ids.add(point.id);
  }
  if (surface.kind === 'uv') {
    if (surface.resolvedRevision === undefined && path.pointIds.length) throw new Error('未求解或过期的UV路径不能保留可执行程序点。');
    if (surface.resolvedRevision !== undefined) {
      const resolved = resolveProgramPathPoints(path, points);
      if (resolved.length !== surface.points.length || resolved.some((point, index) => !samePose(point.pose, surface.points[index].pose, 0) || point.motionRole === 'transfer')) throw new Error('UV程序点必须完整对应原始表面目标。');
    }
  }
}

function requireSurfacePath(program: RobotProgram, pathId: string): ProgramPath & { surface: SurfacePathData } {
  const path = program.paths.find(item => item.id === pathId);
  if (!path?.surface) throw new Error('该Path没有表面区域或UV目标。');
  return path as ProgramPath & { surface: SurfacePathData };
}
function clearSurfaceSolution(path: ProgramPath, invalidated: boolean): ProgramPath {
  const surface = path.surface!;
  const { resolvedRevision: _resolved, ...geometry } = surface;
  const next = validateSurfacePath({ ...geometry, ...(invalidated ? { invalidated: true } : {}) });
  return pathObject(path.id, path.name, surface.kind === 'uv' ? [] : path.pointIds, path.planning, next);
}
function invalidateSurfaceDependants(program: RobotProgram, sourcePathId: string): RobotProgram {
  const changed = new Set(program.paths.filter(path => path.surface?.source?.pathId === sourcePathId && (!path.surface.invalidated || path.pointIds.length)).map(path => path.id));
  if (!changed.size) return program;
  return update(program, { paths: Object.freeze(program.paths.map(path => changed.has(path.id) ? clearSurfaceSolution(path, true) : path)),
    points: Object.freeze(program.points.filter(point => !changed.has(point.pathId))) });
}
function surfacePointCount(program: RobotProgram): number { return program.paths.reduce((sum, path) => sum + (path.surface?.points.length ?? 0), 0); }
function checkTotalPoints(program: RobotProgram): void {
  if (surfacePointCount(program) + program.points.length > 4096) throw new Error('项目中的程序点和表面点合计不能超过4096个。');
}
function withSurface(program: RobotProgram, pathId: string, surface: SurfacePathData | undefined): RobotProgram {
  const previous = program.paths.find(path => path.id === pathId);
  if (!previous) throw new Error('Path已不存在。');
  const clearPoints = previous.surface?.kind === 'uv';
  const updated = update(program, { paths: Object.freeze(program.paths.map(path => path.id === pathId
    ? pathObject(path.id, path.name, clearPoints ? [] : path.pointIds, path.planning, surface) : path)),
    ...(clearPoints ? { points: Object.freeze(program.points.filter(point => point.pathId !== pathId)) } : {}) });
  return invalidateSurfaceDependants(updated, pathId);
}
function sampledPoints(program: RobotProgram, samples: readonly SurfacePathSample[], ids?: readonly string[], startNumber = 1): readonly SurfacePathPoint[] {
  if (ids && ids.length !== samples.length) throw new Error('表面点标识数量不匹配。');
  const allocated = new Set<string>();
  return Object.freeze(samples.map((sample, index) => {
    const id = ids?.[index] ?? crypto.randomUUID(); checkNewId(program, id);
    if (allocated.has(id)) throw new Error('表面点标识重复。'); allocated.add(id);
    return Object.freeze({ ...validateSurfacePathSample(sample), id, name: `S${String(startNumber + index).padStart(3, '0')}` });
  }));
}

/** Append a whole pick batch atomically. No robot solve is invented for these targets. */
export function appendSurfacePathPoints(program: RobotProgram, pathId: string, samples: readonly SurfacePathSample[], ids?: readonly string[]): ProgramChange<readonly SurfacePathPoint[]> {
  const path = program.paths.find(item => item.id === pathId);
  if (!path) throw new Error('Path已不存在。');
  if (!samples.length || samples.length > SURFACE_PATH_LIMITS.regionPoints) throw new Error('点选区域需要1至4096个表面点。');
  const existing = path.surface;
  if (existing?.kind === 'uv') throw new Error('生成的UV路径不能作为点选区域，请新建Path。');
  if (existing?.invalidated) throw new Error('原点选区域已过期，请先清空区域或新建Path。');
  const nextNumber = Math.max(0, ...(existing?.points ?? []).map(point => Number(/^S(\d+)$/.exec(point.name)?.[1] ?? 0))) + 1;
  const points = sampledPoints(program, samples, ids, nextNumber), first = points[0].pick;
  const data = validateSurfacePath({ kind: 'region', revision: existing ? existing.revision + 1 : 0,
    surfaceId: existing?.surfaceId ?? first.surfaceId, surfaceRevision: existing?.surfaceRevision ?? first.surfaceRevision,
    points: [...existing?.points ?? [], ...points], controlPointIds: existing?.controlPointIds ?? [] });
  const changed = withSurface(program, pathId, data); checkTotalPoints(changed);
  return { program: changed, value: points };
}
export function setSurfacePathControl(program: RobotProgram, pathId: string, pointId: string, enabled: boolean): RobotProgram {
  const { surface } = requireSurfacePath(program, pathId);
  if (surface.kind !== 'region' || surface.invalidated) throw new Error('只能设置有效点选区域的控制点。');
  if (!surface.points.some(point => point.id === pointId)) throw new Error('表面点已不存在。');
  const selected = new Set(surface.controlPointIds);
  if (selected.has(pointId) === enabled) return program;
  if (enabled) selected.add(pointId); else selected.delete(pointId);
  if (selected.size > 4) throw new Error('一个UV区域最多设置四个控制点。');
  return withSurface(program, pathId, validateSurfacePath({ ...surface, revision: surface.revision + 1,
    controlPointIds: surface.points.filter(point => selected.has(point.id)).map(point => point.id) }));
}
export function removeSurfacePathPoint(program: RobotProgram, pathId: string, pointId: string): RobotProgram {
  const { surface } = requireSurfacePath(program, pathId);
  if (!surface.points.some(point => point.id === pointId)) return program;
  const points = surface.points.filter(point => point.id !== pointId);
  const { resolvedRevision: _resolved, ...geometry } = surface;
  return withSurface(program, pathId, points.length ? validateSurfacePath({ ...geometry, revision: surface.revision + 1, points,
    controlPointIds: surface.controlPointIds.filter(id => id !== pointId), ...(surface.kind === 'uv' ? { invalidated: true } : {}) }) : undefined);
}
export function clearSurfacePath(program: RobotProgram, pathId: string): RobotProgram {
  const path = program.paths.find(item => item.id === pathId);
  if (!path) throw new Error('Path已不存在。');
  return path.surface ? withSurface(program, pathId, undefined) : program;
}
export interface CreateSurfaceUvOptions {
  readonly name?: string;
  readonly id?: string;
  readonly pointIds?: readonly string[];
  readonly resolvedSnapshots?: readonly ProgramSnapshot[];
}
/** Create geometry and optional solved targets before returning any changed program. */
export function createSurfaceUvPath(program: RobotProgram, sourcePathId: string, expectedRevision: number, samples: readonly SurfacePathSample[], settings: SurfaceUvSettings, options: CreateSurfaceUvOptions = {}): ProgramChange<ProgramPath> {
  const source = requireSurfacePath(program, sourcePathId);
  if (source.surface.kind !== 'region' || source.surface.invalidated || source.surface.revision !== expectedRevision || source.surface.controlPointIds.length !== 4) throw new Error('源区域或四个控制点已改变，请重新生成。');
  if (!samples.length || samples.length > SURFACE_PATH_LIMITS.uvPoints) throw new Error('一次UV路径需包含1至256个表面目标，请调整间距。');
  const added = createProgramPath(program, options.name ?? `${[...source.name].slice(0, 77).join('')} UV`, options.id);
  const points = sampledPoints(added.program, samples, options.pointIds);
  const surface = validateSurfacePath({ kind: 'uv', revision: 0, surfaceId: source.surface.surfaceId, surfaceRevision: source.surface.surfaceRevision,
    points, controlPointIds: [], source: { pathId: sourcePathId, revision: expectedRevision }, settings: validateSurfaceUvSettings(settings) });
  let changed = withSurface(added.program, added.value.id, surface);
  if (options.resolvedSnapshots) changed = resolveSurfacePath(changed, added.value.id, surface.revision, options.resolvedSnapshots).program;
  checkTotalPoints(changed);
  return { program: changed, value: changed.paths.find(path => path.id === added.value.id)! };
}
/** A successful solve commits every point, preserving the exact requested surface poses. */
export function resolveSurfacePath(program: RobotProgram, pathId: string, expectedRevision: number, snapshots: readonly ProgramSnapshot[]): ProgramChange<readonly ProgramPoint[]> {
  const path = requireSurfacePath(program, pathId), surface = path.surface;
  const source = program.paths.find(item => item.id === surface.source?.pathId)?.surface;
  if (surface.kind !== 'uv' || surface.invalidated || surface.revision !== expectedRevision || !source || source.kind !== 'region' || source.invalidated || source.controlPointIds.length !== 4
    || source.revision !== surface.source?.revision || source.surfaceId !== surface.surfaceId || source.surfaceRevision !== surface.surfaceRevision) throw new Error('表面路径或源区域已改变，不能保存过期求解。');
  if (snapshots.length !== surface.points.length || snapshots.length > SURFACE_PATH_LIMITS.uvPoints) throw new Error('必须完整求解全部表面目标。');
  const cloned = snapshots.map((snapshot, index) => {
    const valid = cloneSnapshot(program.modelId, snapshot);
    if (snapshot.motionRole === 'transfer' || !samePhysicalPose(surface.points[index].pose, valid.pose, 1e-6)) throw new Error('求解结果偏离了原始表面目标，不能保存绕行点或部分路径。');
    return cloneSnapshot(program.modelId, { ...valid, pose: surface.points[index].pose, motionRole: 'process' });
  });
  if (program.nextPointNumber + cloned.length - 1 > 99999) throw new Error('程序点编号已超出P001至P99999范围。');
  const allocated = new Set<string>();
  const points = Object.freeze(cloned.map((snapshot, index) => {
    const id = crypto.randomUUID(); checkNewId(program, id); if (allocated.has(id)) throw new Error('程序点标识重复。'); allocated.add(id);
    return Object.freeze({ ...snapshot, id, pointName: `P${String(program.nextPointNumber + index).padStart(3, '0')}`, kind: 'MoveL' as const, pathId, revision: 0 });
  }));
  const changed = update(program, { points: Object.freeze([...program.points.filter(point => point.pathId !== pathId), ...points]),
    paths: Object.freeze(program.paths.map(item => item.id === pathId ? pathObject(item.id, item.name, points.map(point => point.id), item.planning,
      validateSurfacePath({ ...surface, resolvedRevision: surface.revision })) : item)), nextPointNumber: program.nextPointNumber + points.length });
  checkTotalPoints(changed);
  return { program: changed, value: points };
}
/** Robot base/tool edits invalidate solved joints without discarding usable surface geometry. */
export function invalidateSurfaceSolutions(program: RobotProgram): RobotProgram {
  const changed = new Set(program.paths.filter(path => path.surface?.kind === 'uv' && (path.surface.resolvedRevision !== undefined || path.pointIds.length)).map(path => path.id));
  if (!changed.size) return program;
  return update(program, { paths: Object.freeze(program.paths.map(path => changed.has(path.id) ? clearSurfaceSolution(path, false) : path)),
    points: Object.freeze(program.points.filter(point => !changed.has(point.pathId))) });
}
/** Mesh replacement, deletion or movement preserves visible stale geometry, never executable poses. */
export function invalidateSurfacePaths(program: RobotProgram, surfaceId: string): RobotProgram {
  const changed = new Set(program.paths.filter(path => path.surface?.surfaceId === surfaceId && (!path.surface.invalidated || path.surface.resolvedRevision !== undefined || path.surface.kind === 'uv' && path.pointIds.length)).map(path => path.id));
  if (!changed.size) return program;
  return update(program, { paths: Object.freeze(program.paths.map(path => changed.has(path.id) ? clearSurfaceSolution(path, true) : path)),
    points: Object.freeze(program.points.filter(point => !changed.has(point.pathId) || program.paths.find(path => path.id === point.pathId)?.surface?.kind !== 'uv')) });
}
/** Loading a project retains expired geometry and source text, clearing only unusable robot solutions. */
export function normalizeSurfacePathReferences(program: RobotProgram, surfaces: readonly { id: string; revision: number }[]): RobotProgram {
  const revisions = new Map(surfaces.map(surface => [surface.id, surface.revision]));
  let next = program;
  for (const path of next.paths) if (path.surface && revisions.get(path.surface.surfaceId) !== path.surface.surfaceRevision) next = invalidateSurfacePaths(next, path.surface.surfaceId);
  const stale = new Set(next.paths.filter(path => {
    const data = path.surface;
    if (data?.kind !== 'uv') return false;
    const source = next.paths.find(item => item.id === data.source?.pathId)?.surface;
    return !source || source.kind !== 'region' || source.invalidated || source.controlPointIds.length !== 4 || source.revision !== data.source?.revision
      || source.surfaceId !== data.surfaceId || source.surfaceRevision !== data.surfaceRevision;
  }).map(path => path.id));
  if (![...stale].some(id => { const path = next.paths.find(item => item.id === id)!; return !path.surface!.invalidated || path.pointIds.length; })) return next;
  return update(next, { paths: Object.freeze(next.paths.map(path => stale.has(path.id) ? clearSurfaceSolution(path, true) : path)),
    points: Object.freeze(next.points.filter(point => !stale.has(point.pathId))) });
}
