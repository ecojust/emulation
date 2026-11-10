import type { JointAngles, Pose, RobConf } from '../robot/types';
import { getProgramProfile, PROGRAM_MODEL_IDS } from './program-templates';

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
  if (!id.trim() || id.length > 128 || program.points.some(point => point.id === id) || program.paths.some(path => path.id === id)) throw new Error('程序对象标识必须非空且唯一。');
}
function update(program: RobotProgram, patch: Partial<Omit<RobotProgram, 'modelId' | 'revision'>>): RobotProgram {
  return Object.freeze({ ...program, ...patch, revision: program.revision + 1 });
}
function pathObject(id: string, name: string, pointIds: readonly string[], planning?: PathPlanningSettings): ProgramPath {
  return Object.freeze({ id, name, pointIds: Object.freeze([...pointIds]), ...(planning ? { planning: Object.freeze({ ...planning }) } : {}) });
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
  const point: ProgramPoint = Object.freeze({ ...previous, ...cloneSnapshot(program.modelId, snapshot), revision: previous.revision + 1 });
  return { program: update(program, { points: Object.freeze(program.points.map(current => current.id === id ? point : current)) }), value: point };
}

export function removeProgramPoint(program: RobotProgram, id: string): RobotProgram {
  if (!program.points.some(point => point.id === id)) return program;
  const paths = program.paths.map(path => path.pointIds.includes(id)
    ? pathObject(path.id, path.name, path.pointIds.filter(pointId => pointId !== id), path.planning) : path);
  return update(program, { points: Object.freeze(program.points.filter(point => point.id !== id)), paths: Object.freeze(paths) });
}

/** Path points belong to their container; deleting it must not leave orphan poses. */
export function removeProgramPath(program: RobotProgram, id: string): RobotProgram {
  if (!program.paths.some(path => path.id === id)) return program;
  return update(program, { paths: Object.freeze(program.paths.filter(path => path.id !== id)),
    points: Object.freeze(program.points.filter(point => point.pathId !== id)) });
}

export function renameProgramPath(program: RobotProgram, id: string, name: string): RobotProgram {
  if (!program.paths.some(path => path.id === id)) throw new Error('Path 已不存在。');
  const nextName = pathName(name);
  if (program.paths.find(path => path.id === id)!.name === nextName) return program;
  return update(program, { paths: Object.freeze(program.paths.map(path => path.id === id ? pathObject(path.id, nextName, path.pointIds, path.planning) : path)) });
}

/** Validate the entire replacement before committing; failed planning never partly rewrites a Path. */
export function replaceProgramPathPoints(program: RobotProgram, pathId: string, snapshots: readonly ProgramSnapshot[], planning?: PathPlanningSettings): ProgramChange<readonly ProgramPoint[]> {
  const path = program.paths.find(item => item.id === pathId);
  if (!path) throw new Error('路径已不存在。');
  if (!snapshots.length || snapshots.length > 256) throw new Error('路径需包含1至256个目标点。');
  if (planning && (planning.kind !== 'grinding' || !Number.isFinite(planning.clearanceMm) || planning.clearanceMm < 0 || planning.clearanceMm > 1000)) throw new Error('路径规划间距无效。');
  snapshots.forEach(snapshot => cloneSnapshot(program.modelId, snapshot));
  let changed = update(program, { points: Object.freeze(program.points.filter(point => point.pathId !== pathId)),
    paths: Object.freeze(program.paths.map(item => item.id === pathId ? pathObject(item.id, item.name, [], planning) : item)) });
  const points: ProgramPoint[] = [];
  for (const snapshot of snapshots) { const next = captureProgramPointToPath(changed, pathId, snapshot); changed = next.program; points.push(next.value); }
  return { program: changed, value: Object.freeze(points) };
}
