import type { JointAngles, Pose, RobConf } from '../robot/types';
import { getProgramProfile, PROGRAM_MODEL_IDS } from './program-templates';

export interface ProgramSnapshot {
  pose: Pose;
  joints: JointAngles;
  configuration: RobConf | null;
  speedMmPerSecond: number;
}
export interface ProgramPoint extends ProgramSnapshot {
  readonly id: string;
  readonly pointName: string;
  readonly kind: 'MoveL';
  readonly pathId: string | null;
  readonly revision: number;
}
export interface ProgramPath {
  readonly id: string;
  readonly name: string;
  /** References in program execution order, never selection-click order. */
  readonly pointIds: readonly string[];
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
  return { pose, joints, configuration, speedMmPerSecond: snapshot.speedMmPerSecond };
}
function checkNewId(program: RobotProgram, id: string): void {
  if (!id.trim() || id.length > 128 || program.points.some(point => point.id === id) || program.paths.some(path => path.id === id)) throw new Error('程序对象标识必须非空且唯一。');
}
function update(program: RobotProgram, patch: Partial<Omit<RobotProgram, 'modelId' | 'revision'>>): RobotProgram {
  return Object.freeze({ ...program, ...patch, revision: program.revision + 1 });
}
function pathObject(id: string, name: string, pointIds: readonly string[]): ProgramPath {
  return Object.freeze({ id, name, pointIds: Object.freeze([...pointIds]) });
}
function pathName(name: string): string {
  const trimmed = name.trim();
  if (!trimmed || Array.from(trimmed).length > 80) throw new Error('Path 名称须为 1 至 80 个字符。');
  return trimmed;
}

export function captureProgramPoint(program: RobotProgram, snapshot: ProgramSnapshot, afterPointId: string | null = null, id: string = crypto.randomUUID()): ProgramChange<ProgramPoint> {
  checkNewId(program, id);
  const afterIndex = afterPointId === null ? program.points.length - 1 : program.points.findIndex(point => point.id === afterPointId);
  if (afterPointId !== null && afterIndex < 0) throw new Error('插入位置已不存在，请重新选择程序行。');
  const point: ProgramPoint = Object.freeze({ ...cloneSnapshot(program.modelId, snapshot), id, pointName: `P${String(program.nextPointNumber).padStart(3, '0')}`, kind: 'MoveL', pathId: null, revision: 0 });
  const points = [...program.points]; points.splice(afterIndex + 1, 0, point);
  return { program: update(program, { points: Object.freeze(points), nextPointNumber: program.nextPointNumber + 1 }), value: point };
}

export function updateProgramPoint(program: RobotProgram, id: string, snapshot: ProgramSnapshot): ProgramChange<ProgramPoint> {
  const previous = program.points.find(point => point.id === id);
  if (!previous) throw new Error('目标点已不存在。');
  const point: ProgramPoint = Object.freeze({ ...previous, ...cloneSnapshot(program.modelId, snapshot), revision: previous.revision + 1 });
  return { program: update(program, { points: Object.freeze(program.points.map(current => current.id === id ? point : current)) }), value: point };
}

export function removeProgramPoint(program: RobotProgram, id: string): RobotProgram {
  if (!program.points.some(point => point.id === id)) return program;
  const paths = program.paths.flatMap(path => {
    if (!path.pointIds.includes(id)) return [path];
    const remaining = path.pointIds.filter(pointId => pointId !== id);
    return remaining.length ? [pathObject(path.id, path.name, remaining)] : [];
  });
  return update(program, { points: Object.freeze(program.points.filter(point => point.id !== id)), paths: Object.freeze(paths) });
}

export function convertProgramPointsToPath(program: RobotProgram, pointIds: readonly string[], name?: string, id: string = crypto.randomUUID()): ProgramChange<ProgramPath> {
  checkNewId(program, id);
  const selected = new Set(pointIds);
  if (!selected.size || [...selected].some(pointId => !program.points.some(point => point.id === pointId))) throw new Error('请先选择当前程序中存在的目标点。');
  const ordered = program.points.filter(point => selected.has(point.id)).map(point => point.id);
  const path = pathObject(id, pathName(name ?? `Path ${program.nextPathNumber}`), ordered);
  const paths = program.paths.flatMap(previous => {
    const remaining = previous.pointIds.filter(pointId => !selected.has(pointId));
    return remaining.length ? [remaining.length === previous.pointIds.length ? previous : pathObject(previous.id, previous.name, remaining)] : [];
  });
  paths.push(path);
  const points = program.points.map(point => selected.has(point.id) ? Object.freeze({ ...point, pathId: id }) : point);
  return { program: update(program, { points: Object.freeze(points), paths: Object.freeze(paths), nextPathNumber: program.nextPathNumber + 1 }), value: path };
}

export function removeProgramPath(program: RobotProgram, id: string): RobotProgram {
  if (!program.paths.some(path => path.id === id)) return program;
  return update(program, { paths: Object.freeze(program.paths.filter(path => path.id !== id)),
    points: Object.freeze(program.points.map(point => point.pathId === id ? Object.freeze({ ...point, pathId: null }) : point)) });
}

export function renameProgramPath(program: RobotProgram, id: string, name: string): RobotProgram {
  if (!program.paths.some(path => path.id === id)) throw new Error('Path 已不存在。');
  const nextName = pathName(name);
  if (program.paths.find(path => path.id === id)!.name === nextName) return program;
  return update(program, { paths: Object.freeze(program.paths.map(path => path.id === id ? pathObject(path.id, nextName, path.pointIds) : path)) });
}
