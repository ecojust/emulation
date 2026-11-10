import type { JointAngles, MotionSample, Pose } from '../robot/types';
import type { ProgramPoint } from './program';

export interface PlannedLeg { path: MotionSample[]; durationSeconds: number; resolvedTarget?: { joints: JointAngles; pose: Pose } }
export interface ProgramLeg extends PlannedLeg {
  pointId: string;
  lineNumber: number;
  startSeconds: number;
}
export interface ProgramTimeline { legs: readonly ProgramLeg[]; durationSeconds: number }
export interface ProgramRunPlan {
  initial: ProgramTimeline;
  /** Starts at the final point: close to point one, then replay the remaining commands. */
  cycle: ProgramTimeline | null;
}
export interface ProgramPlanningProgress { completed: number; total: number; pointId: string; lineNumber: number; closing: boolean }
export class ProgramRunError extends Error {
  constructor(public readonly pointId: string, public readonly lineNumber: number, message: string, public readonly closing = false) {
    super(`第 ${lineNumber} 条指令${closing ? '（循环回到首点）' : ''}：${message}`);
    this.name = 'ProgramRunError';
  }
}
const MAX_POINTS = 256;
const MAX_SAMPLES = 200_000;
export const PROGRAM_JOINT_TOLERANCE_DEG = 1e-3;
const finiteJoints = (value: unknown): value is JointAngles => Array.isArray(value) && value.length === 6 && Array.from(value).every(v => typeof v === 'number' && Number.isFinite(v));
const near = (a: JointAngles, b: JointAngles, tolerance = PROGRAM_JOINT_TOLERANCE_DEG) => a.every((angle, index) => Math.abs(angle - b[index]) <= tolerance);
function checkAbort(signal: AbortSignal) { if (signal.aborted) throw new DOMException('程序运行已取消', 'AbortError'); }

function verifyLeg(value: PlannedLeg, from: JointAngles, target: ProgramPoint): PlannedLeg {
  if (!value || !Array.isArray(value.path) || !value.path.length || value.path.length > 20001
    || !Number.isFinite(value.durationSeconds) || value.durationSeconds < 0) throw new Error('运动规划返回了无效轨迹');
  let previous = -1;
  let previousJoints: JointAngles | undefined;
  for (const sample of value.path) {
    if (!sample || !Number.isFinite(sample.t) || sample.t < 0 || sample.t > 1 || sample.t <= previous || !finiteJoints(sample.joints)
      || !sample.pose || !Array.isArray(sample.pose.position) || sample.pose.position.length !== 3
      || !Array.from(sample.pose.position).every(Number.isFinite) || !Array.isArray(sample.pose.orientation) || sample.pose.orientation.length !== 4
      || !Array.from(sample.pose.orientation).every(Number.isFinite) || Math.abs(Math.hypot(...sample.pose.orientation) - 1) > 1e-6) throw new Error('运动规划返回了无效采样');
    if (previousJoints && sample.joints.some((angle, index) => Math.abs(angle - previousJoints![index]) > 15 + 1e-8)) throw new Error('运动采样包含不连续的关节跳变');
    previous = sample.t;
    previousJoints = sample.joints;
  }
  const first = value.path[0], last = value.path.at(-1)!;
  const targetJoints = value.resolvedTarget?.joints ?? target.joints;
  if (value.resolvedTarget) {
    const orientation = (q: readonly number[], p: readonly number[]) => Math.abs(q.reduce((sum, n, i) => sum + n * p[i], 0));
    if (!finiteJoints(targetJoints) || Math.hypot(...last.pose.position.map((n, i) => n - target.pose.position[i])) > .2
      || orientation(last.pose.orientation, target.pose.orientation) < Math.cos(.1 * Math.PI / 360)) throw new Error('变换后的轨迹没有到达目标工具位姿');
  }
  if (!near(first.joints, from) || !near(last.joints, targetJoints)) throw new Error('轨迹没有连续连接已记录的关节姿态');
  if (value.path.length > 1 && (Math.abs(first.t) > 1e-9 || Math.abs(last.t - 1) > 1e-9)) throw new Error('轨迹采样未覆盖完整运动段');
  if (value.path.length === 1 && !near(from, targetJoints)) throw new Error('单点轨迹不能代表非零运动');
  // A coincident target still occupies one display frame, so a one-point loop
  // never becomes a zero-duration busy loop. It does not invent robot motion.
  return { durationSeconds: Math.max(1 / 60, value.durationSeconds), path: value.path.map(sample => ({
    t: sample.t, joints: [...sample.joints], pose: { position: [...sample.pose.position], orientation: [...sample.pose.orientation] },
  })) };
}

function timeline(legs: readonly ProgramLeg[]): ProgramTimeline {
  let elapsed = 0;
  const placed = legs.map(leg => { const placed = { ...leg, startSeconds: elapsed }; elapsed += leg.durationSeconds; return placed; });
  return { legs: placed, durationSeconds: elapsed };
}

/** All legs, including loop closure, are planned before any playback starts. */
export async function planProgramRun(options: {
  points: readonly ProgramPoint[];
  startJoints: JointAngles;
  loop: boolean;
  signal: AbortSignal;
  planLeg: (from: JointAngles, target: ProgramPoint, signal: AbortSignal) => Promise<PlannedLeg>;
  onProgress?: (progress: ProgramPlanningProgress) => void;
}): Promise<ProgramRunPlan> {
  const { signal, planLeg, onProgress } = options;
  checkAbort(signal);
  const points = [...options.points];
  if (!points.length || points.length > MAX_POINTS || !finiteJoints(options.startJoints)) throw new Error('请选择包含 1 至 256 条指令的程序，并设置有效机器人姿态');
  const ids = new Set<string>();
  for (const point of points) {
    if (!point || !point.id || ids.has(point.id) || !finiteJoints(point.joints) || !point.pose || point.pose.position.length !== 3 || !point.pose.position.every(Number.isFinite) || point.pose.orientation.length !== 4 || !point.pose.orientation.every(Number.isFinite) || Math.abs(Math.hypot(...point.pose.orientation) - 1) > 1e-6) throw new Error('程序包含无效或重复的目标点');
    ids.add(point.id);
  }
  const total = points.length + Number(options.loop);
  let current = [...options.startJoints] as JointAngles;
  let sampleCount = 0;
  const initial: ProgramLeg[] = [];
  const runLeg = async (point: ProgramPoint, index: number, closing: boolean): Promise<ProgramLeg> => {
    checkAbort(signal);
    onProgress?.({ completed: closing ? points.length : index, total, pointId: point.id, lineNumber: index + 1, closing });
    checkAbort(signal);
    try {
      const result = await planLeg([...current], point, signal);
      checkAbort(signal);
      const checked = verifyLeg(result, current, point);
      sampleCount += checked.path.length;
      if (sampleCount > MAX_SAMPLES) throw new Error('程序采样过多，请拆分后运行');
      current = [...checked.path.at(-1)!.joints];
      return { ...checked, pointId: point.id, lineNumber: index + 1, startSeconds: 0 };
    } catch (cause) {
      checkAbort(signal);
      if (cause instanceof DOMException && cause.name === 'AbortError') throw cause;
      throw new ProgramRunError(point.id, index + 1, cause instanceof Error ? cause.message : '运动段无法完成', closing);
    }
  };
  for (let index = 0; index < points.length; index++) initial.push(await runLeg(points[index], index, false));
  const closing = options.loop ? await runLeg(points[0], 0, true) : null;
  checkAbort(signal);
  if (closing && !near(closing.path.at(-1)!.joints, initial[0].path.at(-1)!.joints)) throw new ProgramRunError(points[0].id, 1, '循环闭合姿态不连续', true);
  onProgress?.({ completed: total, total, pointId: points.at(-1)!.id, lineNumber: points.length, closing: false });
  return { initial: timeline(initial), cycle: closing ? timeline([closing, ...initial.slice(1)]) : null };
}

export interface ProgramFrame { joints: JointAngles; pointId: string; lineNumber: number; cycle: number; progress: number; finished: boolean }

/** Pure time sampling: React owns the animation clock and cancellation. */
export function sampleProgramRun(plan: ProgramRunPlan, elapsedSeconds: number): ProgramFrame {
  const time = Math.max(0, Number.isFinite(elapsedSeconds) ? elapsedSeconds : 0);
  let active = plan.initial, offset = time, cycle = 0;
  const finished = !plan.cycle && time >= plan.initial.durationSeconds;
  if (plan.cycle && time >= plan.initial.durationSeconds) {
    active = plan.cycle;
    const afterApproach = time - plan.initial.durationSeconds;
    cycle = Math.floor(afterApproach / active.durationSeconds) + 1;
    offset = afterApproach % active.durationSeconds;
  }
  const leg = active.legs.find(item => offset < item.startSeconds + item.durationSeconds) ?? active.legs.at(-1)!;
  const fraction = Math.max(0, Math.min(1, (offset - leg.startSeconds) / leg.durationSeconds));
  const path = leg.path;
  let low = 0, high = path.length - 1;
  while (low < high) { const middle = Math.floor((low + high) / 2); if (path[middle].t < fraction) low = middle + 1; else high = middle; }
  const after = path[low], before = path[Math.max(0, low - 1)];
  const ratio = after.t === before.t ? 0 : Math.max(0, Math.min(1, (fraction - before.t) / (after.t - before.t)));
  const joints = before.joints.map((angle, axis) => angle + (after.joints[axis] - angle) * ratio) as JointAngles;
  return { joints, pointId: leg.pointId, lineNumber: leg.lineNumber, cycle, progress: Math.max(0, Math.min(1, offset / active.durationSeconds)), finished };
}

/** View-only positioning, deliberately separate from linear-motion planning. */
export function sampleJointPositioning(from: JointAngles, to: JointAngles, fraction: number): JointAngles {
  if (!finiteJoints(from) || !finiteJoints(to) || !Number.isFinite(fraction)) throw new Error('定位姿态无效');
  const t = Math.max(0, Math.min(1, fraction));
  const eased = t * t * (3 - 2 * t);
  return from.map((angle, index) => angle + (to[index] - angle) * eased) as JointAngles;
}
