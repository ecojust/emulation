import { invoke, isTauri } from '@tauri-apps/api/core';
import type { GrindingRobot, GrindingRequest } from '../robot/grinding-types';
import { finiteTuple, validateGrindingRequest } from '../robot/grinding-validation';
import { forwardKinematics, quaternionDistanceDeg, quaternionToMatrix } from '../robot/kinematics';
import { forwardGenericKinematics } from '../robot/generic-kinematics';
import { getRobotModel } from '../robot/model';
import { composePose } from '../workspace/frame-graph';
import type { CollisionZone } from '../robot/collision-geometry';
import type { JointAngles, Pose, Vec3 } from '../robot/types';
import type { PendantInput } from './types';

export interface JogContext {
  robot: GrindingRobot;
  basePose: Pose;
  toolPose: Pose;
  toolRadiusMm: number;
  zones: CollisionZone[];
}
export interface JogRequest extends JogContext {
  startJoints: JointAngles;
  mode: 'linear' | 'joint';
  space: 'world' | 'tool';
  axes: JointAngles;
  speedScale: number;
  stepSeconds: number;
}
export interface JogSuccess { status: 'success'; joints: JointAngles; pose: Pose }
export interface JogFailure { status: 'failure'; code: string; message: string }
export type JogResult = JogSuccess | JogFailure;
const IDENTITY: Pose = { position: [0, 0, 0], orientation: [1, 0, 0, 0] };
export const PENDANT_STEP_SECONDS = .05;
export const PENDANT_INPUT_TTL_MS = 350;

function limits(request: JogRequest) {
  return request.robot.kind === 'abb' ? getRobotModel(request.robot.modelId).limits : request.robot.chain.limits;
}
function worldTcp(request: JogContext, joints: JointAngles): Pose {
  const flange = request.robot.kind === 'abb'
    ? forwardKinematics(getRobotModel(request.robot.modelId), joints).pose
    : forwardGenericKinematics(request.robot.chain, joints).pose;
  return composePose(composePose(request.basePose, flange), request.toolPose);
}
export function validateJogRequest(input: JogRequest): JogRequest {
  if (!input || !['linear', 'joint'].includes(input.mode) || !['world', 'tool'].includes(input.space)
    || !finiteTuple(input.axes, 6) || input.axes.some(axis => Math.abs(axis) > 1)
    || input.mode === 'linear' && input.axes.slice(3).some(axis => axis !== 0)
    || !Number.isFinite(input.speedScale) || input.speedScale < 0 || input.speedScale > 1
    || !Number.isFinite(input.stepSeconds) || input.stepSeconds <= 0 || input.stepSeconds > PENDANT_STEP_SECONDS) throw new Error('示教方向、速度或步长无效。');
  const validated = validateGrindingRequest({ ...input, targets: [IDENTITY], clearanceMm: 0, speedMmPerSecond: 100, allowDetours: false } as GrindingRequest);
  if (!finiteTuple(input.startJoints, 6) || input.startJoints.some((joint, index) => joint < limits(input)[index].minDeg || joint > limits(input)[index].maxDeg)) throw new Error('当前关节角超出型号限位。');
  return { robot: validated.robot, basePose: validated.basePose, toolPose: validated.toolPose,
    toolRadiusMm: validated.toolRadiusMm, zones: validated.zones, startJoints: [...validated.startJoints],
    mode: input.mode, space: input.space, axes: [...input.axes], speedScale: input.speedScale, stepSeconds: input.stepSeconds };
}
export function validateJogResult(value: unknown, request: JogRequest): JogResult {
  if (!value || typeof value !== 'object') throw new Error('原生示教结果无效。');
  const result = value as JogResult;
  if (result.status === 'failure') {
    if (typeof result.code !== 'string' || !result.code || typeof result.message !== 'string' || !result.message || Object.hasOwn(result, 'joints') || Object.hasOwn(result, 'pose')) throw new Error('原生示教失败结果无效。');
    return { status: 'failure', code: result.code, message: result.message };
  }
  if (result.status !== 'success' || !finiteTuple(result.joints, 6)
    || !result.pose || !finiteTuple(result.pose.position, 3) || !finiteTuple(result.pose.orientation, 4)
    || Math.abs(Math.hypot(...result.pose.orientation) - 1) > 1e-6) throw new Error('原生示教位姿无效。');
  const modelLimits = limits(request);
  if (result.joints.some((joint, index) => joint < modelLimits[index].minDeg || joint > modelLimits[index].maxDeg
    || Math.abs(joint - request.startJoints[index]) > modelLimits[index].maxSpeedDeg * request.stepSeconds + 1e-7)) throw new Error('原生示教关节超出限位或速度。');
  const actual = worldTcp(request, result.joints);
  if (Math.hypot(...actual.position.map((value, index) => value - result.pose.position[index])) > .002
    || quaternionDistanceDeg(actual.orientation, result.pose.orientation) > .002) throw new Error('原生示教关节与TCP结果不一致。');
  const magnitude = Math.max(1, Math.hypot(...request.axes));
  if (request.mode === 'joint') {
    const expected = request.startJoints.map((joint, index) => joint + request.axes[index] / magnitude * request.speedScale * request.stepSeconds * Math.min(30, modelLimits[index].maxSpeedDeg));
    if (result.joints.some((joint, index) => Math.abs(joint - expected[index]) > 1e-7)) throw new Error('原生轴模式没有保留指定关节增量。');
  } else {
    const before = worldTcp(request, request.startJoints);
    let direction = request.axes.slice(0, 3).map(axis => axis / magnitude) as Vec3;
    if (request.space === 'tool') {
      const matrix = quaternionToMatrix(before.orientation);
      direction = [0, 1, 2].map(row => direction.reduce((sum, axis, column) => sum + matrix[row * 4 + column] * axis, 0)) as Vec3;
    }
    const expected = before.position.map((value, index) => value + direction[index] * 100 * request.speedScale * request.stepSeconds);
    if (Math.hypot(...result.pose.position.map((value, index) => value - expected[index])) > .002
      || quaternionDistanceDeg(result.pose.orientation, before.orientation) > .002) throw new Error('原生线模式没有保留目标位置或工具方向。');
  }
  return { status: 'success', joints: [...result.joints], pose: { position: [...result.pose.position], orientation: [...result.pose.orientation] } };
}

/** Desktop-only command. A missing/failed native backend is never a JS fallback. */
export function jogPendant(input: JogRequest, signal?: AbortSignal): Promise<JogResult> {
  if (signal?.aborted) return Promise.reject(new DOMException('示教已停止', 'AbortError'));
  let request: JogRequest;
  try { request = validateJogRequest(input); } catch (error) { return Promise.reject(error); }
  if (!isTauri()) return Promise.reject(new Error('手机示教需要使用桌面客户端。'));
  return new Promise((resolve, reject) => {
    const requestId = crypto.randomUUID(); let settled = false;
    const cleanup = () => { clearTimeout(timer); signal?.removeEventListener('abort', abort); };
    const fail = (error: Error) => { if (settled) return; settled = true; cleanup(); void invoke('cancel_motion', { requestId }).catch(() => {}); reject(error); };
    const abort = () => fail(new DOMException('示教已停止', 'AbortError'));
    const timer = setTimeout(() => fail(new Error('示教求解超时，已停止移动。')), 1500);
    signal?.addEventListener('abort', abort, { once: true });
    if (signal?.aborted) { abort(); return; }
    void invoke('pendant_jog', { requestId, request }).then(value => {
      if (settled) return;
      try { const result = validateJogResult(value, request); settled = true; cleanup(); resolve(result); }
      catch (error) { fail(error instanceof Error ? error : new Error('原生示教结果无效。')); }
    }, error => fail(new Error(typeof error === 'string' ? error : '原生示教求解失败。')));
  });
}

export interface PendantMotionStatus { moving: boolean; error: string | null }
export interface PendantMotionOptions {
  sessionId: string;
  context: JogContext;
  initialJoints: JointAngles;
  solve?: (request: JogRequest, signal: AbortSignal) => Promise<JogResult>;
  onAccept(result: JogSuccess): void;
  onStatus(status: PendantMotionStatus): void;
}
export interface PendantMotionController {
  input(packet: PendantInput): void;
  /** External context/body changes discard pending work and require a release. */
  reset(context: JogContext, joints: JointAngles): void;
  stop(): void;
  dispose(): void;
}
/** One latest velocity, one bounded increment in flight. Slow solvers reduce
 * travel speed; elapsed backlog never becomes a larger catch-up displacement. */
export function createPendantMotionController(options: PendantMotionOptions): PendantMotionController {
  let context = structuredClone(options.context), accepted: JointAngles = [...options.initialJoints];
  let latest: PendantInput | null = null, active: AbortController | null = null;
  let epoch = -1, sequence = -1, generation = 0, armed = false, disposed = false;
  let lastDispatch = -Infinity, error: string | null = null;
  let previous: PendantMotionStatus = { moving: false, error: null };
  const fresh = (packet: PendantInput) => Date.now() - packet.receivedAtMs <= PENDANT_INPUT_TTL_MS && packet.receivedAtMs - Date.now() <= 100;
  function publish() {
    if (disposed) return;
    const next = { moving: Boolean(latest?.active && armed && latest.speed > 0 && Math.hypot(...latest.axes) > 1e-12), error };
    if (next.moving !== previous.moving || next.error !== previous.error) { previous = next; options.onStatus(next); }
  }
  function halt(message: string | null = null) {
    generation++; active?.abort(); active = null; latest = null; armed = false;
    lastDispatch = -Infinity; error = message; publish();
  }
  function tick() {
    if (disposed || !latest || !armed) return;
    if (!fresh(latest)) { halt('示教连接超时，已停止移动。'); return; }
    if (active || Date.now() - lastDispatch < PENDANT_STEP_SECONDS * 1000) return;
    if (!latest.active || latest.speed === 0 || Math.hypot(...latest.axes) < 1e-12) return;
    const packet = latest, expected = generation, controller = new AbortController();
    const request: JogRequest = { ...structuredClone(context), startJoints: [...accepted], mode: packet.mode,
      space: packet.space, axes: [...packet.axes], speedScale: packet.speed, stepSeconds: PENDANT_STEP_SECONDS };
    active = controller; lastDispatch = Date.now(); publish();
    void (async () => {
      try {
        const result = await (options.solve ?? jogPendant)(request, controller.signal);
        if (disposed || generation !== expected || controller.signal.aborted || !latest || !armed) return;
        if (!fresh(latest)) { halt('示教连接超时，已停止移动。'); return; }
        const verified = validateJogResult(result, request);
        if (verified.status === 'failure') { halt(verified.message); return; }
        accepted = [...verified.joints]; options.onAccept(verified);
      } catch (cause) {
        if (!disposed && generation === expected && !controller.signal.aborted) halt(cause instanceof Error ? cause.message : '示教求解失败，已停止移动。');
      } finally { if (active === controller) active = null; }
    })();
  }
  const timer = setInterval(tick, 25);
  return {
    input(packet) {
      if (disposed || packet.sessionId !== options.sessionId) return;
      if (!Number.isSafeInteger(packet.epoch) || packet.epoch < 0 || !Number.isSafeInteger(packet.sequence) || packet.sequence < 0
        || !Number.isFinite(packet.receivedAtMs) || !['linear', 'joint'].includes(packet.mode) || !['world', 'tool'].includes(packet.space)
        || !finiteTuple(packet.axes, 6) || packet.axes.some(axis => Math.abs(axis) > 1)
        || packet.mode === 'linear' && packet.axes.slice(3).some(axis => axis !== 0)
        || typeof packet.active !== 'boolean' || !Number.isFinite(packet.speed) || packet.speed < 0 || packet.speed > 1
        || !packet.active && packet.axes.some(axis => axis !== 0)) { halt('收到无效示教输入，已停止移动。'); return; }
      if (packet.epoch < epoch) return;
      if (packet.epoch > epoch) { halt(); epoch = packet.epoch; sequence = -1; }
      // A release stops even if it duplicates the final sequence; only a new
      // valid inactive packet can arm the next gesture.
      if (!packet.active) {
        const canArm = fresh(packet) && packet.sequence > sequence;
        halt(); sequence = Math.max(sequence, packet.sequence); armed = canArm; return;
      }
      if (packet.sequence <= sequence) return;
      sequence = packet.sequence;
      if (!fresh(packet)) { halt('示教输入已过期，请松开摇杆后重试。'); return; }
      if (!armed) return;
      if (latest && (packet.mode !== latest.mode || packet.space !== latest.space)) { halt('示教模式已改变，请松开摇杆后重试。'); return; }
      latest = structuredClone(packet); error = null; publish(); tick();
    },
    reset(nextContext, joints) { if (disposed) return; halt(); context = structuredClone(nextContext); accepted = [...joints]; },
    stop() { if (!disposed) halt(); },
    dispose() { if (disposed) return; halt(); disposed = true; clearInterval(timer); },
  };
}
