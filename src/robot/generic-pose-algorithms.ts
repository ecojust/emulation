import { composePose, identityPose, inversePose, poseMatrix } from '../workspace/frame-graph';
import { forwardGenericUnchecked, GenericMotionCancelled, genericJointsWithinLimits, solveGenericTaskUnchecked, solveGenericUnchecked, validateGenericChain } from './generic-kinematics';
import { quaternionDistanceDeg, slerpQuaternion } from './kinematics';
import type { GenericEndpointRequest, GenericIkSolution, GenericPlanFailure, GenericPosePlanRequest, GenericPosePlanResult } from './generic-motion-types';
import type { JointAngles, MotionSample, Pose, Vec3 } from './types';

const NEVER_CANCEL = () => false;
export function validGenericPose(value: unknown): value is Pose {
  if (!value || typeof value !== 'object') return false;
  const pose = value as Pose;
  return Array.isArray(pose.position) && pose.position.length === 3 && Array.from(pose.position).every(v => Number.isFinite(v) && Math.abs(v) <= 1e6)
    && Array.isArray(pose.orientation) && pose.orientation.length === 4 && Array.from(pose.orientation).every(Number.isFinite)
    && Math.abs(Math.hypot(...pose.orientation) - 1) <= 1e-6;
}
export function genericSolution(joints: JointAngles, pose: Pose, target: Pose, seed: JointAngles): GenericIkSolution {
  // Round halves away from zero to match Rust, including negative zero canonicalization.
  const rounded = (value: number) => String(Math.sign(value) * Math.floor(Math.abs(value) * 1000 + 0.5));
  return { id: `ik:${joints.map(rounded).join(':')}`, joints, pose,
    positionErrorMm: Math.hypot(...pose.position.map((v, i) => v - target.position[i])),
    orientationErrorDeg: quaternionDistanceDeg(pose.orientation, target.orientation),
    score: Math.hypot(...joints.map((v, i) => v - seed[i])) };
}
function halton(index: number, base: number): number {
  let fraction = 1, result = 0;
  while (index > 0) { fraction /= base; result += fraction * (index % base); index = Math.floor(index / base); }
  return result;
}
/** A bounded set of numerical candidates, never an exhaustive controller configuration list. */
export function inspectGenericEndpointReference(request: GenericEndpointRequest, cancel = NEVER_CANCEL): GenericIkSolution[] {
  if (cancel()) throw new GenericMotionCancelled();
  validateGenericChain(request.chain);
  if (!validGenericPose(request.pose) || !genericJointsWithinLimits(request.chain, request.seedJoints)) throw new Error('数值逆解的姿态或起始关节无效。');
  const clamp = (joints: JointAngles) => joints.map((v, i) => Math.max(request.chain.limits[i].minDeg, Math.min(request.chain.limits[i].maxDeg, v))) as JointAngles;
  const seeds = [request.seedJoints, clamp([0, 0, 0, 0, 0, 0]), request.chain.limits.map(limit => (limit.minDeg + limit.maxDeg) / 2) as JointAngles];
  for (let axis = 0; axis < 6; axis++) for (const offset of [-90, 90]) { const joints = [...request.seedJoints] as JointAngles; joints[axis] += offset; seeds.push(clamp(joints)); }
  const remainder = 64 - seeds.length;
  for (let index = 1; index <= remainder; index++) seeds.push(request.chain.limits.map((limit, axis) => limit.minDeg + halton(index, [2, 3, 5, 7, 11, 13][axis]) * (limit.maxDeg - limit.minDeg)) as JointAngles);
  const candidates: GenericIkSolution[] = [];
  for (const seed of seeds) {
    if (cancel()) throw new GenericMotionCancelled();
    const joints = solveGenericUnchecked(request.chain, request.pose, seed, cancel);
    if (!joints || candidates.some(candidate => candidate.joints.every((v, i) => Math.abs(v - joints[i]) <= 0.001))) continue;
    const fk = forwardGenericUnchecked(request.chain, joints, cancel);
    const candidate = genericSolution(joints, fk.pose, request.pose, request.seedJoints);
    if (genericJointsWithinLimits(request.chain, joints) && candidate.positionErrorMm <= 0.001001 && candidate.orientationErrorDeg <= 0.00058) candidates.push(candidate);
    if (candidates.length >= 32) break;
  }
  if (cancel()) throw new GenericMotionCancelled();
  return candidates.sort((a, b) => a.score - b.score || a.joints.map((v, i) => v - b.joints[i]).find(value => value !== 0) || 0);
}
function failure(code: GenericPlanFailure['code'], message: string, failedAt?: number): GenericPlanFailure { return { status: 'failure', code, message, ...(failedAt === undefined ? {} : { failedAt }) }; }
function cancelled(): GenericPlanFailure { return failure('cancelled', '运动计算已取消，未返回部分路径。'); }

function planTarget(request: GenericPosePlanRequest, start: Pose, target: Pose, cancel: () => boolean): GenericPosePlanResult {
  const base = request.basePose ?? identityPose(), tool = request.toolPose ?? identityPose();
  const baseInverse = inversePose(base), toolMatrix = poseMatrix(tool);
  const difference = target.position.map((v, i) => v - start.position[i]);
  const lengthMm = Math.hypot(...difference), orientationTravelDeg = quaternionDistanceDeg(start.orientation, target.orientation);
  const segments = Math.max(1, Math.ceil(lengthMm / 5), Math.ceil(orientationTravelDeg / 2));
  if (segments >= 1601) return failure('invalid_input', '运动长度或姿态变化超出 1601 个采样点的单段上限。');
  const path: MotionSample[] = [{ t: 0, pose: start, joints: [...request.startJoints] }];
  let previous = request.startJoints, durationSeconds = lengthMm / request.speedMmPerSecond;
  for (let index = 1; index <= segments; index++) {
    if (cancel()) return cancelled();
    const t = index / segments;
    const desired: Pose = { position: start.position.map((v, i) => v + difference[i] * t) as Vec3, orientation: slerpQuaternion(start.orientation, target.orientation, t) };
    const baseTcpTarget = composePose(baseInverse, desired);
    const joints = solveGenericTaskUnchecked(request.chain, baseTcpTarget, previous, toolMatrix, false, cancel);
    if (!joints) return failure('path_unreachable', 'TCP 直线上的数值逆解未收敛；没有返回部分运动。', t);
    if (joints.some((v, i) => Math.abs(v - previous[i]) > 15 + 1e-8)) return failure('configuration_change', '相邻采样的关节变化超过 15°，未通过连续性检查。', t);
    const actual = composePose(composePose(base, forwardGenericUnchecked(request.chain, joints, cancel).pose), tool);
    if (Math.hypot(...actual.position.map((v, i) => v - desired.position[i])) > 0.001001 || quaternionDistanceDeg(actual.orientation, desired.orientation) > 0.00058) return failure('path_unreachable', '带工具偏置的 TCP 采样未达到位置精度要求。', t);
    for (let axis = 0; axis < 6; axis++) durationSeconds = Math.max(durationSeconds, Math.abs(joints[axis] - previous[axis]) / request.chain.limits[axis].maxSpeedDeg * segments);
    path.push({ t, pose: actual, joints }); previous = joints;
  }
  if (cancel()) return cancelled();
  const last = path.at(-1)!;
  return { status: 'success', path, lengthMm, orientationTravelDeg, durationSeconds,
    selected: genericSolution(last.joints, last.pose, target, request.startJoints), automaticOrientation: request.target.orientation === null,
    warnings: ['URDF 数值逆解与离散 TCP 直线采样；未验证完整路径碰撞、控制器配置、加速度或动力学。'] };
}
/** World-TCP straight interpolation, with previous joints as the seed at every accepted sample. */
export function planGenericPoseMoveL(request: GenericPosePlanRequest, cancel = NEVER_CANCEL): GenericPosePlanResult {
  if (cancel()) return cancelled();
  try { validateGenericChain(request?.chain); } catch (error) { return failure('invalid_input', error instanceof Error ? error.message : 'URDF 运动链无效。'); }
  if (!genericJointsWithinLimits(request.chain, request.startJoints)) return failure('joint_limit', '起始关节超出 URDF 限位。');
  if (!validGenericPose({ position: request.target?.position, orientation: request.target?.orientation ?? [1, 0, 0, 0] })
    || !validGenericPose(request.basePose ?? identityPose()) || !validGenericPose(request.toolPose ?? identityPose())
    || !Number.isFinite(request.speedMmPerSecond) || request.speedMmPerSecond < 1 || request.speedMmPerSecond > 2000) return failure('invalid_input', 'TCP 目标、坐标变换或运动速度无效。');
  try {
    const base = request.basePose ?? identityPose(), tool = request.toolPose ?? identityPose();
    const start = composePose(composePose(base, forwardGenericUnchecked(request.chain, request.startJoints, cancel).pose), tool);
    const target: Pose = { position: request.target.position, orientation: request.target.orientation ?? start.orientation };
    const first = planTarget(request, start, target, cancel);
    if (request.target.orientation !== null || first.status === 'success') return first;
    if (cancel()) return cancelled();
    const joints = solveGenericTaskUnchecked(request.chain, composePose(inversePose(base), target), request.startJoints, poseMatrix(tool), true, cancel);
    if (!joints) return first;
    const relaxed = composePose(composePose(base, forwardGenericUnchecked(request.chain, joints, cancel).pose), tool);
    return planTarget(request, start, { position: target.position, orientation: relaxed.orientation }, cancel);
  } catch (error) { if (error instanceof GenericMotionCancelled) return cancelled(); throw error; }
}
