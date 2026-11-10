import { forwardGenericKinematics, genericJointsWithinLimits, validateGenericChain } from './generic-kinematics';
import { quaternionDistanceDeg, slerpQuaternion } from './kinematics';
import type { GenericChain, GenericPlanRequest, GenericPlanResult } from './generic-motion-types';
import type { JointAngles, Pose } from './types';

function record(value: unknown): value is Record<string, unknown> {
  return value !== null && typeof value === 'object' && !Array.isArray(value);
}
function finiteTuple(value: unknown, length: number): value is number[] {
  return Array.isArray(value) && value.length === length && Array.from(value).every(item => typeof item === 'number' && Number.isFinite(item));
}
function finite(value: unknown): value is number { return typeof value === 'number' && Number.isFinite(value); }
function validPose(value: unknown): value is Pose {
  return record(value) && finiteTuple(value.position, 3) && finiteTuple(value.orientation, 4)
    && value.position.every(coordinate => Math.abs(coordinate) <= 1_000_000)
    && Math.abs(Math.hypot(...value.orientation) - 1) <= 1e-6;
}
function distance(left: number[], right: number[]): number { return Math.hypot(...left.map((value, index) => value - right[index])); }
function close(actual: number, expected: number, tolerance = 1e-6): boolean { return Math.abs(actual - expected) <= tolerance * Math.max(1, Math.abs(expected)); }

/** Keep an immutable-by-ownership copy across the async worker/IPC boundary. */
export function validateGenericMotionRequest(value: unknown): GenericPlanRequest {
  if (!record(value) || !record(value.chain)) throw new Error('URDF 运动请求无效');
  const chain = value.chain as unknown as GenericChain;
  validateGenericChain(chain);
  if (!finiteTuple(value.startJoints, 6) || !finiteTuple(value.targetJoints, 6)
    || !genericJointsWithinLimits(chain, value.startJoints as JointAngles)
    || !genericJointsWithinLimits(chain, value.targetJoints as JointAngles)) throw new Error('记录的起点或目标关节超出 URDF 范围');
  if (!finite(value.speedMmPerSecond) || value.speedMmPerSecond < 1 || value.speedMmPerSecond > 2000) throw new Error('运动速度须在 1 至 2000 mm/s 之间');
  return {
    chain: { modelId: chain.modelId,
      steps: chain.steps.map(step => ({ origin: [...step.origin], axis: [...step.axis], jointIndex: step.jointIndex })),
      limits: chain.limits.map(limit => ({ ...limit })) as GenericChain['limits'] },
    startJoints: [...value.startJoints] as JointAngles, targetJoints: [...value.targetJoints] as JointAngles,
    speedMmPerSecond: value.speedMmPerSecond,
  };
}

const FAILURE_CODES = new Set(['invalid_input', 'joint_limit', 'unreachable', 'singular_start', 'singular_path', 'configuration_change', 'path_unreachable', 'cancelled']);

/** Independently verify every reported sample against URDF FK and the requested Cartesian line. */
export function validateGenericMotionResult(value: unknown, request: GenericPlanRequest): GenericPlanResult {
  if (!record(value)) throw new Error('URDF 运动结果无效');
  if (value.status === 'failure') {
    if (typeof value.code !== 'string' || !FAILURE_CODES.has(value.code)
      || typeof value.message !== 'string' || !value.message.trim() || value.message.length > 4096
      || (value.failedAt !== undefined && (!finite(value.failedAt) || value.failedAt < 0 || value.failedAt > 1))
      || Object.hasOwn(value, 'path')) throw new Error('URDF 运动失败结果无效');
    return value as unknown as GenericPlanResult;
  }
  if (value.status !== 'success' || !Array.isArray(value.path) || value.path.length < 2 || value.path.length > 1601
    || !finite(value.lengthMm) || value.lengthMm < 0 || !finite(value.orientationTravelDeg) || value.orientationTravelDeg < 0
    || !finite(value.durationSeconds) || value.durationSeconds < 0
    || !Array.isArray(value.warnings) || value.warnings.length > 32
    || !Array.from(value.warnings).every(item => typeof item === 'string' && item.length <= 4096)) throw new Error('URDF 运动采样或统计无效');
  const start = forwardGenericKinematics(request.chain, request.startJoints).pose;
  const target = forwardGenericKinematics(request.chain, request.targetJoints).pose;
  const length = distance(start.position, target.position);
  const angle = quaternionDistanceDeg(start.orientation, target.orientation);
  if (!close(value.lengthMm, length) || !close(value.orientationTravelDeg, angle, 1e-5)) throw new Error('URDF 运动长度与记录端点不一致');
  let previousT = -1;
  let previousJoints = request.startJoints;
  let duration = length / request.speedMmPerSecond;
  for (let index = 0; index < value.path.length; index++) {
    const sample: unknown = value.path[index];
    if (!record(sample) || !finite(sample.t) || sample.t < 0 || sample.t > 1 || sample.t <= previousT
      || !finiteTuple(sample.joints, 6) || !genericJointsWithinLimits(request.chain, sample.joints as JointAngles)
      || !validPose(sample.pose)) throw new Error('URDF 运动返回无效的关节或姿态采样');
    const joints = sample.joints as JointAngles;
    const t = sample.t;
    if (index === 0 && (sample.t !== 0 || joints.some((joint, axis) => Math.abs(joint - request.startJoints[axis]) > 1e-6))) throw new Error('URDF 运动起点不匹配');
    if (index === value.path.length - 1 && (sample.t !== 1 || joints.some((joint, axis) => Math.abs(joint - request.targetJoints[axis]) > 1e-6))) throw new Error('URDF 运动终点与记录的关节分支不匹配');
    const actual = forwardGenericKinematics(request.chain, joints).pose;
    const expectedPosition = start.position.map((coordinate, axis) => coordinate + (target.position[axis] - coordinate) * t);
    const expectedOrientation = slerpQuaternion(start.orientation, target.orientation, sample.t);
    if (distance(actual.position, sample.pose.position) > 1e-5 || quaternionDistanceDeg(actual.orientation, sample.pose.orientation) > 1e-4
      || distance(actual.position, expectedPosition) > 0.001001 || quaternionDistanceDeg(actual.orientation, expectedOrientation) > 0.00058) throw new Error('URDF 运动采样未通过实际 FK 与 TCP 直线复核');
    if (index > 0) {
      const deltaT = sample.t - previousT;
      if (length * deltaT > 5 + 1e-6 || angle * deltaT > 2 + 1e-6) throw new Error('URDF 运动采样间距超出范围');
      for (let axis = 0; axis < 6; axis++) {
        const delta = Math.abs(joints[axis] - previousJoints[axis]);
        if (delta > 15 + 1e-8) throw new Error('URDF 运动关节不连续');
        duration = Math.max(duration, delta / request.chain.limits[axis].maxSpeedDeg / deltaT);
      }
    }
    previousT = sample.t; previousJoints = joints;
  }
  if (!close(value.durationSeconds, duration)) throw new Error('URDF 运动时间未满足 TCP 与关节速度限制');
  return value as unknown as GenericPlanResult;
}
