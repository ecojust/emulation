import { composePose, identityPose } from '../workspace/frame-graph';
import { forwardGenericKinematics, genericJointsWithinLimits } from './generic-kinematics';
import { validateGenericMotionRequest, validateGenericMotionResult } from './generic-motion-validation';
import { genericSolution, validGenericPose } from './generic-pose-algorithms';
import { quaternionDistanceDeg, slerpQuaternion } from './kinematics';
import type { GenericEndpointRequest, GenericIkSolution, GenericPosePlanRequest, GenericPosePlanResult } from './generic-motion-types';
import type { JointAngles, Pose } from './types';

function record(value: unknown): value is Record<string, unknown> { return !!value && typeof value === 'object' && !Array.isArray(value); }
function finite(value: unknown): value is number { return typeof value === 'number' && Number.isFinite(value); }
function joints(value: unknown): value is JointAngles { return Array.isArray(value) && value.length === 6 && Array.from(value).every(finite); }
function clonePose(pose: Pose): Pose { return { position: [...pose.position], orientation: [...pose.orientation] }; }
function distance(left: number[], right: number[]): number { return Math.hypot(...left.map((v, i) => v - right[i])); }
function close(actual: number, expected: number, tolerance = 1e-6): boolean { return Math.abs(actual - expected) <= tolerance * Math.max(1, Math.abs(expected)); }

export function validateGenericEndpointRequest(value: unknown): GenericEndpointRequest {
  if (!record(value) || !validGenericPose(value.pose)) throw new Error('端点数值逆解请求无效');
  const validated = validateGenericMotionRequest({ chain: value.chain, startJoints: value.seedJoints, targetJoints: value.seedJoints, speedMmPerSecond: 100 });
  return { chain: validated.chain, seedJoints: validated.startJoints, pose: clonePose(value.pose) };
}
function validCandidate(value: unknown): value is GenericIkSolution {
  return record(value) && typeof value.id === 'string' && value.id.length > 0 && value.id.length <= 128
    && joints(value.joints) && validGenericPose(value.pose)
    && finite(value.positionErrorMm) && value.positionErrorMm >= 0 && value.positionErrorMm <= 0.001001
    && finite(value.orientationErrorDeg) && value.orientationErrorDeg >= 0 && value.orientationErrorDeg <= 0.00058
    && finite(value.score) && value.score >= 0 && !Object.hasOwn(value, 'configuration') && !Object.hasOwn(value, 'robconf');
}
export function validateGenericEndpointResult(value: unknown, request: GenericEndpointRequest): GenericIkSolution[] {
  if (!Array.isArray(value) || value.length > 32) throw new Error('端点数值逆解候选无效');
  const ids = new Set<string>(); let previousScore = -1;
  for (let index = 0; index < value.length; index++) {
    const candidate: unknown = value[index];
    if (!validCandidate(candidate) || ids.has(candidate.id) || candidate.score + 1e-8 < previousScore || !genericJointsWithinLimits(request.chain, candidate.joints)) throw new Error('端点候选的标识、关节或误差无效');
    const actual = forwardGenericKinematics(request.chain, candidate.joints).pose;
    const expected = genericSolution(candidate.joints, actual, request.pose, request.seedJoints);
    if (distance(actual.position, candidate.pose.position) > 1e-5 || quaternionDistanceDeg(actual.orientation, candidate.pose.orientation) > 1e-4
      || expected.positionErrorMm > 0.001001 || expected.orientationErrorDeg > 0.00058
      || !close(candidate.positionErrorMm, expected.positionErrorMm) || Math.abs(candidate.orientationErrorDeg - expected.orientationErrorDeg) > 1e-4
      || !close(candidate.score, expected.score)
      || value.slice(0, index).some((other: GenericIkSolution) => other.joints.every((v, i) => Math.abs(v - candidate.joints[i]) <= 0.001))) throw new Error('端点候选未通过独立 FK、目标精度或去重复核');
    ids.add(candidate.id); previousScore = candidate.score;
  }
  return value as GenericIkSolution[];
}
export function validateGenericPoseRequest(value: unknown): GenericPosePlanRequest {
  if (!record(value) || !record(value.target) || !(value.target.orientation === null || Array.isArray(value.target.orientation))
    || !validGenericPose({ position: value.target.position, orientation: value.target.orientation ?? [1, 0, 0, 0] })
    || !validGenericPose(value.basePose ?? identityPose()) || !validGenericPose(value.toolPose ?? identityPose())) throw new Error('TCP 目标或坐标变换无效');
  const validated = validateGenericMotionRequest({ chain: value.chain, startJoints: value.startJoints, targetJoints: value.startJoints, speedMmPerSecond: value.speedMmPerSecond });
  return { chain: validated.chain, startJoints: validated.startJoints, speedMmPerSecond: validated.speedMmPerSecond,
    target: { position: [...value.target.position as Pose['position']], orientation: value.target.orientation === null ? null : [...value.target.orientation] as Pose['orientation'] },
    basePose: clonePose((value.basePose ?? identityPose()) as Pose), toolPose: clonePose((value.toolPose ?? identityPose()) as Pose) };
}
/** Independently checks world-TCP FK, line/orientation, endpoint, continuity and speed for every sample. */
export function validateGenericPoseResult(value: unknown, request: GenericPosePlanRequest): GenericPosePlanResult {
  if (!record(value)) throw new Error('TCP 运动结果无效');
  if (value.status === 'failure') {
    if (Object.hasOwn(value, 'selected')) throw new Error('TCP 运动失败不能包含可应用的终点候选');
    const result = validateGenericMotionResult(value, { ...request, targetJoints: request.startJoints });
    if (result.status === 'failure') return result;
    throw new Error('TCP 运动失败结果无效');
  }
  if (value.status !== 'success' || !Array.isArray(value.path) || value.path.length < 2 || value.path.length > 1601
    || !finite(value.lengthMm) || value.lengthMm < 0 || !finite(value.orientationTravelDeg) || value.orientationTravelDeg < 0 || value.orientationTravelDeg > 180.000001
    || !finite(value.durationSeconds) || value.durationSeconds < 0 || value.automaticOrientation !== (request.target.orientation === null)
    || !validCandidate(value.selected) || !Array.isArray(value.warnings) || value.warnings.length > 32 || !Array.from(value.warnings).every(item => typeof item === 'string' && item.length <= 4096)) throw new Error('TCP 运动采样或统计无效');
  const base = request.basePose ?? identityPose(), tool = request.toolPose ?? identityPose();
  const worldTcp = (angles: JointAngles) => composePose(composePose(base, forwardGenericKinematics(request.chain, angles).pose), tool);
  const start = worldTcp(request.startJoints), target: Pose = { position: request.target.position, orientation: request.target.orientation ?? value.selected.pose.orientation };
  const length = distance(start.position, target.position), angle = quaternionDistanceDeg(start.orientation, target.orientation);
  if (!close(value.lengthMm, length) || Math.abs(value.orientationTravelDeg - angle) > 0.00059) throw new Error('TCP 路径长度或姿态变化不匹配');
  let previousT = -1, previousJoints = request.startJoints, duration = length / request.speedMmPerSecond;
  for (let index = 0; index < value.path.length; index++) {
    const sample: unknown = value.path[index];
    if (!record(sample) || !finite(sample.t) || sample.t < 0 || sample.t > 1 || sample.t <= previousT || !joints(sample.joints)
      || !genericJointsWithinLimits(request.chain, sample.joints) || !validGenericPose(sample.pose)) throw new Error('TCP 采样关节或姿态无效');
    if (index === 0 && (sample.t !== 0 || sample.joints.some((v, axis) => Math.abs(v - request.startJoints[axis]) > 1e-6))) throw new Error('TCP 路径起点不匹配');
    const actual = worldTcp(sample.joints);
    const expectedPosition = start.position.map((v, axis) => v + (target.position[axis] - v) * (sample.t as number));
    if (distance(actual.position, sample.pose.position) > 1e-5 || quaternionDistanceDeg(actual.orientation, sample.pose.orientation) > 1e-4
      || distance(actual.position, expectedPosition) > 0.001001
      || quaternionDistanceDeg(actual.orientation, slerpQuaternion(start.orientation, target.orientation, sample.t)) > (request.target.orientation === null ? 0.00116 : 0.00058)) throw new Error('TCP 采样未通过带工具和基座变换的 FK 直线复核');
    if (index === value.path.length - 1 && (sample.t !== 1 || distance(sample.joints, value.selected.joints) > 1e-6 || distance(sample.pose.position, value.selected.pose.position) > 1e-5 || quaternionDistanceDeg(sample.pose.orientation, value.selected.pose.orientation) > 1e-4)) throw new Error('TCP 终点候选与路径不匹配');
    if (index > 0) {
      const deltaT = sample.t - previousT;
      if (length * deltaT > 5 + 1e-6 || value.orientationTravelDeg * deltaT > 2 + 1e-6) throw new Error('TCP 采样间距超出范围');
      for (let axis = 0; axis < 6; axis++) { const delta = Math.abs(sample.joints[axis] - previousJoints[axis]); if (delta > 15 + 1e-8) throw new Error('TCP 关节采样不连续'); duration = Math.max(duration, delta / request.chain.limits[axis].maxSpeedDeg / deltaT); }
    }
    previousT = sample.t; previousJoints = sample.joints;
  }
  if (!close(value.durationSeconds, duration) || !close(value.selected.score, distance(value.selected.joints, request.startJoints))
    || !close(value.selected.positionErrorMm, distance(value.selected.pose.position, request.target.position))
    || (request.target.orientation !== null && Math.abs(value.selected.orientationErrorDeg - quaternionDistanceDeg(value.selected.pose.orientation, request.target.orientation)) > 1e-4)) throw new Error('TCP 端点误差或速度限制统计无效');
  return value as unknown as GenericPosePlanResult;
}
