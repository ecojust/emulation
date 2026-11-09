import { getConfiguration } from './configuration';
import { forwardKinematics, IK_TOLERANCES, jointDistance, jointsWithinLimits, normalizeQuaternion, quaternionDistanceDeg } from './kinematics';
import { ROBOT_MODELS } from './model';
import type { IKSolution, JointAngles, Pose, Quaternion, RobotId } from './types';

export interface EndpointInspectionRequest {
  modelId: RobotId;
  pose: Pose;
  seedJoints: JointAngles;
}

export const MAX_ENDPOINT_SOLUTIONS = 256;
export const MAX_ENDPOINT_COORDINATE_MM = 1_000_000;

function record(value: unknown): value is Record<string, unknown> {
  return value !== null && typeof value === 'object' && !Array.isArray(value);
}

function finiteTuple(value: unknown, length: number): value is number[] {
  return Array.isArray(value) && value.length === length && Array.from(value).every(component => typeof component === 'number' && Number.isFinite(component));
}

function validPose(value: unknown): value is Pose {
  return record(value) && finiteTuple(value.position, 3) && finiteTuple(value.orientation, 4)
    && value.position.every(component => Math.abs(component) <= MAX_ENDPOINT_COORDINATE_MM)
    && Math.abs(value.orientation.reduce((sum, component) => sum + component * component, 0) - 1) <= 0.1 + 1e-12;
}

/** Normalize a private request copy; neither native nor browser may mutate a point object. */
export function validateEndpointInspectionRequest(value: unknown): EndpointInspectionRequest {
  if (!record(value) || Object.keys(value).some(key => !['modelId', 'pose', 'seedJoints'].includes(key))
    || typeof value.modelId !== 'string' || !Object.hasOwn(ROBOT_MODELS, value.modelId)) throw new Error('该机器人尚未适配端点逆解检查');
  const modelId = value.modelId as RobotId;
  if (!validPose(value.pose)) throw new Error('端点位置须为 ±1000000 mm 内的有限坐标，姿态须为有效单位四元数');
  if (!finiteTuple(value.seedJoints, 6) || !jointsWithinLimits(ROBOT_MODELS[modelId], value.seedJoints as JointAngles)) throw new Error('端点参考关节无效或超出机器人关节范围');
  return { modelId, pose: { position: [...value.pose.position], orientation: normalizeQuaternion(value.pose.orientation) }, seedJoints: [...value.seedJoints] as JointAngles };
}

/** Validate every returned candidate against local FK and ABB configuration, not claimed success flags. */
export function validateEndpointSolutions(value: unknown, request: EndpointInspectionRequest): IKSolution[] {
  if (!Array.isArray(value) || value.length > MAX_ENDPOINT_SOLUTIONS) throw new Error('端点逆解返回数量无效');
  const model = ROBOT_MODELS[request.modelId];
  const identifiers = new Set<string>();
  const jointKeys = new Set<string>();
  for (const solution of value) {
    if (!record(solution) || typeof solution.id !== 'string' || solution.id.length > 128 || !solution.id.trim() || new TextEncoder().encode(solution.id).length > 128 || identifiers.has(solution.id)
      || !finiteTuple(solution.joints, 6) || !jointsWithinLimits(model, solution.joints as JointAngles)
      || !validPose(solution.pose) || !finiteTuple(solution.configuration, 4) || !solution.configuration.every(Number.isInteger)
      || typeof solution.positionErrorMm !== 'number' || !Number.isFinite(solution.positionErrorMm) || solution.positionErrorMm < 0 || solution.positionErrorMm > IK_TOLERANCES.positionMm
      || typeof solution.orientationErrorDeg !== 'number' || !Number.isFinite(solution.orientationErrorDeg) || solution.orientationErrorDeg < 0 || solution.orientationErrorDeg > IK_TOLERANCES.orientationDeg
      || typeof solution.score !== 'number' || !Number.isFinite(solution.score) || solution.score < 0) throw new Error('端点逆解返回了无效的关节、姿态或误差数据');
    identifiers.add(solution.id);
    const joints = solution.joints as JointAngles;
    const reportedPose = solution.pose;
    const reportedConfiguration = solution.configuration;
    const key = joints.map(angle => angle.toFixed(6)).join('/');
    if (jointKeys.has(key)) throw new Error('端点逆解返回了重复的关节姿态');
    jointKeys.add(key);
    const fk = forwardKinematics(model, joints);
    const configuration = getConfiguration(model, joints, fk);
    const positionError = Math.hypot(...fk.pose.position.map((coordinate, index) => coordinate - request.pose.position[index]));
    const orientationError = quaternionDistanceDeg(fk.pose.orientation, request.pose.orientation);
    const reportedPositionError = Math.hypot(...fk.pose.position.map((coordinate, index) => coordinate - reportedPose.position[index]));
    const reportedOrientationError = quaternionDistanceDeg(fk.pose.orientation, reportedPose.orientation as Quaternion);
    const score = jointDistance(joints, request.seedJoints);
    if (configuration.singular || !configuration.configuration || configuration.configuration.some((part, index) => part !== reportedConfiguration[index])
      || positionError > IK_TOLERANCES.positionMm || orientationError > IK_TOLERANCES.orientationDeg
      || reportedPositionError > 1e-6 || reportedOrientationError > 1e-5
      || Math.abs(solution.positionErrorMm - positionError) > 1e-6
      || Math.abs(solution.orientationErrorDeg - orientationError) > 1e-5
      || Math.abs(solution.score - score) > 1e-6 * Math.max(1, score)) throw new Error('端点逆解未通过正运动学、配置或目标姿态复核');
  }
  return value as IKSolution[];
}
