import { quaternionDistanceDeg, slerpQuaternion } from './kinematics';
import { forwardGenericUnchecked, GenericMotionCancelled, genericJointsWithinLimits, solveGenericUnchecked, validateGenericChain } from './generic-kinematics';
import type { GenericPlanFailure, GenericPlanRequest, GenericPlanResult } from './generic-motion-types';
import type { MotionSample, Pose, Vec3 } from './types';

const NEVER_CANCEL = () => false;

function failure(code: GenericPlanFailure['code'], message: string, failedAt?: number): GenericPlanFailure {
  return { status: 'failure', code, message, ...(failedAt === undefined ? {} : { failedAt }) };
}
function cancelled(): GenericPlanFailure { return failure('cancelled', '运动计算已取消，未返回部分路径。'); }

/** Browser reference for the native URDF planner. Samples always contain actual FK poses. */
export function planGenericMoveL(request: GenericPlanRequest, cancel = NEVER_CANCEL): GenericPlanResult {
  if (cancel()) return cancelled();
  try { validateGenericChain(request?.chain); }
  catch (error) { return failure('invalid_input', error instanceof Error ? error.message : 'URDF 运动链无效。'); }
  if (!Number.isFinite(request.speedMmPerSecond) || request.speedMmPerSecond < 1 || request.speedMmPerSecond > 2000) {
    return failure('invalid_input', '运动速度须在 1 至 2000 mm/s 之间。');
  }
  const { chain } = request;
  if (!genericJointsWithinLimits(chain, request.startJoints) || !genericJointsWithinLimits(chain, request.targetJoints)) {
    return failure('joint_limit', '起点或记录的目标关节超出 URDF 限位。');
  }
  try {
    const start = forwardGenericUnchecked(chain, request.startJoints, cancel);
    const target = forwardGenericUnchecked(chain, request.targetJoints, cancel);
    const difference = target.pose.position.map((value, i) => value - start.pose.position[i]);
    const lengthMm = Math.sqrt(difference.reduce((sum, value) => sum + value * value, 0));
    const orientationTravelDeg = quaternionDistanceDeg(start.pose.orientation, target.pose.orientation);
    const segments = Math.max(1, Math.ceil(lengthMm / 5), Math.ceil(orientationTravelDeg / 2));
    if (segments >= 1601) return failure('invalid_input', '运动长度或姿态变化超出 1601 个采样点的单段上限。');
    const path: MotionSample[] = [{ t: 0, pose: start.pose, joints: [...request.startJoints] }];
    let previous = request.startJoints;
    let durationSeconds = lengthMm / request.speedMmPerSecond;
    for (let index = 1; index <= segments; index++) {
      if (cancel()) return cancelled();
      const t = index / segments;
      const pose: Pose = {
        position: start.pose.position.map((value, axis) => value + difference[axis] * t) as Vec3,
        orientation: slerpQuaternion(start.pose.orientation, target.pose.orientation, t),
      };
      let joints = solveGenericUnchecked(chain, pose, previous, cancel);
      if (!joints) return failure('path_unreachable', 'TCP 直线上的数值逆解未收敛；没有返回部分运动。', t);
      if (index === segments) {
        if (joints.some((actual, axis) => Math.abs(actual - request.targetJoints[axis]) > 0.001)) {
          return failure('configuration_change', '直线终点无法连续到达记录的关节姿态，不能替换为另一逆解分支。', t);
        }
        joints = [...request.targetJoints];
      }
      if (joints.some((current, axis) => Math.abs(current - previous[axis]) > 15 + 1e-8)) {
        return failure('configuration_change', '相邻采样的关节变化超过 15°，未通过连续性检查。', t);
      }
      const fk = forwardGenericUnchecked(chain, joints, cancel);
      for (let axis = 0; axis < 6; axis++) {
        durationSeconds = Math.max(durationSeconds, Math.abs(joints[axis] - previous[axis]) / chain.limits[axis].maxSpeedDeg * segments);
      }
      path.push({ t, pose: fk.pose, joints });
      previous = joints;
    }
    if (cancel()) return cancelled();
    return { status: 'success', path, lengthMm, orientationTravelDeg, durationSeconds,
      warnings: ['URDF 数值逆解与离散 TCP 直线采样；未验证完整路径碰撞、控制器配置、加速度或动力学。'] };
  } catch (error) {
    if (error instanceof GenericMotionCancelled) return cancelled();
    throw error;
  }
}
