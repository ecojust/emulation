import { forwardKinematics, multiplyMatrices, normalizeQuaternion, quaternionDistanceDeg, quaternionToMatrix, matrixToQuaternion, slerpQuaternion } from './kinematics';
import { getRobotModel } from './model';
import { getConfiguration } from './configuration';
import { forwardGenericUnchecked, validateGenericChain } from './generic-kinematics';
import type { GrindingRequest, GrindingResult, GrindingSuccess } from './grinding-types';
import type { Pose, Vec3 } from './types';

export const GRINDING_LIMITS = { targets: 64, samples: 8192, ikCalls: 32768, intervalChecks: 100000, certDepth: 12, translationStepMm: 2, orientationStepDeg: 1, jointStepDeg: 15, beamWidth: 4, legRefinements: 3 } as const;
export const finiteTuple = (value: unknown, length: number): value is number[] => Array.isArray(value) && value.length === length && Array.from(value).every(item => typeof item === 'number' && Number.isFinite(item));
export function validateGrindingPose(pose: Pose): Pose {
  if (!pose || !finiteTuple(pose.position, 3) || pose.position.some(value => Math.abs(value) > 1e6)
    || !finiteTuple(pose.orientation, 4) || Math.hypot(...pose.orientation) < 1e-10) throw new Error('位置或四元数无效。');
  return { position: [...pose.position], orientation: normalizeQuaternion(pose.orientation) };
}
export function validateGrindingRequest(value: GrindingRequest): GrindingRequest {
  if (!value || !value.robot || !finiteTuple(value.startJoints, 6)) throw new Error('路径规划需要机器人和六个有效起始关节角。');
  if (value.robot.kind === 'abb') getRobotModel(value.robot.modelId);
  else if (value.robot.kind === 'generic') {
    const robot = value.robot;
    validateGenericChain(robot.chain);
    if (!Array.isArray(value.robot.bodies) || !value.robot.bodies.length || value.robot.bodies.length > 128
      || Array.from(value.robot.bodies).some(body => !body || !Number.isInteger(body.stepIndex) || body.stepIndex < 0 || body.stepIndex > robot.chain.steps.length
        || !finiteTuple(body.centerMm, 3) || body.centerMm.some(v => Math.abs(v) > 100000)
        || !finiteTuple(body.halfSizeMm, 3) || body.halfSizeMm.some(v => v <= 0 || v > 100000))) throw new Error('机器人碰撞包络缺失或无效。');
    const stages = new Set(robot.bodies.map(body => robot.chain.steps.slice(0, body.stepIndex).filter(step => step.jointIndex !== null).length));
    if (stages.size !== 7) throw new Error('机器人几何包络必须包含基座和全部六轴刚体。');
  } else throw new Error('不支持的机器人规划类型。');
  if (!Array.isArray(value.targets) || !value.targets.length || value.targets.length > GRINDING_LIMITS.targets) throw new Error('一次最多规划 64 个目标点。');
  if (!Number.isFinite(value.speedMmPerSecond) || value.speedMmPerSecond < 1 || value.speedMmPerSecond > 2000
    || !Number.isFinite(value.clearanceMm) || value.clearanceMm < 0 || value.clearanceMm > 1000
    || !Number.isFinite(value.toolRadiusMm) || value.toolRadiusMm < 0 || value.toolRadiusMm > 1000) throw new Error('速度、安全间距或工具半径超出范围。');
  if (value.allowDetours !== undefined && typeof value.allowDetours !== 'boolean') throw new Error('绕行选项无效。');
  if (!Array.isArray(value.zones) || value.zones.length > 64) throw new Error('一次最多检查 64 个禁入区。');
  const ids = new Set<string>();
  for (const zone of value.zones) {
    if (!zone || typeof zone.id !== 'string' || !zone.id.trim() || new TextEncoder().encode(zone.id).length > 128 || ids.has(zone.id)
      || typeof zone.enabled !== 'boolean' || !finiteTuple(zone.centerMm, 3) || zone.centerMm.some(v => Math.abs(v) > 1e6)
      || !finiteTuple(zone.sizeMm, 3) || zone.sizeMm.some(v => v <= 0 || v > 1e6)) throw new Error('禁入区标识、位置或尺寸无效。');
    ids.add(zone.id);
  }
  const basePose = validateGrindingPose(value.basePose), toolPose = validateGrindingPose(value.toolPose);
  const orientation = value.orientation == null ? null : validateGrindingPose({ position: [0, 0, 0], orientation: value.orientation }).orientation;
  const targets = Array.from(value.targets).map(validateGrindingPose);
  if (orientation && targets.some(target => quaternionDistanceDeg(target.orientation, orientation) > 1e-5)) throw new Error('旧版统一方向参数与目标姿态不一致；不会覆盖输入目标。');
  // Snapshot the request so later caller edits cannot change the submitted context.
  return structuredClone({ ...value, basePose, toolPose, orientation, targets });
}
export function validateGrindingResult(value: unknown, request?: GrindingRequest): GrindingResult {
  if (!value || typeof value !== 'object') throw new Error('未取得有效的路径规划结果。');
  const result = value as GrindingResult;
  if (result.status === 'failure') {
    if (typeof result.code !== 'string' || typeof result.message !== 'string' || Object.hasOwn(result, 'path') || Object.hasOwn(result, 'waypoints')) throw new Error('路径规划失败信息无效。');
    return result;
  }
  if (result.status !== 'success') throw new Error('路径规划状态无效。');
  const plan = result as GrindingSuccess;
  if (!Array.isArray(plan.path) || plan.path.length < 2 || plan.path.length > GRINDING_LIMITS.samples
    || !Array.isArray(plan.waypoints) || !plan.waypoints.length || plan.waypoints.length > GRINDING_LIMITS.samples
    || !Number.isFinite(plan.durationSeconds) || plan.durationSeconds <= 0 || !Number.isFinite(plan.lengthMm) || plan.lengthMm < 0
    || !Number.isSafeInteger(plan.checkedIntervals) || plan.checkedIntervals < 1 || plan.checkedIntervals > GRINDING_LIMITS.intervalChecks || !Array.isArray(plan.warnings) || !Array.from(plan.warnings).every(v => typeof v === 'string')) throw new Error('路径规划的轨迹或统计信息无效。');
  let previous = -1;
  for (const sample of plan.path) {
    if (!sample || !Number.isFinite(sample.t) || sample.t < previous || sample.t < 0 || sample.t > 1 || !finiteTuple(sample.joints, 6)) throw new Error('路径采样时间或关节角无效。');
    validateGrindingPose(sample.pose);
    if (Math.abs(Math.hypot(...sample.pose.orientation) - 1) > 1e-6 || (previous >= 0 && sample.t <= previous)) throw new Error('路径姿态或采样顺序无效。');
    previous = sample.t;
  }
  if (plan.path[0].t !== 0 || Math.abs(plan.path.at(-1)!.t - 1) > 1e-8) throw new Error('路径采样不完整。');
  for (const point of plan.waypoints) {
    validateGrindingPose(point.pose);
    if (!finiteTuple(point.joints, 6) || (point.configuration !== null && (!finiteTuple(point.configuration, 4) || !point.configuration.every(Number.isInteger)))
      || !['process', 'transfer'].includes(point.kind) || (point.targetIndex !== null && (!Number.isInteger(point.targetIndex) || point.targetIndex < 0 || point.targetIndex >= GRINDING_LIMITS.targets))) throw new Error('规划路径点无效。');
  }
  if (plan.orientation !== null) validateGrindingPose({ position: [0, 0, 0], orientation: plan.orientation });
  if (request) {
    const poseMatrix = (pose: Pose) => { const m = quaternionToMatrix(pose.orientation); pose.position.forEach((v, i) => { m[i * 4 + 3] = v; }); return m; };
    const base = poseMatrix(request.basePose), tool = poseMatrix(request.toolPose);
    const distance = (a: Pose, b: Pose) => Math.hypot(...a.position.map((v, i) => v - b.position[i]));
    const robot = request.robot;
    const limits = robot.kind === 'abb' ? getRobotModel(robot.modelId).limits : robot.chain.limits;
    if (plan.path[0].joints.some((v, i) => Math.abs(v - request.startJoints[i]) > 1e-6)) throw new Error('规划结果起点与请求不一致。');
    const initialConfiguration = robot.kind === 'abb' ? getConfiguration(getRobotModel(robot.modelId), request.startJoints, forwardKinematics(getRobotModel(robot.modelId), request.startJoints)).configuration : null;
    for (const [index, sample] of plan.path.entries()) {
      if (sample.joints.some((v, i) => v < limits[i].minDeg - 1e-7 || v > limits[i].maxDeg + 1e-7
        || (index > 0 && Math.abs(v - plan.path[index - 1].joints[i]) > GRINDING_LIMITS.jointStepDeg + 1e-6))) throw new Error('规划关节限位或连续性复核失败。');
      if (robot.kind === 'abb') {
        const model = getRobotModel(robot.modelId), config = getConfiguration(model, sample.joints, forwardKinematics(model, sample.joints));
        if (config.singular || !config.configuration || !initialConfiguration || config.configuration[3] !== initialConfiguration[3]) throw new Error('规划轨迹发生机器人分支变化。');
      }
      if (index > 0 && sample.joints.some((v,i) => Math.abs(v-plan.path[index-1].joints[i]) / ((sample.t-plan.path[index-1].t)*plan.durationSeconds) > limits[i].maxSpeedDeg + 1e-5)) throw new Error('规划轨迹超过关节速度限制。');
      const flange = robot.kind === 'abb' ? forwardKinematics(getRobotModel(robot.modelId), sample.joints).frames[6]
        : forwardGenericUnchecked(robot.chain, sample.joints).transform;
      const world = multiplyMatrices(multiplyMatrices(base, flange), tool);
      const actual: Pose = { position: [world[3], world[7], world[11]] as Vec3, orientation: matrixToQuaternion(world) };
      if (distance(actual, sample.pose) > 0.02 || quaternionDistanceDeg(actual.orientation, sample.pose.orientation) > 0.002) throw new Error('规划结果的实际工具姿态复核失败。');
    }
    const targets = plan.waypoints.filter(point => point.targetIndex !== null);
    if (targets.length !== request.targets.length || targets.some((point, i) => point.targetIndex !== i
      || point.pose.position.some((v,axis) => v !== request.targets[i].position[axis])
      || Math.min(...[1,-1].map(sign => Math.hypot(...point.pose.orientation.map((v,axis) => v-sign*normalizeQuaternion(request.targets[i].orientation)[axis])))) > 1e-12)) throw new Error('规划结果未按顺序包含全部目标。');
    let pathIndex = 0;
    for (const [pointIndex, point] of plan.waypoints.entries()) {
      const expectedKind = point.targetIndex === null || point.targetIndex === 0 || (pointIndex > 0 && plan.waypoints[pointIndex - 1].targetIndex === null) ? 'transfer' : 'process';
      if (point.kind !== expectedKind) throw new Error('规划点的接近／转移运动标记无效。');
      const matches = (sample: GrindingSuccess['path'][number]) => sample.joints.every((v, axis) => Math.abs(v - point.joints[axis]) < 1e-6)
        && distance(sample.pose, point.pose) < 0.02 && quaternionDistanceDeg(sample.pose.orientation, point.pose.orientation) < 0.002;
      // Every leg contributes at least one new sample, even for a stationary or
      // repeated target. The final waypoint must match the actual final sample.
      const found = pointIndex === plan.waypoints.length - 1
        ? (plan.path.length - 1 > pathIndex && matches(plan.path.at(-1)!) ? plan.path.length - 1 : -1)
        : plan.path.findIndex((sample, i) => i > pathIndex && matches(sample));
      if (found < 0) throw new Error('规划路径点与轨迹不一致。');
      const origin = plan.path[pathIndex], end = plan.path[found], interval = end.t-origin.t;
      const length = distance(origin.pose,point.pose), angle = quaternionDistanceDeg(origin.pose.orientation,point.pose.orientation);
      for (let i=pathIndex+1;i<=found;i++) {
        const sample=plan.path[i],fraction=(sample.t-origin.t)/interval;
        const expectedPosition=origin.pose.position.map((v,axis)=>v+(point.pose.position[axis]-v)*fraction);
        const delta=(sample.t-plan.path[i-1].t)/interval;
        if (Math.hypot(...sample.pose.position.map((v,axis)=>v-expectedPosition[axis]))>0.0021
          || quaternionDistanceDeg(sample.pose.orientation,slerpQuaternion(origin.pose.orientation,point.pose.orientation,fraction))>0.0021
          || length*delta>GRINDING_LIMITS.translationStepMm+1e-6 || angle*delta>GRINDING_LIMITS.orientationStepDeg+1e-6) throw new Error('规划采样未按目标间直线和姿态插值生成。');
      }
      if (robot.kind === 'abb') {
        const model = getRobotModel(robot.modelId), actual = getConfiguration(model, point.joints, forwardKinematics(model, point.joints));
        if (actual.singular || !actual.configuration || !point.configuration || actual.configuration.some((v, i) => v !== point.configuration![i])) throw new Error('规划点的机器人配置复核失败。');
      } else if (point.configuration !== null) throw new Error('通用机器人不得使用其他品牌的配置数据。');
      pathIndex = found;
    }
    if (pathIndex !== plan.path.length - 1) throw new Error('规划终点与轨迹不一致。');
    if (request.orientation ? !plan.orientation || quaternionDistanceDeg(request.orientation, plan.orientation) > 0.002 : plan.orientation !== null) throw new Error('规划结果的兼容方向约束不匹配。');
  }
  return plan;
}
