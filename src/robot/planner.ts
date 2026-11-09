import { getConfiguration } from "./configuration";
import { getRobotModel } from "./model";
import { forwardKinematics, inverseKinematics, jointDistance, jointsWithinLimits, matrixToQuaternion, quaternionDistanceDeg, quaternionToMatrix, slerpQuaternion, solveIKCandidates } from "./kinematics";
import { normalizeOrientation } from "./rapid";
import type { IKSolution, JointAngles, MotionSample, PlanFailure, PlanRequest, PlanResult, PlanSuccess, Pose, Quaternion, RobotModel, Vec3 } from "./types";

export const PLANNING_LIMITS = { translationStepMm: 5, orientationStepDeg: 2, maxSamples: 1601, maxJointStepDeg: 15, minSpeed: 1, maxSpeed: 2000 } as const;

const failure = (code: PlanFailure["code"], message: string, extra: Partial<PlanFailure> = {}): PlanFailure => ({ status: "failure", code, message, ...extra });
const positionDistance = (a: Vec3, b: Vec3) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
const cross = (a: Vec3, b: Vec3): Vec3 => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const dot = (a: Vec3, b: Vec3) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const unit = (a: Vec3): Vec3 | null => { const size = Math.hypot(...a); return size < 1e-10 ? null : a.map((v) => v / size) as Vec3; };

function automaticOrientations(model: RobotModel, start: Pose, position: Vec3): Quaternion[] {
  const result: Quaternion[] = [start.orientation], startR = quaternionToMatrix(start.orientation);
  const initialX: Vec3 = [startR[0], startR[4], startR[8]];
  const radial = unit([position[0], position[1], 0]) ?? [1, 0, 0];
  const shoulderDirection = unit([position[0], position[1], position[2] - model.geometry.shoulderHeightMm]);
  const directions: Vec3[] = [
    ...(shoulderDirection ? [shoulderDirection] : []), radial, [-radial[0], -radial[1], 0], [0, 0, -1], [0, 0, 1],
    [radial[0], radial[1], -1], [radial[0], radial[1], 1], [-radial[0], -radial[1], -1], [-radial[0], -radial[1], 1],
  ];
  for (const direction of directions) {
    const z = unit(direction)!;
    const projection = dot(initialX, z);
    const x = unit(initialX.map((v, i) => v - projection * z[i]) as Vec3) ?? unit(cross(Math.abs(z[2]) < 0.9 ? [0, 0, 1] : [0, 1, 0], z))!;
    const y = cross(z, x);
    for (const angle of [0, Math.PI / 4, -Math.PI / 4, Math.PI / 2, -Math.PI / 2, Math.PI]) {
      const xx = x.map((v, i) => v * Math.cos(angle) + y[i] * Math.sin(angle));
      const yy = y.map((v, i) => v * Math.cos(angle) - x[i] * Math.sin(angle));
      const orientation = matrixToQuaternion([xx[0], yy[0], z[0], 0, xx[1], yy[1], z[1], 0, xx[2], yy[2], z[2], 0, 0, 0, 0, 1]);
      if (!result.some((q) => quaternionDistanceDeg(q, orientation) < 0.01)) result.push(orientation);
    }
  }
  return result.sort((a, b) => quaternionDistanceDeg(a, start.orientation) - quaternionDistanceDeg(b, start.orientation));
}

function evaluatePath(model: RobotModel, startJoints: JointAngles, target: Pose, speed: number, automaticOrientation: boolean): PlanResult {
  const startFK = forwardKinematics(model, startJoints), startConf = getConfiguration(model, startJoints, startFK);
  if (!startConf.configuration || startConf.singular) return failure("singular_start", "起始姿态处于肩、肘或腕部奇异区域，请先调整关节或选择示例起始姿态。");
  const solutions = inverseKinematics(model, target, startJoints);
  if (!solutions.length) {
    const raw = solveIKCandidates(model, target, startJoints);
    return raw.some((candidate) => candidate.singular)
      ? failure("singular_path", "目标姿态处于奇异区域，无法确定连续的机器人配置。", { failedAt: 1, solutions })
      : failure("unreachable", "目标位置和方向没有满足六轴关节限位的逆解。", { solutions });
  }
  const sameBranch = solutions.filter((candidate) => candidate.configuration[3] === startConf.configuration![3]);
  if (!sameBranch.length) return failure("configuration_change", "目标姿态有逆解，但需切换 cfx 分支；不能从当前姿态直接执行保持配置的 MoveL。", { solutions });
  const lengthMm = positionDistance(startFK.pose.position, target.position), orientationTravelDeg = quaternionDistanceDeg(startFK.pose.orientation, target.orientation);
  const intervals = Math.max(1, Math.ceil(lengthMm / PLANNING_LIMITS.translationStepMm), Math.ceil(orientationTravelDeg / PLANNING_LIMITS.orientationStepDeg));
  if (intervals + 1 > PLANNING_LIMITS.maxSamples) return failure("invalid_input", "路径超过本次仿真的采样范围，请缩小目标距离。", { solutions });
  const path: MotionSample[] = [{ t: 0, pose: startFK.pose, joints: [...startJoints] as JointAngles }];
  let previous = startJoints, selected: IKSolution | undefined;
  let jointDuration = 0;
  for (let step = 1; step <= intervals; step++) {
    const t = step / intervals;
    const pose: Pose = { position: startFK.pose.position.map((v, i) => v + t * (target.position[i] - v)) as Vec3, orientation: slerpQuaternion(startFK.pose.orientation, target.orientation, t) };
    const candidates = step === intervals ? [...solutions].sort((a, b) => jointDistance(a.joints, previous) - jointDistance(b.joints, previous)) : inverseKinematics(model, pose, previous);
    const matching = candidates.filter((candidate) => candidate.configuration[3] === startConf.configuration![3]);
    if (!matching.length) {
      const singular = solveIKCandidates(model, pose, previous).some((candidate) => candidate.singular);
      return failure(singular ? "singular_path" : candidates.length ? "configuration_change" : "path_unreachable",
        singular ? "直线路径进入奇异区域，无法保持连续的机器人配置。" : candidates.length ? "直线路径需要切换机器人分支，已停止生成 MoveL。" : "目标端点有逆解，但直线路径中间无法满足关节限位或可达范围。", { failedAt: t, solutions });
    }
    const withinTurn = matching.filter((candidate) => [0, 3, 5].every((axis) => Math.abs(candidate.joints[axis] - startJoints[axis]) <= 180 + 1e-8));
    if (!withinTurn.length) return failure("configuration_change", "直线路径要求 A1、A4 或 A6 相对起点转动超过 180°，不符合当前 ConfL 配置约束。", { failedAt: t, solutions });
    selected = withinTurn[0];
    if (selected.joints.some((q, i) => Math.abs(q - previous[i]) > PLANNING_LIMITS.maxJointStepDeg)) return failure("singular_path", "相邻路径采样的关节变化过大，无法确认连续可达；请调整目标方向或起始姿态。", { failedAt: t, solutions });
    for (let axis = 0; axis < 6; axis++) jointDuration = Math.max(jointDuration, Math.abs(selected.joints[axis] - previous[axis]) / model.limits[axis].maxSpeedDeg * intervals);
    path.push({ t, pose: selected.pose, joints: selected.joints });
    previous = selected.joints;
  }
  const idealDuration = lengthMm / speed;
  // Matches the explicit v_ori=500 deg/s in the generated RAPID speeddata.
  const durationSeconds = Math.max(0.05, idealDuration, orientationTravelDeg / 500, jointDuration);
  const warnings: string[] = [];
  if (jointDuration > idealDuration + 0.01) warnings.push("预览时长已按关节额定速度延长；实际控制器还会考虑加速度和负载。");
  return { status: "success", selected: selected!, solutions, path, lengthMm, orientationTravelDeg, durationSeconds, warnings, automaticOrientation };
}

/** Point-by-point linear TCP and quaternion-SLERP planning. This is a geometric
 * sampled check, not a collision detector or a certified RobotWare trajectory.
 */
export function planMoveL(request: PlanRequest): PlanResult {
  if (!request || !Array.isArray(request.startJoints) || request.startJoints.length !== 6 || request.startJoints.some((v) => !Number.isFinite(v))
    || !request.target || !Array.isArray(request.target.position) || request.target.position.length !== 3 || request.target.position.some((v) => !Number.isFinite(v))
    || !Number.isFinite(request.speedMmPerSecond) || request.speedMmPerSecond < PLANNING_LIMITS.minSpeed || request.speedMmPerSecond > PLANNING_LIMITS.maxSpeed) {
    return failure("invalid_input", "请输入有限的 XYZ、六个关节角，以及 1–2000 mm/s 的速度。");
  }
  let model: RobotModel;
  try { model = getRobotModel(request.modelId); } catch { return failure("invalid_input", "不支持的机器人型号。"); }
  if (!jointsWithinLimits(model, request.startJoints)) return failure("joint_limit", "起始关节角超出该型号的实际关节限位。");
  const start = forwardKinematics(model, request.startJoints), configuration = getConfiguration(model, request.startJoints, start);
  if (configuration.singular || !configuration.configuration) return failure("singular_start", "起点处于奇异区域，请先调整关节或选择示例起始姿态；机械零位不能直接作为这条 MoveL 的起点。");
  if (request.target.orientation !== null) {
    let orientation: Quaternion;
    try { orientation = normalizeOrientation(request.target.orientation); } catch { return failure("invalid_input", "目标四元数不是有效的单位姿态，请检查 [q1,q2,q3,q4]，其中 q1 为实部。"); }
    return evaluatePath(model, request.startJoints, { position: request.target.position, orientation }, request.speedMmPerSecond, false);
  }
  let bestFailure: PlanFailure | undefined;
  for (const orientation of automaticOrientations(model, start.pose, request.target.position)) {
    const result = evaluatePath(model, request.startJoints, { position: request.target.position, orientation }, request.speedMmPerSecond, true);
    if (result.status === "success") return result;
    if (!bestFailure || (result.solutions?.length ?? 0) > (bestFailure.solutions?.length ?? 0)) bestFailure = result;
  }
  return { ...bestFailure!, message: "自动姿态搜索未找到满足当前起点与直线路径的方向。可调整目标、指定方向或更换起始姿态。" + (bestFailure?.message ?? "") };
}
