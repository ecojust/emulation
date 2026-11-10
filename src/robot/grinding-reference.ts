import { getRobotModel } from './model';
import { forwardKinematics, inverseKinematics, jointsWithinLimits, matrixToQuaternion, multiplyMatrices, normalizeQuaternion, quaternionDistanceDeg, quaternionToMatrix, slerpQuaternion } from './kinematics';
import { getConfiguration } from './configuration';
import { forwardGenericUnchecked, GenericMotionCancelled, genericJointsWithinLimits, solveGenericUnchecked } from './generic-kinematics';
import { intersectsDangerZone, linkBoundingBoxes, type LinkBoundingBox } from './collision-geometry';
import { GRINDING_LIMITS as LIMIT, validateGrindingRequest } from './grinding-validation';
import type { GrindingFailure, GrindingRequest, GrindingResult, GrindingSuccess, GrindingWaypoint } from './grinding-types';
import type { JointAngles, Mat4, MotionSample, Pose, Quaternion, RobConf, Vec3 } from './types';

const IDENTITY: Mat4 = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
const norm = (value: readonly number[]) => Math.hypot(...value);
class PlanningFailure extends Error {
  constructor(readonly code: string, message: string, public failedTarget?: number) { super(message); }
}
const fail = (code: string, message: string): never => { throw new PlanningFailure(code, message); };
function poseMatrix(pose: Pose): Mat4 {
  const m = quaternionToMatrix(pose.orientation);
  for (let i = 0; i < 3; i++) m[i * 4 + 3] = pose.position[i];
  return m;
}
const matrixPose = (m: Mat4): Pose => ({ position: [m[3], m[7], m[11]], orientation: matrixToQuaternion(m) });
function inverse(m: Mat4): Mat4 {
  const out = [...IDENTITY];
  for (let row = 0; row < 3; row++) {
    for (let col = 0; col < 3; col++) out[row * 4 + col] = m[col * 4 + row];
    out[row * 4 + 3] = -[0, 1, 2].reduce((sum, k) => sum + out[row * 4 + k] * m[k * 4 + 3], 0);
  }
  return out;
}
function rotation(axis: Vec3, degrees: number): Mat4 {
  const [x, y, z] = axis, s = Math.sin(degrees * Math.PI / 180), c = Math.cos(degrees * Math.PI / 180), d = 1 - c;
  return [c+x*x*d,x*y*d-z*s,x*z*d+y*s,0,y*x*d+z*s,c+y*y*d,y*z*d-x*s,0,z*x*d-y*s,z*y*d+x*s,c+z*z*d,0,0,0,0,1];
}
function transformedBox(m: Mat4, center: Vec3, half: Vec3, index = 0): LinkBoundingBox {
  return { index, centerMm: [0, 1, 2].map(row => m[row * 4 + 3] + center.reduce((sum, p, k) => sum + m[row * 4 + k] * p, 0)) as Vec3,
    axes: [[m[0], m[4], m[8]], [m[1], m[5], m[9]], [m[2], m[6], m[10]]], halfSizeMm: half };
}
interface Leg { samples: MotionSample[]; duration: number; length: number }
class Context {
  base: Mat4; baseInverse: Mat4; tool: Mat4; toolInverse: Mat4;
  radiusBound: number; speedLimits: number[];
  ikCalls = 0; checkedIntervals = 0;
  constructor(readonly request: GrindingRequest, readonly cancel: () => boolean) {
    this.base = poseMatrix(request.basePose); this.baseInverse = inverse(this.base);
    this.tool = poseMatrix(request.toolPose); this.toolInverse = inverse(this.tool);
    let reach = 0;
    if (request.robot.kind === 'abb') {
      const model = getRobotModel(request.robot.modelId);
      if (!jointsWithinLimits(model, request.startJoints)) fail('joint_limit', '起点关节超出限位。');
      const boxes = linkBoundingBoxes(model.id, [0, 0, 0, 0, 0, 0]);
      reach = 2 * model.dh.reduce((sum, row) => sum + Math.hypot(row.aMm, row.dMm), 0)
        + Math.max(...boxes.map(box => norm(box.centerMm) + norm(box.halfSizeMm)));
      this.speedLimits = model.limits.map(limit => limit.maxSpeedDeg);
    } else {
      const { chain, bodies } = request.robot;
      if (!genericJointsWithinLimits(chain, request.startJoints)) fail('joint_limit', '起点关节超出限位。');
      reach = chain.steps.reduce((sum, step) => sum + Math.hypot(step.origin[3], step.origin[7], step.origin[11]), 0)
        + Math.max(...bodies.map(body => norm(body.centerMm) + norm(body.halfSizeMm)));
      this.speedLimits = chain.limits.map(limit => limit.maxSpeedDeg);
    }
    this.radiusBound = reach + norm(request.toolPose.position) + Math.sqrt(3) * Math.max(request.toolRadiusMm, 1e-6);
  }
  check() { if (this.cancel()) fail('cancelled', '路径规划已取消，未返回部分结果。'); }
  flange(q: JointAngles): Mat4 {
    this.check();
    return this.request.robot.kind === 'abb' ? forwardKinematics(getRobotModel(this.request.robot.modelId), q).frames[6]
      : forwardGenericUnchecked(this.request.robot.chain, q, this.cancel).transform;
  }
  tcp(q: JointAngles): Pose { return matrixPose(multiplyMatrices(multiplyMatrices(this.base, this.flange(q)), this.tool)); }
  config(q: JointAngles): RobConf | null {
    if (this.request.robot.kind !== 'abb') return null;
    const model = getRobotModel(this.request.robot.modelId), result = getConfiguration(model, q, forwardKinematics(model, q));
    if (result.singular) fail('singular_path', '路径进入奇异区域，未生成连续路径。');
    return result.configuration;
  }
  ik(pose: Pose, seed: JointAngles): JointAngles {
    this.check();
    if (++this.ikCalls > LIMIT.ikCalls) fail('resource_limit', '路径搜索达到逆解预算，未返回部分路径。');
    const target = matrixPose(multiplyMatrices(multiplyMatrices(this.baseInverse, poseMatrix(pose)), this.toolInverse));
    let joints: JointAngles | null | undefined;
    if (this.request.robot.kind === 'abb') {
      const config = this.config(seed);
      if (!config) fail('singular_path', '起点没有有效机器人配置。');
      joints = inverseKinematics(getRobotModel(this.request.robot.modelId), target, seed).find(solution => solution.configuration[3] === config![3])?.joints;
    } else joints = solveGenericUnchecked(this.request.robot.chain, target, seed, this.cancel);
    if (!joints) fail('unreachable', '目标方向或路径上的逆解不可达，不能跳换关节分支。');
    if (joints!.some((value, i) => Math.abs(value - seed[i]) > LIMIT.jointStepDeg + 1e-8)) fail('configuration_change', '相邻采样关节变化超过15°，未生成跳跃姿态。');
    return joints!;
  }
  boxes(q: JointAngles): LinkBoundingBox[] {
    this.check();
    let boxes: LinkBoundingBox[];
    if (this.request.robot.kind === 'abb') {
      boxes = linkBoundingBoxes(this.request.robot.modelId, q).map(box => {
        const a = box.axes, p = box.centerMm;
        const m = [a[0][0],a[1][0],a[2][0],p[0],a[0][1],a[1][1],a[2][1],p[1],a[0][2],a[1][2],a[2][2],p[2],0,0,0,1];
        return transformedBox(multiplyMatrices(this.base, m), [0, 0, 0], box.halfSizeMm, box.index);
      });
    } else {
      const { chain, bodies } = this.request.robot, frames: Mat4[] = [[...IDENTITY]];
      let m = [...IDENTITY];
      for (const step of chain.steps) {
        this.check(); m = multiplyMatrices(m, step.origin);
        if (step.jointIndex !== null) m = multiplyMatrices(m, rotation(step.axis, q[step.jointIndex]));
        frames.push(m);
      }
      boxes = bodies.map((body, i) => transformedBox(multiplyMatrices(this.base, frames[body.stepIndex]), body.centerMm, body.halfSizeMm, i));
    }
    const offset = this.request.toolPose.position, radius = Math.max(this.request.toolRadiusMm, 1e-6);
    boxes.push(transformedBox(multiplyMatrices(this.base, this.flange(q)), offset.map(v => v / 2) as Vec3, offset.map(v => Math.abs(v) / 2 + radius) as Vec3));
    return boxes;
  }
  clear(boxes: LinkBoundingBox[], padding: number): boolean {
    return boxes.every(box => !this.request.zones.some(zone => zone.enabled && intersectsDangerZone({ ...box, halfSizeMm: box.halfSizeMm.map(value => value + padding) as Vec3 }, zone)));
  }
  certify(a: JointAngles, b: JointAngles, depth = 0, fixed: Quaternion | null = null) {
    this.check();
    if (++this.checkedIntervals > LIMIT.intervalChecks) fail('resource_limit', '连续包络检查达到预算，不能确认路径安全。');
    const mid = a.map((value, i) => (value + b[i]) / 2) as JointAngles;
    const boxes = this.boxes(mid), sweep = this.radiusBound * a.reduce((sum, value, i) => sum + Math.abs(value - b[i]) * Math.PI / 180, 0) / 2;
    let orientationClear = true;
    if (fixed) {
      const error = quaternionDistanceDeg(this.tcp(mid).orientation, fixed);
      if (error > 0.1 + 1e-8) fail('orientation_tolerance', '关节插值的工具方向偏差超过0.1°，未生成路径。');
      orientationClear = error + a.reduce((sum, value, i) => sum + Math.abs(value - b[i]), 0) / 2 <= 0.1;
    }
    if (orientationClear && this.clear(boxes, this.request.clearanceMm + sweep + 1e-7)) return;
    if (!this.clear(boxes, this.request.clearanceMm)) fail('collision', '机器人或工具的保守包络与禁入区相交。');
    if (depth >= LIMIT.certDepth) fail('uncertain', '连续运动包络或固定方向无法在检查精度内完成验证，未生成路径。');
    this.certify(a, mid, depth + 1, fixed); this.certify(mid, b, depth + 1, fixed);
  }
  leg(start: JointAngles, target: Pose): Leg {
    const origin = this.tcp(start), length = norm(target.position.map((v, i) => v - origin.position[i]));
    const angle = quaternionDistanceDeg(origin.orientation, target.orientation);
    const count = Math.max(1, Math.ceil(length / LIMIT.translationStepMm), Math.ceil(angle / LIMIT.orientationStepDeg));
    if (count >= LIMIT.samples) fail('resource_limit', '单段路径超过采样上限。');
    const samples: MotionSample[] = [];
    let previous = start, duration = Math.max(length / this.request.speedMmPerSecond, angle / 500);
    for (let index = 1; index <= count; index++) {
      this.check();
      const t = index / count, wanted: Pose = { position: origin.position.map((value, i) => value + t * (target.position[i] - value)) as Vec3,
        orientation: slerpQuaternion(origin.orientation, target.orientation, t) };
      const joints = this.ik(wanted, previous);
      this.certify(previous, joints, 0, angle <= 0.01 ? target.orientation : null);
      for (let j = 0; j < 6; j++) duration = Math.max(duration, Math.abs(joints[j] - previous[j]) / this.speedLimits[j] * count);
      samples.push({ t, pose: this.tcp(joints), joints }); previous = joints;
    }
    if (!Number.isFinite(duration)) fail('invalid_input', '关节速度产生非有限运动时间，未生成路径。');
    return { samples, duration: Math.max(duration, 0.001), length };
  }
}
const retryable = (error: unknown): error is PlanningFailure => error instanceof PlanningFailure && ['collision', 'uncertain', 'unreachable', 'configuration_change', 'singular_path'].includes(error.code);
/** Browser reference matches Rust's bounded route search and swept OBB proof. */
export function planGrindingReference(input: GrindingRequest, cancel = () => false): GrindingResult {
  try {
    if (cancel()) fail('cancelled', '路径规划已取消，未返回部分结果。');
    let request: GrindingRequest;
    try { request = validateGrindingRequest(input); } catch (error) { return { status: 'failure', code: 'invalid_input', message: (error as Error).message }; }
    const context = new Context(request, cancel), orientation: Quaternion | null = request.orientation ? normalizeQuaternion(request.orientation) : null;
    context.config(request.startJoints); context.certify(request.startJoints, request.startJoints);
    let current = request.startJoints, duration = 0, length = 0, usedDetour = false;
    const path: MotionSample[] = [{ t: 0, pose: context.tcp(current), joints: current }], waypoints: GrindingWaypoint[] = [];
    for (const [targetIndex, inputTarget] of request.targets.entries()) {
      const target: Pose = { position: inputTarget.position, orientation: orientation ?? normalizeQuaternion(inputTarget.orientation) };
      let route: Leg[];
      try { route = [context.leg(current, target)]; }
      catch (error) {
        if (error instanceof PlanningFailure) error.failedTarget = targetIndex;
        if (request.allowDetours === false || !retryable(error) || !request.zones.some(zone => zone.enabled)) throw error;
        const start = context.tcp(current); let found: Leg[] | null = null;
        search: for (const extra of [25, 75]) for (let axis = 0; axis < 3; axis++) for (const sign of [-1, 1]) {
          context.check();
          const edges = request.zones.filter(zone => zone.enabled).map(zone => zone.centerMm[axis] + sign * zone.sizeMm[axis] / 2);
          const edge = sign < 0 ? Math.min(...edges) : Math.max(...edges), plane = edge + sign * (request.toolRadiusMm + request.clearanceMm + extra);
          const p1: Pose = { position: [...start.position], orientation: target.orientation }, p2: Pose = { position: [...target.position], orientation: target.orientation };
          p1.position[axis] = plane; p2.position[axis] = plane;
          if ([p1, p2].some(pose => pose.position.some(value => !Number.isFinite(value) || Math.abs(value) > 1e6))) continue;
          const poses = [start, p1, p2, target];
          const estimate = poses.slice(1).reduce((sum, pose, i) => sum + Math.max(1,
            Math.ceil(norm(pose.position.map((v, axis) => v - poses[i].position[axis])) / LIMIT.translationStepMm),
            Math.ceil(quaternionDistanceDeg(poses[i].orientation, pose.orientation) / LIMIT.orientationStepDeg)), 0);
          if (estimate + path.length > LIMIT.samples) continue;
          let q = current; const candidate: Leg[] = []; let valid = true;
          for (const pose of [p1, p2, target]) {
            try { const leg = context.leg(q, pose); q = leg.samples.at(-1)!.joints; candidate.push(leg); }
            catch (next) { if (!retryable(next)) throw next; valid = false; break; }
          }
          if (valid) { found = candidate; break search; }
        }
        if (!found) { error.message += ' 有界绕行搜索未找到完整可验证路径。'; throw error; }
        route = found; usedDetour = true;
      }
      for (const [index, leg] of route.entries()) {
        if (path.length + leg.samples.length > LIMIT.samples) fail('resource_limit', '整条路径超过采样上限，未返回部分结果。');
        const endpoint = leg.samples.at(-1)!; current = endpoint.joints;
        waypoints.push({ pose: endpoint.pose, joints: current, configuration: context.config(current), targetIndex: index === route.length - 1 ? targetIndex : null, kind: route.length === 1 && targetIndex > 0 ? 'process' : 'transfer' });
        for (const sample of leg.samples) path.push({ ...sample, t: duration + sample.t * leg.duration });
        duration += leg.duration; length += leg.length;
        if (!Number.isFinite(duration) || !Number.isFinite(length)) fail('resource_limit', '累计路径长度或时间超过数值范围。');
      }
    }
    context.check();
    let previousTime = -1;
    for (const sample of path) {
      sample.t /= duration;
      if (!Number.isFinite(sample.t) || sample.t < 0 || sample.t > 1 || sample.t <= previousTime) fail('resource_limit', '路径采样时间超出数值精度，未返回部分路径。');
      previousTime = sample.t;
    }
    const warnings = ['已验证声明的机器人／工具保守包络与禁入区在关节插值全程分离；未包含自碰撞、实际工件接触、控制器或动力学验收。'];
    if (usedDetour) warnings.push('绕行点为转移运动，须在加工工艺中关闭打磨接触；不得将绕行当作表面加工轨迹。');
    if (orientation && request.targets.some(pose => quaternionDistanceDeg(pose.orientation, orientation) > 0.1)) warnings.push('输入点的方向已按统一工具方向调整；原输入数据未修改。');
    const success: GrindingSuccess = { status: 'success', path, waypoints, durationSeconds: duration, lengthMm: length, orientation, checkedIntervals: context.checkedIntervals, warnings };
    return success;
  } catch (error) {
    if (error instanceof GenericMotionCancelled) return { status: 'failure', code: 'cancelled', message: '路径规划已取消，未返回部分结果。' };
    if (error instanceof PlanningFailure) { const result: GrindingFailure = { status: 'failure', code: error.code, message: error.message }; if (error.failedTarget !== undefined) result.failedTarget = error.failedTarget; return result; }
    throw error;
  }
}
