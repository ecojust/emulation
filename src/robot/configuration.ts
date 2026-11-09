import type { ConfigurationResult, FKResult, JointAngles, RobotModel, Vec3 } from "./types";

/** Application tolerances around ABB's three serial-arm configuration singularities. */
export const CONFIGURATION_TOLERANCES = {
  shoulderMm: 0.1,
  elbowMm: 0.1,
  wristDeg: 0.1,
} as const;

const dot = (a: Vec3, b: Vec3) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const sub = (a: Vec3, b: Vec3): Vec3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const cross = (a: Vec3, b: Vec3): Vec3 => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];

/** ABB 6-axis quadrants keep complete turns: -1 degree belongs to quadrant -1. */
export function jointQuadrant(angleDeg: number): number {
  if (!Number.isFinite(angleDeg)) throw new Error("关节角必须是有限数值");
  // DH round trips can turn exact 0/90-degree values into tiny signed errors.
  const boundary = Math.round(angleDeg / 90) * 90;
  const value = Math.abs(angleDeg - boundary) <= 1e-10 ? boundary : angleDeg;
  const quadrant = Math.floor(value / 90);
  return Object.is(quadrant, -0) ? 0 : quadrant;
}

/** ABB 3HAC050917-001 Rev AC, confdata pp.1649–1655 (6-axis serial link robots).
 * cfx = 4 * wrist-behind-A1 + 2 * wrist-behind-lower-arm + (A5 < 0).
 * The lower arm is the physical A2→A3 link, not the offset DH frame-3 segment.
 * FK axes and points are all expressed in the base/world frame.
 */
export function getConfiguration(model: RobotModel, joints: JointAngles, fk: FKResult): ConfigurationResult {
  const points = [fk.shoulder, fk.elbow, fk.wristCenter, fk.jointOrigins[0], fk.jointAxes[1]];
  if (joints.length !== 6 || !joints.every(Number.isFinite) || points.some(p => !p || p.length !== 3 || !p.every(Number.isFinite))) {
    return { configuration: null, singular: false, reason: "关节或正运动学数据无效" };
  }
  const angle = (joints[0] * model.dh[0].thetaSign + model.dh[0].thetaOffsetDeg) * Math.PI / 180;
  const forward: Vec3 = [Math.cos(angle), Math.sin(angle), 0];
  const shoulderSide = dot(sub(fk.wristCenter, fk.jointOrigins[0]), forward);
  if (Math.abs(shoulderSide) <= CONFIGURATION_TOLERANCES.shoulderMm) {
    return { configuration: null, singular: true, reason: "腕心接近轴 1，肩部配置不确定" };
  }
  const front = cross(fk.jointAxes[1], sub(fk.elbow, fk.shoulder));
  const scale = Math.hypot(...front);
  if (scale < 1e-10) return { configuration: null, singular: true, reason: "下臂方向无效，无法确定肘部配置" };
  const elbowSide = dot(sub(fk.wristCenter, fk.elbow), front) / scale;
  if (Math.abs(elbowSide) <= CONFIGURATION_TOLERANCES.elbowMm) {
    return { configuration: null, singular: true, reason: "腕心接近下臂延长线，肘部配置不确定" };
  }
  if (Math.abs(Math.sin(joints[4] * Math.PI / 180)) <= Math.sin(CONFIGURATION_TOLERANCES.wristDeg * Math.PI / 180)) {
    return { configuration: null, singular: true, reason: "轴 5 接近腕部奇异位形，无法稳定确定 robconf" };
  }
  const cfx = (shoulderSide < 0 ? 4 : 0) + (elbowSide < 0 ? 2 : 0) + (joints[4] < 0 ? 1 : 0);
  return { configuration: [jointQuadrant(joints[0]), jointQuadrant(joints[3]), jointQuadrant(joints[5]), cfx], singular: false };
}
