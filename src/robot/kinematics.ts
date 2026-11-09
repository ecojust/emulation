import { getConfiguration } from "./configuration";
import type { FKResult, IKSolution, JointAngles, Mat4, Pose, Quaternion, RobConf, RobotModel, Vec3 } from "./types";

const RAD = Math.PI / 180;
const DEG = 180 / Math.PI;
const IDENTITY: Mat4 = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
export const IK_TOLERANCES = { positionMm: 0.01, orientationDeg: 0.001 } as const;

export function multiplyMatrices(a: Mat4, b: Mat4): Mat4 {
  const out = Array<number>(16).fill(0);
  for (let row = 0; row < 4; row++) for (let col = 0; col < 4; col++) {
    for (let k = 0; k < 4; k++) out[row * 4 + col] += a[row * 4 + k] * b[k * 4 + col];
  }
  return out;
}

export function normalizeQuaternion(value: Quaternion): Quaternion {
  if (!Array.isArray(value) || value.length !== 4 || value.some((v) => !Number.isFinite(v))) throw new Error("四元数需要四个有限数值");
  const scale = Math.max(...value.map(Math.abs));
  if (scale < 1e-12) throw new Error("四元数不能为零");
  const scaled = value.map((v) => v / scale);
  const length = Math.hypot(...scaled);
  const result = scaled.map((v) => v / length) as Quaternion;
  // q and -q describe the same pose. A consistent sign keeps exported targets stable.
  const first = result.find((v) => Math.abs(v) > 1e-12) ?? 1;
  return result.map((v) => (first < 0 ? -v : v)) as Quaternion;
}

export function quaternionToMatrix(value: Quaternion): Mat4 {
  const [w, x, y, z] = normalizeQuaternion(value);
  return [
    1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y), 0,
    2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x), 0,
    2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y), 0,
    0, 0, 0, 1,
  ];
}

export function matrixToQuaternion(m: Mat4): Quaternion {
  const trace = m[0] + m[5] + m[10];
  let q: Quaternion;
  if (trace > 0) {
    const s = Math.sqrt(Math.max(0, trace + 1)) * 2;
    q = [s / 4, (m[9] - m[6]) / s, (m[2] - m[8]) / s, (m[4] - m[1]) / s];
  } else if (m[0] > m[5] && m[0] > m[10]) {
    const s = Math.sqrt(Math.max(0, 1 + m[0] - m[5] - m[10])) * 2;
    q = [(m[9] - m[6]) / s, s / 4, (m[1] + m[4]) / s, (m[2] + m[8]) / s];
  } else if (m[5] > m[10]) {
    const s = Math.sqrt(Math.max(0, 1 + m[5] - m[0] - m[10])) * 2;
    q = [(m[2] - m[8]) / s, (m[1] + m[4]) / s, s / 4, (m[6] + m[9]) / s];
  } else {
    const s = Math.sqrt(Math.max(0, 1 + m[10] - m[0] - m[5])) * 2;
    q = [(m[4] - m[1]) / s, (m[2] + m[8]) / s, (m[6] + m[9]) / s, s / 4];
  }
  return normalizeQuaternion(q);
}

export function quaternionDistanceDeg(a: Quaternion, b: Quaternion): number {
  const aa = normalizeQuaternion(a), bb = normalizeQuaternion(b);
  const dot = Math.min(1, Math.max(-1, Math.abs(aa.reduce((sum, v, i) => sum + v * bb[i], 0))));
  return 2 * Math.acos(dot) * DEG;
}

export function slerpQuaternion(a: Quaternion, b: Quaternion, t: number): Quaternion {
  const aa = normalizeQuaternion(a);
  let bb = normalizeQuaternion(b);
  let dot = aa.reduce((sum, v, i) => sum + v * bb[i], 0);
  if (dot < 0) { bb = bb.map((v) => -v) as Quaternion; dot = -dot; }
  dot = Math.min(1, Math.max(-1, dot));
  if (dot > 0.9995) return normalizeQuaternion(aa.map((v, i) => v + t * (bb[i] - v)) as Quaternion);
  const angle = Math.acos(dot), sin = Math.sin(angle);
  return normalizeQuaternion(aa.map((v, i) => (Math.sin((1 - t) * angle) * v + Math.sin(t * angle) * bb[i]) / sin) as Quaternion);
}

function translation(m: Mat4): Vec3 { return [m[3], m[7], m[11]]; }

/** Standard DH: Rz(theta) Tz(d) Tx(a) Rx(alpha), using the manufacturer's joint signs. */
export function forwardKinematics(model: RobotModel, joints: JointAngles): FKResult {
  if (!Array.isArray(joints) || joints.length !== 6 || joints.some((v) => !Number.isFinite(v))) throw new Error("关节角需要六个有限数值");
  const frames: Mat4[] = [[...IDENTITY]], jointOrigins: Vec3[] = [], jointAxes: Vec3[] = [];
  for (let i = 0; i < 6; i++) {
    const previous = frames[i], row = model.dh[i];
    jointOrigins.push(translation(previous));
    jointAxes.push([previous[2] * row.thetaSign, previous[6] * row.thetaSign, previous[10] * row.thetaSign]);
    const theta = (joints[i] * row.thetaSign + row.thetaOffsetDeg) * RAD, alpha = row.alphaDeg * RAD;
    const c = Math.cos(theta), s = Math.sin(theta), ca = Math.cos(alpha), sa = Math.sin(alpha);
    frames.push(multiplyMatrices(previous, [
      c, -s * ca, s * sa, row.aMm * c,
      s, c * ca, -c * sa, row.aMm * s,
      0, sa, ca, row.dMm,
      0, 0, 0, 1,
    ]));
  }
  return {
    pose: { position: translation(frames[6]), orientation: matrixToQuaternion(frames[6]) },
    frames, jointOrigins, jointAxes,
    shoulder: translation(frames[1]), elbow: translation(frames[2]), wristCenter: translation(frames[4]),
  };
}

export function jointsWithinLimits(model: RobotModel, joints: JointAngles, tolerance = 1e-8): boolean {
  return joints.length === 6 && joints.every((v, i) => Number.isFinite(v) && v >= model.limits[i].minDeg - tolerance && v <= model.limits[i].maxDeg + tolerance);
}

/** Unwrapped distance: a complete A6 revolution is not a zero-length move. */
export function jointDistance(a: JointAngles, b: JointAngles): number {
  const weights = [1, 1, 1, 0.7, 0.8, 0.45];
  return Math.sqrt(a.reduce((sum, q, i) => sum + weights[i] * (q - b[i]) ** 2, 0));
}

function equivalentAngles(degrees: number, min: number, max: number): number[] {
  const result: number[] = [];
  const first = Math.ceil((min - degrees - 1e-8) / 360), last = Math.floor((max - degrees + 1e-8) / 360);
  for (let turn = first; turn <= last; turn++) {
    const q = Math.max(min, Math.min(max, degrees + turn * 360));
    result.push(Math.abs(q) < 1e-10 ? 0 : q);
  }
  return result;
}

interface RawCandidate {
  joints: JointAngles;
  pose: Pose;
  configuration: RobConf | null;
  singular: boolean;
  positionErrorMm: number;
  orientationErrorDeg: number;
  score: number;
}

/** Analytic position and spherical-wrist solutions for the two IRB 1100 DH models.
 * All 360-degree representations inside real axis limits are retained; every
 * candidate is independently checked through standard-DH forward kinematics.
 */
export function solveIKCandidates(model: RobotModel, pose: Pose, seed: JointAngles = model.readyJoints): RawCandidate[] {
  if (!pose || !Array.isArray(pose.position) || pose.position.length !== 3 || pose.position.some((v) => !Number.isFinite(v))) return [];
  let targetRotation: Mat4;
  try { targetRotation = quaternionToMatrix(pose.orientation); } catch { return []; }
  const { shoulderHeightMm: height, upperArmMm: upper, elbowOffsetMm: offset, forearmMm: forearm, flangeMm: flange } = model.geometry;
  const wrist: Vec3 = [pose.position[0] - flange * targetRotation[2], pose.position[1] - flange * targetRotation[6], pose.position[2] - flange * targetRotation[10]];
  const rho = Math.hypot(wrist[0], wrist[1]), vertical = wrist[2] - height;
  const lower = Math.hypot(offset, forearm), offsetAngle = Math.atan2(offset, forearm);
  const cosine = (rho * rho + vertical * vertical - upper * upper - lower * lower) / (2 * upper * lower);
  if (cosine < -1 - 1e-10 || cosine > 1 + 1e-10) return [];
  const elbowAngle = Math.acos(Math.max(-1, Math.min(1, cosine)));
  const shoulderBase = rho < 1e-8 ? seed[0] * RAD : Math.atan2(wrist[1], wrist[0]);
  const candidates: RawCandidate[] = [], seen = new Set<string>();
  const axisValues = (angle: number, i: number) => equivalentAngles(angle, model.limits[i].minDeg, model.limits[i].maxDeg);

  for (const shoulder of [shoulderBase, shoulderBase + Math.PI]) {
    const horizontal = wrist[0] * Math.cos(shoulder) + wrist[1] * Math.sin(shoulder);
    for (const bend of [elbowAngle, -elbowAngle]) {
      const shoulderAngle = Math.PI / 2 - Math.atan2(vertical, horizontal) + Math.atan2(lower * Math.sin(bend), upper + lower * Math.cos(bend));
      const q3 = (offsetAngle - Math.PI / 2 - bend) * DEG;
      for (const q1 of axisValues(shoulder * DEG, 0)) for (const q2 of axisValues(shoulderAngle * DEG, 1)) for (const elbow of axisValues(q3, 2)) {
        const armFrame = forwardKinematics(model, [q1, q2, elbow, 0, 0, 0]).frames[3];
        const relative = [...IDENTITY];
        for (let row = 0; row < 3; row++) for (let col = 0; col < 3; col++) {
          relative[row * 4 + col] = 0;
          for (let k = 0; k < 3; k++) relative[row * 4 + col] += armFrame[k * 4 + row] * targetRotation[k * 4 + col];
        }
        // R36 = Rz(q4) Ry(-q5) Rz(q6 + 180 degrees).
        const middle = Math.acos(Math.max(-1, Math.min(1, relative[10])));
        let wrists: [number, number, number][];
        if (Math.abs(Math.sin(middle)) < 1e-9) {
          if (relative[10] < 0) continue; // A5=180 degrees lies outside both models' limits.
          const sum = Math.atan2(relative[4], relative[0]) * DEG;
          wrists = [[seed[3], 0, sum - seed[3] - 180]];
        } else {
          const a = Math.atan2(relative[6], relative[2]) * DEG, c = Math.atan2(relative[9], -relative[8]) * DEG;
          wrists = [[a, -middle * DEG, c - 180], [a + 180, middle * DEG, c]];
        }
        for (const [a4, a5, a6] of wrists) for (const q4 of axisValues(a4, 3)) for (const q5 of axisValues(a5, 4)) for (const q6 of axisValues(a6, 5)) {
          const joints: JointAngles = [q1, q2, elbow, q4, q5, q6];
          const key = joints.map((q) => q.toFixed(6)).join(",");
          if (seen.has(key)) continue;
          seen.add(key);
          const fk = forwardKinematics(model, joints);
          const positionErrorMm = Math.hypot(...fk.pose.position.map((v, i) => v - pose.position[i]));
          const orientationErrorDeg = quaternionDistanceDeg(fk.pose.orientation, pose.orientation);
          if (positionErrorMm > IK_TOLERANCES.positionMm || orientationErrorDeg > IK_TOLERANCES.orientationDeg) continue;
          const configuration = getConfiguration(model, joints, fk);
          candidates.push({ joints, pose: fk.pose, configuration: configuration.configuration, singular: configuration.singular,
            positionErrorMm, orientationErrorDeg, score: jointDistance(joints, seed) });
        }
      }
    }
  }
  return candidates.sort((a, b) => a.score - b.score || a.joints.join(",").localeCompare(b.joints.join(",")));
}

export function inverseKinematics(model: RobotModel, pose: Pose, seed: JointAngles = model.readyJoints): IKSolution[] {
  return solveIKCandidates(model, pose, seed).filter((candidate) => candidate.configuration && !candidate.singular).map((candidate) => ({
    ...candidate,
    id: candidate.joints.map((q) => q.toFixed(4)).join("/"),
    configuration: candidate.configuration!,
  }));
}
