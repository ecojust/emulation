import { matrixToQuaternion, multiplyMatrices } from './kinematics';
import type { GenericChain, GenericFkResult } from './generic-motion-types';
import type { JointAngles, Mat4, Pose, Quaternion, Vec3 } from './types';

const IDENTITY: Mat4 = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
const RAD = Math.PI / 180;
const DEG = 180 / Math.PI;
const NEVER_CANCEL = () => false;
export const GENERIC_IK_TOLERANCES = { positionMm: 0.001, orientationRadians: 1e-5 } as const;

function finiteArray(value: unknown, length: number): value is number[] {
  return Array.isArray(value) && value.length === length
    && Array.from({ length }, (_, index) => value[index]).every(Number.isFinite);
}
function dot(a: Vec3, b: Vec3): number { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
function cross(a: Vec3, b: Vec3): Vec3 { return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]; }
function sub(a: Vec3, b: Vec3): Vec3 { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
function norm(value: Vec3): number { return Math.sqrt(dot(value, value)); }
function translation(matrix: Mat4): Vec3 { return [matrix[3], matrix[7], matrix[11]]; }

/** Reject malformed/unbounded chains before they reach either numerical implementation. */
export function validateGenericChain(chain: GenericChain): void {
  if (!chain || typeof chain.modelId !== 'string' || !chain.modelId.trim()
    || new TextEncoder().encode(chain.modelId).byteLength > 128
    || !Array.isArray(chain.steps) || chain.steps.length < 6 || chain.steps.length > 32) {
    throw new Error('URDF 运动链需要有效型号与 6 至 32 个步骤。');
  }
  const seen = new Set<number>();
  let reach = 0;
  for (const step of chain.steps) {
    if (!step || !finiteArray(step.origin, 16)) throw new Error('URDF 固定变换不是有界刚体矩阵。');
    const m = step.origin;
    if (Math.abs(m[12]) > 1e-10 || Math.abs(m[13]) > 1e-10 || Math.abs(m[14]) > 1e-10 || Math.abs(m[15] - 1) > 1e-10
      || [m[3], m[7], m[11]].some(value => Math.abs(value) > 100_000)) {
      throw new Error('URDF 固定变换不是有界刚体矩阵。');
    }
    const columns: Vec3[] = [[m[0], m[4], m[8]], [m[1], m[5], m[9]], [m[2], m[6], m[10]]];
    if (columns.some((column, i) => Math.abs(dot(column, column) - 1) > 1e-8
      || columns.slice(0, i).some(previous => Math.abs(dot(column, previous)) > 1e-8))
      || Math.abs(dot(columns[0], cross(columns[1], columns[2])) - 1) > 1e-8
      || !finiteArray(step.axis, 3) || Math.abs(norm(step.axis) - 1) > 1e-8) {
      throw new Error('URDF 旋转与关节轴必须正交、右手且归一化。');
    }
    reach += norm(translation(m));
    if (step.jointIndex !== null) {
      const index = step.jointIndex;
      if (!Number.isInteger(index) || index < 0 || index >= 6 || seen.has(index)) {
        throw new Error('URDF 必须恰好包含六个不重复的旋转关节。');
      }
      seen.add(index);
    }
  }
  if (seen.size !== 6 || reach > 1_000_000) throw new Error('URDF 运动链关节缺失或长度超出范围。');
  if (!Array.isArray(chain.limits) || chain.limits.length !== 6) throw new Error('URDF 关节范围或速度无效。');
  for (const limit of chain.limits) {
    if (!limit || ![limit.minDeg, limit.maxDeg, limit.maxSpeedDeg].every(Number.isFinite)
      || limit.minDeg >= limit.maxDeg || Math.abs(limit.minDeg) > 3600 || Math.abs(limit.maxDeg) > 3600
      || limit.maxSpeedDeg <= 0 || limit.maxSpeedDeg > 10_000) {
      throw new Error('URDF 关节范围或速度无效。');
    }
  }
}

export function genericJointsWithinLimits(chain: GenericChain, joints: JointAngles): boolean {
  return finiteArray(joints, 6) && joints.every((angle, index) => angle >= chain.limits[index].minDeg - 1e-9
    && angle <= chain.limits[index].maxDeg + 1e-9);
}

function rotation(axis: Vec3, angle: number): Mat4 {
  const [x, y, z] = axis, s = Math.sin(angle), c = Math.cos(angle), d = 1 - c;
  return [c + x * x * d, x * y * d - z * s, x * z * d + y * s, 0,
    y * x * d + z * s, c + y * y * d, y * z * d - x * s, 0,
    z * x * d - y * s, z * y * d + x * s, c + z * z * d, 0,
    0, 0, 0, 1];
}

/** Internal cancellation is never converted into an unreachable-pose result. */
export class GenericMotionCancelled extends Error {}

/** @internal Callers must validate the complete request before entering the sample loop. */
export function forwardGenericUnchecked(chain: GenericChain, joints: JointAngles, cancel = NEVER_CANCEL): GenericFkResult {
  let transform = [...IDENTITY];
  const jointOrigins = Array.from({ length: 6 }, () => [0, 0, 0]) as GenericFkResult['jointOrigins'];
  const jointAxes = Array.from({ length: 6 }, () => [0, 0, 0]) as GenericFkResult['jointAxes'];
  for (const step of chain.steps) {
    if (cancel()) throw new GenericMotionCancelled();
    transform = multiplyMatrices(transform, step.origin);
    if (step.jointIndex !== null) {
      const index = step.jointIndex;
      jointOrigins[index] = translation(transform);
      jointAxes[index] = [0, 1, 2].map(row => {
        let value = 0;
        for (let column = 0; column < 3; column++) value += transform[row * 4 + column] * step.axis[column];
        return value;
      }) as Vec3;
      transform = multiplyMatrices(transform, rotation(step.axis, joints[index] * RAD));
    }
  }
  return { pose: { position: translation(transform), orientation: matrixToQuaternion(transform) }, transform, jointOrigins, jointAxes };
}

export function forwardGenericKinematics(chain: GenericChain, joints: JointAngles): GenericFkResult {
  validateGenericChain(chain);
  if (!genericJointsWithinLimits(chain, joints)) throw new Error('关节角无效或超出 URDF 限位。');
  return forwardGenericUnchecked(chain, joints);
}

/** Spatial shortest rotation vector: log(target * inverse(current)), in radians. */
function orientationError(target: Quaternion, current: Quaternion): Vec3 {
  const [a, b, c, d] = target, [w, x, y, z] = current;
  let relative = [a * w + b * x + c * y + d * z, -a * x + b * w - c * z + d * y,
    -a * y + b * z + c * w - d * x, -a * z - b * y + c * x + d * w];
  if (relative[0] < 0) relative = relative.map(value => -value);
  const sine = norm([relative[1], relative[2], relative[3]]);
  const factor = sine < 1e-12 ? 2 : 2 * Math.atan2(sine, Math.max(0, relative[0])) / sine;
  return [relative[1] * factor, relative[2] * factor, relative[3] * factor];
}

function residual(target: Pose, current: Pose, scale: number, positionOnly: boolean): { error: number[]; converged: boolean } {
  const position = sub(target.position, current.position), orientation: Vec3 = positionOnly ? [0, 0, 0] : orientationError(target.orientation, current.orientation);
  return { error: [position[0] / scale, position[1] / scale, position[2] / scale, ...orientation],
    converged: norm(position) <= GENERIC_IK_TOLERANCES.positionMm && norm(orientation) <= GENERIC_IK_TOLERANCES.orientationRadians };
}

/** In-place partial-pivot elimination; the last equal pivot matches the Rust implementation. */
function solveLinear(matrix: number[][], rhs: number[]): number[] | null {
  for (let column = 0; column < 6; column++) {
    let pivot = column;
    for (let row = column + 1; row < 6; row++) if (Math.abs(matrix[row][column]) >= Math.abs(matrix[pivot][column])) pivot = row;
    if (Math.abs(matrix[pivot][column]) < 1e-16) return null;
    [matrix[column], matrix[pivot]] = [matrix[pivot], matrix[column]];
    [rhs[column], rhs[pivot]] = [rhs[pivot], rhs[column]];
    for (let row = column + 1; row < 6; row++) {
      const factor = matrix[row][column] / matrix[column][column];
      matrix[row][column] = 0;
      for (let j = column + 1; j < 6; j++) matrix[row][j] -= factor * matrix[column][j];
      rhs[row] -= factor * rhs[column];
    }
  }
  const result = Array<number>(6).fill(0);
  for (let row = 5; row >= 0; row--) {
    let sum = 0;
    for (let column = row + 1; column < 6; column++) sum += matrix[row][column] * result[column];
    result[row] = (rhs[row] - sum) / matrix[row][row];
  }
  return result.every(Number.isFinite) ? result : null;
}

/** @internal Damped least squares seeded from the preceding accepted Cartesian sample. */
export function solveGenericUnchecked(chain: GenericChain, target: Pose, seed: JointAngles, cancel = NEVER_CANCEL): JointAngles | null {
  return solveGenericTaskUnchecked(chain, target, seed, undefined, false, cancel);
}

/** @internal The optional tool transform participates in both the FK objective and TCP Jacobian. */
export function solveGenericTaskUnchecked(chain: GenericChain, target: Pose, seed: JointAngles, tool: Mat4 | undefined, positionOnly: boolean, cancel = NEVER_CANCEL): JointAngles | null {
  const taskPose = (fk: GenericFkResult): Pose => {
    if (!tool) return fk.pose;
    const transform = multiplyMatrices(fk.transform, tool);
    return { position: translation(transform), orientation: matrixToQuaternion(transform) };
  };
  const scale = Math.max(100, chain.steps.reduce((sum, step) => sum + norm(translation(step.origin)), 0));
  let joints = [...seed] as JointAngles, damping = 0.001;
  for (let iteration = 0; iteration < 80; iteration++) {
    if (cancel()) throw new GenericMotionCancelled();
    const fk = forwardGenericUnchecked(chain, joints, cancel);
    const current = taskPose(fk);
    const { error, converged } = residual(target, current, scale, positionOnly);
    if (converged) return joints;
    const cost = error.reduce((sum, value) => sum + value * value, 0);
    const jacobian = Array.from({ length: 6 }, () => Array<number>(6).fill(0));
    for (let column = 0; column < 6; column++) {
      const linear = cross(fk.jointAxes[column], sub(current.position, fk.jointOrigins[column]));
      for (let row = 0; row < 3; row++) {
        jacobian[row][column] = linear[row] / scale;
        jacobian[row + 3][column] = positionOnly ? 0 : fk.jointAxes[column][row];
      }
    }
    const normal = Array.from({ length: 6 }, (_, row) => Array.from({ length: 6 }, (_, column) => {
      let value = 0;
      for (let k = 0; k < 6; k++) value += jacobian[k][row] * jacobian[k][column];
      return value + (row === column ? damping * damping : 0);
    }));
    const rhs = Array.from({ length: 6 }, (_, column) => jacobian.reduce((sum, row, index) => sum + row[column] * error[index], 0));
    let delta = solveLinear(normal, rhs);
    if (!delta) return null;
    const largest = Math.max(...delta.map(Math.abs)), cap = 4 * RAD;
    if (largest > cap) delta = delta.map(value => value * cap / largest);
    let accepted = false;
    for (let attempt = 0; attempt < 8; attempt++) {
      if (cancel()) throw new GenericMotionCancelled();
      const alpha = 0.5 ** attempt;
      const trial = joints.map((value, i) => Math.max(chain.limits[i].minDeg,
        Math.min(chain.limits[i].maxDeg, value + delta![i] * alpha * DEG))) as JointAngles;
      const trialFk = forwardGenericUnchecked(chain, trial, cancel);
      const next = residual(target, taskPose(trialFk), scale, positionOnly);
      const trialCost = next.error.reduce((sum, value) => sum + value * value, 0);
      if (next.converged || trialCost < cost * (1 - 1e-10)) {
        joints = trial;
        accepted = true;
        damping = Math.max(1e-6, damping * 0.5);
        if (next.converged) return joints;
        break;
      }
    }
    if (!accepted) {
      if (damping >= 1) return null;
      damping = Math.min(1, damping * 10);
    }
  }
  return null;
}

export function inverseGenericKinematics(chain: GenericChain, target: Pose, seed: JointAngles, cancel = NEVER_CANCEL): JointAngles | null {
  if (cancel()) throw new GenericMotionCancelled();
  validateGenericChain(chain);
  if (!genericJointsWithinLimits(chain, seed) || !target || !finiteArray(target.position, 3)
    || target.position.some(value => Math.abs(value) > 1_000_000) || !finiteArray(target.orientation, 4)
    || Math.abs(Math.sqrt(target.orientation.reduce((sum, value) => sum + value * value, 0)) - 1) > 1e-6) {
    throw new Error('数值逆解的姿态或起始关节无效。');
  }
  return solveGenericUnchecked(chain, target, seed, cancel);
}
