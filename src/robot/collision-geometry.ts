import bounds from './link-bounds.json';
import { forwardKinematics, jointsWithinLimits, multiplyMatrices, quaternionToMatrix } from './kinematics';
import { getRobotModel } from './model';
import type { JointAngles, Mat4, Pose, RobotId, Vec3 } from './types';
import { CONTACT_EPSILON_MM, inspectMeshZone, loadCollisionModel } from './collision-mesh';
import type { CollisionMeshModel } from './collision-mesh';

/** World-aligned warning volume, in millimetres. This is not a controller safety zone. */
export interface DangerZone { enabled: boolean; centerMm: Vec3; sizeMm: Vec3 }
export interface CollisionZone extends DangerZone { id: string }
export interface CollisionRequest { basePose?: Pose; modelId: RobotId; zones: CollisionZone[]; solutions: { id: string; joints: JointAngles }[] }
export interface CollisionResult { id: string; collidingLinks: number[]; uncertainLinks: number[]; zones: { zoneId: string; collidingLinks: number[]; uncertainLinks: number[] }[] }
/** Conservative box enclosing a complete rigid mesh; axes are world-space unit vectors. */
export interface LinkBoundingBox { index: number; centerMm: Vec3; halfSizeMm: Vec3; axes: [Vec3, Vec3, Vec3] }

export const COLLISION_LIMITS = { maximumSolutions: 256, maximumZones: 64, maximumIdBytes: 128, coordinateMm: 1e6, sizeMm: 1e6 } as const;
const SAT_EPSILON = 1e-10;
const dot = (a: Vec3, b: Vec3): number => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const finiteNumber = (value: unknown): value is number => typeof value === 'number' && Number.isFinite(value);
const vector = (value: unknown): value is Vec3 => Array.isArray(value) && value.length === 3 && Array.from(value).every(finiteNumber);
const validId = (value: unknown): value is string => typeof value === 'string' && !!value.trim() && new TextEncoder().encode(value).length <= COLLISION_LIMITS.maximumIdBytes;

function baseTransform(pose: Pose): Mat4 {
  if (!pose || !vector(pose.position) || pose.position.some(value => Math.abs(value) > COLLISION_LIMITS.coordinateMm)
    || !Array.isArray(pose.orientation) || pose.orientation.length !== 4 || !Array.from(pose.orientation).every(finiteNumber)) throw new Error('机器人基座位姿必须为有效的世界坐标与四元数');
  const transform = quaternionToMatrix(pose.orientation);
  [transform[3], transform[7], transform[11]] = pose.position;
  return transform;
}

export function validateCollisionRequest(request: CollisionRequest): void {
  if (!request || typeof request.modelId !== 'string' || !Object.hasOwn(bounds.models, request.modelId)) throw new Error('干涉检测需要有效机器人型号');
  if (request.basePose !== undefined) baseTransform(request.basePose);
  const model = getRobotModel(request.modelId);
  if (!Array.isArray(request.zones) || request.zones.length > COLLISION_LIMITS.maximumZones) throw new Error('干涉检测一次最多检查 64 个危险区');
  const zoneIds = new Set<string>();
  for (const zone of request.zones) {
    if (!zone || !validId(zone.id) || zoneIds.has(zone.id)) throw new Error('危险区标识须非空、唯一且不超过 128 字节');
    zoneIds.add(zone.id);
    if (typeof zone.enabled !== 'boolean' || !vector(zone.centerMm) || !vector(zone.sizeMm)
      || zone.centerMm.some(value => Math.abs(value) > COLLISION_LIMITS.coordinateMm)
      || zone.sizeMm.some(value => value <= 0 || value > COLLISION_LIMITS.sizeMm)) {
      throw new Error('危险区中心须在 ±1000000 mm 内，三个尺寸须大于 0 且不超过 1000000 mm');
    }
  }
  if (!Array.isArray(request.solutions) || request.solutions.length > COLLISION_LIMITS.maximumSolutions) throw new Error('干涉检测一次最多检查 256 个姿态');
  const ids = new Set<string>();
  for (const solution of request.solutions) {
    if (!solution || !validId(solution.id) || ids.has(solution.id)) {
      throw new Error('干涉检测的姿态标识须非空、唯一且不超过 128 字节');
    }
    ids.add(solution.id);
    if (!Array.isArray(solution.joints) || solution.joints.length !== 6 || !Array.from(solution.joints).every(finiteNumber) || !jointsWithinLimits(model, solution.joints)) {
      throw new Error('干涉检测的关节角须为六个有限数值且位于该型号限位内');
    }
  }
}

function inverseRigid(frame: Mat4): Mat4 {
  const output = [
    frame[0], frame[4], frame[8], 0,
    frame[1], frame[5], frame[9], 0,
    frame[2], frame[6], frame[10], 0,
    0, 0, 0, 1,
  ];
  const position: Vec3 = [frame[3], frame[7], frame[11]];
  for (let row = 0; row < 3; row++) output[row * 4 + 3] = -dot(output.slice(row * 4, row * 4 + 3) as Vec3, position);
  return output;
}

const zeroInverse = new Map<RobotId, Mat4[]>();
function linkTransforms(modelId: RobotId, joints: JointAngles, basePose?: Pose): Mat4[] {
  const model = getRobotModel(modelId);
  const fk = forwardKinematics(model, joints);
  let inverse = zeroInverse.get(modelId);
  if (!inverse) {
    inverse = forwardKinematics(model, model.homeJoints).frames.map(inverseRigid);
    zeroInverse.set(modelId, inverse);
  }
  const base = basePose === undefined ? null : baseTransform(basePose);
  return fk.frames.map((frame, index) => { const local = multiplyMatrices(frame, inverse![index]); return base ? multiplyMatrices(base, local) : local; });
}

function boxesFromTransforms(modelId: RobotId, transforms: Mat4[]): LinkBoundingBox[] {
  return bounds.models[modelId].parts.map(part => {
    const transform = transforms[part.index];
    const zeroCenter = part.minMm.map((min, axis) => (min + part.maxMm[axis]) / 2);
    const centerMm = [0, 1, 2].map(row => transform[row * 4 + 3] + zeroCenter.reduce((sum, value, column) => sum + transform[row * 4 + column] * value, 0)) as Vec3;
    return {
      index: part.index, centerMm,
      halfSizeMm: part.minMm.map((min, axis) => (part.maxMm[axis] - min) / 2) as Vec3,
      axes: [[transform[0], transform[4], transform[8]], [transform[1], transform[5], transform[9]], [transform[2], transform[6], transform[10]]],
    };
  });
}

/** Same worldBase * F_i(q) * inverse(F_i(0)) placement used by the STL scene. */
export function linkBoundingBoxes(modelId: RobotId, joints: JointAngles, basePose?: Pose): LinkBoundingBox[] {
  if (!Object.hasOwn(bounds.models, modelId)) throw new Error('不支持的机器人型号');
  return boxesFromTransforms(modelId, linkTransforms(modelId, joints, basePose));
}

/** 15-axis OBB/AABB separating-axis test. Contact counts as possible interference.
 * A positive result is conservative: an enclosing box can overlap empty space in a mesh.
 */
export function intersectsDangerZone(box: LinkBoundingBox, zone: DangerZone): boolean {
  const a = box.halfSizeMm;
  const b = zone.sizeMm.map(value => value / 2);
  const rotation = box.axes;
  const absolute = rotation.map(axis => axis.map(value => Math.abs(value) + SAT_EPSILON));
  const difference = zone.centerMm.map((value, axis) => value - box.centerMm[axis]) as Vec3;
  const translation = rotation.map(axis => dot(difference, axis));
  // A's three face normals.
  for (let i = 0; i < 3; i++) {
    const radiusB = b.reduce((sum, value, j) => sum + value * absolute[i][j], 0);
    if (Math.abs(translation[i]) > a[i] + radiusB) return false;
  }
  // B's three world-aligned face normals.
  for (let j = 0; j < 3; j++) {
    const radiusA = a.reduce((sum, value, i) => sum + value * absolute[i][j], 0);
    if (Math.abs(difference[j]) > radiusA + b[j]) return false;
  }
  // Nine edge cross-products. The epsilon also handles almost parallel axes.
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) {
    const i1 = (i + 1) % 3, i2 = (i + 2) % 3, j1 = (j + 1) % 3, j2 = (j + 2) % 3;
    const radiusA = a[i1] * absolute[i2][j] + a[i2] * absolute[i1][j];
    const radiusB = b[j1] * absolute[i][j2] + b[j2] * absolute[i][j1];
    const distance = Math.abs(translation[i2] * rotation[i1][j] - translation[i1] * rotation[i2][j]);
    if (distance > radiusA + radiusB) return false;
  }
  return true;
}

/** Browser/reference computation only. The desktop always calls the Rust engine. */
export async function checkCollisionReference(request: CollisionRequest, options: { loadModel?: (modelId: RobotId) => Promise<CollisionMeshModel> } = {}): Promise<CollisionResult[]> {
  validateCollisionRequest(request);
  const activeZones = request.zones.filter(zone => zone.enabled);
  if (!activeZones.length || !request.solutions.length) return request.solutions.map(({ id }) => ({ id, collidingLinks: [], uncertainLinks: [], zones: [] }));
  const meshModel = await (options.loadModel ?? loadCollisionModel)(request.modelId);
  if (meshModel.modelId !== request.modelId || meshModel.links.length !== 7) throw new Error('三角网格与机器人型号不匹配');
  return request.solutions.map(solution => {
    const transforms = linkTransforms(request.modelId, solution.joints, request.basePose);
    const boxes = boxesFromTransforms(request.modelId, transforms);
    const zones = activeZones.flatMap(zone => {
      const collidingLinks: number[] = [], uncertainLinks: number[] = [];
      const expanded = { ...zone, sizeMm: zone.sizeMm.map(size => size + 2 * CONTACT_EPSILON_MM) as Vec3 };
      for (const box of boxes) {
        if (!intersectsDangerZone(box, expanded)) continue;
        const contact = inspectMeshZone(meshModel.links[box.index], transforms[box.index], zone);
        if (contact === 'hit') collidingLinks.push(box.index);
        else if (contact === 'uncertain') uncertainLinks.push(box.index);
      }
      return collidingLinks.length || uncertainLinks.length ? [{ zoneId: zone.id, collidingLinks, uncertainLinks }] : [];
    });
    const collidingLinks = [...new Set(zones.flatMap(zone => zone.collidingLinks))].sort((a, b) => a - b);
    const uncertainLinks = [...new Set(zones.flatMap(zone => zone.uncertainLinks))].sort((a, b) => a - b);
    return { id: solution.id, collidingLinks, uncertainLinks, zones };
  });
}

