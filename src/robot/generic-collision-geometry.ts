import { BufferAttribute, Matrix4, Vector3 } from 'three';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import descriptors from '../catalog/generic-collision.json';
import type { GenericCollisionDescriptor } from '../catalog/collision-descriptors';
import { readVerifiedCatalogFiles } from '../catalog/client';
import { buildCollisionMesh, inspectMeshZone, type CollisionMesh } from './collision-mesh';
import type { CollisionZone, CollisionResult } from './collision-geometry';
import { genericJointsWithinLimits } from './generic-kinematics';
import { quaternionToMatrix } from './kinematics';
import type { JointAngles, Mat4, Pose } from './types';
export interface GenericCollisionRequest { modelId: string; basePose?: Pose; zones: CollisionZone[]; solutions: { id: string; joints: JointAngles }[] }
export interface GenericCollisionModel { descriptor: GenericCollisionDescriptor; meshes: CollisionMesh[] }
export function genericCollisionDescriptor(modelId: string): GenericCollisionDescriptor {
  const result = (descriptors as unknown as GenericCollisionDescriptor[]).find(value => value.modelId === modelId);
  if (!result) throw new Error('型号没有已验证的三角网格描述。');
  return result;
}
export function genericCollisionParts(modelId: string): readonly { index: number; name: string }[] { return genericCollisionDescriptor(modelId).parts; }
function baseMatrix(pose?: Pose): Matrix4 {
  if (!pose) return new Matrix4();
  if (!Array.isArray(pose.position) || pose.position.length !== 3 || !pose.position.every(v => Number.isFinite(v) && Math.abs(v) <= 1e6)) throw new Error('基座世界坐标无效。');
  return new Matrix4().set(...quaternionToMatrix(pose.orientation) as Parameters<Matrix4['set']>).setPosition(...pose.position);
}
export function validateGenericCollisionRequest(request: GenericCollisionRequest): void {
  if (!request) throw new Error('缺少干涉请求。');
  const descriptor = genericCollisionDescriptor(request.modelId); baseMatrix(request.basePose);
  if (!Array.isArray(request.zones) || request.zones.length > 64 || !Array.isArray(request.solutions) || request.solutions.length > 256) throw new Error('干涉请求超过区域或姿态数量限制。');
  const validId = (id: unknown): id is string => typeof id === 'string' && !!id.trim() && new TextEncoder().encode(id).length <= 128;
  const ids = new Set<string>();
  for (const zone of request.zones) {
    if (!zone || !validId(zone.id) || ids.has(zone.id) || typeof zone.enabled !== 'boolean'
      || !Array.isArray(zone.centerMm) || zone.centerMm.length !== 3 || !zone.centerMm.every(v => Number.isFinite(v) && Math.abs(v) <= 1e6)
      || !Array.isArray(zone.sizeMm) || zone.sizeMm.length !== 3 || !zone.sizeMm.every(v => Number.isFinite(v) && v > 0 && v <= 1e6)) throw new Error('危险区标识或坐标尺寸无效。');
    ids.add(zone.id);
  }
  ids.clear();
  for (const row of request.solutions) {
    if (!row || !validId(row.id) || ids.has(row.id) || !genericJointsWithinLimits(descriptor.chain, row.joints)) throw new Error('候选姿态标识或关节角无效。');
    ids.add(row.id);
  }
}
export function genericCollisionTransforms(modelId: string, joints: JointAngles, basePose?: Pose): Mat4[] {
  const descriptor = genericCollisionDescriptor(modelId);
  if (!genericJointsWithinLimits(descriptor.chain, joints)) throw new Error('关节角超出型号限位。');
  return descriptor.parts.map(part => {
    const transform = baseMatrix(basePose);
    for (const step of part.steps) {
      transform.multiply(new Matrix4().set(...step.origin as Parameters<Matrix4['set']>));
      if (step.jointIndex !== null) transform.multiply(new Matrix4().makeRotationAxis(new Vector3(...step.axis), joints[step.jointIndex] * Math.PI / 180));
    }
    return transform.transpose().toArray() as Mat4;
  });
}
/** Trusted factory; production caller verifies all catalog bytes before parsing. */
export function buildGenericCollisionModel(modelId: string, files: ReadonlyMap<string, ArrayBuffer>): GenericCollisionModel {
  const descriptor = genericCollisionDescriptor(modelId), meshes: CollisionMesh[] = [];
  try {
    for (const part of descriptor.parts) {
      const bytes = files.get(part.path);
      if (!bytes || bytes.byteLength !== part.bytes || bytes.byteLength < 84 || new DataView(bytes).getUint32(80, true) * 50 + 84 !== bytes.byteLength) throw new Error('机器人网格文件长度或三角数量不符。');
      const geometry = new STLLoader().parse(bytes);
      try {
        const position = geometry.getAttribute('position'), scaled = new Float64Array(position.count * 3);
        for (let index = 0; index < position.count; index++) for (let axis = 0; axis < 3; axis++) scaled[index * 3 + axis] = position.getComponent(index, axis) * part.scale[axis];
        geometry.setAttribute('position', new BufferAttribute(scaled, 3));
        meshes.push(buildCollisionMesh(geometry));
      } finally { geometry.dispose(); }
    }
    return { descriptor, meshes };
  } catch (error) { for (const mesh of meshes) mesh.geometry.dispose(); throw error; }
}
const cache = new Map<string, Promise<GenericCollisionModel>>();
async function loadModel(modelId: string): Promise<GenericCollisionModel> {
  let value = cache.get(modelId);
  if (!value) {
    genericCollisionDescriptor(modelId);
    value = readVerifiedCatalogFiles(modelId).then(files => buildGenericCollisionModel(modelId, files));
    cache.set(modelId, value);
    void value.catch(() => { if (cache.get(modelId) === value) cache.delete(modelId); });
  }
  return value;
}
export async function checkGenericCollisionReference(request: GenericCollisionRequest, supplied?: GenericCollisionModel): Promise<CollisionResult[]> {
  validateGenericCollisionRequest(request);
  const model = supplied ?? await loadModel(request.modelId);
  if (model.descriptor.modelId !== request.modelId) throw new Error('网格缓存型号不匹配。');
  return request.solutions.map(row => {
    const frames = genericCollisionTransforms(request.modelId, row.joints, request.basePose), hits = new Set<number>(), unknowns = new Set<number>();
    const zones: CollisionResult['zones'] = [];
    for (const zone of request.zones.filter(zone => zone.enabled)) {
      const collidingLinks: number[] = [], uncertainLinks: number[] = [];
      for (const [index, mesh] of model.meshes.entries()) {
        const contact = inspectMeshZone(mesh, frames[index], zone);
        if (contact === 'hit') { collidingLinks.push(index); hits.add(index); }
        if (contact === 'uncertain') { uncertainLinks.push(index); unknowns.add(index); }
      }
      if (collidingLinks.length || uncertainLinks.length) zones.push({ zoneId: zone.id, collidingLinks, uncertainLinks });
    }
    return { id: row.id, collidingLinks: [...hits].sort((a,b)=>a-b), uncertainLinks: [...unknowns].sort((a,b)=>a-b), zones };
  });
}
