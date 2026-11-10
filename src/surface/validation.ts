import { quaternionToMatrix } from '../robot/kinematics';
import type { Pose, Vec3 } from '../robot/types';
import { normalizePose } from '../workspace/frame-graph';
import { SURFACE_LIMITS, type SceneSurface, type SurfaceAnchor, type SurfaceMesh, type SurfacePick } from './types';

const record = (value: unknown): value is Record<string, any> => !!value && typeof value === 'object' && !Array.isArray(value);
const requireValue = (valid: unknown, message: string): void => { if (!valid) throw new Error('工件模型无效：' + message); };
function finiteArray(value: unknown, length?: number, bound = Number.MAX_VALUE): value is number[] {
  return Array.isArray(value) && (length === undefined || value.length === length) && Array.from(value).every(n => typeof n === 'number' && Number.isFinite(n) && Math.abs(n) <= bound);
}
function label(value: unknown, maximum: number): value is string {
  return typeof value === 'string' && !!value.trim() && [...value].length <= maximum && !/[\x00-\x1f\x7f]/.test(value);
}
export function validateSurfaceMesh(value: unknown): SurfaceMesh {
  requireValue(record(value), '缺少三角网格。');
  const raw = value as Record<string, any>;
  requireValue(Array.isArray(raw.positions) && raw.positions.length > 0 && raw.positions.length % 9 === 0 && raw.positions.length <= SURFACE_LIMITS.triangles * 9, '三角形数量必须在1至100000之间。');
  requireValue(finiteArray(raw.positions, undefined, SURFACE_LIMITS.coordinateMm), '顶点必须是有界的毫米坐标。');
  requireValue(finiteArray(raw.normals, raw.positions.length, 1.00001), '每个顶点都需要有效的单位法向量。');
  const p = raw.positions as number[], n = raw.normals as number[];
  for (let offset = 0; offset < p.length; offset += 9) {
    const ax = p[offset + 3] - p[offset], ay = p[offset + 4] - p[offset + 1], az = p[offset + 5] - p[offset + 2];
    const bx = p[offset + 6] - p[offset], by = p[offset + 7] - p[offset + 1], bz = p[offset + 8] - p[offset + 2];
    const cross = [ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx], area = Math.hypot(...cross);
    requireValue(Number.isFinite(area) && area > SURFACE_LIMITS.minimumDoubleAreaMm2, '存在退化或重复顶点三角形。');
    for (let corner = 0; corner < 9; corner += 3) {
      const length = Math.hypot(n[offset + corner], n[offset + corner + 1], n[offset + corner + 2]);
      requireValue(Math.abs(length - 1) <= 1e-5, '法向量未归一化。');
      requireValue((cross[0] * n[offset + corner] + cross[1] * n[offset + corner + 1] + cross[2] * n[offset + corner + 2]) / area > 1e-5, '法向量与三角形绕序不一致。');
    }
  }
  return { positions: p.slice(), normals: n.slice() };
}
export function validateSceneSurface(value: unknown): SceneSurface {
  requireValue(record(value), '缺少模型数据。'); const raw = value as Record<string, any>;
  requireValue(label(raw.id, 128) && !['__proto__', 'constructor', 'prototype'].includes(raw.id), '对象标识。');
  requireValue(label(raw.name, 80) && Number.isSafeInteger(raw.revision) && raw.revision >= 0 && typeof raw.visible === 'boolean', '名称、修订或显示设置。');
  requireValue(record(raw.source) && label(raw.source.name, 256) && ['stl', 'obj', 'glb'].includes(raw.source.format) && ['mm', 'm'].includes(raw.source.unit), '模型来源。');
  requireValue(raw.source.format !== 'glb' || raw.source.unit === 'm', 'GLB使用标准米单位。');
  requireValue(record(raw.pose) && finiteArray(raw.pose.position, 3, SURFACE_LIMITS.coordinateMm) && finiteArray(raw.pose.orientation, 4) && Math.abs(Math.hypot(...raw.pose.orientation) - 1) <= 1e-6, '模型世界位姿。');
  const pose = normalizePose(raw.pose as Pose), mesh = validateSurfaceMesh(raw.mesh);
  const matrix = quaternionToMatrix(pose.orientation);
  for (let i = 0; i < mesh.positions.length; i += 3) for (let axis = 0; axis < 3; axis++) {
    const world = matrix[axis * 4] * mesh.positions[i] + matrix[axis * 4 + 1] * mesh.positions[i + 1] + matrix[axis * 4 + 2] * mesh.positions[i + 2] + pose.position[axis];
    requireValue(Number.isFinite(world) && Math.abs(world) <= SURFACE_LIMITS.coordinateMm, '模型变换后的顶点超出场景范围。');
  }
  return { id: raw.id, name: raw.name.trim(), revision: raw.revision, visible: raw.visible, pose, source: { name: raw.source.name, format: raw.source.format, unit: raw.source.unit }, mesh };
}
export function validateSceneSurfaces(value: unknown): SceneSurface[] {
  if (value === undefined) return [];
  requireValue(Array.isArray(value) && value.length <= SURFACE_LIMITS.surfaces, '工件数量不能超过32。');
  const values = value as unknown[];
  const total = values.reduce<number>((sum, item) => sum + (record(item) && record(item.mesh) && Array.isArray(item.mesh.positions) ? item.mesh.positions.length / 9 : 0), 0);
  requireValue(Number.isFinite(total) && total <= SURFACE_LIMITS.totalTriangles, '项目工件三角形总数不能超过200000。');
  const result = values.map(validateSceneSurface), ids = new Set<string>();
  for (const surface of result) { requireValue(!ids.has(surface.id), '工件对象标识重复。'); ids.add(surface.id); }
  return result;
}
/** Apply the model's rigid world placement once, keeping stored triangle indices unchanged. */
export function surfaceWorldMesh(surface: SceneSurface): SurfaceMesh {
  const matrix = quaternionToMatrix(surface.pose.orientation), positions: number[] = [], normals: number[] = [];
  for (let i = 0; i < surface.mesh.positions.length; i += 3) for (let axis = 0; axis < 3; axis++) {
    positions.push(matrix[axis * 4] * surface.mesh.positions[i] + matrix[axis * 4 + 1] * surface.mesh.positions[i + 1] + matrix[axis * 4 + 2] * surface.mesh.positions[i + 2] + surface.pose.position[axis]);
    normals.push(matrix[axis * 4] * surface.mesh.normals[i] + matrix[axis * 4 + 1] * surface.mesh.normals[i + 1] + matrix[axis * 4 + 2] * surface.mesh.normals[i + 2]);
  }
  return { positions, normals };
}
export function evaluateSurfaceAnchor(surface: SceneSurface, anchor: SurfaceAnchor): { position: Vec3; normal: Vec3 } {
  requireValue(Number.isSafeInteger(anchor.triangleIndex) && anchor.triangleIndex >= 0 && anchor.triangleIndex < surface.mesh.positions.length / 9, '表面点引用的三角形不存在。');
  requireValue(finiteArray(anchor.barycentric, 3) && anchor.barycentric.every(weight => weight >= -1e-7 && weight <= 1 + 1e-7) && Math.abs(anchor.barycentric.reduce((sum, weight) => sum + weight, 0) - 1) <= 1e-6, '表面点的重心坐标无效。');
  const position: Vec3 = [0, 0, 0], normal: Vec3 = [0, 0, 0], matrix = quaternionToMatrix(surface.pose.orientation);
  const local: Vec3 = [0, 0, 0], direction: Vec3 = [0, 0, 0], offset = anchor.triangleIndex * 9;
  for (let corner = 0; corner < 3; corner++) for (let axis = 0; axis < 3; axis++) {
    local[axis] += surface.mesh.positions[offset + corner * 3 + axis] * anchor.barycentric[corner];
    direction[axis] += surface.mesh.normals[offset + corner * 3 + axis] * anchor.barycentric[corner];
  }
  for (let axis = 0; axis < 3; axis++) {
    position[axis] = matrix[axis * 4] * local[0] + matrix[axis * 4 + 1] * local[1] + matrix[axis * 4 + 2] * local[2] + surface.pose.position[axis];
    normal[axis] = matrix[axis * 4] * direction[0] + matrix[axis * 4 + 1] * direction[1] + matrix[axis * 4 + 2] * direction[2];
  }
  const length = Math.hypot(...normal); requireValue(length > 1e-8 && Number.isFinite(length), '表面点法向量退化。');
  return { position, normal: normal.map(n => n / length) as Vec3 };
}
/** Only validate current references; old references stay readable and are marked stale by path logic. */
export function validateSurfacePickReference(pick: SurfacePick, surface: SceneSurface): void {
  if (pick.surfaceId !== surface.id || pick.surfaceRevision !== surface.revision) return;
  const actual = evaluateSurfaceAnchor(surface, pick);
  requireValue(finiteArray(pick.position, 3, SURFACE_LIMITS.coordinateMm) && Math.hypot(...actual.position.map((n, i) => n - pick.position[i])) <= 1e-4, '表面点位置与模型锚点不一致。');
  requireValue(finiteArray(pick.normal, 3, 1.00001) && Math.hypot(...actual.normal.map((n, i) => n - pick.normal[i])) <= 1e-5, '表面点法向与模型锚点不一致。');
}
