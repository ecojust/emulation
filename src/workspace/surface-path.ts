import type { Pose, Vec3 } from '../robot/types';
import type { SurfacePick } from '../surface/types';

export const SURFACE_PATH_LIMITS = { regionPoints: 4096, uvPoints: 256, coordinateMm: 1_000_000 } as const;
export interface SurfaceUvSettings {
  readonly uSpacingMm: number;
  readonly vSpacingMm: number;
  readonly reverseNormal: boolean;
}
/** Geometry only: arbitrary surface targets have no invented robot joints. */
export interface SurfacePathSample {
  /** TCP pose in the owning Path frame. Its positive Z follows the chosen surface normal. */
  readonly pose: Pose;
  /** Capture-time world position/normal; triangle and barycentric coordinates remain mesh-local. */
  readonly pick: SurfacePick;
}
export interface SurfacePathPoint extends SurfacePathSample {
  readonly id: string;
  readonly name: string;
}
export interface SurfacePathData {
  readonly kind: 'region' | 'uv';
  readonly revision: number;
  readonly surfaceId: string;
  readonly surfaceRevision: number;
  readonly points: readonly SurfacePathPoint[];
  /** In boundary order: U0V0, U1V0, U1V1, U0V1. */
  readonly controlPointIds: readonly string[];
  readonly source?: { readonly pathId: string; readonly revision: number };
  readonly settings?: SurfaceUvSettings;
  /** Present only after every generated target has a validated joint solution. */
  readonly resolvedRevision?: number;
  /** Keep stale geometry visible while prohibiting motion and export. */
  readonly invalidated?: true;
}
const record = (value: unknown): value is Record<string, unknown> => Boolean(value) && typeof value === 'object' && !Array.isArray(value);
function requireValue(value: unknown, message: string): asserts value { if (!value) throw new Error('表面路径数据无效：' + message); }
function identifier(value: unknown): asserts value is string {
  requireValue(typeof value === 'string' && value.trim() && value.length <= 128 && !['__proto__', 'constructor', 'prototype'].includes(value), '对象标识');
}
function revision(value: unknown): asserts value is number { requireValue(Number.isSafeInteger(value) && Number(value) >= 0, '修订编号'); }
function tuple(value: unknown, size: number, bound = Number.MAX_VALUE): asserts value is number[] {
  requireValue(Array.isArray(value) && value.length === size && Array.from(value).every(n => typeof n === 'number' && Number.isFinite(n) && Math.abs(n) <= bound), '坐标');
}
function pose(value: unknown): Pose {
  requireValue(record(value), '目标位姿'); tuple(value.position, 3, SURFACE_PATH_LIMITS.coordinateMm); tuple(value.orientation, 4);
  requireValue(Math.abs(Math.hypot(...value.orientation) - 1) <= 1e-6, '目标四元数');
  const result: Pose = { position: [...value.position] as Vec3, orientation: [...value.orientation] as Pose['orientation'] };
  Object.freeze(result.position); Object.freeze(result.orientation); return Object.freeze(result);
}
export function validateSurfaceUvSettings(value: unknown): SurfaceUvSettings {
  requireValue(record(value), 'UV设置');
  for (const spacing of [value.uSpacingMm, value.vSpacingMm]) requireValue(typeof spacing === 'number' && Number.isFinite(spacing) && spacing > 0 && spacing <= SURFACE_PATH_LIMITS.coordinateMm, 'UV间距必须为有限正数（mm）');
  requireValue(typeof value.reverseNormal === 'boolean', '法向方向');
  return Object.freeze({ uSpacingMm: value.uSpacingMm as number, vSpacingMm: value.vSpacingMm as number, reverseNormal: value.reverseNormal });
}
export function validateSurfacePathSample(value: unknown): SurfacePathSample {
  requireValue(record(value) && record(value.pick), '表面点');
  const p = value.pick;
  identifier(p.surfaceId); revision(p.surfaceRevision); revision(p.triangleIndex);
  tuple(p.barycentric, 3); tuple(p.position, 3, SURFACE_PATH_LIMITS.coordinateMm); tuple(p.normal, 3);
  requireValue(p.barycentric.every(n => n >= -1e-8 && n <= 1 + 1e-8) && Math.abs(p.barycentric.reduce((a, b) => a + b, 0) - 1) <= 1e-6, '三角形重心坐标');
  requireValue(Math.abs(Math.hypot(...p.normal) - 1) <= 1e-6, '表面法向');
  const pick: SurfacePick = { surfaceId: p.surfaceId, surfaceRevision: p.surfaceRevision, triangleIndex: p.triangleIndex,
    barycentric: [...p.barycentric] as Vec3, position: [...p.position] as Vec3, normal: [...p.normal] as Vec3 };
  Object.freeze(pick.barycentric); Object.freeze(pick.position); Object.freeze(pick.normal); Object.freeze(pick);
  return Object.freeze({ pose: pose(value.pose), pick });
}
/** Validation is bounded and preserves exact poses instead of normalizing/round-tripping them. */
export function validateSurfacePath(value: unknown): SurfacePathData {
  requireValue(record(value) && (value.kind === 'region' || value.kind === 'uv'), '区域类型');
  revision(value.revision); identifier(value.surfaceId); revision(value.surfaceRevision);
  requireValue(Array.isArray(value.points) && value.points.length <= (value.kind === 'uv' ? SURFACE_PATH_LIMITS.uvPoints : SURFACE_PATH_LIMITS.regionPoints), '表面点数量');
  const ids = new Set<string>();
  const points = Array.from(value.points, raw => {
    requireValue(record(raw), '表面点'); identifier(raw.id); requireValue(!ids.has(raw.id), '重复表面点'); ids.add(raw.id);
    requireValue(typeof raw.name === 'string' && raw.name.trim() && [...raw.name].length <= 80 && !/[\x00-\x1f\x7f]/.test(raw.name), '表面点名称');
    const sample = validateSurfacePathSample(raw);
    requireValue(sample.pick.surfaceId === value.surfaceId && sample.pick.surfaceRevision === value.surfaceRevision, '区域必须来自同一模型修订');
    return Object.freeze({ ...sample, id: raw.id, name: raw.name });
  });
  requireValue(Array.isArray(value.controlPointIds) && value.controlPointIds.length <= 4, '最多四个控制点');
  const controls = Array.from(value.controlPointIds);
  requireValue(controls.every(id => typeof id === 'string' && ids.has(id)) && new Set(controls).size === controls.length, '控制点引用');
  requireValue(points.filter(point => controls.includes(point.id)).every((point, index) => point.id === controls[index]), '控制点须按区域边界顺序排列');
  const base = { kind: value.kind as SurfacePathData['kind'], revision: value.revision, surfaceId: value.surfaceId, surfaceRevision: value.surfaceRevision,
    points: Object.freeze(points), controlPointIds: Object.freeze(controls as string[]) };
  requireValue(value.invalidated === undefined || value.invalidated === true, '失效标记');
  if (value.kind === 'region') {
    requireValue(value.source === undefined && value.settings === undefined && value.resolvedRevision === undefined, '点选区域不能伪装已求解路径');
    return Object.freeze({ ...base, ...(value.invalidated ? { invalidated: true as const } : {}) });
  }
  requireValue(points.length > 0 && controls.length === 0 && record(value.source), 'UV路径来源');
  identifier(value.source.pathId); revision(value.source.revision);
  if (value.resolvedRevision !== undefined) { revision(value.resolvedRevision); requireValue(value.resolvedRevision === value.revision && value.invalidated !== true, '已求解修订'); }
  return Object.freeze({ ...base, source: Object.freeze({ pathId: value.source.pathId, revision: value.source.revision }), settings: validateSurfaceUvSettings(value.settings),
    ...(value.resolvedRevision !== undefined ? { resolvedRevision: value.resolvedRevision } : {}), ...(value.invalidated ? { invalidated: true as const } : {}) });
}
