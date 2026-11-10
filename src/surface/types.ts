import type { Pose, Vec3 } from '../robot/types';

/** Serializable, non-indexed triangles in model-local millimetres. Each face occupies 9 numbers. */
export interface SurfaceMesh {
  positions: number[];
  /** One unit, outward shading normal per vertex; sharp boundaries keep distinct normals. */
  normals: number[];
}
export type SurfaceFormat = 'stl' | 'obj' | 'glb';
export type SurfaceUnit = 'mm' | 'm';
export interface SceneSurface {
  id: string;
  name: string;
  revision: number;
  visible: boolean;
  /** World <- model-local, with all scale already baked into mesh positions. */
  pose: Pose;
  source: { name: string; format: SurfaceFormat; unit: SurfaceUnit };
  mesh: SurfaceMesh;
}
export interface SurfaceAnchor {
  triangleIndex: number;
  /** Weights for the triangle's three vertices, in the mesh's stable stored order. */
  barycentric: Vec3;
}
export interface SurfacePick extends SurfaceAnchor {
  surfaceId: string;
  surfaceRevision: number;
  /** Pick position and interpolated normal in world coordinates. */
  position: Vec3;
  normal: Vec3;
}
export const SURFACE_LIMITS = {
  fileBytes: 20 * 1024 * 1024,
  triangles: 100_000,
  totalTriangles: 200_000,
  surfaces: 32,
  coordinateMm: 1_000_000,
  minimumDoubleAreaMm2: 1e-10,
  smoothingAngleDegrees: 45,
} as const;
