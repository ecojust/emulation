import type { Pose, Vec3 } from '../robot/types';
import type { SurfaceAnchor, SurfaceMesh } from './types';

/** Entire request is a world-space mm snapshot, independent of a mutable scene. */
export interface SurfaceRequest {
  mesh: SurfaceMesh;
  boundary: SurfaceAnchor[];
  /** Four corners in boundary traversal order, defining (0,0), (1,0), (1,1), (0,1). */
  controlIndices: [number, number, number, number];
  sampling: { mode: 'spacing' | 'divisions'; u: number; v: number };
  normalSign: 1 | -1;
  maxNormalStepDeg: number;
  maxPoints: number;
}
export interface SurfaceGeneratedPoint {
  pose: Pose;
  normal: Vec3;
  anchor: SurfaceAnchor;
  uv: [number, number];
  row: number;
}
export interface SurfaceSuccess {
  status: 'success';
  points: SurfaceGeneratedPoint[];
  rowOffsets: number[];
  maxSpacingErrorMm: number;
  maxRegularSpacingErrorMm: number;
  maxOrientationStepDeg: number;
  warnings: string[];
}
export interface SurfaceFailure { status: 'failure'; code: string; message: string }
export type SurfaceResult = SurfaceSuccess | SurfaceFailure;
