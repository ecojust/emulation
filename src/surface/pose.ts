import { Matrix4, Quaternion as ThreeQuaternion, Vector3 } from 'three';
import type { Pose, Quaternion } from '../robot/types';
import type { SurfacePick } from './types';

/** Preserve roll from the preceding point while placing tool +Z on the surface normal. */
export function surfacePickPose(pick: SurfacePick, previous: Quaternion = [1, 0, 0, 0]): Pose {
  const z = new Vector3(...pick.normal).normalize();
  if (!Number.isFinite(z.lengthSq()) || z.lengthSq() < .99) throw new Error('表面法向无效。');
  const q = new ThreeQuaternion(previous[1], previous[2], previous[3], previous[0]);
  const x = new Vector3(1, 0, 0).applyQuaternion(q);
  x.addScaledVector(z, -x.dot(z));
  if (x.lengthSq() < 1e-12) {
    x.set(0, 1, 0).applyQuaternion(q); x.addScaledVector(z, -x.dot(z));
  }
  x.normalize(); const y = new Vector3().crossVectors(z, x).normalize();
  const orientation = new ThreeQuaternion().setFromRotationMatrix(new Matrix4().makeBasis(x, y, z)).normalize();
  const result: Quaternion = [orientation.w, orientation.x, orientation.y, orientation.z];
  if (result.reduce((sum, n, i) => sum + n * previous[i], 0) < 0) for (let i = 0; i < 4; i++) result[i] *= -1;
  return { position: [...pick.position], orientation: result };
}
