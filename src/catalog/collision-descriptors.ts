import type { PreparedUrdfModel } from './urdf-types';
import { urdfOriginMatrix } from './kinematics';
import { buildGenericChain } from '../robot/generic-motion-chain';
import type { GenericChain, GenericChainStep } from '../robot/generic-motion-types';
import type { Mat4, Vec3 } from '../robot/types';
export interface GenericMeshSource { path: string; bytes: number; sha256: string; url: string }
export interface GenericCollisionPart extends GenericMeshSource { index: number; name: string; steps: GenericChainStep[]; scale: Vec3 }
export interface GenericCollisionDescriptor { modelId: string; chain: GenericChain; parts: GenericCollisionPart[] }
/** Every visual, including fixed side branches, retains its full original ancestry. */
export function buildGenericCollisionDescriptor(prepared: PreparedUrdfModel, files: GenericMeshSource[]): GenericCollisionDescriptor {
  const robot = prepared.description, parents = new Map(robot.joints.map(joint => [joint.child, joint]));
  const indices = new Map(robot.actuatedJoints.map((joint, index) => [joint.name, index]));
  const matrix = (origin: Parameters<typeof urdfOriginMatrix>[0]) => {
    const value = urdfOriginMatrix(origin).transpose().toArray() as Mat4;
    for (const index of [3, 7, 11]) value[index] *= 1000;
    return value;
  };
  const parts: GenericCollisionPart[] = [];
  for (const link of robot.links) for (const [visualIndex, visual] of link.visuals.entries()) {
    const ancestry = []; let name = link.name;
    while (name !== robot.rootLink) {
      const joint = parents.get(name);
      if (!joint || ancestry.length >= 32) throw new Error('Visual ancestry is incomplete');
      ancestry.push(joint); name = joint.parent;
    }
    const steps: GenericChainStep[] = ancestry.reverse().map(joint => ({ origin: matrix(joint.origin), axis: [...joint.axis], jointIndex: joint.type === 'fixed' ? null : indices.get(joint.name)! }));
    steps.push({ origin: matrix(visual.origin), axis: [1, 0, 0], jointIndex: null });
    const file = files.find(file => file.path === visual.meshPath);
    if (!file || !visual.scale.every(value => Number.isFinite(value) && value > 0)) throw new Error('Visual file or scale is invalid');
    parts.push({ index: parts.length, name: link.name + (link.visuals.length > 1 ? ` (${visualIndex + 1})` : ''), ...file, steps, scale: visual.scale.map(value => value * 1000) as Vec3 });
  }
  if (!parts.length || parts.length > 64) throw new Error('Visual count is invalid');
  return { modelId: prepared.id, chain: buildGenericChain(prepared), parts };
}
