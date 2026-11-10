import { urdfOriginMatrix } from '../catalog/kinematics';
import type { PreparedUrdfModel } from '../catalog/urdf-types';
import type { GenericChain } from './generic-motion-types';
import { validateGenericChain } from './generic-kinematics';
import type { Mat4 } from './types';

/** Preserve each fixed transform and the original actuator index; side branches are visual only. */
export function buildGenericChain(prepared: PreparedUrdfModel): GenericChain {
  const robot = prepared.description;
  if (!robot.toolLink || robot.actuatedJoints.length !== 6) throw new Error('当前 URDF 直线仿真需要完整的六轴 tool0/flange 关节链');
  const parents = new Map(robot.joints.map(joint => [joint.child, joint]));
  const actuatorIndices = new Map(robot.actuatedJoints.map((joint, index) => [joint.name, index]));
  const ancestry = [];
  const seen = new Set<string>();
  let link = robot.toolLink;
  while (link !== robot.rootLink) {
    if (seen.has(link) || ancestry.length >= 32) throw new Error('URDF 工具关节链包含循环或超过 32 个变换');
    seen.add(link);
    const joint = parents.get(link);
    if (!joint) throw new Error('URDF 工具关节链与基座未连接');
    ancestry.push(joint);
    link = joint.parent;
  }
  const steps = ancestry.reverse().map(joint => {
    const origin = urdfOriginMatrix(joint.origin).transpose().toArray() as Mat4;
    for (const index of [3, 7, 11]) origin[index] *= 1000;
    const jointIndex = joint.type === 'fixed' ? null : actuatorIndices.get(joint.name);
    if (jointIndex === undefined) throw new Error('URDF 工具链的可动关节索引无效');
    return { origin, axis: [...joint.axis] as [number, number, number], jointIndex };
  });
  const chain: GenericChain = {
    modelId: prepared.id,
    steps,
    limits: robot.actuatedJoints.map(joint => ({ minDeg: joint.minDeg, maxDeg: joint.maxDeg, maxSpeedDeg: joint.maxSpeedDeg })) as GenericChain['limits'],
  };
  validateGenericChain(chain);
  return chain;
}
