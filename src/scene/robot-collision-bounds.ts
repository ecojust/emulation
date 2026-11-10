import { Box3, Matrix4, Vector3 } from 'three';
import type { PreparedUrdfModel, UrdfJoint } from '../catalog/urdf-types';
import { urdfLinkMatrices, urdfOriginMatrix } from '../catalog/kinematics';
import { buildGenericChain } from '../robot/generic-motion-chain';
import type { Vec3 } from '../robot/types';

/** An enclosure in the frame after `stepIndex` complete chain steps; 0 is the root. */
export interface GenericCollisionBody {
  stepIndex: number;
  centerMm: Vec3;
  halfSizeMm: Vec3;
}

/** Enclose every loaded visual, including fixed branches, in its moving chain frame.
 * Geometry remains owned by the model loader. No renderer, world base, or TCP offset
 * participates in these local bounds; the planner applies those transforms once.
 */
export function buildGenericCollisionBodies(prepared: PreparedUrdfModel): GenericCollisionBody[] {
  const chain = buildGenericChain(prepared), robot = prepared.description;
  const parentByChild = new Map(robot.joints.map(joint => [joint.child, joint]));
  const ancestry: UrdfJoint[] = [];
  for (let link = robot.toolLink!; link !== robot.rootLink;) {
    const joint = parentByChild.get(link);
    if (!joint || ancestry.length >= chain.steps.length) throw new Error('机器人包络缺少完整运动链');
    ancestry.push(joint); link = joint.parent;
  }
  ancestry.reverse();
  const chainIndices = new Map([[robot.rootLink, 0], ...ancestry.map((joint, i): [string, number] => [joint.child, i + 1])]);
  const zero = urdfLinkMatrices(robot, robot.actuatedJoints.map(() => 0));
  const boxes = new Map<number, Box3>();
  for (const link of robot.links) {
    if (!link.visuals.length) continue;
    let ancestor = link.name;
    const visited = new Set<string>();
    while (!chainIndices.has(ancestor)) {
      if (visited.has(ancestor)) throw new Error('机器人包络分支存在环路');
      visited.add(ancestor);
      const joint = parentByChild.get(ancestor);
      if (!joint || joint.type !== 'fixed') throw new Error('机器人包络包含未纳入运动链的可动部件');
      ancestor = joint.parent;
    }
    const stepIndex = chainIndices.get(ancestor)!;
    const parentTransform = zero.get(ancestor), linkTransform = zero.get(link.name);
    if (!parentTransform || !linkTransform) throw new Error('机器人网格与运动链未连接');
    const relative = parentTransform.clone().invert().multiply(linkTransform);
    const combined = boxes.get(stepIndex) ?? new Box3();
    for (const visual of link.visuals) {
      const geometry = prepared.geometries.get(visual.meshPath);
      if (!geometry || !geometry.getAttribute('position')?.count) throw new Error('机器人网格尚未完整加载，无法检查包络');
      if (!geometry.boundingBox) geometry.computeBoundingBox();
      const transform = relative.clone().multiply(urdfOriginMatrix(visual.origin))
        .multiply(new Matrix4().makeScale(...visual.scale));
      const visualBox = geometry.boundingBox!.clone().applyMatrix4(transform);
      if (visualBox.isEmpty() || ![...visualBox.min.toArray(), ...visualBox.max.toArray()].every(Number.isFinite)) throw new Error('机器人网格包络包含无效坐标');
      combined.union(visualBox);
    }
    boxes.set(stepIndex, combined);
  }
  if (!boxes.size) throw new Error('机器人没有可用于避障检查的网格');
  return [...boxes].sort(([a], [b]) => a - b).map(([stepIndex, box]) => {
    const centerMm = box.getCenter(new Vector3()).multiplyScalar(1000).toArray() as Vec3;
    const halfSizeMm = box.getSize(new Vector3()).multiplyScalar(500).toArray().map(size => Math.max(size, 1e-6)) as Vec3;
    if ([...centerMm, ...halfSizeMm].some(value => !Number.isFinite(value) || Math.abs(value) > 1e6)) throw new Error('机器人网格包络超出规划范围');
    return { stepIndex, centerMm, halfSizeMm };
  });
}
