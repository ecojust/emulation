import { describe, expect, it } from 'vitest';
import { BoxGeometry, Vector3 } from 'three';
import { buildGenericCollisionBodies } from '../src/scene/robot-collision-bounds';
import { urdfLinkMatrices, urdfOriginMatrix } from '../src/catalog/kinematics';
import type { PreparedUrdfModel, UrdfJoint, UrdfVisual } from '../src/catalog/urdf-types';

const origin = () => ({ xyz: [0, 0, 0] as [number, number, number], rpy: [0, 0, 0] as [number, number, number] });
function fixture(): PreparedUrdfModel {
  const joints: UrdfJoint[] = Array.from({ length: 6 }, (_, i) => ({ name: `j${i + 1}`, type: 'revolute', parent: i ? `l${i}` : 'base', child: `l${i + 1}`,
    origin: { xyz: [.01 * (i + 1), -.02, .1], rpy: [.12, -.16, .09 * i] }, axis: [0, 0, 1], minDeg: -180, maxDeg: 180, maxSpeedDeg: 90 }));
  const branch: UrdfJoint = { ...joints[0], name: 'side', type: 'fixed', parent: 'l2', child: 'branch', origin: { xyz: [.2, .05, -.01], rpy: [.3, .4, .5] } };
  const visual: UrdfVisual = { meshPath: 'box.stl', origin: { xyz: [.04, -.03, .02], rpy: [.2, .1, .4] }, scale: [2, .5, 1.2] };
  const links = ['base', 'l1', 'l2', 'l3', 'l4', 'l5', 'l6', 'branch'].map(name => ({ name, visuals: [{ ...visual }] }));
  return { id: 'test-six-axis', name: 'test', geometries: new Map([['box.stl', new BoxGeometry(.06, .08, .1)]]), description: {
    name: 'test', rootLink: 'base', toolLink: 'l6', links, joints: [...joints, branch], actuatedJoints: joints, homeJoints: [0, 0, 0, 0, 0, 0],
  } };
}

describe('real visual collision enclosures', () => {
  it('encloses every transformed visual vertex, including a rotated fixed branch, at different joint poses', () => {
    const model = fixture(), bodies = buildGenericCollisionBodies(model);
    expect(bodies).toHaveLength(7);
    for (const angles of [[0, 0, 0, 0, 0, 0], [35, -40, 75, 90, -25, 13], [-170, 20, 60, -35, 110, -15]]) {
      const transforms = urdfLinkMatrices(model.description, angles);
      for (const link of model.description.links) {
        const ancestor = link.name === 'branch' ? 'l2' : link.name;
        const stepIndex = ancestor === 'base' ? 0 : Number(ancestor.slice(1));
        const body = bodies.find(item => item.stepIndex === stepIndex)!;
        for (const visual of link.visuals) {
          const toWorld = transforms.get(link.name)!.clone().multiply(urdfOriginMatrix(visual.origin)).scale(new Vector3(...visual.scale));
          const positions = model.geometries.get(visual.meshPath)!.getAttribute('position');
          for (let i = 0; i < positions.count; i++) {
            // Evaluate a world visual vertex independently, then inspect the planner's moving enclosure.
            const local = new Vector3().fromBufferAttribute(positions, i).applyMatrix4(toWorld)
              .applyMatrix4(transforms.get(ancestor)!.clone().invert()).multiplyScalar(1000).toArray();
            for (let axis = 0; axis < 3; axis++) expect(Math.abs(local[axis] - body.centerMm[axis])).toBeLessThanOrEqual(body.halfSizeMm[axis] + 1e-6);
          }
        }
      }
    }
  });

  it('rejects missing geometry instead of certifying an incomplete model', () => {
    const model = fixture(); model.geometries = new Map();
    expect(() => buildGenericCollisionBodies(model)).toThrow(/网格尚未完整加载/);
  });

  it('includes root geometry and refuses unsupported movable side branches', () => {
    const model = fixture();
    const root = buildGenericCollisionBodies(model).find(body => body.stepIndex === 0)!;
    expect(root.halfSizeMm.every(size => size > 0)).toBe(true);
    model.description.joints.at(-1)!.type = 'revolute';
    expect(() => buildGenericCollisionBodies(model)).toThrow(/未纳入运动链的可动部件/);
  });

  it('does not mutate mesh vertices while computing conservative boxes', () => {
    const model = fixture(), geometry = model.geometries.get('box.stl')!;
    const before = [...geometry.getAttribute('position').array];
    buildGenericCollisionBodies(model);
    expect([...geometry.getAttribute('position').array]).toEqual(before);
  });
});
