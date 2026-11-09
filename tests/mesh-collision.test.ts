import { BoxGeometry, BufferGeometry, Float32BufferAttribute } from 'three';
import { describe, expect, it } from 'vitest';
import { buildCollisionMesh, inspectMeshZone } from '../src/robot/collision-mesh';
import type { DangerZone } from '../src/robot/collision-geometry';
import type { Mat4, Vec3 } from '../src/robot/types';

const identity: Mat4 = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
const zone = (centerMm: Vec3, sizeMm: Vec3): DangerZone => ({ enabled: true, centerMm, sizeMm });
const tetrahedron = [
  0, 0, 0, 0, 2, 0, 2, 0, 0,
  0, 0, 0, 2, 0, 0, 0, 0, 2,
  0, 0, 0, 0, 0, 2, 0, 2, 0,
  2, 0, 0, 0, 2, 0, 0, 0, 2,
];
function geometry(vertices: readonly number[]) {
  return new BufferGeometry().setAttribute('position', new Float32BufferAttribute(vertices, 3));
}
function reversed(vertices: readonly number[]) {
  const output: number[] = [];
  for (let i = 0; i < vertices.length; i += 9) output.push(...vertices.slice(i, i + 3), ...vertices.slice(i + 6, i + 9), ...vertices.slice(i + 3, i + 6));
  return output;
}

describe('actual triangle surfaces and closed-mesh volume', () => {
  it('rejects empty space inside an enclosing OBB and detects complete containment in either direction', () => {
    const mesh = buildCollisionMesh(geometry(tetrahedron));
    // This box is within [0,2]^3, but every corner has x+y+z>2.
    expect(inspectMeshZone(mesh, identity, zone([1.5, 1.5, 1.5], [.1, .1, .1]))).toBe('clear');
    // Every box corner has x,y,z>0 and x+y+z<2: no triangle crosses the box.
    expect(inspectMeshZone(mesh, identity, zone([.2, .2, .2], [.05, .05, .05]))).toBe('hit');
    expect(inspectMeshZone(mesh, identity, zone([.5, .5, .5], [5, 5, 5]))).toBe('hit');
  });

  it('counts face, edge and vertex contact and penetration while preserving a real gap', () => {
    const mesh = buildCollisionMesh(geometry(tetrahedron));
    expect(inspectMeshZone(mesh, identity, zone([-.1, .3, .3], [.2, .1, .1]))).toBe('hit');
    expect(inspectMeshZone(mesh, identity, zone([-.101, .3, .3], [.2, .1, .1]))).toBe('clear');
    expect(inspectMeshZone(mesh, identity, zone([-.09, .3, .3], [.2, .1, .1]))).toBe('hit');
    expect(inspectMeshZone(mesh, identity, zone([-.1, -.1, .5], [.2, .2, .2]))).toBe('hit');
    expect(inspectMeshZone(mesh, identity, zone([-.1, -.1, -.1], [.2, .2, .2]))).toBe('hit');
  });

  it('detects a triangle crossing the box even when all its vertices are outside', () => {
    const mesh = buildCollisionMesh(geometry([-2, -2, 0, 2, -2, 0, 0, 2, 0]));
    expect(inspectMeshZone(mesh, identity, zone([0, 0, 0], [.2, .2, .2]))).toBe('hit');
    expect(inspectMeshZone(mesh, identity, zone([0, 0, 1], [.2, .2, .2]))).toBe('clear');
  });

  it('does not infer a closed solid through a missing face or a caller-supplied closed flag', () => {
    const mesh = buildCollisionMesh(geometry(tetrahedron.slice(9)), true);
    expect(inspectMeshZone(mesh, identity, zone([.2, .2, .2], [.05, .05, .05]))).toBe('uncertain');
    expect(inspectMeshZone(mesh, identity, zone([-.09, .3, .3], [.2, .1, .1]))).toBe('hit');
    expect(inspectMeshZone(mesh, identity, zone([3, 3, 3], [.1, .1, .1]))).toBe('clear');
  });

  it('handles disconnected closed components independently of their opposite winding', () => {
    const other = reversed(tetrahedron).map((value, index) => value + (index % 3 === 0 ? 4 : 0));
    const mesh = buildCollisionMesh(geometry([...tetrahedron, ...other]));
    expect(inspectMeshZone(mesh, identity, zone([.2, .2, .2], [.05, .05, .05]))).toBe('hit');
    expect(inspectMeshZone(mesh, identity, zone([4.2, .2, .2], [.05, .05, .05]))).toBe('hit');
    expect(inspectMeshZone(mesh, identity, zone([3, .2, .2], [.05, .05, .05]))).toBe('clear');
  });

  it('preserves an enclosed cavity instead of treating every closed component as filled', () => {
    const outer = new BoxGeometry(4, 4, 4).toNonIndexed();
    const inner = new BoxGeometry(2, 2, 2).toNonIndexed();
    const mesh = buildCollisionMesh(geometry([...outer.getAttribute('position').array, ...reversed([...inner.getAttribute('position').array])]));
    expect(inspectMeshZone(mesh, identity, zone([0, 0, 0], [.1, .1, .1]))).toBe('clear');
    expect(inspectMeshZone(mesh, identity, zone([1.5, 0, 0], [.1, .1, .1]))).toBe('hit');
    outer.dispose(); inner.dispose();
  });

  it('applies the complete rigid transform while keeping the obstacle world-aligned', () => {
    const mesh = buildCollisionMesh(geometry(tetrahedron));
    const transform: Mat4 = [0, -1, 0, 10, 1, 0, 0, 20, 0, 0, 1, 30, 0, 0, 0, 1];
    expect(inspectMeshZone(mesh, transform, zone([9.8, 20.2, 30.2], [.05, .05, .05]))).toBe('hit');
    expect(inspectMeshZone(mesh, transform, zone([8.5, 21.5, 31.5], [.1, .1, .1]))).toBe('clear');
    expect(inspectMeshZone(mesh, transform, zone([.2, .2, .2], [.05, .05, .05]))).toBe('clear');
  });
});
