import { describe, expect, test } from 'vitest';
import * as THREE from 'three';
import { createSurfaceVisual, disposeSurfaceVisual, pickSurface, surfacePickFromRay,
  createSurfaceRegionVisual, updateSurfaceRegionVisual, updateSurfaceVisual } from '../src/scene/surface-visuals';
import type { SceneSurface, SurfacePick } from '../src/surface/types';

const mesh = { positions: [0, 0, 0, 100, 0, 0, 0, 100, 0], normals: [0, 0, 1, 0, 0, 1, 0, 0, 1] };
function surface(patch: Partial<SceneSurface> = {}): SceneSurface {
  return { id: 'part', name: 'Part', revision: 1, visible: true,
    pose: { position: [0, 0, 0], orientation: [1, 0, 0, 0] }, source: { name: 'part.stl', format: 'stl', unit: 'mm' }, mesh, ...patch };
}
function downward(x: number, y: number, z = 1000) {
  return new THREE.Raycaster(new THREE.Vector3(x, y, z).multiplyScalar(.001), new THREE.Vector3(0, 0, -1));
}
function close(actual: readonly number[], expected: readonly number[], precision = 9) {
  expect(actual).toHaveLength(expected.length);
  actual.forEach((value, index) => expect(value).toBeCloseTo(expected[index], precision));
}
function anchored(point: SurfacePick, source: SceneSurface) {
  const local = new THREE.Vector3();
  for (let i = 0; i < 3; i++) local.addScaledVector(new THREE.Vector3().fromArray(source.mesh.positions, point.triangleIndex * 9 + i * 3), point.barycentric[i]);
  const [w, x, y, z] = source.pose.orientation;
  local.applyQuaternion(new THREE.Quaternion(x, y, z, w)).add(new THREE.Vector3(...source.pose.position));
  close(point.position, local.toArray());
  expect(point.barycentric.reduce((a, b) => a + b, 0)).toBeCloseTo(1, 12);
}

describe('imported surface rendering and pointer geometry', () => {
  test('records original double precision surface coordinates and outward normal from either side', () => {
    const source = surface({ mesh: { positions: [0, 0, .123456789123, 100, 0, .123456789123, 0, 100, .123456789123], normals: mesh.normals } });
    const visual = createSurfaceVisual(source);
    const front = pickSurface(downward(20, 30), [visual])!;
    expect(front.surfaceId).toBe('part'); expect(front.surfaceRevision).toBe(1); expect(front.triangleIndex).toBe(0);
    close(front.position, [20, 30, .123456789123], 11); close(front.normal, [0, 0, 1]); anchored(front, source);
    const back = pickSurface(new THREE.Raycaster(new THREE.Vector3(.02, .03, -.1), new THREE.Vector3(0, 0, 1)), [visual])!;
    close(back.normal, [0, 0, 1]); anchored(back, source);
    disposeSurfaceVisual(visual);
  });

  test('transforms positions and interpolated normals through the world pose', () => {
    const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI / 2);
    const source = surface({ pose: { position: [200, 300, 400], orientation: [q.w, q.x, q.y, q.z] } });
    const visual = createSurfaceVisual(source);
    const ray = new THREE.Raycaster(new THREE.Vector3(.22, -.7, .43), new THREE.Vector3(0, 1, 0));
    const result = pickSurface(ray, [visual])!;
    close(result.position, [220, 300, 430]); close(result.normal, [0, -1, 0]); anchored(result, source);
    disposeSurfaceVisual(visual);
  });

  test('BVH indirect mode retains original stored triangle indices for shuffled faces', () => {
    const positions: number[] = [], normals: number[] = [];
    const sequence = [7, 1, 8, 3, 9, 2, 6, 0, 5, 4];
    for (const n of sequence) {
      positions.push(n * 200, 0, 0, n * 200 + 100, 0, 0, n * 200, 100, 0); normals.push(...mesh.normals);
    }
    const source = surface({ mesh: { positions, normals } }), visual = createSurfaceVisual(source);
    for (let index = 0; index < sequence.length; index++) {
      const result = pickSurface(downward(sequence[index] * 200 + 25, 25), [visual])!;
      expect(result.triangleIndex).toBe(index); anchored(result, source);
    }
    expect(source.mesh.positions).toEqual(positions);
    disposeSurfaceVisual(visual);
  });

  test('does not pick hidden surfaces or through another visible workpiece when region is restricted', () => {
    const lower = createSurfaceVisual(surface({ id: 'lower' }));
    const upper = createSurfaceVisual(surface({ id: 'upper', pose: { position: [0, 0, 100], orientation: [1, 0, 0, 0] } }));
    const ray = downward(20, 30);
    expect(pickSurface(ray, [lower, upper])?.surfaceId).toBe('upper');
    expect(pickSurface(ray, [lower, upper], 'lower')).toBeNull();
    updateSurfaceVisual(upper, { ...upper.source, visible: false });
    expect(pickSurface(ray, [lower, upper], 'lower')?.surfaceId).toBe('lower');
    expect(surfacePickFromRay(upper, ray.ray, 0)).toBeNull();
    expect(surfacePickFromRay(lower, ray.ray, 99)).toBeNull();
    disposeSurfaceVisual(lower); disposeSurfaceVisual(upper);
  });

  test('pose edits reuse geometry, geometry edits dispose and replace it, deletion releases geometry and material', () => {
    const source = surface(), visual = createSurfaceVisual(source), geometry = visual.mesh.geometry;
    let disposedGeometry = 0, disposedMaterial = 0;
    geometry.addEventListener('dispose', () => { disposedGeometry++; });
    visual.mesh.material.addEventListener('dispose', () => { disposedMaterial++; });
    updateSurfaceVisual(visual, { ...source, revision: 2, pose: { ...source.pose, position: [0, 0, 200] } }, true);
    expect(visual.mesh.geometry).toBe(geometry); expect(disposedGeometry).toBe(0);
    expect(pickSurface(downward(20, 30), [visual])?.surfaceRevision).toBe(2);
    updateSurfaceVisual(visual, { ...source, revision: 3, mesh: { positions: [...mesh.positions], normals: [...mesh.normals] } });
    expect(visual.mesh.geometry).not.toBe(geometry); expect(disposedGeometry).toBe(1);
    disposeSurfaceVisual(visual); expect(disposedMaterial).toBe(1);
  });

  test('region overlays retain exact point coordinates and hide stale or hidden anchors', () => {
    const source = surface(), model = createSurfaceVisual(source);
    const points = [pickSurface(downward(10, 10), [model])!, pickSurface(downward(60, 10), [model])!, pickSurface(downward(10, 60), [model])!];
    const overlay = createSurfaceRegionVisual();
    updateSurfaceRegionVisual(overlay, { points, controlIndices: [0, 2], closed: true }, [source]);
    expect(overlay.root.visible).toBe(true);
    expect(overlay.line.geometry.getAttribute('position').count).toBe(4);
    expect(overlay.controls.geometry.getAttribute('position').count).toBe(2);
    expect(overlay.normals.geometry.getAttribute('position').count).toBe(6);
    expect(overlay.points.geometry.getAttribute('position').getZ(0)).toBe(0);
    updateSurfaceRegionVisual(overlay, { points, controlIndices: [] }, [{ ...source, revision: 2 }]);
    expect(overlay.root.visible).toBe(false);
    expect(overlay.points.geometry.getAttribute('position').count).toBe(0);
    disposeSurfaceVisual(model);
    for (const object of [overlay.line, overlay.points, overlay.controls, overlay.normals]) { object.geometry.dispose(); object.material.dispose(); }
  });
});
