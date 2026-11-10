import * as THREE from 'three';
import { acceleratedRaycast, MeshBVH } from 'three-mesh-bvh';
import type { Vec3 } from '../robot/types';
import type { SceneSurface, SurfacePick } from '../surface/types';

const MM_TO_METRES = .001;

export interface SurfaceVisual {
  root: THREE.Group;
  mesh: THREE.Mesh<THREE.BufferGeometry, THREE.MeshStandardMaterial>;
  source: SceneSurface;
}
export interface SceneSurfaceRegion {
  points: readonly SurfacePick[];
  controlIndices: readonly number[];
  closed?: boolean;
}
export interface SurfaceRegionVisual {
  root: THREE.Group;
  line: THREE.Line<THREE.BufferGeometry, THREE.LineDashedMaterial>;
  points: THREE.Points<THREE.BufferGeometry, THREE.PointsMaterial>;
  controls: THREE.Points<THREE.BufferGeometry, THREE.PointsMaterial>;
  normals: THREE.LineSegments<THREE.BufferGeometry, THREE.LineBasicMaterial>;
}

function geometryFor(surface: SceneSurface): THREE.BufferGeometry {
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(surface.mesh.positions, 3));
  geometry.setAttribute('normal', new THREE.Float32BufferAttribute(surface.mesh.normals, 3));
  geometry.computeBoundingBox();
  geometry.computeBoundingSphere();
  // The indirect tree accelerates pointer raycasts without reordering stored
  // triangles. Provenance refers to original project triangles throughout.
  geometry.boundsTree = new MeshBVH(geometry, { indirect: true });
  return geometry;
}

function applySurface(visual: SurfaceVisual, surface: SceneSurface, selected: boolean): void {
  visual.source = surface;
  visual.root.name = `Imported model ${surface.name}`;
  visual.root.visible = surface.visible;
  visual.root.position.fromArray(surface.pose.position).multiplyScalar(MM_TO_METRES);
  const [w, x, y, z] = surface.pose.orientation;
  visual.root.quaternion.set(x, y, z, w).normalize();
  visual.mesh.material.color.setHex(selected ? 0x7cabc0 : 0x9eafb9);
  visual.mesh.material.emissive.setHex(selected ? 0x10212c : 0x000000);
  visual.mesh.material.emissiveIntensity = .35;
  visual.root.updateMatrixWorld(true);
}

export function createSurfaceVisual(surface: SceneSurface, selected = false): SurfaceVisual {
  const root = new THREE.Group();
  const mesh = new THREE.Mesh(geometryFor(surface), new THREE.MeshStandardMaterial({
    color: 0x9eafb9, roughness: .68, metalness: .12, side: THREE.DoubleSide,
    // Coplanar region points are rendered independently; no surface geometry
    // or stored position is offset to improve display.
    polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 1,
  }));
  mesh.name = 'Imported model triangles';
  mesh.scale.setScalar(MM_TO_METRES);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  mesh.raycast = acceleratedRaycast;
  root.add(mesh);
  const visual = { root, mesh, source: surface };
  applySurface(visual, surface, selected);
  return visual;
}

export function updateSurfaceVisual(visual: SurfaceVisual, surface: SceneSurface, selected = false): void {
  if (visual.source.mesh !== surface.mesh) {
    visual.mesh.geometry.boundsTree = undefined;
    visual.mesh.geometry.dispose();
    visual.mesh.geometry = geometryFor(surface);
  }
  applySurface(visual, surface, selected);
}

export function disposeSurfaceVisual(visual: SurfaceVisual): void {
  visual.root.removeFromParent();
  visual.mesh.geometry.boundsTree = undefined;
  visual.mesh.geometry.dispose();
  visual.mesh.material.dispose();
}

/** Reconstruct a pick on the original double-precision triangle, rather than
 * recording the rounded Float32 render vertex/fragment coordinates. */
export function surfacePickFromRay(visual: SurfaceVisual, ray: THREE.Ray, triangleIndex: number): SurfacePick | null {
  const { source, mesh } = visual;
  if (!source.visible || !visual.root.visible || !Number.isSafeInteger(triangleIndex)
    || triangleIndex < 0 || triangleIndex * 9 + 8 >= source.mesh.positions.length) return null;
  mesh.updateWorldMatrix(true, false);
  const localRay = ray.clone().applyMatrix4(mesh.matrixWorld.clone().invert());
  const offset = triangleIndex * 9;
  const a = new THREE.Vector3().fromArray(source.mesh.positions, offset);
  const b = new THREE.Vector3().fromArray(source.mesh.positions, offset + 3);
  const c = new THREE.Vector3().fromArray(source.mesh.positions, offset + 6);
  const localPoint = localRay.intersectTriangle(a, b, c, false, new THREE.Vector3());
  if (!localPoint) return null;
  const barycentric = THREE.Triangle.getBarycoord(localPoint, a, b, c, new THREE.Vector3());
  if (!barycentric || !barycentric.toArray().every(Number.isFinite)) return null;
  const normal = new THREE.Vector3().fromArray(source.mesh.normals, offset).multiplyScalar(barycentric.x)
    .addScaledVector(new THREE.Vector3().fromArray(source.mesh.normals, offset + 3), barycentric.y)
    .addScaledVector(new THREE.Vector3().fromArray(source.mesh.normals, offset + 6), barycentric.z);
  if (normal.lengthSq() < 1e-16) return null;
  normal.applyNormalMatrix(new THREE.Matrix3().getNormalMatrix(mesh.matrixWorld));
  if (!normal.toArray().every(Number.isFinite) || normal.lengthSq() < .99) return null;
  // Do not use Intersection.normal: Three flips it towards the camera for
  // back-face hits. A machining point must keep the model's original normal.
  const point = a.multiplyScalar(barycentric.x).addScaledVector(b, barycentric.y).addScaledVector(c, barycentric.z)
    .applyMatrix4(mesh.matrixWorld).multiplyScalar(1 / MM_TO_METRES);
  return {
    surfaceId: source.id, surfaceRevision: source.revision, triangleIndex,
    barycentric: barycentric.toArray() as Vec3,
    position: point.toArray() as Vec3, normal: normal.toArray() as Vec3,
  };
}

/** The nearest visible imported surface wins; restricting a region never picks
 * through another imported model lying in front of the selected one. */
export function pickSurface(raycaster: THREE.Raycaster, visuals: readonly SurfaceVisual[], requiredId?: string | null): SurfacePick | null {
  const visible = visuals.filter(visual => visual.source.visible && visual.root.visible);
  for (const visual of visible) visual.mesh.updateWorldMatrix(true, false);
  const byObject = new Map(visible.map(visual => [visual.mesh, visual]));
  const previous = raycaster.firstHitOnly;
  raycaster.firstHitOnly = true;
  let hit: THREE.Intersection | undefined;
  try { hit = raycaster.intersectObjects(visible.map(visual => visual.mesh), false)[0]; }
  finally { raycaster.firstHitOnly = previous; }
  if (!hit || hit.faceIndex === undefined || hit.faceIndex === null) return null;
  const visual = byObject.get(hit.object as SurfaceVisual['mesh']);
  if (!visual || requiredId && visual.source.id !== requiredId) return null;
  return surfacePickFromRay(visual, raycaster.ray, hit.faceIndex);
}

export function createSurfaceRegionVisual(): SurfaceRegionVisual {
  const root = new THREE.Group();
  root.name = 'Selected surface region';
  const line = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineDashedMaterial({
    color: 0x3b917b, dashSize: .009, gapSize: .005, transparent: true, opacity: .9, depthWrite: false,
  }));
  const points = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial({
    color: 0x29977b, size: 9, sizeAttenuation: false, depthTest: false, depthWrite: false,
  }));
  const controls = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial({
    color: 0xecaa36, size: 14, sizeAttenuation: false, depthTest: false, depthWrite: false,
  }));
  const normals = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({
    color: 0x6d9e7f, transparent: true, opacity: .8, depthTest: false, depthWrite: false,
  }));
  line.renderOrder = 7; normals.renderOrder = 8; points.renderOrder = 9; controls.renderOrder = 10;
  root.add(line, normals, points, controls);
  root.visible = false;
  return { root, line, points, controls, normals };
}

/** Update region markers only from non-stale surface anchors. */
export function updateSurfaceRegionVisual(visual: SurfaceRegionVisual, region: SceneSurfaceRegion | null | undefined,
  surfaces: readonly SceneSurface[]): void {
  const points = region?.points ?? [];
  const current = points.every(point => surfaces.some(surface => surface.id === point.surfaceId
    && surface.revision === point.surfaceRevision && surface.visible));
  visual.root.visible = current && points.length > 0;
  const positions = current ? points.map(point => new THREE.Vector3(...point.position).multiplyScalar(MM_TO_METRES)) : [];
  const boundary = positions.slice();
  if (region?.closed && boundary.length > 2) boundary.push(boundary[0]);
  const controlIndices = new Set(region?.controlIndices ?? []);
  const normalEnds = current ? points.flatMap((point, index) => [positions[index], positions[index].clone().addScaledVector(new THREE.Vector3(...point.normal), .025)]) : [];
  for (const [drawable, vertices] of [[visual.line, boundary], [visual.points, positions],
    [visual.controls, positions.filter((_, index) => controlIndices.has(index))], [visual.normals, normalEnds]] as const) {
    drawable.geometry.dispose();
    drawable.geometry = new THREE.BufferGeometry().setFromPoints(vertices);
  }
  visual.line.computeLineDistances();
  visual.line.visible = boundary.length > 1;
}

/** Read-only screen anchors for pointer tests. Anchors remain inside actual
 * triangles and are checked by the same raycaster used for user clicks. */
export function surfaceDiagnostics(visuals: readonly SurfaceVisual[], camera: THREE.Camera, rect: { left: number; top: number; width: number; height: number }) {
  const raycaster = new THREE.Raycaster();
  return visuals.map(visual => {
    visual.mesh.updateWorldMatrix(true, false);
    const bounds = new THREE.Box3().setFromObject(visual.root);
    const projected: { triangleIndex: number; barycentric: Vec3; position: Vec3; normal: Vec3; x: number; y: number }[] = [];
    if (visual.root.visible) {
      const count = visual.source.mesh.positions.length / 9;
      const stride = Math.max(1, Math.ceil(count / 12));
      const weights: Vec3[] = [[1 / 3, 1 / 3, 1 / 3], [.7, .15, .15], [.15, .7, .15], [.15, .15, .7]];
      for (let triangle = 0; triangle < count && projected.length < 24; triangle += stride) {
        for (const weight of weights) {
          const point = new THREE.Vector3();
          for (let vertex = 0; vertex < 3; vertex++) point.addScaledVector(new THREE.Vector3().fromArray(visual.source.mesh.positions, triangle * 9 + vertex * 3), weight[vertex]);
          const ndc = point.applyMatrix4(visual.mesh.matrixWorld).project(camera);
          if (Math.abs(ndc.x) > .98 || Math.abs(ndc.y) > .98 || ndc.z < -1 || ndc.z > 1) continue;
          raycaster.setFromCamera(new THREE.Vector2(ndc.x, ndc.y), camera);
          const hit = pickSurface(raycaster, visuals, visual.source.id);
          if (!hit || hit.triangleIndex !== triangle) continue;
          projected.push({ triangleIndex: triangle, barycentric: hit.barycentric, position: hit.position, normal: hit.normal,
            x: rect.left + (ndc.x + 1) * rect.width / 2, y: rect.top + (1 - ndc.y) * rect.height / 2 });
        }
      }
    }
    return { id: visual.source.id, revision: visual.source.revision, visible: visual.root.visible,
      triangles: visual.source.mesh.positions.length / 9,
      minMm: bounds.min.toArray().map(n => n / MM_TO_METRES), maxMm: bounds.max.toArray().map(n => n / MM_TO_METRES), projected };
  });
}
