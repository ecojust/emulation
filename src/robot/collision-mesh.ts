import { Box3, BufferGeometry, Matrix4, Triangle, Vector3 } from 'three';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { MeshBVH, SAH } from 'three-mesh-bvh';
import source from '../../vendor/work-r-labs/SOURCE.json';
import type { DangerZone } from './collision-geometry';
import type { Mat4, RobotId } from './types';

export const CONTACT_EPSILON_MM = 1e-6;
const MAX_TRIANGLES = 500_000;
const VERTEX_KEY_STRIDE = MAX_TRIANGLES * 3 + 1;
const RAY_DIRECTIONS = [
  new Vector3(1, 0.3713906763541037, 0.7135814171270493).normalize(),
  new Vector3(-0.519697, 1, 0.291731).normalize(),
  new Vector3(0.213127, -0.638273, 1).normalize(),
];
export type MeshContact = 'hit' | 'clear' | 'uncertain';
export interface CollisionMesh {
  readonly geometry: BufferGeometry;
  readonly bvh: MeshBVH;
  /** Exact shared vertices, every edge used twice in opposite directions, no degenerate facets. */
  readonly closed: boolean;
}
export interface CollisionMeshModel { readonly modelId: RobotId; readonly links: readonly CollisionMesh[] }

/** No welding or hole repair: these checks describe the supplied triangles as they are. */
function hasClosedOrientedTopology(geometry: BufferGeometry): boolean {
  const position = geometry.getAttribute('position');
  const indices = geometry.index;
  const count = indices?.count ?? position?.count ?? 0;
  if (!position || position.itemSize !== 3 || count === 0 || count % 3 !== 0 || count / 3 > MAX_TRIANGLES) throw new Error('机器人三角网格数量或顶点格式无效');
  const vertices = new Map<string, number>();
  const edges = new Map<number, { count: number; balance: number }>();
  const triangle = new Triangle(), edge1 = new Vector3(), edge2 = new Vector3();
  const ids = [0, 0, 0], points = [triangle.a, triangle.b, triangle.c];
  let closed = true;
  for (let offset = 0; offset < count; offset += 3) {
    for (let vertex = 0; vertex < 3; vertex++) {
      const index = indices ? indices.getX(offset + vertex) : offset + vertex;
      if (!Number.isInteger(index) || index < 0 || index >= position.count) throw new Error('机器人三角网格的顶点索引无效');
      const point = points[vertex].fromBufferAttribute(position, index);
      if (![point.x, point.y, point.z].every(value => Number.isFinite(value) && Math.abs(value) <= 1e6)) throw new Error('机器人三角网格包含无效顶点');
      // JavaScript serializes both signs of zero to "0"; Float32 coordinates retain exact values.
      const key = `${point.x},${point.y},${point.z}`;
      let id = vertices.get(key);
      if (id === undefined) { id = vertices.size; vertices.set(key, id); }
      ids[vertex] = id;
    }
    const area = edge1.subVectors(triangle.b, triangle.a).cross(edge2.subVectors(triangle.c, triangle.a)).length();
    if (area <= 1e-12) { closed = false; continue; }
    for (let edge = 0; edge < 3; edge++) {
      const a = ids[edge], b = ids[(edge + 1) % 3];
      const key = Math.min(a, b) * VERTEX_KEY_STRIDE + Math.max(a, b);
      const previous = edges.get(key);
      if (previous) { previous.count++; previous.balance += a < b ? 1 : -1; }
      else edges.set(key, { count: 1, balance: a < b ? 1 : -1 });
    }
  }
  for (const edge of edges.values()) if (edge.count !== 2 || edge.balance !== 0) return false;
  return closed;
}

/** Millimetre geometry. `closed=false` can require uncertainty; true never bypasses topology checks. */
export function buildCollisionMesh(geometry: BufferGeometry, closed?: boolean): CollisionMesh {
  const copy = geometry.clone();
  copy.clearGroups();
  copy.setDrawRange(0, Infinity);
  const verifiedClosed = hasClosedOrientedTopology(copy);
  copy.computeBoundingBox();
  return { geometry: copy, closed: verifiedClosed && closed !== false, bvh: new MeshBVH(copy, { strategy: SAH, targetLeafSize: 8 }) };
}

/** Trusted-byte factory for tests and the verified bundled-file loader; files are base, link1…6. */
export function buildCollisionMeshModel(modelId: RobotId, files: ArrayBuffer[]): CollisionMeshModel {
  if (!['irb1100-4-0.475', 'irb1100-4-0.58'].includes(modelId) || files.length !== 7) throw new Error('机器人三角网格型号或部件数量无效');
  const loader = new STLLoader();
  const links = files.map(bytes => {
    if (!(bytes instanceof ArrayBuffer) || bytes.byteLength < 84) throw new Error('机器人 STL 文件不完整');
    const count = new DataView(bytes).getUint32(80, true);
    if (!count || count > MAX_TRIANGLES || bytes.byteLength !== 84 + count * 50) throw new Error('机器人 STL 三角数量或文件长度无效');
    const geometry = loader.parse(bytes);
    try { return buildCollisionMesh(geometry); } finally { geometry.dispose(); }
  });
  return { modelId, links };
}

const cachedModels = new Map<RobotId, Promise<CollisionMeshModel>>();
/** A persistent browser worker reuses these verified assets and BVHs across successful checks. */
export function loadCollisionModel(modelId: RobotId): Promise<CollisionMeshModel> {
  let promise = cachedModels.get(modelId);
  if (!promise) {
    promise = (async () => {
      const files: ArrayBuffer[] = [];
      for (let link = 0; link < 7; link++) {
        const name = link ? `link_${link}` : 'base_link';
        const logicalPath = `public/models/${modelId}/${name}.stl`;
        const pinned = source.files.find(file => file.modelId === modelId && file.localPath === logicalPath);
        if (!pinned) throw new Error('该机器人尚无经过核对的三角网格');
        const response = await fetch(`${import.meta.env.BASE_URL}models/${modelId}/${name}.stl`);
        if (!response.ok) throw new Error(`机器人网格读取失败：${name}`);
        if (!response.body) throw new Error(`机器人网格未返回文件：${name}`);
        const reader = response.body.getReader(), data = new Uint8Array(pinned.bytes);
        let received = 0;
        try {
          for (;;) {
            const { value, done } = await reader.read();
            if (done) break;
            if (value.byteLength > pinned.bytes - received) throw new Error(`机器人网格文件超出规定大小：${name}`);
            data.set(value, received); received += value.byteLength;
          }
          if (received !== pinned.bytes) throw new Error(`机器人网格文件长度不符：${name}`);
        } finally { await reader.cancel().catch(() => {}); reader.releaseLock(); }
        const bytes = data.buffer;
        const digest = Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256', bytes)), byte => byte.toString(16).padStart(2, '0')).join('');
        if (digest !== pinned.sha256) throw new Error(`机器人网格文件校验失败：${name}`);
        files.push(bytes);
      }
      return buildCollisionMeshModel(modelId, files);
    })();
    cachedModels.set(modelId, promise);
    void promise.catch(() => { if (cachedModels.get(modelId) === promise) cachedModels.delete(modelId); });
  }
  return promise;
}

function rigidInverse(frame: Mat4): Matrix4 {
  return new Matrix4().set(
    frame[0], frame[4], frame[8], -(frame[0] * frame[3] + frame[4] * frame[7] + frame[8] * frame[11]),
    frame[1], frame[5], frame[9], -(frame[1] * frame[3] + frame[5] * frame[7] + frame[9] * frame[11]),
    frame[2], frame[6], frame[10], -(frame[2] * frame[3] + frame[6] * frame[7] + frame[10] * frame[11]),
    0, 0, 0, 1,
  );
}

/** Matches the Rust BVH ray bounds, including its 1e-6 mm contact expansion. */
function rayIntersectsBounds(bounds: Box3, origin: Vector3, direction: Vector3): boolean {
  let near = 0, far = Infinity;
  for (let axis = 0; axis < 3; axis++) {
    const component = direction.getComponent(axis), coordinate = origin.getComponent(axis);
    if (Math.abs(component) < 1e-15) {
      if (coordinate < bounds.min.getComponent(axis) - CONTACT_EPSILON_MM || coordinate > bounds.max.getComponent(axis) + CONTACT_EPSILON_MM) return false;
    } else {
      const a = (bounds.min.getComponent(axis) - CONTACT_EPSILON_MM - coordinate) / component;
      const b = (bounds.max.getComponent(axis) + CONTACT_EPSILON_MM - coordinate) / component;
      near = Math.max(near, Math.min(a, b)); far = Math.min(far, Math.max(a, b));
      if (near > far) return false;
    }
  }
  return true;
}

/** Row-major rigid placement in mm; detects actual triangle contact before solid containment. */
export function inspectMeshZone(mesh: CollisionMesh, localToWorld: Mat4, zone: DangerZone): MeshContact {
  if (!zone.enabled) return 'clear';
  const world = new Matrix4().set(...localToWorld as Parameters<Matrix4['set']>);
  const inverse = rigidInverse(localToWorld);
  const center = new Vector3(...zone.centerMm);
  const worldBox = new Box3().setFromCenterAndSize(center, new Vector3(...zone.sizeMm)).expandByScalar(CONTACT_EPSILON_MM);
  const localBounds = worldBox.clone().applyMatrix4(inverse).expandByScalar(CONTACT_EPSILON_MM);
  if (!mesh.geometry.boundingBox!.intersectsBox(localBounds)) return 'clear';
  const triangle = new Triangle(), edge1 = new Vector3(), edge2 = new Vector3();
  const surface = mesh.bvh.shapecast({
    intersectsBounds: bounds => bounds.intersectsBox(localBounds),
    intersectsTriangle: local => {
      if (edge1.subVectors(local.b, local.a).cross(edge2.subVectors(local.c, local.a)).length() <= 1e-12) return false;
      triangle.a.copy(local.a).applyMatrix4(world);
      triangle.b.copy(local.b).applyMatrix4(world);
      triangle.c.copy(local.c).applyMatrix4(world);
      return worldBox.intersectsTriangle(triangle);
    },
  });
  if (surface) return 'hit';
  if (!mesh.closed) return 'uncertain';

  // One convex box with no surface contact has a constant interior classification.
  // Count the entire mesh (including reverse-oriented cavity shells), not component unions.
  const origin = center.applyMatrix4(inverse);
  const triangleBounds = new Box3(), normal = new Vector3(), p = new Vector3(), q = new Vector3(), displacement = new Vector3();
  let classification: boolean | undefined;
  let confirmations = 0;
  for (const direction of RAY_DIRECTIONS) {
    let crossings = 0, ambiguous = false;
    mesh.bvh.shapecast({
      intersectsBounds: bounds => rayIntersectsBounds(bounds, origin, direction),
      intersectsTriangle: local => {
        triangleBounds.makeEmpty().expandByPoint(local.a).expandByPoint(local.b).expandByPoint(local.c);
        if (!rayIntersectsBounds(triangleBounds, origin, direction)) return false;
        edge1.subVectors(local.b, local.a); edge2.subVectors(local.c, local.a);
        normal.crossVectors(edge1, edge2);
        const area = normal.length(), determinant = edge1.dot(p.crossVectors(direction, edge2));
        displacement.subVectors(origin, local.a);
        if (Math.abs(determinant) <= area * 1e-12) {
          ambiguous = Math.abs(normal.dot(displacement)) <= area * CONTACT_EPSILON_MM;
          return ambiguous;
        }
        const u = displacement.dot(p) / determinant;
        if (u < -1e-10 || u > 1 + 1e-10) return false;
        q.crossVectors(displacement, edge1);
        const v = direction.dot(q) / determinant;
        if (v < -1e-10 || u + v > 1 + 1e-10) return false;
        const distance = edge2.dot(q) / determinant;
        if (distance < -CONTACT_EPSILON_MM) return false;
        if (distance <= CONTACT_EPSILON_MM || u <= 1e-9 || v <= 1e-9 || 1 - u - v <= 1e-9) { ambiguous = true; return true; }
        crossings++;
        return false;
      },
    });
    if (ambiguous) continue;
    const inside = crossings % 2 !== 0;
    if (classification !== undefined && classification !== inside) return 'uncertain';
    classification = inside; confirmations++;
  }
  return confirmations < 2 ? 'uncertain' : classification ? 'hit' : 'clear';
}
