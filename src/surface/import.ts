import { BufferGeometry, Matrix3, Matrix4, Mesh, Object3D, Vector3, LoadingManager, type BufferAttribute, type InterleavedBufferAttribute } from 'three';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { identityPose } from '../workspace/frame-graph';
import { SURFACE_LIMITS, type SceneSurface, type SurfaceFormat, type SurfaceUnit, type SurfaceMesh } from './types';
import { validateSceneSurface } from './validation';

export interface SurfaceImportOptions { name?: string; unit?: SurfaceUnit; id?: string; signal?: AbortSignal }
export type SurfaceImportFile = Pick<File, 'name' | 'size' | 'arrayBuffer'>;
const check = (valid: unknown, message: string): void => { if (!valid) throw new Error(message); };
const abort = (signal?: AbortSignal): void => { if (signal?.aborted) throw new DOMException('已取消模型导入。', 'AbortError'); };
const record = (v: unknown): v is Record<string, any> => !!v && typeof v === 'object' && !Array.isArray(v);

/** Reject external resources before invoking a loader. Materials are intentionally omitted. */
function geometryGlb(bytes: ArrayBuffer): ArrayBuffer {
  const view = new DataView(bytes);
  check(bytes.byteLength >= 20 && view.getUint32(0, true) === 0x46546c67 && view.getUint32(4, true) === 2 && view.getUint32(8, true) === bytes.byteLength, '需要完整的GLB 2.0文件。');
  let offset = 12, json: Record<string, any> | null = null, binary: Uint8Array | null = null;
  while (offset < bytes.byteLength) {
    check(offset + 8 <= bytes.byteLength, 'GLB数据块被截断。');
    const length = view.getUint32(offset, true), kind = view.getUint32(offset + 4, true); offset += 8;
    check(length % 4 === 0 && offset + length <= bytes.byteLength, 'GLB数据块长度无效。');
    if (kind === 0x4e4f534a) {
      check(json === null && binary === null, 'GLB需要唯一的首个JSON数据块。');
      try { json = JSON.parse(new TextDecoder().decode(new Uint8Array(bytes, offset, length)).trim()); } catch { throw new Error('GLB描述数据损坏。'); }
    } else if (kind === 0x004e4942) { check(json !== null && binary === null, 'GLB需要唯一的内嵌几何数据块。'); binary = new Uint8Array(bytes, offset, length); }
    else throw new Error('GLB包含不支持的数据块。');
    offset += length;
  }
  check(record(json) && record(json.asset) && json.asset.version === '2.0' && binary !== null, 'GLB缺少有效的内嵌几何。');
  const doc = json!;
  // No URI may reach GLTFLoader, including an unused external image or extension.
  const scan: unknown[] = [doc];
  while (scan.length) {
    const value = scan.pop(); if (!value || typeof value !== 'object') continue;
    for (const [key, child] of Object.entries(value)) { check(key !== 'uri', 'GLB必须自包含，不能引用外部文件或网址。'); if (child && typeof child === 'object') scan.push(child); }
  }
  check(Array.isArray(doc.buffers) && doc.buffers.length === 1 && Number.isSafeInteger(doc.buffers[0]?.byteLength) && doc.buffers[0].byteLength > 0 && doc.buffers[0].byteLength <= binary!.byteLength, 'GLB几何缓冲区无效。');
  for (const [key, maximum] of [['nodes', 4096], ['meshes', 1024], ['accessors', 4096], ['bufferViews', 4096], ['scenes', 16]] as const) check(Array.isArray(doc[key]) && doc[key].length > 0 && doc[key].length <= maximum, `GLB的${key}数量超出支持范围。`);
  check(!doc.animations?.length && !doc.skins?.length, '请将动画或蒙皮模型导出为静态工件后导入。');
  check(!doc.extensionsRequired?.length, '当前仅支持未压缩的标准GLB静态三角网格，请先导出为普通GLB。');
  for (const accessor of doc.accessors) check(record(accessor) && Number.isSafeInteger(accessor.count) && accessor.count > 0 && accessor.count <= SURFACE_LIMITS.triangles * 3, 'GLB顶点或索引数量超出支持范围。');
  for (const mesh of doc.meshes) {
    check(record(mesh) && Array.isArray(mesh.primitives) && mesh.primitives.length > 0 && mesh.primitives.length <= 1024, 'GLB网格结构无效。');
    check(!mesh.weights?.length, '请将形变网格导出为静态工件后导入。');
    for (const primitive of mesh.primitives) {
      check(record(primitive) && (primitive.mode === undefined || primitive.mode === 4) && record(primitive.attributes) && Number.isSafeInteger(primitive.attributes.POSITION), 'GLB仅支持三角网格。');
      check(!primitive.targets?.length && !primitive.extensions?.KHR_draco_mesh_compression, '不支持形变或Draco压缩网格，请导出为普通静态GLB。');
      delete primitive.material;
      primitive.attributes = { POSITION: primitive.attributes.POSITION, ...(primitive.attributes.NORMAL === undefined ? {} : { NORMAL: primitive.attributes.NORMAL }) };
    }
  }
  const state = new Uint8Array(doc.nodes.length);
  function visit(index: number, depth: number): void {
    check(Number.isSafeInteger(index) && index >= 0 && index < doc.nodes.length && depth <= 256, 'GLB节点层级超出支持范围。');
    check(state[index] !== 1, 'GLB节点层级存在循环。'); if (state[index] === 2) return;
    const node = doc.nodes[index]; check(record(node) && node.skin === undefined && !node.weights?.length && !node.extensions?.EXT_mesh_gpu_instancing, '不支持蒙皮、形变或GPU实例模型，请先烘焙为静态工件。');
    if (node.children !== undefined) check(Array.isArray(node.children) && node.children.length <= doc.nodes.length, 'GLB子节点列表无效。');
    for (const [field, length] of [['matrix', 16], ['translation', 3], ['rotation', 4], ['scale', 3]] as const) if (node[field] !== undefined) check(Array.isArray(node[field]) && node[field].length === length && node[field].every((n: unknown) => typeof n === 'number' && Number.isFinite(n)), 'GLB节点变换无效。');
    state[index] = 1; for (const child of node.children ?? []) visit(child, depth + 1); state[index] = 2;
  }
  for (let i = 0; i < doc.nodes.length; i++) visit(i, 0);
  for (const buffer of doc.bufferViews) check(!buffer.extensions?.EXT_meshopt_compression && !buffer.extensions?.KHR_meshopt_compression, '不支持Meshopt压缩网格，请导出为普通GLB。');
  delete doc.materials; delete doc.textures; delete doc.images; delete doc.samplers; delete doc.extensions; delete doc.extensionsUsed;
  const jsonBytes = new TextEncoder().encode(JSON.stringify(doc)), jsonLength = Math.ceil(jsonBytes.length / 4) * 4;
  const output = new ArrayBuffer(12 + 8 + jsonLength + 8 + binary!.byteLength), data = new DataView(output), array = new Uint8Array(output);
  data.setUint32(0, 0x46546c67, true); data.setUint32(4, 2, true); data.setUint32(8, output.byteLength, true);
  data.setUint32(12, jsonLength, true); data.setUint32(16, 0x4e4f534a, true); array.fill(32, 20, 20 + jsonLength); array.set(jsonBytes, 20);
  data.setUint32(20 + jsonLength, binary!.byteLength, true); data.setUint32(24 + jsonLength, 0x004e4942, true); array.set(binary!, 28 + jsonLength);
  return output;
}

function disposeTree(root: Object3D): void {
  const geometries = new Set<BufferGeometry>(), materials = new Set<{ dispose(): void }>();
  root.traverse(object => { const mesh = object as Mesh; if (mesh.geometry) geometries.add(mesh.geometry); if (mesh.material) for (const material of Array.isArray(mesh.material) ? mesh.material : [mesh.material]) materials.add(material); });
  geometries.forEach(geometry => geometry.dispose()); materials.forEach(material => material.dispose());
}
/** Generate smooth normals only across consistently wound shared edges below the crease angle. */
function completeNormals(positions: number[], supplied: (number[] | null)[], signal?: AbortSignal): number[] {
  const corners = positions.length / 3, faces = corners / 3, parent = new Uint32Array(corners), sums = new Float64Array(corners * 3);
  const faceNormals: number[][] = [], faceCross: number[][] = [];
  for (let i = 0; i < corners; i++) parent[i] = i;
  const find = (i: number): number => { while (parent[i] !== i) { parent[i] = parent[parent[i]]; i = parent[i]; } return i; };
  const join = (a: number, b: number): void => { if (!supplied[a] && !supplied[b]) parent[find(b)] = find(a); };
  const vertices = new Map<string, number>(), ids = new Uint32Array(corners);
  for (let c = 0; c < corners; c++) {
    const key = `${positions[c * 3]},${positions[c * 3 + 1]},${positions[c * 3 + 2]}`;
    let id = vertices.get(key); if (id === undefined) { id = vertices.size; vertices.set(key, id); } ids[c] = id;
  }
  const edges = new Map<string, { face: number; a: number; b: number }[]>();
  for (let face = 0; face < faces; face++) {
    if (face % 512 === 0) abort(signal);
    const offset = face * 9, a = new Vector3().fromArray(positions, offset), b = new Vector3().fromArray(positions, offset + 3), c = new Vector3().fromArray(positions, offset + 6);
    const cross = b.sub(a).cross(c.sub(a)), length = cross.length();
    check(Number.isFinite(length) && length > SURFACE_LIMITS.minimumDoubleAreaMm2, '模型包含退化三角形，请修复模型后重试。');
    faceCross.push(cross.toArray()); faceNormals.push(cross.multiplyScalar(1 / length).toArray());
    for (let i = 0; i < 3; i++) {
      const a = face * 3 + i, b = face * 3 + (i + 1) % 3, va = ids[a], vb = ids[b], key = va < vb ? `${va}:${vb}` : `${vb}:${va}`;
      const list = edges.get(key) ?? []; list.push({ face, a, b }); edges.set(key, list);
    }
  }
  const cosine = Math.cos(SURFACE_LIMITS.smoothingAngleDegrees * Math.PI / 180);
  for (const list of edges.values()) if (list.length === 2) {
    const [a, b] = list, na = faceNormals[a.face], nb = faceNormals[b.face];
    if (ids[a.a] === ids[b.b] && ids[a.b] === ids[b.a] && na.reduce((sum, n, i) => sum + n * nb[i], 0) >= cosine) { join(a.a, b.b); join(a.b, b.a); }
  }
  for (let corner = 0; corner < corners; corner++) { const root = find(corner), normal = faceCross[Math.floor(corner / 3)]; for (let axis = 0; axis < 3; axis++) sums[root * 3 + axis] += normal[axis]; }
  const normals: number[] = [];
  for (let corner = 0; corner < corners; corner++) {
    const root = find(corner), vector = supplied[corner] ?? [sums[root * 3], sums[root * 3 + 1], sums[root * 3 + 2]], length = Math.hypot(...vector);
    check(Number.isFinite(length) && length > 1e-10, '模型法向量退化，请检查面绕序。'); normals.push(...vector.map(value => value / length));
  }
  return normals;
}
function bake(root: Object3D, scale: number, glb: boolean, preserveNormals: boolean, signal?: AbortSignal): SurfaceMesh {
  root.updateMatrixWorld(true);
  const positions: number[] = [], supplied: (number[] | null)[] = [], conversion = new Matrix4().makeRotationX(glb ? Math.PI / 2 : 0).multiply(new Matrix4().makeScale(scale, scale, scale));
  root.traverse(object => {
    if (!(object instanceof Mesh)) return;
    abort(signal); check(!(object as any).isSkinnedMesh && !(object as any).isInstancedMesh, '请将蒙皮或实例网格烘焙后导入。');
    const geometry = object.geometry, attribute = geometry.getAttribute('position') as BufferAttribute | InterleavedBufferAttribute | undefined, normal = preserveNormals ? geometry.getAttribute('normal') : undefined, index = geometry.index;
    check(attribute && attribute.itemSize === 3 && attribute.count > 0, '模型缺少有效顶点。');
    const count = index?.count ?? attribute!.count;
    check(count % 3 === 0 && count > 0 && positions.length / 9 + count / 3 <= SURFACE_LIMITS.triangles, '模型三角形不能超过100000，请先简化模型。');
    if (normal) check(normal.itemSize === 3 && normal.count === attribute!.count, '模型法向量数量无效。');
    const matrix = conversion.clone().multiply(object.matrixWorld), determinant = matrix.determinant();
    check(Number.isFinite(determinant) && Math.abs(determinant) > 1e-12, '模型变换包含退化缩放。');
    const normalMatrix = new Matrix3().getNormalMatrix(matrix), order = determinant < 0 ? [0, 2, 1] : [0, 1, 2];
    for (let face = 0; face < count; face += 3) {
      if (face % 1536 === 0) abort(signal);
      for (const corner of order) {
        const i = index ? index.getX(face + corner) : face + corner;
        check(Number.isSafeInteger(i) && i >= 0 && i < attribute!.count, '模型索引越界。');
        const p = new Vector3().fromBufferAttribute(attribute!, i).applyMatrix4(matrix);
        check(p.toArray().every(value => Number.isFinite(value) && Math.abs(value) <= SURFACE_LIMITS.coordinateMm), '导入后的模型超出毫米坐标范围，请检查单位。');
        positions.push(p.x, p.y, p.z);
        if (normal) {
          const n = new Vector3().fromBufferAttribute(normal, i).applyMatrix3(normalMatrix), length = n.length();
          check(Number.isFinite(length) && length > 1e-12, '模型包含无效法向量。'); supplied.push(n.multiplyScalar(1 / length).toArray());
        } else supplied.push(null);
      }
    }
  });
  check(positions.length > 0, '模型中没有可导入的三角表面。');
  return { positions, normals: completeNormals(positions, supplied, signal) };
}

/** Import self-contained geometry only; project data owns all coordinates after this resolves. */
export async function importSurface(file: SurfaceImportFile, options: SurfaceImportOptions = {}): Promise<SceneSurface> {
  abort(options.signal);
  const extension = file.name.split('.').pop()?.toLowerCase();
  check(extension === 'stl' || extension === 'obj' || extension === 'glb', '请选择STL、OBJ或GLB模型。');
  check(Number.isSafeInteger(file.size) && file.size > 0 && file.size <= SURFACE_LIMITS.fileBytes, '模型文件需在20 MiB以内。');
  const format = extension as SurfaceFormat, unit = format === 'glb' ? 'm' : options.unit ?? 'mm';
  check(options.unit === undefined || options.unit === 'mm' || options.unit === 'm', '模型单位只支持毫米或米。');
  check(format !== 'glb' || options.unit === undefined || options.unit === 'm', 'GLB使用标准米单位，无需手动更改。');
  const bytes = await file.arrayBuffer(); abort(options.signal); check(bytes.byteLength === file.size, '模型文件未完整读取。');
  let root: Object3D | null = null;
  try {
    if (format === 'stl') {
      const count = bytes.byteLength >= 84 ? new DataView(bytes).getUint32(80, true) : -1;
      if (84 + count * 50 === bytes.byteLength) check(count > 0 && count <= SURFACE_LIMITS.triangles, '模型三角形不能超过100000，请先简化模型。');
      else { const text = new TextDecoder().decode(bytes); check((text.match(/\bvertex\s+/g)?.length ?? 0) <= SURFACE_LIMITS.triangles * 3, '模型三角形不能超过100000，请先简化模型。'); }
      root = new Mesh(new STLLoader().parse(bytes));
    } else if (format === 'obj') {
      const text = new TextDecoder('utf-8', { fatal: true }).decode(bytes).replace(/\\\r?\n/g, '').replace(/#.*$/gm, '');
      check(!/^[ \t]*(?:call|csh)\b/m.test(text), 'OBJ包含不支持的外部命令。');
      check((text.match(/^[ \t]*v[ \t]+/gm)?.length ?? 0) <= SURFACE_LIMITS.triangles * 3, 'OBJ顶点过多，请先简化模型。');
      const lines = text.split(/\r?\n/), vertices = lines.filter(line => /^[ \t]*v[ \t]+/.test(line)).map(line => line.trim().split(/\s+/).slice(1, 4).map(Number));
      let triangles = 0, availableVertices = 0;
      for (const line of lines) {
        if (/^[ \t]*v[ \t]+/.test(line)) availableVertices++;
        if (!/^[ \t]*f[ \t]+/.test(line)) continue;
        const corners = line.trim().split(/\s+/).slice(1);
        check(corners.length >= 3 && corners.every(corner => /^-?\d+(?:\/-?\d*)?(?:\/-?\d+)?$/.test(corner)), 'OBJ面索引无效。');
        triangles += corners.length - 2; check(triangles <= SURFACE_LIMITS.triangles, '模型三角形不能超过100000，请先简化模型。');
        check(corners.length <= 256, 'OBJ单面顶点过多，请先将多边形三角化。');
        const points = corners.map(corner => {
          const index = Number(corner.split('/')[0]), vertex = vertices[index < 0 ? availableVertices + index : index - 1];
          check(index !== 0 && vertex?.length === 3 && vertex.every(Number.isFinite), 'OBJ面索引或顶点坐标无效。');
          return new Vector3(vertex[0], vertex[1], vertex[2]);
        });
        if (points.length > 3) {
          const edge = points[1].clone().sub(points[0]), normal = edge.clone().cross(points[2].clone().sub(points[0])), length = normal.length();
          check(length > 1e-15, 'OBJ多边形包含退化边，请先三角化。'); normal.multiplyScalar(1 / length);
          const diameter = Math.max(...points.map(point => point.distanceTo(points[0]))), tolerance = Math.max(1e-10, diameter * 1e-6);
          check(points.every(point => Math.abs(point.clone().sub(points[0]).dot(normal)) <= tolerance), 'OBJ包含非平面多边形，请先三角化后导出。');
          let turning = 0;
          for (let i = 0; i < points.length; i++) {
            const a = points[(i + 1) % points.length].clone().sub(points[i]), b = points[(i + 2) % points.length].clone().sub(points[(i + 1) % points.length]);
            check(a.length() > 1e-12 && b.length() > 1e-12, 'OBJ多边形包含重复顶点，请先修复模型。');
            const cross = a.clone().cross(b).dot(normal); check(cross >= -tolerance * diameter, 'OBJ包含非凸多边形，请先三角化后导出。');
            turning += Math.atan2(cross, a.dot(b));
          }
          check(Math.abs(turning - 2 * Math.PI) < 1e-5, 'OBJ多边形存在自交，请先三角化后导出。');
        }
      }
      // OBJLoader never loads mtllib/usemtl resources; only the vertex/face geometry is retained.
      root = new OBJLoader().parse(text);
    } else {
      const manager = new LoadingManager(); manager.setURLModifier(() => { throw new Error('模型导入不允许访问外部资源。'); });
      root = (await new GLTFLoader(manager).parseAsync(geometryGlb(bytes), '')).scene;
    }
    abort(options.signal);
    const explicitObjNormals = format === 'obj' && /^[ \t]*vn[ \t]+/m.test(new TextDecoder().decode(bytes));
    const mesh = bake(root, unit === 'm' ? 1000 : 1, format === 'glb', format === 'glb' || explicitObjNormals, options.signal);
    abort(options.signal);
    return validateSceneSurface({ id: options.id ?? crypto.randomUUID(), name: options.name ?? file.name.replace(/\.[^.]+$/, ''), revision: 0, visible: true,
      pose: identityPose(), source: { name: file.name, format, unit }, mesh });
  } catch (error) {
    if (error instanceof DOMException && error.name === 'AbortError') throw error;
    throw new Error(error instanceof Error ? '模型导入失败：' + error.message : '模型导入失败，请检查文件是否完整。');
  } finally { if (root) disposeTree(root); }
}
