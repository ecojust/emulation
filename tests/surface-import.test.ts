import { describe, expect, it, vi } from 'vitest';
import { importSurface, type SurfaceImportFile } from '../src/surface/import';
import { evaluateSurfaceAnchor, surfaceWorldMesh, validateSceneSurface, validateSceneSurfaces, validateSurfacePickReference } from '../src/surface/validation';
import { SURFACE_LIMITS } from '../src/surface/types';

function file(name: string, data: string | ArrayBuffer): SurfaceImportFile {
  const bytes = typeof data === 'string' ? new TextEncoder().encode(data).buffer : data;
  return { name, size: bytes.byteLength, arrayBuffer: async () => bytes.slice(0) };
}
const triangle = 'v 0 0 0\nv 10 0 0\nv 0 10 0\nf 1 2 3\n';
function glb(options: { scale?: number[]; positions?: number[]; normals?: number[]; json?: (value: any) => void } = {}): ArrayBuffer {
  const positions = options.positions ?? [0, 0, 0, 1, 0, 0, 0, 1, 0], normals = options.normals;
  const floats = new Float32Array([...positions, ...(normals ?? [])]);
  const doc: any = { asset: { version: '2.0' }, scene: 0, scenes: [{ nodes: [0] }],
    nodes: [{ mesh: 0, translation: [.01, .02, .03], scale: options.scale ?? [2, 3, 4] }],
    meshes: [{ primitives: [{ attributes: { POSITION: 0, ...(normals ? { NORMAL: 1 } : {}) } }] }],
    buffers: [{ byteLength: floats.byteLength }], bufferViews: [{ buffer: 0, byteOffset: 0, byteLength: positions.length * 4 }, ...(normals ? [{ buffer: 0, byteOffset: positions.length * 4, byteLength: normals.length * 4 }] : [])],
    accessors: [{ bufferView: 0, componentType: 5126, count: 3, type: 'VEC3', min: [0, 0, 0], max: [1, 1, 1] }, ...(normals ? [{ bufferView: 1, componentType: 5126, count: 3, type: 'VEC3' }] : [])] };
  options.json?.(doc);
  const json = new TextEncoder().encode(JSON.stringify(doc)), jsonSize = Math.ceil(json.byteLength / 4) * 4;
  const bytes = new ArrayBuffer(28 + jsonSize + floats.byteLength), view = new DataView(bytes), array = new Uint8Array(bytes);
  view.setUint32(0, 0x46546c67, true); view.setUint32(4, 2, true); view.setUint32(8, bytes.byteLength, true);
  view.setUint32(12, jsonSize, true); view.setUint32(16, 0x4e4f534a, true); array.fill(32, 20, 20 + jsonSize); array.set(json, 20);
  view.setUint32(20 + jsonSize, floats.byteLength, true); view.setUint32(24 + jsonSize, 0x004e4942, true); array.set(new Uint8Array(floats.buffer), 28 + jsonSize);
  return bytes;
}
function stl(triangles: number[][]): ArrayBuffer {
  const bytes = new ArrayBuffer(84 + triangles.length * 50), view = new DataView(bytes); view.setUint32(80, triangles.length, true);
  triangles.forEach((vertices, face) => vertices.forEach((n, index) => view.setFloat32(84 + face * 50 + 12 + index * 4, n, true)));
  return bytes;
}
function closeArray(actual: number[], expected: number[], tolerance = 1e-7) { expect(actual).toHaveLength(expected.length); actual.forEach((n, i) => expect(Math.abs(n - expected[i])).toBeLessThan(tolerance)); }

describe('self-contained workpiece import', () => {
  it('imports OBJ triangles, explicit units and disconnected objects without loading materials', async () => {
    const spy = vi.spyOn(globalThis, 'fetch');
    try {
      const model = await importSurface(file('plate.obj', 'mtllib https://example.invalid/private.mtl\n' + triangle), { id: 'surface', unit: 'm', name: '工件' });
      expect(model.source).toEqual({ name: 'plate.obj', format: 'obj', unit: 'm' });
      expect(model.pose).toEqual({ position: [0, 0, 0], orientation: [1, 0, 0, 0] });
      expect(model.mesh.positions).toEqual([0, 0, 0, 10000, 0, 0, 0, 10000, 0]);
      expect(model.mesh.normals).toEqual([0, 0, 1, 0, 0, 1, 0, 0, 1]); expect(spy).not.toHaveBeenCalled();
      const two = await importSurface(file('parts.OBJ', triangle + 'o second\nv 0 0 5\nv 10 0 5\nv 0 10 5\nf -3 -2 -1\n'));
      expect(two.mesh.positions).toHaveLength(18); expect(two.mesh.positions.slice(9)).toEqual([0, 0, 5, 10, 0, 5, 0, 10, 5]);
    } finally { spy.mockRestore(); }
  });
  it('triangulates convex OBJ polygons but rejects concavity or warped faces before inventing a surface', async () => {
    const quad = 'v 0 0 0\nv 10 0 0\nv 10 10 0\nv 0 10 0\nf 1 2 3 4 # planar quad\n';
    expect((await importSurface(file('quad.obj', quad))).mesh.positions).toHaveLength(18);
    await expect(importSurface(file('concave.obj', 'v 0 0 0\nv 10 0 0\nv 3 3 0\nv 10 10 0\nv 0 10 0\nf 1 2 3 4 5\n'))).rejects.toThrow(/非凸/);
    await expect(importSurface(file('warped.obj', quad.replace('v 0 10 0', 'v 0 10 1')))).rejects.toThrow(/非平面/);
  });
  it('bakes GLB metres, Y-up, node transforms and nonuniform normals into millimetre triangles', async () => {
    const normal = [0, -Math.SQRT1_2, Math.SQRT1_2];
    const model = await importSurface(file('curved.glb', glb({ positions: [0, 0, 0, 1, 0, 0, 0, 1, 1], normals: [...normal, ...normal, ...normal] })));
    closeArray(model.mesh.positions, [10, -30, 20, 2010, -30, 20, 10, -4030, 3020]);
    closeArray(model.mesh.normals, [0, -.6, -.8, 0, -.6, -.8, 0, -.6, -.8]);
    expect(model.source.unit).toBe('m');
  });
  it('keeps outward winding under mirrored GLB node transforms', async () => {
    const model = await importSurface(file('mirror.glb', glb({ scale: [-2, 3, 4], normals: [0, 0, 1, 0, 0, 1, 0, 0, 1] })));
    closeArray(model.mesh.positions, [10, -30, 20, 10, -30, 3020, -1990, -30, 20]);
    closeArray(model.mesh.normals, [0, -1, 0, 0, -1, 0, 0, -1, 0]);
    expect(() => validateSceneSurface(model)).not.toThrow();
  });
  it('ignores embedded GLB material images but rejects external resources before any fetch', async () => {
    const spy = vi.spyOn(globalThis, 'fetch');
    try {
      const embedded = glb({ json: doc => { doc.images = [{ bufferView: 0, mimeType: 'image/png' }]; doc.textures = [{ source: 0 }]; doc.materials = [{ pbrMetallicRoughness: { baseColorTexture: { index: 0 } } }]; doc.meshes[0].primitives[0].material = 0; } });
      await expect(importSurface(file('embedded.glb', embedded))).resolves.toHaveProperty('mesh');
      await expect(importSurface(file('remote.glb', glb({ json: doc => { doc.images = [{ uri: 'https://example.invalid/image.png' }]; } })))).rejects.toThrow(/自包含|外部/);
      expect(spy).not.toHaveBeenCalled();
    } finally { spy.mockRestore(); }
  });
  it('smooths connected STL facets while retaining ninety-degree creases and separate vertex contacts', async () => {
    const shallow = await importSurface(file('smooth.stl', stl([[0, 0, 0, 10, 0, 0, 0, 10, 0], [10, 0, 0, 0, 0, 0, 0, -10, 2]])));
    closeArray(shallow.mesh.normals.slice(0, 3), shallow.mesh.normals.slice(12, 15));
    expect(shallow.mesh.normals[1]).toBeGreaterThan(0); expect(shallow.mesh.normals[1]).toBeLessThan(.2);
    const hard = await importSurface(file('crease.stl', stl([[0, 0, 0, 10, 0, 0, 0, 10, 0], [10, 0, 0, 0, 0, 0, 0, 0, 10]])));
    closeArray(hard.mesh.normals.slice(0, 3), [0, 0, 1]); closeArray(hard.mesh.normals.slice(9, 12), [0, 1, 0]);
    const separate = await importSurface(file('contact.stl', stl([[0, 0, 0, 10, 0, 0, 0, 10, 0], [0, 0, 0, -10, 0, 0, 0, -10, 2]])));
    closeArray(separate.mesh.normals.slice(0, 3), [0, 0, 1]); expect(separate.mesh.normals[10]).toBeGreaterThan(.1);
  });
  it('rejects malformed, degenerate, animated, unsupported and oversized inputs without partial assets', async () => {
    await expect(importSurface(file('bad.obj', 'v 0 0 0\nf 1 1 1'))).rejects.toThrow(/退化/);
    await expect(importSurface(file('bad.obj', 'v 0 0 NaN\nv 1 0 0\nv 0 1 0\nf 1 2 3'))).rejects.toThrow(/坐标|无效/);
    await expect(importSurface(file('bad.obj', triangle.replace('f 1 2 3', 'f 1 2 100')))).rejects.toThrow(/坐标|无效/);
    await expect(importSurface(file('bad.glb', glb({ scale: [0, 1, 1] })))).rejects.toThrow(/退化/);
    await expect(importSurface(file('animation.glb', glb({ json: doc => { doc.animations = [{}]; } })))).rejects.toThrow(/静态/);
    await expect(importSurface(file('cycle.glb', glb({ json: doc => { doc.nodes[0].children = [0]; } })))).rejects.toThrow(/循环/);
    await expect(importSurface(file('primitive.glb', glb({ json: doc => { doc.meshes[0].primitives[0].mode = 1; } })))).rejects.toThrow(/三角/);
    await expect(importSurface(file('plate.step', triangle))).rejects.toThrow(/STL/);
    const read = vi.fn(); await expect(importSurface({ name: 'large.stl', size: SURFACE_LIMITS.fileBytes + 1, arrayBuffer: read })).rejects.toThrow(/20 MiB/); expect(read).not.toHaveBeenCalled();
    await expect(importSurface(file('many.obj', 'v 0 0 0\nf ' + '1 '.repeat(SURFACE_LIMITS.triangles + 3)))).rejects.toThrow(/100000/);
  });
  it('cancels before reading and after an asynchronous file read', async () => {
    const controller = new AbortController(); controller.abort(); const read = vi.fn();
    await expect(importSurface({ name: 'plate.obj', size: 100, arrayBuffer: read }, { signal: controller.signal })).rejects.toMatchObject({ name: 'AbortError' }); expect(read).not.toHaveBeenCalled();
    const next = new AbortController(), bytes = new TextEncoder().encode(triangle).buffer;
    await expect(importSurface({ name: 'plate.obj', size: bytes.byteLength, arrayBuffer: async () => { next.abort(); return bytes; } }, { signal: next.signal })).rejects.toMatchObject({ name: 'AbortError' });
  });
});

describe('durable surface coordinates and anchors', () => {
  it('applies pose once to vertices and normals and verifies exact pick provenance', async () => {
    const model = await importSurface(file('plate.obj', triangle), { id: 'part' });
    model.pose = { position: [100, 200, 300], orientation: [Math.SQRT1_2, Math.SQRT1_2, 0, 0] };
    const surface = validateSceneSurface(model), anchor = { triangleIndex: 0, barycentric: [.25, .25, .5] as [number, number, number] };
    const actual = evaluateSurfaceAnchor(surface, anchor);
    closeArray(actual.position, [102.5, 200, 305]); closeArray(actual.normal, [0, -1, 0]);
    const world = surfaceWorldMesh(surface); closeArray(world.positions.slice(6), [100, 200, 310]);
    const pick = { ...anchor, ...actual, surfaceId: surface.id, surfaceRevision: surface.revision };
    expect(() => validateSurfacePickReference(pick, surface)).not.toThrow();
    expect(() => validateSurfacePickReference({ ...pick, position: [0, 0, 0] }, surface)).toThrow(/位置/);
    expect(() => validateSurfacePickReference({ ...pick, normal: [0, 1, 0] }, surface)).toThrow(/法向/);
    expect(() => validateSurfacePickReference({ ...pick, triangleIndex: 1 }, surface)).toThrow(/三角形/);
    expect(() => validateSurfacePickReference({ ...pick, surfaceRevision: 12, position: [0, 0, 0] }, surface)).not.toThrow();
    expect(surface.mesh.positions).toEqual(model.mesh.positions); surface.mesh.positions[0] = 99; expect(model.mesh.positions[0]).toBe(0);
  });
  it('rejects broken stored meshes, normals, duplicate IDs and unbounded composed positions', async () => {
    const model = await importSurface(file('plate.obj', triangle));
    const sparse = structuredClone(model); sparse.mesh.positions = new Array(9); expect(() => validateSceneSurface(sparse)).toThrow();
    const reversed = structuredClone(model); reversed.mesh.normals = model.mesh.normals.map(n => -n); expect(() => validateSceneSurface(reversed)).toThrow(/绕序/);
    const scaled = structuredClone(model); scaled.mesh.normals[2] = .5; expect(() => validateSceneSurface(scaled)).toThrow(/归一化/);
    const moved = structuredClone(model); moved.pose.position[0] = SURFACE_LIMITS.coordinateMm; expect(() => validateSceneSurface(moved)).toThrow(/场景范围/);
    expect(() => validateSceneSurfaces([model, model])).toThrow(/重复/); expect(validateSceneSurfaces(undefined)).toEqual([]);
  });
});
