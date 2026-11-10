import { execFileSync } from 'node:child_process';
import { createHash } from 'node:crypto';
import { existsSync, readFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { buildCollisionMeshModel, type CollisionMeshModel } from '../src/robot/collision-mesh';
import { checkCollisionReference, intersectsDangerZone, linkBoundingBoxes, validateCollisionRequest } from '../src/robot/collision-geometry';
import type { CollisionRequest, CollisionResult, CollisionZone, DangerZone, LinkBoundingBox } from '../src/robot/collision-geometry';
import { getRobotModel } from '../src/robot/model';
import { forwardKinematics, multiplyMatrices } from '../src/robot/kinematics';
import type { JointAngles, Mat4, RobotId, Vec3 } from '../src/robot/types';

const transport = vi.hoisted(() => ({ invoke: vi.fn(), isTauri: vi.fn() }));
vi.mock('@tauri-apps/api/core', () => transport);

const MODELS: RobotId[] = ['irb1100-4-0.58', 'irb1100-4-0.475'];
const zero: JointAngles = [0, 0, 0, 0, 0, 0];
const rotated: JointAngles = [90, 0, 0, 0, 0, 0];
const zone = (centerMm: Vec3, sizeMm: Vec3 = [2, 2, 2]): DangerZone => ({ enabled: true, centerMm, sizeMm });
const identified = (zone: DangerZone, id = 'zone-1'): CollisionZone => ({ ...zone, id });
const request = (modelId: RobotId = MODELS[0]): CollisionRequest => ({
  modelId, zones: [identified(zone([0, 0, 50], [2, 2, 2]))],
  solutions: [{ id: 'zero', joints: zero }, { id: 'turned', joints: rotated }],
});
const nativePath = process.env.EMULATION_NATIVE_CORE || resolve(process.env.CARGO_TARGET_DIR || 'target', 'debug', process.platform === 'win32' ? 'core-cli.exe' : 'core-cli');
function native(requests: CollisionRequest[]): CollisionResult[][] {
  if (!existsSync(nativePath)) throw new Error(`Collision parity requires the actual Rust executable. Run cargo build -p emulation-core --bin core-cli first. Expected: ${nativePath}`);
  const input = requests.map(request => JSON.stringify({ op: 'collision', request })).join('\n') + '\n';
  const output = execFileSync(nativePath, { input, encoding: 'utf8', maxBuffer: 4 * 1024 * 1024, timeout: 30_000 });
  const responses = output.trim().split('\n').map(line => JSON.parse(line));
  expect(responses).toHaveLength(requests.length);
  return responses.map(response => { expect(response.error).toBeUndefined(); return response.result as CollisionResult[]; });
}

const meshFixtures = new Map<RobotId, { model: CollisionMeshModel; files: Buffer[] }>();
function meshFixture(modelId: RobotId) {
  let fixture = meshFixtures.get(modelId);
  if (!fixture) {
    const names = ['base_link', ...Array.from({ length: 6 }, (_, index) => `link_${index + 1}`)];
    const files = names.map(name => readFileSync(resolve('public/models', modelId, `${name}.stl`)));
    fixture = { model: buildCollisionMeshModel(modelId, files.map(file => Uint8Array.from(file).buffer)), files };
    meshFixtures.set(modelId, fixture);
  }
  return fixture;
}
function reference(input: CollisionRequest): Promise<CollisionResult[]> {
  return checkCollisionReference(input, { loadModel: async id => meshFixture(id).model });
}
function clearResults(input: CollisionRequest): CollisionResult[] {
  return input.solutions.map(({ id }) => ({ id, collidingLinks: [], uncertainLinks: [], zones: [] }));
}
function baseHits(input: CollisionRequest): CollisionResult[] {
  return input.solutions.map(({ id }) => ({ id, collidingLinks: [0], uncertainLinks: [], zones: [{ zoneId: input.zones[0].id, collidingLinks: [0], uncertainLinks: [] }] }));
}
function actualFlangeSurface(modelId: RobotId): Vec3 {
  // An actual non-degenerate triangle centroid is a known surface point, unlike
  // an enclosing-box center, which may lie in a hole or outside the solid.
  const file = meshFixture(modelId).files[6];
  for (let index = 0; index < file.readUInt32LE(80); index++) {
    const offset = 84 + index * 50 + 12;
    const triangle = Array.from({ length: 9 }, (_, component) => file.readFloatLE(offset + component * 4));
    const a = triangle.slice(0, 3), b = triangle.slice(3, 6), c = triangle.slice(6, 9);
    const u = b.map((value, i) => value - a[i]), v = c.map((value, i) => value - a[i]);
    if (Math.hypot(u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]) > 1e-8) {
      return a.map((value, i) => (value + b[i] + c[i]) / 3) as Vec3;
    }
  }
  throw new Error('The pinned flange mesh has no non-degenerate surface triangle');
}

function seededJoints(modelId: RobotId): JointAngles[] {
  let state = 287431;
  const random = () => ((state = (Math.imul(state, 1664525) + 1013904223) >>> 0) / 2 ** 32);
  const model = getRobotModel(modelId);
  return [zero, rotated, model.readyJoints, ...Array.from({ length: 16 }, () => model.limits.map(limit => limit.minDeg + 1 + random() * (limit.maxDeg - limit.minDeg - 2)) as JointAngles)];
}
function close(actual: number[], expected: number[], tolerance = 1e-8) {
  expect(actual).toHaveLength(expected.length);
  actual.forEach((value, i) => expect(Math.abs(value - expected[i])).toBeLessThan(tolerance));
}

describe('conservative oriented-box separating axes', () => {
  const cube: LinkBoundingBox = { index: 0, centerMm: [0, 0, 0], halfSizeMm: [1, 1, 1], axes: [[1, 0, 0], [0, 1, 0], [0, 0, 1]] };

  it('counts face contact and containment, but rejects a real gap', () => {
    expect(intersectsDangerZone(cube, zone([2, 0, 0]))).toBe(true);
    expect(intersectsDangerZone(cube, zone([2.001, 0, 0]))).toBe(false);
    expect(intersectsDangerZone(cube, zone([0, 0, 0], [0.01, 0.01, 0.01]))).toBe(true);
    expect(intersectsDangerZone(cube, zone([0, 0, 0], [100, 100, 100]))).toBe(true);
  });

  it('uses the link orientation instead of its enlarged world-aligned bounds', () => {
    const c = Math.SQRT1_2;
    const rod: LinkBoundingBox = { index: 4, centerMm: [0, 0, 0], halfSizeMm: [2, 0.1, 0.1], axes: [[c, c, 0], [-c, c, 0], [0, 0, 1]] };
    expect(intersectsDangerZone(rod, zone([1.4, 1.4, 0], [0.2, 0.2, 0.2]))).toBe(true);
    // Both positions are inside the rod's world AABB, but only one is on the rod.
    expect(intersectsDangerZone(rod, zone([1.4, -1.4, 0], [0.2, 0.2, 0.2]))).toBe(false);
  });

  it('retains the nine edge axes, which can separate boxes despite overlapping all six face projections', () => {
    // Independent Rz(50) Ry(40) Rx(30) fixture. Its separating axis is an edge cross-product.
    const tilted: LinkBoundingBox = {
      index: 4, centerMm: [0, 0, 0], halfSizeMm: [1.2, 0.3, 0.7],
      axes: [
        [0.49240387650610407, 0.5868240888334652, -0.6427876096865393],
        [-0.456825992585671, 0.8028723374794714, 0.38302222155948895],
        [0.7408430568614905, 0.10504046113295196, 0.6634139481689384],
      ],
    };
    const testZone = zone([-1.5, -0.8, 1], [1.2, 0.8, 0.4]);
    expect(intersectsDangerZone(tilted, testZone)).toBe(false);
    expect(intersectsDangerZone(tilted, { ...testZone, centerMm: [0, 0, 0] })).toBe(true);
  });
});

describe.each(MODELS)('%s existing mesh placement', modelId => {
  it('preserves mechanical-zero mesh coordinates and rotates each movable part around axis 1', () => {
    const atZero = linkBoundingBoxes(modelId, zero);
    const atQuarterTurn = linkBoundingBoxes(modelId, rotated);
    expect(atZero).toHaveLength(7);
    close(atZero[0].centerMm, [-35, 0, 79.75]);
    close(atQuarterTurn[0].centerMm, atZero[0].centerMm);
    const flangeX = modelId.endsWith('0.58') ? 360 : 310;
    const flangeZ = modelId.endsWith('0.58') ? 617.010986328125 : 562.010986328125;
    close(atZero[6].centerMm, [flangeX, 0, flangeZ]);
    close(atQuarterTurn[6].centerMm, [0, flangeX, flangeZ]);
    for (let link = 1; link < 7; link++) {
      close(atQuarterTurn[link].centerMm, [-atZero[link].centerMm[1], atZero[link].centerMm[0], atZero[link].centerMm[2]]);
      close(atQuarterTurn[link].halfSizeMm, atZero[link].halfSizeMm);
      close(atQuarterTurn[link].axes[0], [0, 1, 0]);
      close(atQuarterTurn[link].axes[1], [-1, 0, 0]);
      close(atQuarterTurn[link].axes[2], [0, 0, 1]);
    }
  });

  it('distinguishes a real mesh empty corner, surface crossing and contained solid volume', async () => {
    const input = request(modelId);
    const expectedBaseSha = modelId.endsWith('0.58') ? '52d6fd54aa77591f296fdf27378adfc6f1a706bdede649e1dbb3410c6195cb16' : '15b59765b3dd5d49e07c977cec7e206452d65323a135bd28ff325961aab021de';
    expect(createHash('sha256').update(meshFixture(modelId).files[0]).digest('hex')).toBe(expectedBaseSha);
    input.solutions.push({ id: 'ready', joints: getRobotModel(modelId).readyJoints });
    input.zones = [identified(zone([75, 75, 80], [4, 4, 4]))];
    for (const solution of input.solutions) {
      expect(linkBoundingBoxes(modelId, solution.joints).filter(box => intersectsDangerZone(box, input.zones[0])).map(box => box.index)).toEqual([0]);
    }
    // Independently checked against every original triangle and ten ray directions.
    expect(await reference(input)).toEqual(clearResults(input));
    input.zones = [identified(zone([0, 70, 100], [4, 4, 4]))];
    expect(await reference(input)).toEqual(baseHits(input));
    input.zones = [identified(zone([0, 0, 50], [2, 2, 2]))];
    // No surface triangle enters this tiny box; all ten oracle rays are odd.
    expect(await reference(input)).toEqual(baseHits(input));
  });

  it('moves an actual flange triangle with the corresponding rigid link', async () => {
    const input = request(modelId), surface = actualFlangeSurface(modelId);
    input.zones = [identified(zone(surface, [.02, .02, .02]))];
    let rows = await reference(input);
    expect(rows[0].collidingLinks).toContain(6);
    expect(rows[0].uncertainLinks).not.toContain(6);
    expect(rows[1].collidingLinks).not.toContain(6);
    input.zones[0].centerMm = [-surface[1], surface[0], surface[2]];
    rows = await reference(input);
    expect(rows[0].collidingLinks).not.toContain(6);
    expect(rows[1].collidingLinks).toContain(6);
    expect(rows[1].uncertainLinks).not.toContain(6);
  });

  it('retains uncertainty inside the bounds of an actual non-manifold link without a surface crossing', async () => {
    const input = request(modelId);
    input.solutions = [{ id: 'zero', joints: zero }, { id: 'ready', joints: getRobotModel(modelId).readyJoints }];
    input.zones = [identified(zone([0, 85, 180], [4, 4, 4]))];
    expect(createHash('sha256').update(meshFixture(modelId).files[1]).digest('hex')).toBe('05f2ff6948f269f609f8b145955bc2f1017861111c889b64400518f37d8f0f40');
    expect(await reference(input)).toEqual(input.solutions.map(({ id }) => ({
      id, collidingLinks: [], uncertainLinks: [1], zones: [{ zoneId: 'zone-1', collidingLinks: [], uncertainLinks: [1] }],
    })));
  });

  it('returns neither hits nor uncertainty for disabled, absent or distant regions', async () => {
    const input = request(modelId);
    input.zones[0].enabled = false;
    expect(await reference(input)).toEqual(clearResults(input));
    input.zones = [identified(zone([5000, 5000, 5000], [10, 10, 10]))];
    expect(await reference(input)).toEqual(clearResults(input));
    input.zones = [];
    expect(await reference(input)).toEqual(clearResults(input));
  });

  it('keeps zone identity and separate sorted link unions across multiple real mesh checks', async () => {
    const input = request(modelId), flange = actualFlangeSurface(modelId);
    input.zones = [
      identified(zone(flange, [.02, .02, .02]), 'flange'),
      identified(zone([0, 0, 50]), 'base'),
      identified({ ...zone(flange), enabled: false }, 'disabled'),
      identified(zone([5000, 5000, 5000]), 'clear'),
      identified(zone(flange, [.02, .02, .02]), 'same-volume'),
    ];
    const results = await reference(input);
    expect(results[0].zones.map(row => row.zoneId)).toEqual(['flange', 'base', 'same-volume']);
    expect(results[0].collidingLinks).toContain(0);
    expect(results[0].collidingLinks).toContain(6);
    for (const row of results) {
      expect(row.collidingLinks).toEqual([...new Set(row.zones.flatMap(zone => zone.collidingLinks))].sort((a, b) => a - b));
      expect(row.uncertainLinks).toEqual([...new Set(row.zones.flatMap(zone => zone.uncertainLinks))].sort((a, b) => a - b));
    }
    expect(results[1].zones).toEqual([{ zoneId: 'base', collidingLinks: [0], uncertainLinks: [] }]);
  });

  it('matches the real Rust mesh engine across seeded poses, empty corners, containment and multiple zones', async () => {
    const joints = seededJoints(modelId);
    const solutions = joints.map((joints, index) => ({ id: `姿态-${index}`, joints }));
    const boxes = joints.map(pose => linkBoundingBoxes(modelId, pose));
    const regions: DangerZone[] = [
      zone([5000, 5000, 5000], [10, 10, 10]),
      zone([75, 75, 80], [4, 4, 4]),
      zone([0, 70, 100], [4, 4, 4]),
      zone([0, 0, 50], [2, 2, 2]),
      { ...zone([0, 0, 50], [100, 100, 100]), enabled: false },
      zone([0, 0, 0], [10000, 10000, 10000]),
      zone([0, 85, 180], [4, 4, 4]),
    ];
    boxes.forEach((parts, index) => {
      const part = parts[2 + index % 5];
      regions.push(zone(part.centerMm, [.2, .2, .2]));
      const outside = part.centerMm.map((coordinate, axis) => coordinate
        + part.axes[0][axis] * (part.halfSizeMm[0] + 5)
        + part.axes[1][axis] * (part.halfSizeMm[1] + 5)) as Vec3;
      regions.push(zone(outside, [.2, .2, .2]));
    });
    const inputs = regions.map(region => ({ modelId, zones: [identified(region)], solutions }));
    inputs.push({ modelId, zones: regions.map((region, index) => identified(region, `region-${index}`)), solutions });
    inputs.push({ modelId, zones: [], solutions });
    const expected: CollisionResult[][] = [];
    for (const input of inputs) expected.push(await reference(input));
    const actual = native(inputs);
    expect(actual).toEqual(expected);
    expect(actual[0]).toEqual(clearResults(inputs[0]));
    expect(actual[1].every(row => !row.collidingLinks.includes(0) && !row.uncertainLinks.includes(0))).toBe(true);
    expect(actual[2].every(row => row.collidingLinks.includes(0) && !row.uncertainLinks.includes(0))).toBe(true);
    expect(actual[3].every(row => row.collidingLinks.includes(0))).toBe(true);
    expect(actual[4]).toEqual(clearResults(inputs[4]));
    for (const row of actual[5]) {
      expect(row.collidingLinks).toEqual([0, 1, 2, 3, 4, 5, 6]);
      expect(row.uncertainLinks).toEqual([]);
    }
  }, 60_000);
});

it('matches native and reference contact tolerance at a verified rotated flange surface', async () => {
  const modelId: RobotId = 'irb1100-4-0.58', model = getRobotModel(modelId);
  const joints: JointAngles = [20, -10, 0, 35, 15, 20];
  // Original triangle 1112 is on x=364. This centroid is >1.68mm from every
  // triangle edge, so the 0.2mm box cannot slip into a neighbouring flange hole.
  const point: Vec3 = [364, 15.238862037658691, 622.9183146158854];
  const source = meshFixture(modelId).files[6], offset = 84 + 1112 * 50 + 12;
  close(point, [0, 1, 2].map(axis => [0, 1, 2].reduce((sum, vertex) => sum + source.readFloatLE(offset + (vertex * 3 + axis) * 4), 0) / 3));
  const zeroFrame = forwardKinematics(model, zero).frames[6];
  const inverse: Mat4 = [
    zeroFrame[0], zeroFrame[4], zeroFrame[8], 0,
    zeroFrame[1], zeroFrame[5], zeroFrame[9], 0,
    zeroFrame[2], zeroFrame[6], zeroFrame[10], 0,
    0, 0, 0, 1,
  ];
  for (let row = 0; row < 3; row++) inverse[row * 4 + 3] = -[0, 1, 2].reduce((sum, axis) => sum + inverse[row * 4 + axis] * zeroFrame[axis * 4 + 3], 0);
  const placement = multiplyMatrices(forwardKinematics(model, joints).frames[6], inverse);
  const normal: Vec3 = [placement[0], placement[4], placement[8]];
  close(normal, [.8777053016636077, .47743846093752235, -.0410599494544913]);
  const worldPoint = [0, 1, 2].map(row => placement[row * 4 + 3] + point.reduce((sum, value, axis) => sum + placement[row * 4 + axis] * value, 0));
  const projectedHalfSize = .1 * normal.reduce((sum, value) => sum + Math.abs(value), 0);
  const inputs = [.5e-6, 2e-6].map(gap => ({
    modelId, solutions: [{ id: 'rotated-flange', joints }],
    zones: [identified(zone(worldPoint.map((value, axis) => value + normal[axis] * (projectedHalfSize + gap)) as Vec3, [.2, .2, .2]))],
  }));
  const actual = native(inputs);
  expect(actual).toEqual(await Promise.all(inputs.map(reference)));
  expect(actual[0][0].collidingLinks).toContain(6);
  expect(actual[0][0].uncertainLinks).not.toContain(6);
  expect(actual[1][0].collidingLinks).not.toContain(6);
  expect(actual[1][0].uncertainLinks).not.toContain(6);
});

describe('bounded collision requests', () => {
  it('rejects non-finite and oversized regions, invalid joints and ambiguous endpoint IDs', () => {
    const variants: CollisionRequest[] = [
      { ...request(), modelId: '__proto__' as RobotId },
      { ...request(), zones: [identified(zone([NaN, 0, 0]))] },
      { ...request(), zones: [identified(zone([1e308, 0, 0]))] },
      { ...request(), zones: [identified(zone([0, 0, 0], [0, 1, 1]))] },
      { ...request(), zones: [identified(zone([0, 0, 0], [-1, 1, 1]))] },
      { ...request(), zones: [identified(zone([0, 0, 0], [1, 1, 1e6 + 1]))] },
      { ...request(), zones: [identified(zone([0, 0, 0]), ' ')] },
      { ...request(), zones: [identified(zone([0, 0, 0]), '区'.repeat(43))] },
      { ...request(), zones: [identified(zone([0, 0, 0]), 'same'), identified(zone([20, 0, 0]), 'same')] },
      { ...request(), zones: Array.from({ length: 65 }, (_, index) => identified(zone([0, 0, 0]), `zone-${index}`)) },
      { ...request(), solutions: [{ id: 'bad', joints: [0, 120, 0, 0, 0, 0] }] },
      { ...request(), solutions: [{ id: 'bad', joints: [0, 0, NaN, 0, 0, 0] }] },
      { ...request(), solutions: [{ id: 'same', joints: zero }, { id: 'same', joints: rotated }] },
      { ...request(), solutions: [{ id: ' ', joints: zero }] },
      { ...request(), solutions: [{ id: '轴'.repeat(43), joints: zero }] },
      { ...request(), solutions: Array.from({ length: 257 }, (_, i) => ({ id: String(i), joints: zero })) },
    ];
    for (const variant of variants) {
      expect(() => validateCollisionRequest(variant)).toThrow();
      expect(() => validateCollisionRequest({ ...variant, zones: variant.zones.map(zone => ({ ...zone, enabled: false })) })).toThrow();
    }
  });

  it('accepts the maximum bounded volume and an empty endpoint list', async () => {
    const input: CollisionRequest = { ...request(), zones: [identified(zone([1e6, -1e6, 1e6], [1e6, 1e6, 1e6]))], solutions: [] };
    expect(() => validateCollisionRequest(input)).not.toThrow();
    expect(await reference(input)).toEqual([]);
  });
});

describe('collision transport lifecycle', () => {
  let checkCollisions: typeof import('../src/robot/collision')['checkCollisions'];
  beforeEach(async () => {
    vi.resetModules();
    ({ checkCollisions } = await import('../src/robot/collision'));
  });
  afterEach(() => { vi.useRealTimers(); vi.resetAllMocks(); vi.unstubAllGlobals(); });

  function installWorkers() {
    const workers: MockWorker[] = [];
    class MockWorker {
      onmessage: ((event: { data: { requestId: string; results?: CollisionResult[]; error?: string } }) => void) | null = null;
      onerror: (() => void) | null = null;
      postMessage = vi.fn<(message: { requestId: string; request: CollisionRequest }) => void>();
      terminate = vi.fn();
      constructor() { workers.push(this); }
      latest() { return this.postMessage.mock.calls.at(-1)![0]; }
      reply(results: CollisionResult[], requestId = this.latest().requestId) { this.onmessage?.({ data: { requestId, results } }); }
    }
    vi.stubGlobal('Worker', MockWorker);
    return workers;
  }

  it('rejects a desktop engine failure without falling back to JavaScript', async () => {
    transport.isTauri.mockReturnValue(true);
    transport.invoke.mockRejectedValue('engine unavailable');
    const Worker = vi.fn();
    vi.stubGlobal('Worker', Worker);
    await expect(checkCollisions(request())).rejects.toThrow('engine unavailable');
    expect(transport.invoke).toHaveBeenCalledWith('check_collisions', expect.objectContaining({ request: request() }));
    expect(Worker).not.toHaveBeenCalled();
  });

  it('requires explicit uncertainty and rejects malformed per-zone evidence and either inconsistent union', async () => {
    transport.isTauri.mockReturnValue(true);
    const valid = baseHits(request());
    const withoutUncertain = { id: valid[0].id, collidingLinks: [0], zones: valid[0].zones };
    const invalid = [
      [],
      [withoutUncertain, valid[1]],
      [{ ...valid[0], uncertainLinks: undefined }, valid[1]],
      [{ ...valid[0], uncertainLinks: [7] }, valid[1]],
      [{ ...valid[0], zones: [{ zoneId: 'unknown', collidingLinks: [0], uncertainLinks: [] }] }, valid[1]],
      [{ ...valid[0], zones: [{ zoneId: 'zone-1', collidingLinks: [0] }] }, valid[1]],
      [{ ...valid[0], zones: [{ zoneId: 'zone-1', collidingLinks: [0], uncertainLinks: [0] }] }, valid[1]],
      [{ ...valid[0], collidingLinks: [] }, valid[1]],
      [{ ...valid[0], uncertainLinks: [1] }, valid[1]],
      [{ ...valid[0], zones: [{ zoneId: 'zone-1', collidingLinks: [], uncertainLinks: [1] }] }, valid[1]],
      [{ ...valid[0], zones: [] }, valid[1]],
      [{ ...valid[0], collidingLinks: [1, 0] }, valid[1]],
      [{ ...valid[0], uncertainLinks: [1, 1] }, valid[1]],
    ];
    for (const response of invalid) {
      transport.invoke.mockResolvedValueOnce(response);
      await expect(checkCollisions(request())).rejects.toThrow();
    }
  });

  it('accepts uncertainty without calling it a hit and permits the same link to differ across zones', async () => {
    transport.isTauri.mockReturnValue(true);
    const input = request();
    input.zones.push(identified(zone([0, 85, 180], [4, 4, 4]), 'other-zone'));
    const result: CollisionResult[] = input.solutions.map(({ id }) => ({
      id, collidingLinks: [1], uncertainLinks: [1], zones: [
        { zoneId: 'zone-1', collidingLinks: [1], uncertainLinks: [] },
        { zoneId: 'other-zone', collidingLinks: [], uncertainLinks: [1] },
      ],
    }));
    transport.invoke.mockResolvedValue(result);
    await expect(checkCollisions(input)).resolves.toEqual(result);
    const reversed = structuredClone(result);
    reversed[0].zones.reverse();
    transport.invoke.mockResolvedValue(reversed);
    await expect(checkCollisions(input)).rejects.toThrow();
  });

  it('cancels only the matching native request and ignores its late completion', async () => {
    transport.isTauri.mockReturnValue(true);
    const complete: ((value: CollisionResult[]) => void)[] = [];
    transport.invoke.mockImplementation((command: string) => command === 'check_collisions'
      ? new Promise<CollisionResult[]>(resolve => { complete.push(resolve); }) : Promise.resolve());
    const controller = new AbortController();
    const pending = checkCollisions(request(), controller.signal);
    const rejected = expect(pending).rejects.toMatchObject({ name: 'AbortError' });
    const requestId = transport.invoke.mock.calls[0][1].requestId;
    controller.abort();
    await rejected;
    expect(transport.invoke).toHaveBeenCalledWith('cancel_motion', { requestId });
    const replacement = checkCollisions(request());
    let delivered = false;
    void replacement.then(() => { delivered = true; });
    complete[0](baseHits(request()));
    await Promise.resolve();
    expect(delivered).toBe(false);
    complete[1](baseHits(request()));
    await expect(replacement).resolves.toEqual(baseHits(request()));
    await expect(checkCollisions(request(), controller.signal)).rejects.toMatchObject({ name: 'AbortError' });
    expect(transport.invoke).toHaveBeenCalledTimes(3);
  });

  it('reuses a successful worker and ignores an older request ID while the next request is pending', async () => {
    transport.isTauri.mockReturnValue(false);
    const workers = installWorkers();
    const first = checkCollisions(request());
    const oldId = workers[0].latest().requestId;
    workers[0].reply(baseHits(request()));
    await expect(first).resolves.toEqual(baseHits(request()));
    expect(workers[0].terminate).not.toHaveBeenCalled();
    const second = checkCollisions(request());
    expect(workers).toHaveLength(1);
    expect(workers[0].latest().requestId).not.toBe(oldId);
    expect(workers[0].latest().request).toEqual(request());
    let delivered = false;
    void second.then(() => { delivered = true; });
    workers[0].reply(baseHits(request()), oldId);
    await Promise.resolve();
    expect(delivered).toBe(false);
    workers[0].reply(baseHits(request()));
    await expect(second).resolves.toEqual(baseHits(request()));
    expect(workers[0].terminate).not.toHaveBeenCalled();
    expect(transport.invoke).not.toHaveBeenCalled();
  });

  it('terminates an aborted worker without cancelling a concurrent check or accepting its stale result', async () => {
    transport.isTauri.mockReturnValue(false);
    const workers = installWorkers(), controller = new AbortController();
    const old = checkCollisions(request(), controller.signal);
    const rejected = expect(old).rejects.toMatchObject({ name: 'AbortError' });
    const active = checkCollisions(request());
    expect(workers).toHaveLength(2);
    controller.abort();
    await rejected;
    expect(workers[0].terminate).toHaveBeenCalledOnce();
    expect(workers[1].terminate).not.toHaveBeenCalled();
    workers[0].reply(baseHits(request()));
    workers[1].reply(baseHits(request()));
    await expect(active).resolves.toEqual(baseHits(request()));
    const next = checkCollisions(request());
    expect(workers).toHaveLength(2);
    expect(workers[1].postMessage).toHaveBeenCalledTimes(2);
    workers[1].reply(baseHits(request()));
    await expect(next).resolves.toEqual(baseHits(request()));
  });

  it('retains at most one idle worker after concurrent successful checks', async () => {
    transport.isTauri.mockReturnValue(false);
    const workers = installWorkers();
    const first = checkCollisions(request()), second = checkCollisions(request());
    workers[0].reply(baseHits(request()));
    await first;
    workers[1].reply(baseHits(request()));
    await second;
    expect(workers[0].terminate).not.toHaveBeenCalled();
    expect(workers[1].terminate).toHaveBeenCalledOnce();
  });

  it('discards malformed or failed workers and creates a fresh worker for a later check', async () => {
    transport.isTauri.mockReturnValue(false);
    const workers = installWorkers();
    const malformed = checkCollisions(request());
    const rejected = expect(malformed).rejects.toThrow();
    workers[0].reply([]);
    await rejected;
    expect(workers[0].terminate).toHaveBeenCalledOnce();
    const failed = checkCollisions(request());
    const failedResult = expect(failed).rejects.toThrow();
    workers[1].onerror?.();
    await failedResult;
    expect(workers[1].terminate).toHaveBeenCalledOnce();
    const recovered = checkCollisions(request());
    expect(workers).toHaveLength(3);
    workers[2].reply(baseHits(request()));
    await expect(recovered).resolves.toEqual(baseHits(request()));
  });

  it('times out by terminating the worker and never returns a false clear result', async () => {
    vi.useFakeTimers();
    transport.isTauri.mockReturnValue(false);
    const workers = installWorkers();
    const pending = checkCollisions(request());
    const rejected = expect(pending).rejects.toThrow(/超时/);
    await vi.runOnlyPendingTimersAsync();
    await rejected;
    expect(workers[0].terminate).toHaveBeenCalledOnce();
    const next = checkCollisions(request());
    expect(workers).toHaveLength(2);
    workers[1].reply(clearResults(request()));
    await expect(next).resolves.toEqual(clearResults(request()));
  });
});

describe('endpoint meshes under a world robot-base transform', () => {
  it('keeps world boxes axis-aligned while moving and rotating the actual robot meshes', async () => {
    const c = Math.SQRT1_2, inputs: CollisionRequest[] = [];
    for (const modelId of MODELS) {
      const pose = { position: [800, -230, 100] as Vec3, orientation: [Math.cos(Math.PI / 8), 0, 0, Math.sin(Math.PI / 8)] as [number, number, number, number] };
      const input: CollisionRequest = { modelId, basePose: pose, solutions: [{ id: 'zero', joints: zero }], zones: [
        identified(zone([800 - 70 * c, -230 + 70 * c, 200], [4, 4, 4]), 'rotated-surface'),
        identified(zone([800, -230 + 150 * c, 180], [4, 4, 4]), 'empty-corner'),
        identified(zone([800, -230, 150]), 'inside-base'),
        identified(zone([0, 70, 100], [4, 4, 4]), 'old-base-location'),
      ] };
      const result = await reference(input);
      expect(result).toEqual([{ id: 'zero', collidingLinks: [0], uncertainLinks: [], zones: [
        { zoneId: 'rotated-surface', collidingLinks: [0], uncertainLinks: [] },
        { zoneId: 'inside-base', collidingLinks: [0], uncertainLinks: [] },
      ] }]);
      inputs.push(input);
      const worldBoxes = linkBoundingBoxes(modelId, zero, pose), original = linkBoundingBoxes(modelId, zero);
      for (let i = 0; i < worldBoxes.length; i++) {
        const [x, y, z] = original[i].centerMm;
        close(worldBoxes[i].centerMm, [800 + c * (x - y), -230 + c * (x + y), 100 + z]);
      }
    }
    expect(native(inputs)).toEqual(await Promise.all(inputs.map(reference)));
  });

  it('retains omitted-base compatibility and rejects invalid base poses before checking even empty zones', async () => {
    const input = request();
    const explicit = { ...input, basePose: { position: [0, 0, 0] as Vec3, orientation: [1, 0, 0, 0] as [number, number, number, number] } };
    expect(await reference(explicit)).toEqual(await reference(input));
    for (const basePose of [
      { position: [0, 0, 0], orientation: [0, 0, 0, 0] },
      { position: [Infinity, 0, 0], orientation: [1, 0, 0, 0] },
      { position: [1e6 + 1, 0, 0], orientation: [1, 0, 0, 0] },
      { position: [0, 0, 0], orientation: [1, 0, 0] },
    ]) expect(() => validateCollisionRequest({ ...input, zones: [], basePose } as unknown as CollisionRequest)).toThrow();
  });
});
