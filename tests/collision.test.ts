import { execFileSync } from 'node:child_process';
import { existsSync } from 'node:fs';
import { resolve } from 'node:path';
import { afterEach, describe, expect, it, vi } from 'vitest';
import { checkCollisions } from '../src/robot/collision';
import { checkCollisionReference, intersectsDangerZone, linkBoundingBoxes, validateCollisionRequest } from '../src/robot/collision-geometry';
import type { CollisionRequest, CollisionResult, CollisionZone, DangerZone, LinkBoundingBox } from '../src/robot/collision-geometry';
import { getRobotModel } from '../src/robot/model';
import type { JointAngles, RobotId, Vec3 } from '../src/robot/types';

const transport = vi.hoisted(() => ({ invoke: vi.fn(), isTauri: vi.fn() }));
vi.mock('@tauri-apps/api/core', () => transport);

const MODELS: RobotId[] = ['irb1100-4-0.58', 'irb1100-4-0.475'];
const zero: JointAngles = [0, 0, 0, 0, 0, 0];
const rotated: JointAngles = [90, 0, 0, 0, 0, 0];
const zone = (centerMm: Vec3, sizeMm: Vec3 = [2, 2, 2]): DangerZone => ({ enabled: true, centerMm, sizeMm });
const identified = (zone: DangerZone, id = 'zone-1'): CollisionZone => ({ ...zone, id });
const request = (modelId: RobotId = MODELS[0]): CollisionRequest => ({
  modelId, zones: [identified(zone([-35, 0, 80], [10, 10, 10]))],
  solutions: [{ id: 'zero', joints: zero }, { id: 'turned', joints: rotated }],
});
const nativePath = process.env.EMULATION_NATIVE_CORE || resolve(process.env.CARGO_TARGET_DIR || 'target', 'debug', process.platform === 'win32' ? 'core-cli.exe' : 'core-cli');
function native(requests: CollisionRequest[]): CollisionResult[][] {
  if (!existsSync(nativePath)) throw new Error(`Collision parity requires the actual Rust executable. Run cargo build -p emulation-core --bin core-cli first. Expected: ${nativePath}`);
  const input = requests.map(request => JSON.stringify({ op: 'collision', request })).join('\n') + '\n';
  const output = execFileSync(nativePath, { input, encoding: 'utf8', maxBuffer: 4 * 1024 * 1024, timeout: 10_000 });
  const responses = output.trim().split('\n').map(line => JSON.parse(line));
  expect(responses).toHaveLength(requests.length);
  return responses.map(response => { expect(response.error).toBeUndefined(); return response.result as CollisionResult[]; });
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

  it('attributes overlapping link boxes to each corresponding endpoint ID', () => {
    const input = request(modelId);
    expect(checkCollisionReference(input)).toEqual([{ id: 'zero', collidingLinks: [0], zones: [{ zoneId: 'zone-1', collidingLinks: [0] }] }, { id: 'turned', collidingLinks: [0], zones: [{ zoneId: 'zone-1', collidingLinks: [0] }] }]);
    const center = linkBoundingBoxes(modelId, zero)[6].centerMm;
    input.zones = [identified(zone(center))];
    const rows = checkCollisionReference(input);
    expect(rows[0].id).toBe('zero');
    expect(rows[0].collidingLinks).toContain(6);
    expect(rows[1].id).toBe('turned');
    expect(rows[1].collidingLinks).not.toContain(6);
  });

  it('returns no warnings when the region is disabled or distant', () => {
    const input = request(modelId);
    input.zones[0].enabled = false;
    expect(checkCollisionReference(input).every(row => row.collidingLinks.length === 0)).toBe(true);
    input.zones = [identified(zone([5000, 5000, 5000], [10, 10, 10]))];
    expect(checkCollisionReference(input).every(row => row.collidingLinks.length === 0)).toBe(true);
  });

  it('reports hit zones in input order and merges their link indices without disabled or clear zones', () => {
    const input = request(modelId);
    const flange = linkBoundingBoxes(modelId, zero)[6].centerMm;
    input.zones = [
      identified(zone(flange), 'flange'),
      identified(zone([-35, 0, 80], [10, 10, 10]), 'base'),
      identified({ ...zone(flange), enabled: false }, 'disabled'),
      identified(zone([5000, 5000, 5000]), 'clear'),
      identified(zone(flange), 'same-volume'),
    ];
    const results = checkCollisionReference(input);
    expect(results[0].zones.map(row => row.zoneId)).toEqual(['flange', 'base', 'same-volume']);
    expect(results[0].collidingLinks).toContain(0);
    expect(results[0].collidingLinks).toContain(6);
    expect(new Set(results[0].collidingLinks).size).toBe(results[0].collidingLinks.length);
    expect(results[1].zones).toEqual([{ zoneId: 'base', collidingLinks: [0] }]);
    input.zones = [];
    expect(checkCollisionReference(input)).toEqual(input.solutions.map(({ id }) => ({ id, collidingLinks: [], zones: [] })));
  });

  it('matches the actual Rust engine for all link indices over seeded rotated poses and hit, gap, contact and disabled regions', () => {
    const joints = seededJoints(modelId);
    const solutions = joints.map((joints, index) => ({ id: `姿态-${index}`, joints }));
    const boxes = joints.map(pose => linkBoundingBoxes(modelId, pose));
    const regions: DangerZone[] = [
      zone([5000, 5000, 5000], [10, 10, 10]),
      zone([-150, 0, 80]), // Exactly touches the base's x=-149 face.
      zone([-150.001, 0, 80]),
      { ...zone([-35, 0, 80], [100, 100, 100]), enabled: false },
    ];
    boxes.forEach((parts, index) => {
      const part = parts[2 + index % 5];
      regions.push(zone(part.centerMm, [0.2, 0.2, 0.2]));
      // An offset beyond two of this rotated box's faces exercises narrow empty space.
      const outside = part.centerMm.map((coordinate, axis) => coordinate
        + part.axes[0][axis] * (part.halfSizeMm[0] + 5)
        + part.axes[1][axis] * (part.halfSizeMm[1] + 5)) as Vec3;
      regions.push(zone(outside, [0.2, 0.2, 0.2]));
    });
    const inputs = regions.map(region => ({ modelId, zones: [identified(region)], solutions }));
    inputs.push({ modelId, zones: regions.map((region, index) => identified(region, `region-${index}`)), solutions });
    inputs.push({ modelId, zones: [], solutions });
    const expected = inputs.map(checkCollisionReference);
    const actual = native(inputs);
    expect(actual).toEqual(expected);
    expect(actual[0].every(row => row.collidingLinks.length === 0)).toBe(true);
    expect(actual[1].every(row => row.collidingLinks.includes(0))).toBe(true);
    expect(actual[2].every(row => !row.collidingLinks.includes(0))).toBe(true);
    expect(actual[3].every(row => row.collidingLinks.length === 0)).toBe(true);
    boxes.forEach((_parts, index) => {
      expect(actual[4 + 2 * index][index].collidingLinks).toContain(2 + index % 5);
      expect(actual[5 + 2 * index][index].collidingLinks).not.toContain(2 + index % 5);
    });
  });
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

  it('accepts the maximum bounded volume and an empty endpoint list', () => {
    const input: CollisionRequest = { ...request(), zones: [identified(zone([1e6, -1e6, 1e6], [1e6, 1e6, 1e6]))], solutions: [] };
    expect(() => validateCollisionRequest(input)).not.toThrow();
    expect(checkCollisionReference(input)).toEqual([]);
  });
});

describe('collision transport lifecycle', () => {
  afterEach(() => { vi.resetAllMocks(); vi.unstubAllGlobals(); });

  it('rejects a desktop engine failure without falling back to JavaScript', async () => {
    transport.isTauri.mockReturnValue(true);
    transport.invoke.mockRejectedValue('engine unavailable');
    const Worker = vi.fn();
    vi.stubGlobal('Worker', Worker);
    await expect(checkCollisions(request())).rejects.toThrow('engine unavailable');
    expect(transport.invoke).toHaveBeenCalledWith('check_collisions', expect.objectContaining({ request: request() }));
    expect(Worker).not.toHaveBeenCalled();
  });

  it('rejects missing endpoint rows, unknown zones and inconsistent link unions', async () => {
    transport.isTauri.mockReturnValue(true);
    const valid = checkCollisionReference(request());
    const invalid = [
      [],
      [{ ...valid[0], zones: [{ zoneId: 'unknown', collidingLinks: [0] }] }, valid[1]],
      [{ ...valid[0], collidingLinks: [] }, valid[1]],
      [{ ...valid[0], zones: [] }, valid[1]],
    ];
    for (const response of invalid) {
      transport.invoke.mockResolvedValueOnce(response);
      await expect(checkCollisions(request())).rejects.toThrow();
    }
  });

  it('cancels the matching native request and ignores its late completion', async () => {
    transport.isTauri.mockReturnValue(true);
    let complete!: (value: CollisionResult[]) => void;
    transport.invoke.mockImplementation((command: string) => command === 'check_collisions'
      ? new Promise<CollisionResult[]>(resolve => { complete = resolve; }) : Promise.resolve());
    const controller = new AbortController();
    const pending = checkCollisions(request(), controller.signal);
    const rejected = expect(pending).rejects.toMatchObject({ name: 'AbortError' });
    const requestId = transport.invoke.mock.calls[0][1].requestId;
    controller.abort();
    await rejected;
    expect(transport.invoke).toHaveBeenCalledWith('cancel_motion', { requestId });
    complete(checkCollisionReference(request()));
    await Promise.resolve();
    await expect(checkCollisions(request(), controller.signal)).rejects.toMatchObject({ name: 'AbortError' });
    expect(transport.invoke).toHaveBeenCalledTimes(2);
  });

  it('terminates an obsolete browser worker and terminates its replacement after delivery', async () => {
    transport.isTauri.mockReturnValue(false);
    const workers: MockWorker[] = [];
    class MockWorker {
      onmessage: ((event: { data: { results: CollisionResult[] } }) => void) | null = null;
      onerror: (() => void) | null = null;
      postMessage = vi.fn();
      terminate = vi.fn();
      constructor() { workers.push(this); }
    }
    vi.stubGlobal('Worker', MockWorker);
    const controller = new AbortController();
    const old = checkCollisions(request(), controller.signal);
    const rejected = expect(old).rejects.toMatchObject({ name: 'AbortError' });
    controller.abort();
    await rejected;
    expect(workers[0].terminate).toHaveBeenCalledOnce();
    const current = checkCollisions(request());
    const data = { results: checkCollisionReference(request()) };
    workers[0].onmessage?.({ data });
    expect(workers[1].terminate).not.toHaveBeenCalled();
    workers[1].onmessage?.({ data });
    await expect(current).resolves.toEqual(data.results);
    expect(workers[1].postMessage).toHaveBeenCalledWith(request());
    expect(workers[1].terminate).toHaveBeenCalledOnce();
    expect(transport.invoke).not.toHaveBeenCalled();
  });
});
