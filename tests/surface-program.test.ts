import { describe, expect, it } from 'vitest';
import {
  appendSurfacePathPoints, assertProgramPathReady, assertSurfacePathProgram, captureProgramPointToPath, clearSurfacePath, createProgramPath,
  createRobotProgram, createSurfaceUvPath, getProgramPathPoints, invalidateSurfacePaths, invalidateSurfaceSolutions, normalizeSurfacePathReferences,
  removeProgramPath, removeProgramPoint, removeSurfacePathPoint, renameProgramPath, resolveSurfacePath, setSurfacePathControl, surfacePathDisabledReason,
  updateProgramPoint, type ProgramSnapshot, type RobotProgram,
} from '../src/workspace/program';
import { createPathModule } from '../src/workspace/path-module';
import { validateSurfacePath, type SurfacePathSample } from '../src/workspace/surface-path';

const settings = { uSpacingMm: 5, vSpacingMm: 5, reverseNormal: false };
const sample = (x = 10, y = 10): SurfacePathSample => ({ pose: { position: [x, y, 0], orientation: [1, 0, 0, 0] },
  pick: { surfaceId: 'mesh', surfaceRevision: 2, triangleIndex: 0, barycentric: [1 - (x + y) / 100, x / 100, y / 100], position: [x, y, 0], normal: [0, 0, 1] } });
const snapshot = (s = sample()): ProgramSnapshot => ({ pose: s.pose, joints: [0, 20, -30, 0, 40, 0], configuration: [0, 0, 0, 0], speedMmPerSecond: 100 });
function region(manual = false): RobotProgram {
  let program = createProgramPath(createRobotProgram('irb1100-4-0.58'), 'Region', 'region').program;
  if (manual) program = captureProgramPointToPath(program, 'region', snapshot(), 'manual').program;
  program = appendSurfacePathPoints(program, 'region', [sample(10, 10), sample(20, 10), sample(20, 20), sample(10, 20)], ['s1', 's2', 's3', 's4']).program;
  for (const id of ['s3', 's1', 's4', 's2']) program = setSurfacePathControl(program, 'region', id, true);
  return program;
}
function generated(solved = false): RobotProgram {
  const program = region(), revision = program.paths[0].surface!.revision, samples = [sample(12, 12), sample(15, 12)];
  return createSurfaceUvPath(program, 'region', revision, samples, settings,
    { id: 'uv', pointIds: ['u1', 'u2'], ...(solved ? { resolvedSnapshots: samples.map(s => snapshot(s)) } : {}) }).program;
}
const uv = (program: RobotProgram) => program.paths.find(path => path.id === 'uv')!;

describe('surface regions keep geometry separate from solved robot instructions', () => {
  it('stores immutable surface targets without inventing joints or robconf and retains pre-existing taught points', () => {
    const program = region(true), path = program.paths[0];
    expect(path.surface!.points.map(point => point.id)).toEqual(['s1', 's2', 's3', 's4']);
    expect(path.surface!.controlPointIds).toEqual(['s1', 's2', 's3', 's4']);
    expect(path.surface!.points[0]).not.toHaveProperty('joints');
    expect(path.surface!.points[0]).not.toHaveProperty('configuration');
    expect(getProgramPathPoints(program, path.id).map(point => point.id)).toEqual(['manual']);
    expect(() => assertProgramPathReady(path)).toThrow('点选区域');
    expect(() => createPathModule(program.modelId, path, program.points, 'rapid-rw6', [])).toThrow('点选区域');
    expect(() => { path.surface!.points[0].pose.position[0] = 999; }).toThrow();
    expect(() => { path.surface!.points[0].pick.barycentric[0] = 0; }).toThrow();
    const renamed = renameProgramPath(program, 'region', 'Another');
    expect(renamed.paths[0].surface).toBe(path.surface);
    expect(clearSurfacePath(program, 'region').paths[0].pointIds).toEqual(['manual']);
  });

  it('atomically validates entire batches and rejects foreign models, duplicate ids and oversized controls', () => {
    const initial = createProgramPath(createRobotProgram('irb1100-4-0.58'), 'R', 'region').program;
    const foreign = sample(); (foreign.pick as { surfaceId: string }).surfaceId = 'other';
    expect(() => appendSurfacePathPoints(initial, 'region', [sample(), foreign])).toThrow('同一模型');
    expect(() => appendSurfacePathPoints(initial, 'region', [sample(), sample()], ['same', 'same'])).toThrow('重复');
    expect(initial.paths[0].surface).toBeUndefined(); expect(initial.points).toEqual([]);
    const before = region(), more = appendSurfacePathPoints(before, 'region', [sample(15, 20)], ['s5']).program;
    expect(() => setSurfacePathControl(more, 'region', 's5', true)).toThrow('四个');
    expect(() => createProgramPath(before, 'Other', 's1')).toThrow('唯一');
    expect(() => captureProgramPointToPath(before, 'region', snapshot())).toThrow('表面区域');
    expect(before.paths[0].surface!.points).toHaveLength(4);
    expect(more.paths[0].surface!.controlPointIds).toEqual(['s1', 's2', 's3', 's4']);
  });

  it('creates a new unresolved UV Path while leaving the source region untouched', () => {
    const program = generated(), path = uv(program);
    expect(path.surface!.kind).toBe('uv'); expect(path.surface!.source).toEqual({ pathId: 'region', revision: 4 });
    expect(path.pointIds).toEqual([]); expect(program.points).toEqual([]);
    expect(program.paths[0].surface!.points).toHaveLength(4);
    expect(surfacePathDisabledReason(path)).toContain('连续逆解');
    expect(() => createPathModule(program.modelId, path, [], 'rapid-rw6', [])).toThrow('连续逆解');
    expect(() => appendSurfacePathPoints(program, 'uv', [sample()])).toThrow('不能作为点选区域');
  });

  it('commits complete solved targets while preserving exact original surface poses and provenance', () => {
    const before = generated(), path = uv(before), target = path.surface!.points[0];
    const snapshots = path.surface!.points.map(point => snapshot(point));
    snapshots[0].pose = { ...snapshots[0].pose, position: [target.pose.position[0] + 1e-8, target.pose.position[1], target.pose.position[2]] };
    snapshots[1].pose = { ...snapshots[1].pose, orientation: [-1, 0, 0, 0] }; // Equivalent solver sign must not rewrite the original hemisphere.
    const next = resolveSurfacePath(before, 'uv', 0, snapshots).program, current = uv(next);
    expect(current.surface!.resolvedRevision).toBe(0); expect(current.pointIds).toHaveLength(2);
    expect(getProgramPathPoints(next, 'uv').map(point => point.pose)).toEqual(path.surface!.points.map(point => point.pose));
    expect(getProgramPathPoints(next, 'uv').every(point => point.motionRole === 'process')).toBe(true);
    expect(() => assertProgramPathReady(current, { paths: next.paths, surfaceRevision: 2 })).not.toThrow();
    expect(() => assertSurfacePathProgram(current, next.points, next.paths)).not.toThrow();
    expect(() => createPathModule(next.modelId, current, next.points, 'rapid-rw6', [])).not.toThrow();
    expect(before.points).toEqual([]); expect(path.surface!.resolvedRevision).toBeUndefined();
    snapshots[0].joints[0] = 999;
    expect(getProgramPathPoints(next, 'uv')[0].joints[0]).toBe(0);
  });

  it('leaves no half-created UV Path after an incomplete, off-surface, transfer or invalid joint solve', () => {
    const before = region(), revision = before.paths[0].surface!.revision, samples = [sample(12, 12), sample(15, 12)];
    const valid = samples.map(s => snapshot(s));
    const badCases = [valid.slice(0, 1), [{ ...valid[0], pose: sample(50, 50).pose }, valid[1]], [{ ...valid[0], motionRole: 'transfer' as const }, valid[1]], [{ ...valid[0], configuration: null }, valid[1]]];
    for (const resolvedSnapshots of badCases) expect(() => createSurfaceUvPath(before, 'region', revision, samples, settings, { id: 'uv', resolvedSnapshots })).toThrow();
    expect(before.paths.map(path => path.id)).toEqual(['region']); expect(before.points).toEqual([]); expect(before.nextPointNumber).toBe(1);
    expect(() => createSurfaceUvPath(before, 'region', revision - 1, samples, settings)).toThrow('已改变');
    expect(() => createSurfaceUvPath(before, 'region', revision, Array.from({ length: 257 }, () => sample()), settings)).toThrow('256');
  });

  it('permits a candidate joint update only when it preserves the exact surface target pose', () => {
    const program = generated(true), point = getProgramPathPoints(program, 'uv')[0];
    const alternate = { ...point, joints: [5, 20, -30, 0, 40, 0] as ProgramSnapshot['joints'] };
    expect(updateProgramPoint(program, point.id, alternate).value.joints[0]).toBe(5);
    expect(() => updateProgramPoint(program, point.id, { ...alternate, pose: sample(30, 30).pose })).toThrow('位置和法向');
    expect(() => updateProgramPoint(program, point.id, { ...alternate, motionRole: 'transfer' })).toThrow('位置和法向');
  });
});

describe('surface source changes invalidate entire dependent solutions', () => {
  it('clears generated robot points when controls, boundary points or their source Path change', () => {
    const original = generated(true);
    const changes = [setSurfacePathControl(original, 'region', 's1', false), removeSurfacePathPoint(original, 'region', 's1'),
      appendSurfacePathPoints(original, 'region', [sample(10, 15)], ['extra']).program, removeProgramPath(original, 'region')];
    for (const program of changes) {
      expect(uv(program).surface!.invalidated).toBe(true); expect(uv(program).pointIds).toEqual([]);
      expect(program.points).toEqual([]); expect(uv(program).surface!.points).toHaveLength(2);
      expect(() => assertProgramPathReady(uv(program), { paths: program.paths })).toThrow();
    }
    expect(uv(original).surface!.resolvedRevision).toBe(0); expect(original.points).toHaveLength(2);
  });

  it('separates robot-frame invalidation from missing/modified mesh invalidation', () => {
    const original = generated(true), onlyJoints = invalidateSurfaceSolutions(original);
    expect(onlyJoints.paths[0].surface!.invalidated).toBeUndefined(); expect(uv(onlyJoints).surface!.invalidated).toBeUndefined();
    expect(uv(onlyJoints).surface!.resolvedRevision).toBeUndefined(); expect(onlyJoints.points).toEqual([]);
    const noMesh = invalidateSurfacePaths(original, 'mesh');
    expect(noMesh.paths.every(path => path.surface!.invalidated)).toBe(true); expect(noMesh.points).toEqual([]);
    expect(invalidateSurfacePaths(noMesh, 'mesh')).toBe(noMesh);
    expect(invalidateSurfacePaths(original, 'other')).toBe(original);
    expect(surfacePathDisabledReason(uv(original), { surfaceRevision: null })).toContain('已改变');
    expect(surfacePathDisabledReason(uv(original), { surfaceRevision: 3 })).toContain('已改变');
  });

  it('preserves stale geometry when loading deleted models or missing/edited source regions', () => {
    const original = generated(true);
    for (const surfaces of [[], [{ id: 'mesh', revision: 3 }]]) {
      const loaded = normalizeSurfacePathReferences(original, surfaces);
      expect(loaded.points).toEqual([]); expect(uv(loaded).surface!.points).toHaveLength(2); expect(uv(loaded).surface!.invalidated).toBe(true);
    }
    const missingSource = { ...original, paths: original.paths.filter(path => path.id !== 'region') };
    const loaded = normalizeSurfacePathReferences(missingSource, [{ id: 'mesh', revision: 2 }]);
    expect(loaded.points).toEqual([]); expect(uv(loaded).surface!.invalidated).toBe(true);
    expect(normalizeSurfacePathReferences(original, [{ id: 'mesh', revision: 2 }])).toBe(original);
  });

  it('deleting a solved UV target clears its solution and removes the matching geometric target', () => {
    const original = generated(true), first = getProgramPathPoints(original, 'uv')[0];
    const next = removeProgramPoint(original, first.id);
    expect(next.points).toEqual([]); expect(uv(next).surface!.points.map(point => point.id)).toEqual(['u2']);
    expect(uv(next).surface!.invalidated).toBe(true);
    const cleared = removeSurfacePathPoint(next, 'uv', 'u2');
    expect(uv(cleared).surface).toBeUndefined(); expect(uv(cleared).pointIds).toEqual([]);
  });
});

describe('surface path validation rejects malformed persisted metadata', () => {
  it('validates controls, barycentric coordinates, references and complete solved point associations', () => {
    const program = generated(true), path = uv(program), data = path.surface!;
    expect(() => validateSurfacePath({ ...program.paths[0].surface!, controlPointIds: ['s2', 's1'] })).toThrow('顺序');
    expect(() => validateSurfacePath({ ...data, points: [{ ...data.points[0], pick: { ...data.points[0].pick, barycentric: [2, 0, -1] } }] })).toThrow('重心');
    expect(() => validateSurfacePath({ ...data, resolvedRevision: 10 })).toThrow('修订');
    expect(() => validateSurfacePath({ ...data, settings: { ...settings, uSpacingMm: 0 } })).toThrow('间距');
    expect(() => assertSurfacePathProgram({ ...path, pointIds: path.pointIds.slice(0, 1) }, program.points, program.paths)).toThrow('完整对应');
    expect(() => assertSurfacePathProgram({ ...path, surface: { ...data, resolvedRevision: undefined } }, program.points, program.paths)).toThrow('不能保留');
    expect(() => assertSurfacePathProgram(path, program.points, [...program.paths, { id: 'duplicate', name: 'D', pointIds: [], surface: program.paths[0].surface }])).toThrow('标识重复');
  });

  it('allows reversed tool Z without flipping the stored original surface normal', () => {
    const original = region(), reversed = sample(12, 12);
    reversed.pose.orientation = [0, 1, 0, 0];
    const result = createSurfaceUvPath(original, 'region', original.paths[0].surface!.revision, [reversed], { ...settings, reverseNormal: true });
    expect(result.value.surface!.points[0].pick.normal).toEqual([0, 0, 1]);
    expect(result.value.surface!.points[0].pose.orientation).toEqual([0, 1, 0, 0]);
  });
});
