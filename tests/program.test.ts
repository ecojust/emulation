import { describe, expect, it } from 'vitest';
import {
  captureProgramPointToPath, createProgramPath, createProgramStore, createRobotProgram, getProgramPathPoints,
  removeProgramPath, removeProgramPoint, renameProgramPath, replaceRobotProgram, resolveProgramPathPoints, updateProgramPoint,
  type ProgramSnapshot, type RobotProgram,
} from '../src/workspace/program';
import { sanitizeWorkspaceSelection } from '../src/workspace/useWorkspace';
import type { RobConf } from '../src/robot/types';
import { PROGRAM_MODEL_IDS } from '../src/workspace/program-templates';

const snapshot = (): ProgramSnapshot & { configuration: RobConf } => ({ pose: { position: [400, 0, 350], orientation: [1, 0, 0, 0] }, joints: [0, 20, -30, 0, 40, 0], configuration: [0, 0, 0, 0], speedMmPerSecond: 100 });
function points(count: number): RobotProgram {
  let program = createProgramPath(createRobotProgram('irb1100-4-0.58'), undefined, 'path-1').program;
  for (let i = 1; i <= count; i++) program = captureProgramPointToPath(program, 'path-1', snapshot(), `point-${i}`).program;
  return program;
}

describe('Path-first immutable pose capture', () => {
  it('creates and retains an empty Path without inventing a current pose', () => {
    const empty = createRobotProgram('irb1100-4-0.58');
    const first = createProgramPath(empty, undefined, 'path-1');
    expect(first.value).toEqual({ id: 'path-1', name: 'Path 1', pointIds: [] });
    expect(first.program.points).toEqual([]);
    expect(first.program.nextPointNumber).toBe(1);
    expect(getProgramPathPoints(first.program, first.value.id)).toEqual([]);
    expect(empty.paths).toHaveLength(0);
    expect(Object.isFrozen(first.value)).toBe(true);
    expect(Object.isFrozen(first.value.pointIds)).toBe(true);
    const removed = removeProgramPath(first.program, first.value.id);
    const second = createProgramPath(removed, undefined, 'path-2');
    expect(second.value.name).toBe('Path 2');
    expect(second.program.nextPathNumber).toBe(3);
  });

  it('appends snapshots only to the requested Path and preserves each independent order', () => {
    let program = createProgramPath(points(1), 'Other', 'path-2').program;
    program = captureProgramPointToPath(program, 'path-2', snapshot(), 'point-2').program;
    const before = program;
    program = captureProgramPointToPath(program, 'path-1', snapshot(), 'point-3').program;
    expect(program.points.map(point => [point.id, point.pathId])).toEqual([
      ['point-1', 'path-1'], ['point-2', 'path-2'], ['point-3', 'path-1'],
    ]);
    expect(getProgramPathPoints(program, 'path-1').map(point => point.id)).toEqual(['point-1', 'point-3']);
    expect(getProgramPathPoints(program, 'path-2').map(point => point.id)).toEqual(['point-2']);
    expect(before.paths[0].pointIds).toEqual(['point-1']);
    expect(program.paths[1]).toBe(before.paths[1]);
    expect(program.revision).toBe(before.revision + 1);
    const removed = removeProgramPoint(program, 'point-2');
    const next = captureProgramPointToPath(removed, 'path-2', snapshot(), 'point-4');
    expect(next.value.pointName).toBe('P004');
    expect(next.program.paths[1].pointIds).toEqual(['point-4']);
  });

  it('copies and freezes all caller-owned pose, joints and configuration tuples', () => {
    const input = snapshot();
    const { program, value } = captureProgramPointToPath(points(0), 'path-1', input, 'point-1');
    input.pose.position[0] = -999; input.pose.orientation[0] = 0;
    input.joints[0] = 30; input.configuration[0] = -1; input.speedMmPerSecond = 200;
    expect(value.pose.position).toEqual([400, 0, 350]);
    expect(value.pose.orientation).toEqual([1, 0, 0, 0]);
    expect(value.joints[0]).toBe(0); expect(value.configuration![0]).toBe(0); expect(value.speedMmPerSecond).toBe(100);
    expect(value.pathId).toBe('path-1');
    expect(() => { value.pose.position[0] = 999; }).toThrow();
    expect(() => { value.joints[0] = 999; }).toThrow();
    expect(Object.isFrozen(program.points)).toBe(true);
    expect(Object.isFrozen(getProgramPathPoints(program, 'path-1'))).toBe(true);
  });

  it('updates a snapshot without changing identity or Path membership', () => {
    const original = points(2), input = snapshot();
    input.pose.position = [450, 20, 300]; input.speedMmPerSecond = 200;
    const changed = updateProgramPoint(original, 'point-1', input);
    expect(changed.value.id).toBe('point-1'); expect(changed.value.pointName).toBe('P001');
    expect(changed.value.pathId).toBe('path-1'); expect(changed.value.revision).toBe(1);
    expect(changed.program.revision).toBe(original.revision + 1);
    expect(changed.program.points.map(point => point.id)).toEqual(original.points.map(point => point.id));
    expect(changed.program.paths).toBe(original.paths);
    expect(original.points[0].pose.position).toEqual([400, 0, 350]);
    input.pose.position[0] = 800;
    expect(changed.value.pose.position).toEqual([450, 20, 300]);
  });

  it('isolates all model projects and uses each brand configuration convention', () => {
    const original = createProgramStore();
    expect(Object.keys(original)).toEqual(PROGRAM_MODEL_IDS);
    let store = original;
    for (const modelId of PROGRAM_MODEL_IDS) {
      const abb = modelId.startsWith('irb1100-');
      const input: ProgramSnapshot = { ...snapshot(), configuration: abb ? [0, 0, 0, 0] : null };
      const path = createProgramPath(store[modelId], undefined, `${modelId}-path`);
      const captured = captureProgramPointToPath(path.program, path.value.id, input, `${modelId}-point`);
      store = replaceRobotProgram(store, captured.program);
      expect(captured.value.configuration).toEqual(abb ? [0, 0, 0, 0] : null);
      expect(captured.value.pointName).toBe('P001');
      expect(Object.isFrozen(captured.value.pose)).toBe(true);
      input.joints[0] = 999;
      expect(captured.value.joints[0]).toBe(0);
      if (!abb) expect(() => captureProgramPointToPath(store[modelId], path.value.id, snapshot())).toThrow('非 ABB');
    }
    for (const program of Object.values(original)) { expect(program.points).toHaveLength(0); expect(program.paths).toHaveLength(0); }
    const updated = updateProgramPoint(store['yaskawa-gp7'], 'yaskawa-gp7-point', { ...snapshot(), configuration: null });
    expect(updated.value.revision).toBe(1);
    expect(store['yaskawa-gp110'].points[0].revision).toBe(0);
    expect(() => createRobotProgram('kuka-kr6')).toThrow('目录');
  });

  it.each([
    ['missing robconf', (input: ProgramSnapshot) => { input.configuration = null; }],
    ['fractional robconf', (input: ProgramSnapshot) => { input.configuration![0] = .5; }],
    ['invalid branch', (input: ProgramSnapshot) => { input.configuration![3] = 8; }],
    ['invalid quaternion', (input: ProgramSnapshot) => { input.pose.orientation = [0, 0, 0, 0]; }],
    ['nonfinite joints', (input: ProgramSnapshot) => { input.joints[2] = NaN; }],
    ['nonpositive speed', (input: ProgramSnapshot) => { input.speedMmPerSecond = 0; }],
    ['speed below controller range', (input: ProgramSnapshot) => { input.speedMmPerSecond = .1; }],
    ['speed above controller range', (input: ProgramSnapshot) => { input.speedMmPerSecond = 2001; }],
  ])('rejects %s without appending a point or consuming a number', (_label, mutate) => {
    const original = points(1), input = snapshot(); mutate(input);
    expect(() => captureProgramPointToPath(original, 'path-1', input, 'bad')).toThrow();
    expect(original.points).toHaveLength(1); expect(original.paths[0].pointIds).toEqual(['point-1']); expect(original.nextPointNumber).toBe(2);
  });
  it('accepts both supported speed boundaries and rejects exhausted target numbers', () => {
    for (const speed of [1, 2000]) {
      const input = snapshot(); input.speedMmPerSecond = speed;
      expect(captureProgramPointToPath(points(0), 'path-1', input).value.speedMmPerSecond).toBe(speed);
    }
    expect(() => captureProgramPointToPath({ ...points(0), nextPointNumber: 100000 }, 'path-1', snapshot())).toThrow('编号');
  });
});

describe('Path ownership, deletion and lookup', () => {
  it('keeps an empty Path after its last point is deleted', () => {
    const original = points(1), removed = removeProgramPoint(original, 'point-1');
    expect(removed.points).toEqual([]);
    expect(removed.paths).toEqual([{ id: 'path-1', name: 'Path 1', pointIds: [] }]);
    expect(original.paths[0].pointIds).toEqual(['point-1']);
    expect(removeProgramPoint(removed, 'missing')).toBe(removed);
  });

  it('deletes only a Path and its own points, preserving other Path snapshots', () => {
    let program = createProgramPath(points(2), 'Other', 'path-2').program;
    program = captureProgramPointToPath(program, 'path-2', snapshot(), 'other-point').program;
    const removed = removeProgramPath(program, 'path-1');
    expect(removed.paths).toEqual([program.paths[1]]);
    expect(removed.points).toEqual([program.points[2]]);
    expect(removed.points[0]).toBe(program.points[2]);
    expect(program.paths).toHaveLength(2); expect(program.points).toHaveLength(3);
    expect(removeProgramPath(removed, 'missing')).toBe(removed);
    expect(captureProgramPointToPath(removed, 'path-2', snapshot()).value.pointName).toBe('P004');
  });

  it('resolves the explicit path order even when the overall pose collection differs', () => {
    const program = points(3), path = { ...program.paths[0], pointIds: ['point-3', 'point-1'] };
    expect(resolveProgramPathPoints(path, program.points).map(point => point.id)).toEqual(['point-3', 'point-1']);
    expect(program.paths[0].pointIds).toEqual(['point-1', 'point-2', 'point-3']);
  });

  it('fails closed for duplicate, missing or foreign point references', () => {
    const program = points(2), path = program.paths[0];
    expect(() => resolveProgramPathPoints({ ...path, pointIds: ['point-1', 'point-1'] }, program.points)).toThrow('重复');
    expect(() => resolveProgramPathPoints({ ...path, pointIds: ['deleted'] }, program.points)).toThrow('已删除');
    expect(() => resolveProgramPathPoints({ ...path, id: 'foreign' }, program.points)).toThrow('不属于');
    expect(() => resolveProgramPathPoints(path, [...program.points, program.points[0]])).toThrow('标识重复');
    expect(() => getProgramPathPoints(program, 'deleted')).toThrow('Path');
  });

  it('renames metadata without changing geometry and rejects missing or duplicate object IDs', () => {
    const original = points(2), renamed = renameProgramPath(original, 'path-1', '  Station B  ');
    expect(renamed.paths[0].name).toBe('Station B'); expect(renamed.points).toBe(original.points);
    expect(renameProgramPath(renamed, 'path-1', 'Station B')).toBe(renamed);
    expect(() => renameProgramPath(renamed, 'path-1', ' ')).toThrow();
    expect(() => renameProgramPath(renamed, 'path-1', 'x'.repeat(81))).toThrow();
    expect(() => renameProgramPath(renamed, 'deleted', 'Name')).toThrow();
    expect(() => captureProgramPointToPath(original, 'removed', snapshot(), 'new')).toThrow('Path');
    expect(() => captureProgramPointToPath(original, 'path-1', snapshot(), 'point-1')).toThrow('唯一');
    expect(() => captureProgramPointToPath(original, 'path-1', snapshot(), 'path-1')).toThrow('唯一');
    expect(() => createProgramPath(original, 'B', 'point-1')).toThrow('唯一');
    expect(() => createProgramPath(original, 'B', 'path-1')).toThrow('唯一');
    expect(() => updateProgramPoint(original, 'removed', snapshot())).toThrow();
  });
});

describe('workspace selection follows current object scope', () => {
  const scope = { zoneIds: ['zone'], pathIds: ['path'], targetIds: ['point'] };
  it('keeps valid objects and clears deleted or foreign references', () => {
    expect(sanitizeWorkspaceSelection({ kind: 'zone', id: 'zone' }, scope)).toEqual({ kind: 'zone', id: 'zone' });
    expect(sanitizeWorkspaceSelection({ kind: 'path', id: 'path' }, scope)).toEqual({ kind: 'path', id: 'path' });
    expect(sanitizeWorkspaceSelection({ kind: 'target', id: 'point' }, scope)).toEqual({ kind: 'target', id: 'point' });
    for (const kind of ['zone', 'path', 'target'] as const) expect(sanitizeWorkspaceSelection({ kind, id: 'deleted' }, scope)).toEqual({ kind: 'robot' });
  });
  it('clears model-specific selection immediately on model change, even for matching IDs', () => {
    expect(sanitizeWorkspaceSelection({ kind: 'target', id: 'point' }, scope, '580', '475')).toEqual({ kind: 'robot' });
    expect(sanitizeWorkspaceSelection({ kind: 'path', id: 'path' }, scope, '580', 'generic')).toEqual({ kind: 'robot' });
    expect(sanitizeWorkspaceSelection({ kind: 'zone', id: 'zone' }, scope, '580', '475')).toEqual({ kind: 'zone', id: 'zone' });
  });
});
