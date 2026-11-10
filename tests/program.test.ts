import { describe, expect, it } from 'vitest';
import {
  captureProgramPoint, convertProgramPointsToPath, createProgramStore, createRobotProgram,
  removeProgramPath, removeProgramPoint, renameProgramPath, replaceRobotProgram, updateProgramPoint,
  type ProgramSnapshot, type RobotProgram,
} from '../src/workspace/program';
import { sanitizeWorkspaceSelection } from '../src/workspace/useWorkspace';
import type { RobConf } from '../src/robot/types';
import { PROGRAM_MODEL_IDS } from '../src/workspace/program-templates';

const snapshot = (): ProgramSnapshot & { configuration: RobConf } => ({ pose: { position: [400, 0, 350], orientation: [1, 0, 0, 0] }, joints: [0, 20, -30, 0, 40, 0], configuration: [0, 0, 0, 0], speedMmPerSecond: 100 });
function points(count: number): RobotProgram {
  let program = createRobotProgram('irb1100-4-0.58');
  for (let i = 1; i <= count; i++) program = captureProgramPoint(program, snapshot(), null, `point-${i}`).program;
  return program;
}

describe('immutable per-model MoveL programs', () => {
  it('appends and inserts after a row without renumbering saved points', () => {
    const original = points(2);
    const inserted = captureProgramPoint(original, snapshot(), 'point-1', 'point-3').program;
    expect(inserted.points.map(point => point.id)).toEqual(['point-1', 'point-3', 'point-2']);
    expect(inserted.points.map(point => point.pointName)).toEqual(['P001', 'P003', 'P002']);
    expect(original.points.map(point => point.id)).toEqual(['point-1', 'point-2']);
    const removed = removeProgramPoint(inserted, 'point-2');
    expect(captureProgramPoint(removed, snapshot(), null, 'point-4').value.pointName).toBe('P004');
    expect(inserted.revision).toBe(original.revision + 1);
  });

  it('copies and freezes all caller-owned pose, joints and configuration tuples', () => {
    const input = snapshot();
    const { program, value } = captureProgramPoint(createRobotProgram('irb1100-4-0.58'), input, null, 'point-1');
    input.pose.position[0] = -999; input.pose.orientation[0] = 0;
    input.joints[0] = 30; input.configuration[0] = -1; input.speedMmPerSecond = 200;
    expect(value.pose.position).toEqual([400, 0, 350]);
    expect(value.pose.orientation).toEqual([1, 0, 0, 0]);
    expect(value.joints[0]).toBe(0); expect(value.configuration![0]).toBe(0); expect(value.speedMmPerSecond).toBe(100);
    expect(() => { value.pose.position[0] = 999; }).toThrow();
    expect(() => { value.joints[0] = 999; }).toThrow();
    expect(Object.isFrozen(program.points)).toBe(true);
  });

  it('updates a target snapshot while preserving identity, row order and Path references', () => {
    const original = convertProgramPointsToPath(points(2), ['point-1', 'point-2'], 'Weld', 'path-1').program;
    const input = snapshot(); input.pose.position = [450, 20, 300]; input.speedMmPerSecond = 200;
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

  it('keeps model projects and revisions independent across model switching', () => {
    const originalStore = createProgramStore();
    const first = captureProgramPoint(originalStore['irb1100-4-0.58'], snapshot(), null, 'long-point').program;
    const one = replaceRobotProgram(originalStore, first);
    const second = captureProgramPoint(one['irb1100-4-0.475'], snapshot(), null, 'short-point').program;
    const both = replaceRobotProgram(one, second);
    expect(both['irb1100-4-0.58']).toBe(first);
    expect(both['irb1100-4-0.475']).toBe(second);
    expect(originalStore['irb1100-4-0.58'].points).toHaveLength(0);
    expect(one['irb1100-4-0.475'].points).toHaveLength(0);
    expect(second.points[0].pointName).toBe('P001');
    expect(() => createRobotProgram('kuka-kr6')).toThrow('目录');
  });

  it('isolates all catalog projects and captures generic poses without fake ABB metadata', () => {
    const original = createProgramStore();
    expect(Object.keys(original)).toEqual(PROGRAM_MODEL_IDS);
    let store = original;
    for (const modelId of PROGRAM_MODEL_IDS.filter(id => !id.startsWith('irb1100-'))) {
      const input: ProgramSnapshot = { ...snapshot(), configuration: null };
      const captured = captureProgramPoint(store[modelId], input, null, `${modelId}-point`);
      store = replaceRobotProgram(store, captured.program);
      expect(captured.value.configuration).toBeNull();
      expect(captured.value.pointName).toBe('P001');
      expect(Object.isFrozen(captured.value.pose)).toBe(true);
      expect(Object.isFrozen(captured.value.joints)).toBe(true);
      input.joints[0] = 999;
      expect(captured.value.joints[0]).toBe(0);
      expect(() => captureProgramPoint(store[modelId], snapshot())).toThrow('非 ABB');
    }
    expect(store['irb1100-4-0.58']).toBe(original['irb1100-4-0.58']);
    for (const program of Object.values(original)) expect(program.points).toHaveLength(0);
    const updated = updateProgramPoint(store['yaskawa-gp7'], 'yaskawa-gp7-point', { ...snapshot(), configuration: null });
    expect(updated.value.revision).toBe(1);
    expect(updated.value.configuration).toBeNull();
    expect(store['yaskawa-gp110'].points[0].revision).toBe(0);
  });

  it.each([
    ['missing robconf', (input: ProgramSnapshot) => { input.configuration = null as unknown as ProgramSnapshot['configuration']; }],
    ['fractional robconf', (input: ProgramSnapshot) => { input.configuration![0] = .5; }],
    ['invalid branch', (input: ProgramSnapshot) => { input.configuration![3] = 8; }],
    ['invalid quaternion', (input: ProgramSnapshot) => { input.pose.orientation = [0, 0, 0, 0]; }],
    ['nonfinite joints', (input: ProgramSnapshot) => { input.joints[2] = NaN; }],
    ['nonpositive speed', (input: ProgramSnapshot) => { input.speedMmPerSecond = 0; }],
    ['speed below controller range', (input: ProgramSnapshot) => { input.speedMmPerSecond = .1; }],
    ['speed above controller range', (input: ProgramSnapshot) => { input.speedMmPerSecond = 2001; }],
  ])('rejects %s without producing a saved instruction', (_label, mutate) => {
    const original = points(1), input = snapshot(); mutate(input);
    expect(() => captureProgramPoint(original, input, null, 'bad')).toThrow();
    expect(original.points).toHaveLength(1); expect(original.nextPointNumber).toBe(2);
  });
  it('accepts both supported speed boundaries without changing the captured value', () => {
    for (const speed of [1, 2000]) {
      const input = snapshot(); input.speedMmPerSecond = speed;
      expect(captureProgramPoint(createRobotProgram('irb1100-4-0.58'), input).value.speedMmPerSecond).toBe(speed);
    }
  });
});

describe('Path membership preserves execution order', () => {
  it('groups nonconsecutive selection in row order without moving unselected instructions', () => {
    const original = points(4);
    const changed = convertProgramPointsToPath(original, ['point-4', 'point-2', 'point-4'], undefined, 'path-1');
    expect(changed.value.pointIds).toEqual(['point-2', 'point-4']);
    expect(changed.value.name).toBe('Path 1');
    expect(changed.program.points.map(point => point.id)).toEqual(original.points.map(point => point.id));
    expect(changed.program.points.map(point => point.pathId)).toEqual([null, 'path-1', null, 'path-1']);
    expect(original.points.every(point => point.pathId === null)).toBe(true);
    expect(Object.isFrozen(changed.value.pointIds)).toBe(true);
  });

  it('reassigns points atomically and removes only emptied previous Paths', () => {
    let program = convertProgramPointsToPath(points(4), ['point-1', 'point-2'], 'A', 'path-a').program;
    program = convertProgramPointsToPath(program, ['point-3', 'point-4'], 'B', 'path-b').program;
    const changed = convertProgramPointsToPath(program, ['point-4', 'point-2'], 'C', 'path-c').program;
    expect(changed.paths.map(path => [path.id, path.pointIds])).toEqual([
      ['path-a', ['point-1']], ['path-b', ['point-3']], ['path-c', ['point-2', 'point-4']],
    ]);
    const replaced = convertProgramPointsToPath(changed, ['point-1'], 'D', 'path-d').program;
    expect(replaced.paths.map(path => path.id)).toEqual(['path-b', 'path-c', 'path-d']);
    expect(replaced.points.map(point => point.pathId)).toEqual(['path-d', 'path-c', 'path-b', 'path-c']);
    expect(program.paths[0].pointIds).toEqual(['point-1', 'point-2']);
  });

  it('deleting a point removes dangling references and deleting a Path keeps its instructions', () => {
    const grouped = convertProgramPointsToPath(points(3), ['point-1', 'point-3'], 'A', 'path-a').program;
    const one = removeProgramPoint(grouped, 'point-1');
    expect(one.paths[0].pointIds).toEqual(['point-3']);
    const none = removeProgramPoint(one, 'point-3');
    expect(none.paths).toEqual([]); expect(none.points.map(point => point.id)).toEqual(['point-2']);
    const detached = removeProgramPath(grouped, 'path-a');
    expect(detached.points.map(point => point.id)).toEqual(grouped.points.map(point => point.id));
    expect(detached.points.every(point => point.pathId === null)).toBe(true);
    expect(detached.paths).toEqual([]);
    expect(removeProgramPoint(detached, 'missing')).toBe(detached);
    expect(removeProgramPath(detached, 'missing')).toBe(detached);
  });

  it('renames metadata without touching geometry and rejects stale or duplicate IDs', () => {
    const grouped = convertProgramPointsToPath(points(2), ['point-1'], 'A', 'path-a').program;
    const renamed = renameProgramPath(grouped, 'path-a', '  Station B  ');
    expect(renamed.paths[0].name).toBe('Station B'); expect(renamed.points).toBe(grouped.points);
    expect(renameProgramPath(renamed, 'path-a', 'Station B')).toBe(renamed);
    expect(() => renameProgramPath(renamed, 'path-a', ' ')).toThrow();
    expect(() => captureProgramPoint(grouped, snapshot(), 'removed', 'new')).toThrow();
    expect(() => captureProgramPoint(grouped, snapshot(), null, 'point-1')).toThrow();
    expect(() => convertProgramPointsToPath(grouped, ['point-1', 'removed'], 'B', 'path-b')).toThrow();
    expect(() => convertProgramPointsToPath(grouped, [], 'B', 'path-b')).toThrow();
    expect(() => updateProgramPoint(grouped, 'removed', snapshot())).toThrow();
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
