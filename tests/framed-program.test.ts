import { describe, expect, it, vi } from 'vitest';
import type { JointAngles, MotionSample, Pose } from '../src/robot/types';
import {
  captureProgramPointToPath, createProgramPath, createRobotProgram, getProgramPathPoints,
  removeProgramPoint, renameProgramPath, replaceProgramPathPoints, updateProgramPoint,
  type ProgramPoint, type ProgramSnapshot, type RobotProgram,
} from '../src/workspace/program';
import { planProgramRun, sampleProgramRun, type PlannedLeg } from '../src/workspace/program-runner';
import { sanitizeWorkspaceSelection, type WorkspaceSelection } from '../src/workspace/useWorkspace';

const q = (joint1: number): JointAngles => [joint1, 20, -30, 0, 40, 0];
const identity: Pose = { position: [420, 30, 610], orientation: [1, 0, 0, 0] };
function snapshot(offset = 0, role?: ProgramSnapshot['motionRole']): ProgramSnapshot {
  return { pose: { position: [420 + offset, 30, 610], orientation: [1, 0, 0, 0] }, joints: q(offset), configuration: [0, 0, 0, 0], speedMmPerSecond: 50, motionRole: role };
}
function recorded(model = 'irb1100-4-0.58'): RobotProgram {
  const path = createProgramPath(createRobotProgram(model), '加工路径', 'work');
  return captureProgramPointToPath(path.program, path.value.id, { ...snapshot(), configuration: model.startsWith('irb') ? [0, 0, 0, 0] : null }, 'original').program;
}
const settings = { kind: 'grinding' as const, clearanceMm: 3 };

// These tests cover immutable program and playback contracts. Synthetic poses
// intentionally do not claim to validate robot FK, IK or physical clearance.
describe('atomic planned Path replacement', () => {
  it('validates every snapshot before replacing any existing points or consuming their numbers', () => {
    const program = recorded(), before = JSON.stringify(program);
    const invalid = { ...snapshot(5), pose: { ...identity, orientation: [0, 0, 0, 0] as Pose['orientation'] } };
    expect(() => replaceProgramPathPoints(program, 'work', [snapshot(2), invalid, snapshot(8)], settings)).toThrow();
    expect(JSON.stringify(program)).toBe(before);
    expect(getProgramPathPoints(program, 'work').map(point => point.id)).toEqual(['original']);
    expect(captureProgramPointToPath(program, 'work', snapshot(1), 'after').value.pointName).toBe('P002');
  });

  it('replaces only the selected Path and preserves processing/transfer roles and frozen planning settings', () => {
    const otherPath = createProgramPath(recorded(), '另一条路径', 'other');
    const other = captureProgramPointToPath(otherPath.program, 'other', snapshot(2), 'other-point');
    const inputs = [snapshot(4, 'process'), snapshot(5, 'transfer'), snapshot(6, 'process')];
    const mutableSettings = { ...settings };
    const result = replaceProgramPathPoints(other.program, 'work', inputs, mutableSettings);
    const output = getProgramPathPoints(result.program, 'work');
    expect(output).toEqual(result.value);
    expect(output.map(point => point.motionRole)).toEqual(['process', 'transfer', 'process']);
    expect(output.map(point => point.pointName)).toEqual(['P003', 'P004', 'P005']);
    expect(result.program.points.some(point => point.id === 'original')).toBe(false);
    expect(getProgramPathPoints(result.program, 'other')[0]).toBe(other.value);
    expect(result.program.paths.find(path => path.id === 'other')).toBe(other.program.paths.find(path => path.id === 'other'));
    const path = result.program.paths.find(path => path.id === 'work')!;
    expect(path.planning).toEqual(settings);
    expect(Object.isFrozen(path.planning)).toBe(true);
    inputs[0].pose.position[0] = -9000; inputs[0].joints[0] = -9000; mutableSettings.clearanceMm = 999;
    expect(output[0].pose.position[0]).toBe(424);
    expect(output[0].joints[0]).toBe(4);
    expect(path.planning?.clearanceMm).toBe(3);
    expect(other.program.points.some(point => point.id === 'original')).toBe(true);
  });

  it('keeps planning metadata when renaming, editing, appending and removing points', () => {
    const replaced = replaceProgramPathPoints(recorded(), 'work', [snapshot(1, 'process'), snapshot(3, 'transfer')], settings);
    let program = renameProgramPath(replaced.program, 'work', '精磨');
    program = updateProgramPoint(program, replaced.value[0].id, snapshot(2)).program;
    expect(getProgramPathPoints(program, 'work')[0].motionRole).toBe('process');
    program = captureProgramPointToPath(program, 'work', snapshot(5, 'process'), 'extra').program;
    program = removeProgramPoint(program, replaced.value[1].id);
    expect(program.paths.find(path => path.id === 'work')).toMatchObject({ name: '精磨', planning: settings });
    expect(getProgramPathPoints(program, 'work').map(point => point.motionRole)).toEqual(['process', 'process']);
  });

  it.each([
    ['ABB missing configuration', 'irb1100-4-0.58', { ...snapshot(), configuration: null }],
    ['generic robot carrying ABB configuration', 'yaskawa-gp7', snapshot()],
    ['unknown motion role', 'irb1100-4-0.58', { ...snapshot(), motionRole: 'rapid' }],
  ])('rejects %s without modifying the original program', (_label, model, candidate) => {
    const program = recorded(model as string), before = JSON.stringify(program);
    expect(() => replaceProgramPathPoints(program, 'work', [candidate as ProgramSnapshot], settings)).toThrow();
    expect(JSON.stringify(program)).toBe(before);
  });

  it('rejects an invalid clearance, empty result or exceeded point-number budget atomically', () => {
    const program = recorded(), before = JSON.stringify(program);
    for (const clearanceMm of [-1, 1001, Infinity, NaN]) expect(() => replaceProgramPathPoints(program, 'work', [snapshot()], { kind: 'grinding', clearanceMm })).toThrow();
    expect(() => replaceProgramPathPoints(program, 'work', [], settings)).toThrow();
    const full = { ...program, nextPointNumber: 99999 }, saved = JSON.stringify(full);
    expect(() => replaceProgramPathPoints(full, 'work', [snapshot(1), snapshot(2)], settings)).toThrow();
    expect(JSON.stringify(full)).toBe(saved);
    expect(JSON.stringify(program)).toBe(before);
  });
});

function target(id: string, x: number): ProgramPoint {
  return { id, pointName: id, kind: 'MoveL', pathId: 'work', revision: 0, pose: { position: [x, 250, 630], orientation: [Math.SQRT1_2, 0, 0, Math.SQRT1_2] },
    joints: q(90 + x / 100), configuration: [0, 0, 0, 0], speedMmPerSecond: 50 };
}
function planned(from: JointAngles, point: ProgramPoint, joint1: number): PlannedLeg {
  const end = q(joint1), pose = structuredClone(point.pose);
  const sample = (joints: JointAngles, t: number): MotionSample => ({ t, joints: [...joints], pose: structuredClone(pose) });
  return { durationSeconds: 1, path: [sample(from, 0), sample(end, 1)], resolvedTarget: { joints: end, pose } };
}
function run(points: ProgramPoint[], planLeg: (from: JointAngles, point: ProgramPoint) => Promise<PlannedLeg>, loop = false) {
  return planProgramRun({ points, startJoints: q(0), loop, signal: new AbortController().signal, planLeg });
}

describe('running targets resolved from reference frames', () => {
  it('accepts newly solved joints instead of obsolete captured joints and chains every next leg from the actual result', async () => {
    const points = [target('one', 1000), target('two', 1020)], before = JSON.stringify(points);
    const planLeg = vi.fn(async (from: JointAngles, point: ProgramPoint) => planned(from, point, point.id === 'one' ? 7 : 12));
    const result = await run(points, planLeg);
    expect(planLeg.mock.calls.map(([from, point]) => [from[0], point.id])).toEqual([[0, 'one'], [7, 'two']]);
    expect(sampleProgramRun(result, 2).joints).toEqual(q(12));
    expect(JSON.stringify(points)).toBe(before);
    expect(result.initial.legs[0].path.at(-1)?.pose).toEqual(points[0].pose);
  });

  it('does not relax the recorded-joint contract when resolvedTarget is absent', async () => {
    await expect(run([target('one', 1000)], async (from, point) => {
      const { resolvedTarget: _resolved, ...value } = planned(from, point, 7); return value;
    })).rejects.toMatchObject({ name: 'ProgramRunError', pointId: 'one' });
  });

  it.each(['position', 'orientation', 'start joints', 'resolved endpoint joints'] as const)('rejects a mismatched %s even when a resolved target is supplied', async mismatch => {
    await expect(run([target('one', 1000)], async (from, point) => {
      const value = planned(from, point, 7), last = value.path.at(-1)!;
      if (mismatch === 'position') last.pose.position[0] -= 1000;
      else if (mismatch === 'orientation') last.pose.orientation = [1, 0, 0, 0];
      else if (mismatch === 'start joints') value.path[0].joints[0] += 1;
      else value.resolvedTarget!.joints = q(8);
      return value;
    })).rejects.toMatchObject({ name: 'ProgramRunError', pointId: 'one', lineNumber: 1 });
  });

  it('accepts the equivalent negative quaternion for a transformed endpoint', async () => {
    const result = await run([target('one', 1000)], async (from, point) => {
      const value = planned(from, point, 7);
      value.path.at(-1)!.pose.orientation = point.pose.orientation.map(v => -v) as Pose['orientation'];
      return value;
    });
    expect(sampleProgramRun(result, 1).joints).toEqual(q(7));
  });

  it('rejects a scaled quaternion instead of letting a large dot product hide an orientation error', async () => {
    await expect(run([target('one', 1000)], async (from, point) => {
      const value = planned(from, point, 7);
      value.path.at(-1)!.pose.orientation = [2, 0, 0, 0];
      return value;
    })).rejects.toMatchObject({ name: 'ProgramRunError', pointId: 'one' });
  });

  it('closes a loop at the first resolved configuration and does not replay its original approach', async () => {
    const points = [target('one', 1000), target('two', 1020)];
    const result = await run(points, async (from, point) => planned(from, point, point.id === 'one' ? 7 : 12), true);
    expect(result.cycle?.legs.map(leg => leg.pointId)).toEqual(['one', 'two']);
    expect(result.cycle?.legs[0].path.map(sample => sample.joints[0])).toEqual([12, 7]);
    expect(sampleProgramRun(result, 2)).toMatchObject({ joints: q(12), cycle: 1 });
    expect(sampleProgramRun(result, 3)).toMatchObject({ joints: q(7), cycle: 1 });
    expect(sampleProgramRun(result, 4)).toMatchObject({ joints: q(12), cycle: 2 });
  });

  it('rejects loop closure returning to a different IK branch even if Cartesian endpoints match', async () => {
    let calls = 0;
    await expect(run([target('one', 1000), target('two', 1020)], async (from, point) => {
      calls++;
      return planned(from, point, calls === 3 ? 8 : point.id === 'one' ? 7 : 12);
    }, true)).rejects.toMatchObject({ name: 'ProgramRunError', pointId: 'one', closing: true });
  });
});

describe('frame and axis inspector selection scope', () => {
  const scope = { zoneIds: ['zone'], pathIds: ['path'], targetIds: ['point'], frameIds: ['frame'], toolIds: ['tool'] };
  const same = (selection: WorkspaceSelection) => sanitizeWorkspaceSelection(selection, scope, 'a', 'a');
  it('keeps available frames/tools and every valid axis in the same model', () => {
    for (const selection of [{ kind: 'frame', id: 'frame' }, { kind: 'tool', id: 'tool' }, { kind: 'base' }, { kind: 'flange' }] as WorkspaceSelection[]) expect(same(selection)).toEqual(selection);
    for (let index = 0; index < 6; index++) expect(same({ kind: 'joint', index })).toEqual({ kind: 'joint', index });
  });
  it('returns to the robot when a frame/tool disappears or an axis index is invalid', () => {
    for (const selection of [{ kind: 'frame', id: 'deleted' }, { kind: 'tool', id: 'deleted' }, ...[-1, 6, .5, NaN, Infinity].map(index => ({ kind: 'joint', index }))] as WorkspaceSelection[]) expect(same(selection)).toEqual({ kind: 'robot' });
  });
  it('does not leak object selections across robot models while global zones remain selectable', () => {
    for (const selection of [{ kind: 'frame', id: 'frame' }, { kind: 'tool', id: 'tool' }, { kind: 'base' }, { kind: 'flange' }, { kind: 'joint', index: 2 }, { kind: 'path', id: 'path' }, { kind: 'target', id: 'point' }] as WorkspaceSelection[]) expect(sanitizeWorkspaceSelection(selection, scope, 'a', 'b')).toEqual({ kind: 'robot' });
    expect(sanitizeWorkspaceSelection({ kind: 'zone', id: 'zone' }, scope, 'a', 'b')).toEqual({ kind: 'zone', id: 'zone' });
  });
});
