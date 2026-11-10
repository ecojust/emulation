import { describe, expect, it, vi } from 'vitest';
import { planProgramRun, ProgramRunError, sampleJointPositioning, sampleProgramRun, type PlannedLeg } from '../src/workspace/program-runner';
import type { ProgramPoint } from '../src/workspace/program';
import type { JointAngles, MotionSample } from '../src/robot/types';

const joints = (a1: number): JointAngles => [a1, 20, -30, 0, 40, 0];
function sample(q: JointAngles, t: number): MotionSample {
  return { t, joints: [...q], pose: { position: [q[0], 0, 500], orientation: [1, 0, 0, 0] } };
}
function point(id: string, angle: number): ProgramPoint {
  return { id, pointName: id, kind: 'MoveL', pathId: 'path-1', revision: 0, pose: sample(joints(angle), 1).pose,
    joints: joints(angle), configuration: [0, 0, 0, 0], speedMmPerSecond: 100 };
}
// Synthetic callback isolates sequence/timeline mechanics; robot geometry is
// verified in native and browser planner tests, never inferred from this stub.
async function leg(from: JointAngles, target: ProgramPoint): Promise<PlannedLeg> {
  return { durationSeconds: 2, path: [sample(from, 0), sample(target.joints, 1)] };
}
const points = [point('P001', 0), point('P002', 10)];

describe('complete program planning and deterministic playback', () => {
  it('plans in execution order from the actual initial posture and stops at the final point', async () => {
    const planLeg = vi.fn(leg), progress = vi.fn();
    const plan = await planProgramRun({ points, startJoints: joints(-10), loop: false, signal: new AbortController().signal, planLeg, onProgress: progress });
    expect(planLeg.mock.calls.map(([from, target]) => [from[0], target.id])).toEqual([[-10, 'P001'], [0, 'P002']]);
    expect(sampleProgramRun(plan, 0).joints[0]).toBe(-10);
    expect(sampleProgramRun(plan, 1).joints[0]).toBe(-5);
    expect(sampleProgramRun(plan, 3)).toMatchObject({ lineNumber: 2, pointId: 'P002', finished: false });
    expect(sampleProgramRun(plan, 3).joints[0]).toBe(5);
    expect(sampleProgramRun(plan, 4)).toMatchObject({ joints: joints(10), finished: true, progress: 1 });
    expect(sampleProgramRun(plan, 100).joints).toEqual(joints(10));
    expect(progress.mock.calls.at(-1)?.[0]).toMatchObject({ completed: 2, total: 2 });
  });

  it('validates a separate closing leg and never replays the initial approach during loops', async () => {
    const planLeg = vi.fn(leg);
    const plan = await planProgramRun({ points, startJoints: joints(-10), loop: true, signal: new AbortController().signal, planLeg });
    expect(planLeg.mock.calls.map(([from, target]) => [from[0], target.id])).toEqual([[-10, 'P001'], [0, 'P002'], [10, 'P001']]);
    expect(sampleProgramRun(plan, 4)).toMatchObject({ joints: joints(10), cycle: 1, lineNumber: 1, finished: false });
    expect(sampleProgramRun(plan, 5).joints[0]).toBe(5);
    expect(sampleProgramRun(plan, 6).joints[0]).toBe(0);
    expect(sampleProgramRun(plan, 7).joints[0]).toBe(5);
    expect(sampleProgramRun(plan, 8)).toMatchObject({ joints: joints(10), cycle: 2, finished: false });
    for (const boundary of [4, 8, 12, 16]) {
      expect(Math.abs(sampleProgramRun(plan, boundary - 1e-7).joints[0] - sampleProgramRun(plan, boundary).joints[0])).toBeLessThan(1e-5);
    }
  });

  it('reports the failed instruction and rejects an unreachable closing leg before playback', async () => {
    const failed = new Error('不可达');
    await expect(planProgramRun({ points, startJoints: joints(-10), loop: false, signal: new AbortController().signal,
      planLeg: async (from, target) => { if (target.id === 'P002') throw failed; return leg(from, target); },
    })).rejects.toMatchObject({ name: 'ProgramRunError', pointId: 'P002', lineNumber: 2, closing: false });
    let count = 0;
    await expect(planProgramRun({ points, startJoints: joints(-10), loop: true, signal: new AbortController().signal,
      planLeg: async (from, target) => { if (++count === 3) throw failed; return leg(from, target); },
    })).rejects.toMatchObject({ pointId: 'P001', lineNumber: 1, closing: true });
  });

  it('cancels before submission and discards a result arriving after cancellation without scheduling later lines', async () => {
    const controller = new AbortController(), planLeg = vi.fn(leg);
    controller.abort();
    await expect(planProgramRun({ points, startJoints: joints(-10), loop: false, signal: controller.signal, planLeg })).rejects.toMatchObject({ name: 'AbortError' });
    expect(planLeg).not.toHaveBeenCalled();
    const active = new AbortController();
    let resolve!: (value: PlannedLeg) => void;
    const delayed = vi.fn(() => new Promise<PlannedLeg>(done => { resolve = done; }));
    const running = planProgramRun({ points, startJoints: joints(-10), loop: false, signal: active.signal, planLeg: delayed });
    const rejected = expect(running).rejects.toMatchObject({ name: 'AbortError' });
    active.abort(); resolve(await leg(joints(-10), points[0]));
    await rejected;
    expect(delayed).toHaveBeenCalledTimes(1);
  });

  it('rejects a planner that silently changes the recorded joint configuration', async () => {
    await expect(planProgramRun({ points, startJoints: joints(-10), loop: false, signal: new AbortController().signal,
      planLeg: async (from, target) => ({ durationSeconds: 1, path: [sample(from, 0), sample(joints(target.joints[0] + 360), 1)] }),
    })).rejects.toBeInstanceOf(ProgramRunError);
  });

  it('rejects duplicate sample times and an internal joint jump even when endpoints match', async () => {
    for (const path of [
      [sample(joints(-10), 0), sample(joints(-6), .5), sample(joints(-4), .5), sample(joints(0), 1)],
      [sample(joints(-10), 0), sample(joints(30), .5), sample(joints(0), 1)],
    ]) {
      await expect(planProgramRun({ points: [points[0]], startJoints: joints(-10), loop: false, signal: new AbortController().signal,
        planLeg: async () => ({ durationSeconds: 1, path }),
      })).rejects.toBeInstanceOf(ProgramRunError);
    }
  });

  it('keeps a coincident single-point loop finite and separates positioning from linear planning', async () => {
    const plan = await planProgramRun({ points: [points[0]], startJoints: points[0].joints, loop: true, signal: new AbortController().signal,
      planLeg: async (_from, target) => ({ durationSeconds: 0, path: [sample(target.joints, 0)] }),
    });
    expect(plan.cycle!.durationSeconds).toBeGreaterThan(0);
    const frame = sampleProgramRun(plan, 10);
    expect(Number.isFinite(frame.cycle)).toBe(true); expect(frame.joints).toEqual(points[0].joints);
    expect(sampleJointPositioning(joints(-100), joints(100), .5)).toEqual(joints(0));
    expect(sampleJointPositioning(joints(-100), joints(100), 2)).toEqual(joints(100));
    expect(() => sampleJointPositioning(joints(0), joints(1), NaN)).toThrow();
  });
});
