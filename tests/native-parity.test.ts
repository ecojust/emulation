import { execFileSync } from 'node:child_process';
import { existsSync } from 'node:fs';
import { resolve } from 'node:path';
import { describe, expect, it } from 'vitest';
import { forwardKinematics, inverseKinematics, quaternionDistanceDeg } from '../src/robot/kinematics';
import { getConfiguration } from '../src/robot/configuration';
import { ROBOT_MODELS } from '../src/robot/model';
import { planMoveL } from '../src/robot/planner';
import type { FKResult, IKSolution, JointAngles, PlanRequest, PlanResult, Pose, RobotModel } from '../src/robot/types';

const nativePath = process.env.EMULATION_NATIVE_CORE || resolve(process.env.CARGO_TARGET_DIR || 'target', 'debug', process.platform === 'win32' ? 'core-cli.exe' : 'core-cli');
function native<T>(commands: unknown[]): T[] {
  if (!existsSync(nativePath)) throw new Error(`Native parity requires the actual Rust executable. Run cargo build -p emulation-core --bin core-cli first. Expected: ${nativePath}`);
  const output = execFileSync(nativePath, { input: commands.map(c => JSON.stringify(c)).join('\n') + '\n', encoding: 'utf8', maxBuffer: 24 * 1024 * 1024, timeout: 30_000 });
  const responses = output.trim().split('\n').map(line => JSON.parse(line));
  expect(responses).toHaveLength(commands.length);
  return responses.map(response => { expect(response.error).toBeUndefined(); return response.result as T; });
}
function closeVector(actual: number[], expected: number[], tolerance = 1e-7) {
  expect(actual).toHaveLength(expected.length);
  actual.forEach((value, index) => expect(Math.abs(value - expected[index])).toBeLessThanOrEqual(tolerance));
}
function closePose(actual: Pose, expected: Pose) {
  closeVector(actual.position, expected.position, 1e-6);
  expect(quaternionDistanceDeg(actual.orientation, expected.orientation)).toBeLessThan(1e-4);
}
function samples(model: RobotModel): JointAngles[] {
  let state = 287431;
  const random = () => ((state = (Math.imul(state, 1664525) + 1013904223) >>> 0) / 2 ** 32);
  return Array.from({ length: 40 }, () => model.limits.map(limit => limit.minDeg + 2 + random() * (limit.maxDeg - limit.minDeg - 4)) as JointAngles);
}

describe.each(Object.values(ROBOT_MODELS))('$name native Rust compatibility', model => {
  it('matches independent official zero coordinates and the full FK chain across joint ranges', () => {
    const joints = [model.homeJoints, model.readyJoints, ...samples(model)];
    const results = native<FKResult>(joints.map(j => ({ op: 'fk', modelId: model.id, joints: j })));
    closeVector(results[0].pose.position, model.id.endsWith('0.58') ? [364, 0, 617] : [314, 0, 562]);
    results.forEach((actual, i) => {
      const reference = forwardKinematics(model, joints[i]);
      closePose(actual.pose, reference.pose);
      expect(actual.frames).toHaveLength(7);
      actual.frames.forEach((frame, j) => closeVector(frame, reference.frames[j]));
      closeVector(actual.shoulder, reference.shoulder);
      closeVector(actual.elbow, reference.elbow);
      closeVector(actual.wristCenter, reference.wristCenter);
      actual.jointAxes.forEach((axis, j) => closeVector(axis, reference.jointAxes[j]));
    });
  });

  it('preserves all physical inverse branches, configurations and full turns without relying on result order or string IDs', () => {
    const seeds = [model.readyJoints, [20, 20, -40, 25, 40, 370] as JointAngles, ...samples(model)];
    const poses = seeds.map(seed => forwardKinematics(model, seed).pose);
    const all = native<IKSolution[]>(poses.map((pose, i) => ({ op: 'ik', modelId: model.id, pose, seed: seeds[i] })));
    all.forEach((actual, index) => {
      const expected = inverseKinematics(model, poses[index], seeds[index]);
      expect(actual.length, `Branch count at ${seeds[index]}`).toBe(expected.length);
      for (const solution of actual) {
        const match = expected.find(candidate => candidate.joints.every((q, i) => Math.abs(q - solution.joints[i]) < 1e-5));
        expect(match, `Unmatched native joints ${solution.joints}`).toBeDefined();
        expect(solution.configuration).toEqual(match!.configuration);
        closePose(solution.pose, poses[index]);
        expect(solution.positionErrorMm).toBeLessThan(0.01);
        expect(solution.orientationErrorDeg).toBeLessThan(0.001);
      }
    });
  });

  it('matches successful translations, automatic orientation, pure rotation and A6 full-turn continuation', () => {
    const ready = forwardKinematics(model, model.readyJoints).pose;
    const turned: JointAngles = [...model.readyJoints]; turned[5] += 60;
    const wound: JointAngles = [...model.readyJoints]; wound[5] = 360;
    const woundPose = forwardKinematics(model, wound).pose;
    const requests: PlanRequest[] = [
      { modelId: model.id, startJoints: model.readyJoints, target: { position: [ready.position[0] - 15, 30, ready.position[2] - 10], orientation: ready.orientation }, speedMmPerSecond: 100 },
      { modelId: model.id, startJoints: model.readyJoints, target: { position: [ready.position[0], 30, ready.position[2]], orientation: null }, speedMmPerSecond: 100 },
      { modelId: model.id, startJoints: model.readyJoints, target: forwardKinematics(model, turned).pose, speedMmPerSecond: 100 },
      { modelId: model.id, startJoints: wound, target: { position: [woundPose.position[0] - 10, 10, woundPose.position[2]], orientation: woundPose.orientation }, speedMmPerSecond: 50 },
    ];
    const results = native<PlanResult>(requests.map(request => ({ op: 'plan', request })));
    results.forEach((actual, index) => {
      const reference = planMoveL(requests[index]);
      expect(actual.status).toBe('success');
      expect(reference.status).toBe('success');
      if (actual.status !== 'success' || reference.status !== 'success') return;
      closePose(actual.selected.pose, reference.selected.pose);
      closeVector(actual.selected.joints, reference.selected.joints, 1e-5);
      expect(actual.selected.configuration).toEqual(reference.selected.configuration);
      expect(actual.automaticOrientation).toBe(reference.automaticOrientation);
      expect(actual.lengthMm).toBeCloseTo(reference.lengthMm, 6);
      expect(actual.durationSeconds).toBeCloseTo(reference.durationSeconds, 6);
      // Floating-point ceil at exactly 60° can add one interval; verify geometry instead of the count.
      for (const sample of actual.path) {
        const start = forwardKinematics(model, requests[index].startJoints).pose;
        closeVector(sample.pose.position, start.position.map((p, axis) => p + sample.t * (actual.selected.pose.position[axis] - p)), 1e-6);
        const fk = forwardKinematics(model, sample.joints);
        closePose(sample.pose, fk.pose);
        expect(getConfiguration(model, sample.joints, fk).configuration?.[3]).toBe(actual.selected.configuration[3]);
      }
    });
  });

  it('rejects the same singular starts, unreachable targets, invalid orientations and endpoint-only solutions', () => {
    const ready = forwardKinematics(model, model.readyJoints).pose;
    const base: PlanRequest = { modelId: model.id, startJoints: model.readyJoints, target: ready, speedMmPerSecond: 100 };
    const requests: PlanRequest[] = [
      { ...base, startJoints: model.homeJoints },
      { ...base, startJoints: [0, 120, 0, 0, 40, 0] },
      { ...base, target: { position: [2000, 0, 500], orientation: ready.orientation } },
      { ...base, target: { ...ready, orientation: [0, 0, 0, 0] } },
      { ...base, target: { ...ready, orientation: [2, 0, 0, 0] } },
      { ...base, speedMmPerSecond: 0 },
      { ...base, target: forwardKinematics(model, [180, 20, -30, 0, 40, 0]).pose },
    ];
    const results = native<PlanResult>(requests.map(request => ({ op: 'plan', request })));
    results.forEach((actual, i) => {
      const reference = planMoveL(requests[i]);
      expect(actual.status).toBe('failure');
      expect(reference.status).toBe('failure');
      if (actual.status === 'failure' && reference.status === 'failure') expect(actual.code).toBe(reference.code);
    });
  });
});
