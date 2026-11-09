import { describe, expect, it } from "vitest";
import { getConfiguration } from "../src/robot/configuration";
import { forwardKinematics, inverseKinematics, jointsWithinLimits, normalizeQuaternion, quaternionDistanceDeg, slerpQuaternion } from "../src/robot/kinematics";
import { ROBOT_MODELS } from "../src/robot/model";
import type { JointAngles, Quaternion } from "../src/robot/types";

describe.each(Object.values(ROBOT_MODELS))("$name analytic inverse kinematics", (model) => {
  it("recovers deterministic non-singular samples throughout the real six-axis ranges", () => {
    let random = 287431;
    const next = () => ((random = (Math.imul(random, 1664525) + 1013904223) >>> 0) / 2 ** 32);
    let checked = 0;
    for (let n = 0; n < 100; n++) {
      const joints = model.limits.map((limit) => limit.minDeg + 2 + next() * (limit.maxDeg - limit.minDeg - 4)) as JointAngles;
      const fk = forwardKinematics(model, joints);
      if (getConfiguration(model, joints, fk).singular) continue;
      const solutions = inverseKinematics(model, fk.pose, joints);
      expect(solutions.length, `Missing q=${joints.join(",")}`).toBeGreaterThan(0);
      expect(solutions[0].joints.every((value, axis) => Math.abs(value - joints[axis]) < 1e-5), `Wrong branch q=${joints.join(",")}`).toBe(true);
      for (const solution of solutions) {
        expect(jointsWithinLimits(model, solution.joints)).toBe(true);
        expect(solution.positionErrorMm).toBeLessThan(1e-6);
        expect(solution.orientationErrorDeg).toBeLessThan(1e-4);
        expect(getConfiguration(model, solution.joints, forwardKinematics(model, solution.joints)).configuration).toEqual(solution.configuration);
      }
      checked++;
    }
    expect(checked).toBeGreaterThan(95);
  });

  it("retains genuine elbow/wrist alternatives and unwrapped A6 turns", () => {
    const seed: JointAngles = [20, 20, -40, 25, 40, 370];
    const pose = forwardKinematics(model, seed).pose;
    const solutions = inverseKinematics(model, pose, seed);
    expect(solutions[0].joints[5]).toBeCloseTo(370, 8);
    expect(solutions.some((solution) => Math.abs(solution.joints[5] - 10) < 1e-7)).toBe(true);
    expect(new Set(solutions.map((solution) => solution.configuration[3])).size).toBeGreaterThanOrEqual(2);
    expect(new Set(solutions.map((solution) => solution.id)).size).toBe(solutions.length);
  });

  it("rejects unreachable targets and invalid quaternions without fabricated joints", () => {
    expect(inverseKinematics(model, { position: [5000, 0, 300], orientation: [1, 0, 0, 0] })).toEqual([]);
    expect(inverseKinematics(model, { position: [400, 0, 500], orientation: [0, 0, 0, 0] })).toEqual([]);
    expect(inverseKinematics(model, { position: [NaN, 0, 500], orientation: [1, 0, 0, 0] })).toEqual([]);
  });

  it("normalizes input rotation and treats q and -q as the same orientation", () => {
    const pose = forwardKinematics(model, model.readyJoints).pose;
    const scaled = pose.orientation.map((value) => value * -8) as Quaternion;
    const first = inverseKinematics(model, pose, model.readyJoints)[0];
    const second = inverseKinematics(model, { ...pose, orientation: scaled }, model.readyJoints)[0];
    first.joints.forEach((value, axis) => expect(second.joints[axis]).toBeCloseTo(value, 8));
  });
});

it("interpolates the shortest quaternion arc and handles very large finite inputs", () => {
  normalizeQuaternion([1e308, 0, 1e308, 0]).forEach((value, index) => expect(value).toBeCloseTo([Math.SQRT1_2, 0, Math.SQRT1_2, 0][index], 14));
  const a: Quaternion = [1, 0, 0, 0], b: Quaternion = [-Math.SQRT1_2, 0, 0, -Math.SQRT1_2];
  const middle = slerpQuaternion(a, b, 0.5);
  expect(quaternionDistanceDeg(a, middle)).toBeCloseTo(45, 8);
  expect(quaternionDistanceDeg(middle, b)).toBeCloseTo(45, 8);
  expect(quaternionDistanceDeg(a, slerpQuaternion(a, [-1, 0, 0, 0], 0.5))).toBe(0);
});
