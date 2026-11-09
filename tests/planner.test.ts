import { describe, expect, it } from "vitest";
import { getConfiguration } from "../src/robot/configuration";
import { forwardKinematics, inverseKinematics, jointsWithinLimits, quaternionDistanceDeg, slerpQuaternion } from "../src/robot/kinematics";
import { ROBOT_MODELS } from "../src/robot/model";
import { planMoveL, PLANNING_LIMITS } from "../src/robot/planner";
import type { JointAngles, PlanRequest, Vec3 } from "../src/robot/types";

describe.each(Object.values(ROBOT_MODELS))("$name MoveL geometry", (model) => {
  const makeRequest = (): PlanRequest => {
    const start = forwardKinematics(model, model.readyJoints).pose;
    return { modelId: model.id, startJoints: [...model.readyJoints], target: { position: [start.position[0] - 15, 30, start.position[2] - 10], orientation: start.orientation }, speedMmPerSecond: 100 };
  };

  it("validates the complete straight TCP path, orientation and constant cfx", () => {
    const request = makeRequest(), result = planMoveL(request);
    expect(result.status, JSON.stringify(result)).toBe("success");
    if (result.status !== "success") return;
    const initial = forwardKinematics(model, request.startJoints).pose;
    expect(result.path.length).toBeGreaterThan(2);
    expect(result.path[0].joints).toEqual(request.startJoints);
    expect(result.path.at(-1)?.joints).toEqual(result.selected.joints);
    const initialConfig = getConfiguration(model, request.startJoints, forwardKinematics(model, request.startJoints)).configuration!;
    for (let index = 0; index < result.path.length; index++) {
      const sample = result.path[index], fk = forwardKinematics(model, sample.joints);
      expect(jointsWithinLimits(model, sample.joints)).toBe(true);
      sample.pose.position.forEach((value, axis) => expect(value).toBeCloseTo(initial.position[axis] + sample.t * (request.target.position[axis] - initial.position[axis]), 6));
      expect(quaternionDistanceDeg(sample.pose.orientation, slerpQuaternion(initial.orientation, request.target.orientation!, sample.t))).toBeLessThan(1e-4);
      expect(getConfiguration(model, sample.joints, fk).configuration?.[3]).toBe(initialConfig[3]);
      if (index > 0) {
        expect(Math.hypot(...sample.pose.position.map((value, axis) => value - result.path[index - 1].pose.position[axis]))).toBeLessThanOrEqual(PLANNING_LIMITS.translationStepMm + 1e-6);
      }
    }
    expect(result.lengthMm).toBeCloseTo(35, 6);
    expect(result.durationSeconds).toBeGreaterThanOrEqual(result.lengthMm / request.speedMmPerSecond);
    expect(result.automaticOrientation).toBe(false);
  });

  it("prefers preserving the current tool direction when only XYZ is supplied", () => {
    const request = makeRequest(); request.target.orientation = null;
    const result = planMoveL(request);
    expect(result.status, JSON.stringify(result)).toBe("success");
    if (result.status !== "success") return;
    expect(result.automaticOrientation).toBe(true);
    expect(result.orientationTravelDeg).toBeLessThan(1e-4);
  });

  it("plans a pure 60-degree tool rotation at a fixed TCP with the RAPID orientation speed", () => {
    const targetJoints: JointAngles = [...model.readyJoints]; targetJoints[5] += 60;
    const request = makeRequest(); request.target = forwardKinematics(model, targetJoints).pose;
    const result = planMoveL(request);
    expect(result.status, JSON.stringify(result)).toBe("success");
    if (result.status !== "success") return;
    const start = forwardKinematics(model, request.startJoints).pose;
    expect(result.lengthMm).toBeLessThan(1e-8);
    expect(result.orientationTravelDeg).toBeCloseTo(60, 6);
    expect(result.path.length).toBeGreaterThanOrEqual(31);
    expect(result.durationSeconds).toBeGreaterThanOrEqual(60 / 500 - 1e-9);
    for (const sample of result.path) {
      expect(Math.hypot(...sample.pose.position.map((value, axis) => value - start.position[axis]))).toBeLessThan(1e-6);
      expect(quaternionDistanceDeg(start.orientation, sample.pose.orientation)).toBeCloseTo(60 * sample.t, 5);
    }
    expect(result.selected.joints[5]).toBeCloseTo(60, 6);
  });

  it("rejects a singular mechanical zero and actual joint-limit violations", () => {
    const request = makeRequest(); request.startJoints = [0, 0, 0, 0, 0, 0];
    expect(planMoveL(request)).toMatchObject({ status: "failure", code: "singular_start" });
    request.startJoints = [0, 120, 0, 0, 40, 0];
    expect(planMoveL(request)).toMatchObject({ status: "failure", code: "joint_limit" });
  });

  it("does not generate a command for an unreachable point or invalid request", () => {
    const request = makeRequest(); request.target.position = [2000, 0, 500];
    expect(planMoveL(request)).toMatchObject({ status: "failure", code: "unreachable" });
    request.target.orientation = [0, 0, 0, 0];
    expect(planMoveL(request)).toMatchObject({ status: "failure", code: "invalid_input" });
    request.target.orientation = [2, 0, 0, 0];
    expect(planMoveL(request)).toMatchObject({ status: "failure", code: "invalid_input" });
    request.target.orientation = [1, 0, 0, 0]; request.target.position[0] = Infinity;
    expect(planMoveL(request)).toMatchObject({ status: "failure", code: "invalid_input" });
    request.target.position[0] = 400; request.speedMmPerSecond = 0;
    expect(planMoveL(request)).toMatchObject({ status: "failure", code: "invalid_input" });
  });

  it("keeps unwrapped full-turn starts instead of silently unwinding axis six", () => {
    const startJoints: JointAngles = [...model.readyJoints]; startJoints[5] = 360;
    const start = forwardKinematics(model, startJoints).pose;
    const target: Vec3 = [start.position[0] - 10, 10, start.position[2]];
    const result = planMoveL({ modelId: model.id, startJoints, target: { position: target, orientation: start.orientation }, speedMmPerSecond: 50 });
    expect(result.status, JSON.stringify(result)).toBe("success");
    if (result.status !== "success") return;
    expect(result.selected.joints[5]).toBeGreaterThan(300);
    expect(result.selected.configuration[2]).toBeGreaterThanOrEqual(3);
  });

  it("does not mistake a reachable endpoint for a reachable linear move", () => {
    const request = makeRequest();
    const endpoint = forwardKinematics(model, [180, 20, -30, 0, 40, 0]).pose;
    expect(inverseKinematics(model, endpoint, request.startJoints).length).toBeGreaterThan(0);
    request.target = endpoint;
    const result = planMoveL(request);
    expect(result.status, JSON.stringify(result)).toBe("failure");
    if (result.status !== "failure") return;
    expect(["path_unreachable", "singular_path", "configuration_change"]).toContain(result.code);
  });
});
