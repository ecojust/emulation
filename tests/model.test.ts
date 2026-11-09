import { describe, expect, it } from "vitest";
import { forwardKinematics } from "../src/robot/kinematics";
import { getRobotModel } from "../src/robot/model";
import type { RobotId, Vec3 } from "../src/robot/types";

function expectVector(actual: number[], expected: number[], tolerance = 1e-8) {
  expect(actual).toHaveLength(expected.length);
  expected.forEach((value, i) => expect(Math.abs(actual[i] - value)).toBeLessThan(tolerance));
}

// Independent values from ABB 3HAC064993-001 revision V, pp. 30–31, 60–61.
// pos0 in the working-range tables is the flange, despite the wrist-center heading.
const zeroCases: { id: RobotId; elbow: Vec3; wrist: Vec3; flange: Vec3 }[] = [
  { id: "irb1100-4-0.58", elbow: [0, 0, 607], wrist: [300, 0, 617], flange: [364, 0, 617] },
  { id: "irb1100-4-0.475", elbow: [0, 0, 552], wrist: [250, 0, 562], flange: [314, 0, 562] },
];

describe.each(zeroCases)("$id mechanical zero", ({ id, elbow, wrist, flange }) => {
  it("places the base, shoulder, elbow, wrist and tool0 at the specified positions", () => {
    const model = getRobotModel(id);
    expect(model.homeJoints).toEqual([0, 0, 0, 0, 0, 0]);
    const fk = forwardKinematics(model, model.homeJoints);
    expectVector([fk.frames[0][3], fk.frames[0][7], fk.frames[0][11]], [0, 0, 0]);
    expectVector(fk.shoulder, [0, 0, 327]);
    expectVector(fk.elbow, elbow);
    expectVector(fk.wristCenter, wrist);
    expectVector(fk.pose.position, flange);
  });

  it("uses the positive joint axes and the tool0 flange orientation", () => {
    const model = getRobotModel(id);
    const fk = forwardKinematics(model, model.homeJoints);
    const positiveAxes = [[0, 0, 1], [0, 1, 0], [0, 1, 0], [1, 0, 0], [0, 1, 0], [1, 0, 0]];
    positiveAxes.forEach((axis, i) => expectVector(fk.jointAxes[i], axis));
    // Flange local axes are -world Z, +world Y, +world X respectively.
    const flangeFrame = fk.frames[6];
    expectVector([flangeFrame[0], flangeFrame[4], flangeFrame[8]], [0, 0, -1]);
    expectVector([flangeFrame[1], flangeFrame[5], flangeFrame[9]], [0, 1, 0]);
    expectVector([flangeFrame[2], flangeFrame[6], flangeFrame[10]], [1, 0, 0]);
    // q and -q represent the same rotation.
    const expectedQ = [Math.SQRT1_2, 0, Math.SQRT1_2, 0];
    const dot = fk.pose.orientation.reduce((sum, value, i) => sum + value * expectedQ[i], 0);
    expect(Math.abs(dot)).toBeCloseTo(1, 12);
  });
});

type WorkingRangePoint = [position: number, axis2: number, axis3: number, x: number, z: number];
const workingRanges: { id: RobotId; page: number; points: WorkingRangePoint[] }[] = [
  {
    id: "irb1100-4-0.58", page: 61,
    points: [
      [1, 0, -88, 0, 907.2],
      [2, 12.5, 55, 184.6, 327],
      [3, 90, -88, 580, 327],
      [4, 113, -88, 534, 100.3],
      [5, -28.3, -205, -304, 327],
      [6, -115, 55, -112.4, 473.5],
      [7, -90, -88, -580, 327],
      [8, -115, -88, -525.8, 81.8],
      [9, 113, -205, 237.3, 517.1],
    ],
  },
  {
    id: "irb1100-4-0.475", page: 60,
    points: [
      [1, 0, -87.7, 0, 802],
      // Page 60 drawing says 153.8; its table says 53.8, inconsistent with geometry.
      // Use the drawing; this source discrepancy is preserved in docs/robot-model.md.
      [2, 9.7, 55, 153.8, 327],
      [3, 90, -87.7, 475, 327],
      [4, 113, -87.7, 437.4, 141.3],
      [5, -26.4, -205, -248.2, 327],
      [6, -115, 55, -87.6, 453.4],
      [7, -90, -87.7, -475, 327],
      [8, -115, -87.7, -430.7, 126.2],
      [9, 113, -205, 188.4, 488.6],
    ],
  },
];

describe.each(workingRanges)("$id official working range, page $page", ({ id, points }) => {
  it.each(points)("matches wrist position pos%i at axis2=%s°, axis3=%s°", (_position, axis2, axis3, x, z) => {
    const fk = forwardKinematics(getRobotModel(id), [0, axis2, axis3, 0, 0, 0]);
    // The source rounds angles and coordinates; this is not a machine calibration tolerance.
    expectVector(fk.wristCenter, [x, 0, z], 1);
  });
});
