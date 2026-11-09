import { describe, expect, it } from "vitest";
import { getConfiguration, jointQuadrant } from "../src/robot/configuration";
import { ROBOT_MODELS } from "../src/robot/model";
import type { FKResult, JointAngles, Vec3 } from "../src/robot/types";

const model = ROBOT_MODELS["irb1100-4-0.58"];

function geometry(shoulder: Vec3, elbow: Vec3, wristCenter: Vec3): FKResult {
  return { shoulder, elbow, wristCenter, jointOrigins: [[0, 0, 0], shoulder], jointAxes: [[0, 0, 1], [0, 1, 0]], frames: [], pose: { position: wristCenter, orientation: [1, 0, 0, 0] } };
}

describe("ABB 6-axis quadrant intervals", () => {
  it.each([[-400, -5], [-360, -4], [-180, -2], [-90.001, -2], [-90, -1], [-0.000001, -1], [0, 0], [89.999999, 0], [90, 1], [180, 2], [400, 4]])("classifies %s° as %s without wrapping complete turns", (angle, expected) => {
    expect(jointQuadrant(angle)).toBe(expected);
  });
  it("snaps only round-off at an exact boundary, not meaningful near-boundary angles", () => {
    expect(jointQuadrant(-1e-12)).toBe(0);
    expect(jointQuadrant(90 - 1e-12)).toBe(1);
    expect(jointQuadrant(90 - 1e-7)).toBe(0);
    expect(jointQuadrant(-1e-7)).toBe(-1);
  });
});

describe("ABB serial-arm cfx geometry", () => {
  // ABB's configuration figures show the lower arm vertical for cfx 0/1,
  // horizontal toward the wrist for cfx 2/3; reflected wrist positions cover 4–7.
  const geometries = [
    geometry([0, 0, 327], [0, 0, 607], [300, 0, 617]),
    geometry([0, 0, 327], [280, 0, 327], [290, 0, 627]),
    geometry([0, 0, 327], [-280, 0, 327], [-290, 0, 627]),
    geometry([0, 0, 327], [0, 0, 607], [-300, 0, 617]),
  ];

  it.each([0, 1, 2, 3, 4, 5, 6, 7])("matches the official branch table for cfx=%s", cfx => {
    const q: JointAngles = [0, 0, 0, -100, cfx % 2 ? -30 : 30, 380];
    const result = getConfiguration(model, q, geometries[Math.floor(cfx / 2)]);
    expect(result).toEqual({ configuration: [0, -2, 4, cfx], singular: false });
  });

  it("uses the actual axis-2 direction after rotating the entire arm about axis 1", () => {
    const fk = geometry([0, 0, 327], [0, 0, 607], [0, 300, 617]);
    fk.jointAxes[1] = [-1, 0, 0];
    expect(getConfiguration(model, [90, 0, 0, 0, 30, 0], fk).configuration).toEqual([1, 0, 0, 0]);
  });

  it.each([
    { name: "shoulder", fk: geometry([0, 0, 327], [0, 0, 607], [0.01, 0, 617]), q5: 30 },
    { name: "elbow", fk: geometry([0, 0, 327], [280, 0, 327], [580, 0, 327.01]), q5: 30 },
    { name: "wrist", fk: geometries[0], q5: 0 },
    { name: "near wrist", fk: geometries[0], q5: 0.05 },
  ])("rejects $name singularity instead of inventing a cfx", ({ fk, q5 }) => {
    const result = getConfiguration(model, [0, 0, 0, 0, q5, 0], fk);
    expect(result.configuration).toBeNull();
    expect(result.singular).toBe(true);
    expect(result.reason).toBeTruthy();
  });
});
