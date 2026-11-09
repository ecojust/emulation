import { describe, expect, it } from "vitest";
import { eulerZYXToQuaternion, formatMoveL, normalizeOrientation, parseRobtarget } from "../src/robot/rapid";
import type { Quaternion, RobConf } from "../src/robot/types";

const literal = "[[400,-25.5,500],[0.70710678,0,0.70710678,0],[0,-1,2,3],[9E9,9E9,9E9,9E9,9E9,9E9]]";

describe("ABB scalar-first orientations", () => {
  it("matches the ABB manual's +Y quarter-turn example", () => {
    const q = eulerZYXToQuaternion([0, 90, 0]);
    expect(q[0]).toBeCloseTo(Math.SQRT1_2, 12);
    expect(q[1]).toBeCloseTo(0, 12);
    expect(q[2]).toBeCloseTo(Math.SQRT1_2, 12);
    expect(q[3]).toBeCloseTo(0, 12);
    expect(eulerZYXToQuaternion([90, 0, 0])[3]).toBeCloseTo(Math.SQRT1_2, 12);
    expect(eulerZYXToQuaternion([0, 0, 90])[1]).toBeCloseTo(Math.SQRT1_2, 12);
  });

  it("normalizes a rounded ABB quaternion while preserving its equivalent negative representation", () => {
    expect(normalizeOrientation([0.707, 0, 0.707, 0])).toEqual([Math.SQRT1_2, 0, Math.SQRT1_2, 0]);
    expect(normalizeOrientation([-1, 0, 0, 0])[0]).toBe(-1);
  });

  it.each(([[0, 0, 0, 0], [1, 1, 0, 0], [2, 0, 0, 0], [Infinity, 0, 0, 0], [NaN, 0, 0, 0]] as Quaternion[]).map(q => ({ q })))("rejects invalid orientation $q", ({ q }) => {
    expect(() => normalizeOrientation(q)).toThrow();
  });
});

describe("strict robtarget import", () => {
  it.each([literal, `CONST robtarget pTarget := ${literal};`, `  local pers ROBTARGET target_1 :=\n${literal};  `])("accepts a complete constant target", text => {
    const target = parseRobtarget(text);
    expect(target.pose.position).toEqual([400, -25.5, 500]);
    expect(target.pose.orientation[0]).toBeCloseTo(Math.SQRT1_2, 12);
    expect(target.configuration).toEqual([0, -1, 2, 3]);
    expect(target.externalAxes).toEqual([9e9, 9e9, 9e9, 9e9, 9e9, 9e9]);
  });

  it("supports signed decimals and scientific notation without evaluating expressions", () => {
    const target = parseRobtarget(literal.replace("400,-25.5,500", "+4e2,-.255e2,500."));
    expect(target.pose.position).toEqual([400, -25.5, 500]);
  });

  it.each([
    "", "[400,0,500]", literal + "; Stop;", literal + ";;", `MoveL ${literal},v100,fine,tool0;`,
    literal.replace("400", "1+399"), literal.replace("400", "Infinity"), literal.replace("400", "1e999"),
    literal.replace("[400,-25.5,500]", "[400,-25.5,500,1]"),
    literal.replace("[0,-1,2,3]", "[0,-1,2,8]"), literal.replace("[0,-1,2,3]", "[0.5,-1,2,3]"),
    literal.replace("9E9", "0"), literal.replace(",9E9]", "]"), literal.replace("500]", "500,]"),
    literal.replace("[0.70710678,0,0.70710678,0]", "[0,0,0,0]"),
    `CONST robtarget p := ${literal}; console.log('unexpected')`, "[".repeat(100),
  ])("rejects incomplete, executable, malformed or external-axis targets", text => {
    expect(() => parseRobtarget(text)).toThrow();
  });
});

describe("MoveL output", () => {
  it("exports monitored inline target with real speeddata and explicit tool/workobject", () => {
    const target = parseRobtarget(literal);
    const rapid = formatMoveL(target.pose, target.configuration, 123);
    expect(rapid).toMatch(/^ConfL \\On;\nMoveL \[\[/);
    expect(rapid).toContain(",[123,500,5000,1000]".replace(",", ", "));
    expect(rapid).toMatch(/, fine, tool0\\WObj:=wobj0;$/);
    expect(rapid).not.toContain("v123");
    expect(rapid).toContain("[0,-1,2,3]");
  });

  it("keeps generated numeric poses within the stated output precision on round trip", () => {
    const pose = { position: [300.1234567, -250.7654321, 412.9876543] as [number, number, number], orientation: eulerZYXToQuaternion([147.123, -47.678, 19.234]) };
    const conf: RobConf = [-1, 0, -3, 5];
    const rapid = formatMoveL(pose, conf, 100);
    const targetText = rapid.slice(rapid.indexOf("[["), rapid.indexOf(", [100,500,5000,1000]"));
    const back = parseRobtarget(targetText);
    pose.position.forEach((v, i) => expect(Math.abs(v - back.pose.position[i])).toBeLessThanOrEqual(0.0005));
    pose.orientation.forEach((v, i) => expect(Math.abs(v - back.pose.orientation[i])).toBeLessThan(1e-9));
    expect(back.configuration).toEqual(conf);
  });

  it.each([0, 0.5, -1, NaN, Infinity, 2001])("rejects speed %s before producing a command", speed => {
    const target = parseRobtarget(literal);
    expect(() => formatMoveL(target.pose, target.configuration, speed)).toThrow();
  });

  it.each([1, 2000])("accepts the inclusive speed boundary %s", speed => {
    const target = parseRobtarget(literal);
    expect(formatMoveL(target.pose, target.configuration, speed)).toContain(`[${speed},500,5000,1000]`);
  });
});
