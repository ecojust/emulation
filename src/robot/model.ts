import type { RobotId, RobotModel } from "./types";

/** Nominal geometry, limits and speeds: ABB 3HAC064993-001 revision V, pp. 30–31, 62, 66.
 * Standard DH is derived here from that geometry; it is not controller calibration data.
 * A_i = Rz(q_i + offset_i) Tz(d_i) Tx(a_i) Rx(alpha_i), all angles in degrees.
 */
function model(id: RobotId, upperArmMm: number, forearmMm: number, reachMm: number, shoulderSpeed: number): RobotModel {
  return {
    id,
    name: `ABB IRB 1100-4/${reachMm === 580 ? "0.58" : "0.475"}`,
    reachMm,
    payloadKg: 4,
    geometry: { shoulderHeightMm: 327, upperArmMm, elbowOffsetMm: 10, forearmMm, flangeMm: 64 },
    dh: [
      { aMm: 0, dMm: 327, alphaDeg: -90, thetaOffsetDeg: 0, thetaSign: 1 },
      { aMm: upperArmMm, dMm: 0, alphaDeg: 0, thetaOffsetDeg: -90, thetaSign: 1 },
      { aMm: 10, dMm: 0, alphaDeg: -90, thetaOffsetDeg: 0, thetaSign: 1 },
      { aMm: 0, dMm: forearmMm, alphaDeg: 90, thetaOffsetDeg: 0, thetaSign: 1 },
      { aMm: 0, dMm: 0, alphaDeg: -90, thetaOffsetDeg: 0, thetaSign: 1 },
      { aMm: 0, dMm: 64, alphaDeg: 0, thetaOffsetDeg: 180, thetaSign: 1 },
    ],
    limits: [
      { minDeg: -230, maxDeg: 230, maxSpeedDeg: 460 },
      { minDeg: -115, maxDeg: 113, maxSpeedDeg: shoulderSpeed },
      { minDeg: -205, maxDeg: 55, maxSpeedDeg: 280 },
      { minDeg: -230, maxDeg: 230, maxSpeedDeg: 560 },
      { minDeg: -125, maxDeg: 120, maxSpeedDeg: 420 },
      { minDeg: -400, maxDeg: 400, maxSpeedDeg: 750 },
    ],
    homeJoints: [0, 0, 0, 0, 0, 0],
    readyJoints: [0, 20, -30, 0, 40, 0],
  };
}

export const ROBOT_MODELS: Record<RobotId, RobotModel> = {
  "irb1100-4-0.58": model("irb1100-4-0.58", 280, 300, 580, 360),
  "irb1100-4-0.475": model("irb1100-4-0.475", 225, 250, 475, 380),
};

export function getRobotModel(id: RobotId): RobotModel {
  const result = ROBOT_MODELS[id];
  if (!result) throw new Error("不支持的机器人型号");
  return result;
}
