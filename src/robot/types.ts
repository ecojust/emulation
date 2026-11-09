/** Public units: millimetres, degrees, ABB scalar-first quaternions. */
export type Vec3 = [number, number, number];
export type Quaternion = [number, number, number, number];
export type JointAngles = [number, number, number, number, number, number];
/** Row-major homogeneous matrix, translating at indices 3, 7, 11. */
export type Mat4 = number[];
export type RobotId = "irb1100-4-0.58" | "irb1100-4-0.475";
export interface Pose { position: Vec3; orientation: Quaternion }
export interface DHRow { aMm: number; dMm: number; alphaDeg: number; thetaOffsetDeg: number; thetaSign: 1 | -1 }
export interface JointLimit { minDeg: number; maxDeg: number; maxSpeedDeg: number }
export interface RobotModel {
  id: RobotId;
  name: string;
  reachMm: number;
  payloadKg: number;
  dh: DHRow[];
  limits: JointLimit[];
  geometry: { shoulderHeightMm: number; upperArmMm: number; elbowOffsetMm: number; forearmMm: number; flangeMm: number };
  homeJoints: JointAngles;
  /** Application example only; not the manufacturer's mechanical zero. */
  readyJoints: JointAngles;
}
export interface FKResult {
  pose: Pose;
  /** Base followed by the six cumulative standard-DH transforms. */
  frames: Mat4[];
  jointOrigins: Vec3[];
  jointAxes: Vec3[];
  shoulder: Vec3;
  elbow: Vec3;
  wristCenter: Vec3;
}
/** ABB confdata = [cf1, cf4, cf6, cfx]. */
export type RobConf = [number, number, number, number];
export interface ConfigurationResult { configuration: RobConf | null; singular: boolean; reason?: string }
export interface IKSolution {
  id: string;
  joints: JointAngles;
  pose: Pose;
  configuration: RobConf;
  positionErrorMm: number;
  orientationErrorDeg: number;
  score: number;
}
export interface MotionSample { t: number; pose: Pose; joints: JointAngles }
export interface PlanRequest {
  modelId: RobotId;
  startJoints: JointAngles;
  target: { position: Vec3; orientation: Quaternion | null };
  speedMmPerSecond: number;
}
export interface PlanSuccess {
  status: "success";
  selected: IKSolution;
  /** Other endpoint IK solutions are informational, not validated alternative paths. */
  solutions: IKSolution[];
  path: MotionSample[];
  lengthMm: number;
  orientationTravelDeg: number;
  durationSeconds: number;
  warnings: string[];
  automaticOrientation: boolean;
}
export interface PlanFailure {
  status: "failure";
  code: "invalid_input" | "joint_limit" | "unreachable" | "singular_start" | "singular_path" | "configuration_change" | "path_unreachable" | "cancelled";
  message: string;
  failedAt?: number;
  solutions?: IKSolution[];
}
export type PlanResult = PlanSuccess | PlanFailure;
