import type { GenericChain } from './generic-motion-types';
import type { CollisionZone } from './collision-geometry';
import type { JointAngles, MotionSample, Pose, Quaternion, RobConf, RobotId, Vec3 } from './types';

/** Bounds enclose the actual visual geometry, in a chain-step frame, in mm. */
export interface GrindingBody {
  /** Number of chain steps already applied. 0 is the robot base. */
  stepIndex: number;
  centerMm: Vec3;
  halfSizeMm: Vec3;
}
export type GrindingRobot = { kind: 'abb'; modelId: RobotId }
  | { kind: 'generic'; chain: GenericChain; bodies: GrindingBody[] };
export interface GrindingInput {
  /** Preserve every input target position and orientation, in world coordinates. */
  targets: Pose[];
  clearanceMm: number;
  speedMmPerSecond: number;
}
export interface GrindingRequest extends GrindingInput {
  /** Legacy constraint only: must agree with all targets; never replaces their poses. */
  orientation?: Quaternion | null;
  allowDetours?: boolean;
  /** The request boundary converts each target into world coordinates. */
  robot: GrindingRobot;
  basePose: Pose;
  /** Transform from the flange to the active tool TCP. */
  toolPose: Pose;
  toolRadiusMm: number;
  startJoints: JointAngles;
  zones: CollisionZone[];
}
export interface GrindingWaypoint {
  pose: Pose;
  joints: JointAngles;
  configuration: RobConf | null;
  targetIndex: number | null;
  kind: 'process' | 'transfer';
}
export interface GrindingSuccess {
  status: 'success';
  /** All poses use world TCP coordinates; joints remain robot actuator coordinates. */
  path: MotionSample[];
  waypoints: GrindingWaypoint[];
  durationSeconds: number;
  lengthMm: number;
  orientation: Quaternion | null;
  warnings: string[];
  checkedIntervals: number;
}
export interface GrindingFailure {
  status: 'failure';
  code: string;
  message: string;
  failedTarget?: number;
}
export type GrindingResult = GrindingSuccess | GrindingFailure;
