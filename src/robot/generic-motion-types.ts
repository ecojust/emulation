import type { JointAngles, JointLimit, Mat4, MotionSample, PlanFailure, Pose, Quaternion, Vec3 } from './types';

/** Exact URDF ancestry from the root link to tool0/flange; origins are row-major millimetres. */
export interface GenericChainStep {
  origin: Mat4;
  axis: Vec3;
  /** Fixed transforms carry null. Each of the six actuator indices occurs exactly once. */
  jointIndex: number | null;
}

export interface GenericChain {
  modelId: string;
  steps: GenericChainStep[];
  limits: [JointLimit, JointLimit, JointLimit, JointLimit, JointLimit, JointLimit];
}

export interface GenericFkResult {
  pose: Pose;
  transform: Mat4;
  jointOrigins: [Vec3, Vec3, Vec3, Vec3, Vec3, Vec3];
  jointAxes: [Vec3, Vec3, Vec3, Vec3, Vec3, Vec3];
}

export interface GenericPlanRequest {
  chain: GenericChain;
  startJoints: JointAngles;
  targetJoints: JointAngles;
  speedMmPerSecond: number;
}

export interface GenericPlanSuccess {
  status: 'success';
  path: MotionSample[];
  lengthMm: number;
  orientationTravelDeg: number;
  durationSeconds: number;
  warnings: string[];
}

export interface GenericPlanFailure {
  status: 'failure';
  code: PlanFailure['code'];
  message: string;
  failedAt?: number;
}

/** A URDF kinematic simulation; no manufacturer controller configuration is invented. */
export type GenericPlanResult = GenericPlanSuccess | GenericPlanFailure;

/** Endpoint inspection uses the robot-base flange frame, without manufacturer posture codes. */
export interface GenericEndpointRequest { chain: GenericChain; pose: Pose; seedJoints: JointAngles }
export interface GenericIkSolution {
  id: string;
  joints: JointAngles;
  pose: Pose;
  positionErrorMm: number;
  orientationErrorDeg: number;
  score: number;
}

/** Target and returned sample poses are world TCP, including both base and tool transforms. */
export interface GenericPosePlanRequest {
  chain: GenericChain;
  startJoints: JointAngles;
  target: { position: Vec3; orientation: Quaternion | null };
  basePose?: Pose;
  toolPose?: Pose;
  speedMmPerSecond: number;
}
export interface GenericPosePlanSuccess extends GenericPlanSuccess {
  /** Unlike endpoint inspection, selected.pose is the world TCP pose. */
  selected: GenericIkSolution;
  automaticOrientation: boolean;
}
export type GenericPosePlanResult = GenericPosePlanSuccess | GenericPlanFailure;
