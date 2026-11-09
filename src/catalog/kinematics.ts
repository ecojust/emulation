import { Euler, Matrix4, Quaternion, Vector3 } from "three";
import type { Mat4, Pose } from "../robot/types";
import type { UrdfOrigin, UrdfRobot } from "./urdf-types";

/** URDF fixed-axis RPY means Rz(yaw) Ry(pitch) Rx(roll). */
export function urdfOriginMatrix(origin: UrdfOrigin): Matrix4 {
  const [roll, pitch, yaw] = origin.rpy;
  return new Matrix4().makeRotationFromEuler(new Euler(roll, pitch, yaw, "ZYX"))
    .setPosition(...origin.xyz);
}

export function urdfLinkMatrices(robot: UrdfRobot, anglesDeg: readonly number[]): Map<string, Matrix4> {
  if (anglesDeg.length !== robot.actuatedJoints.length || anglesDeg.some(value => !Number.isFinite(value))) {
    throw new Error("URDF 关节角数量不符或包含无效数值。");
  }
  const angles = new Map(robot.actuatedJoints.map((joint, i) => [joint.name, anglesDeg[i] * Math.PI / 180]));
  const transforms = new Map([[robot.rootLink, new Matrix4()]]);
  for (const joint of robot.joints) {
    const transform = transforms.get(joint.parent)!.clone().multiply(urdfOriginMatrix(joint.origin));
    if (joint.type !== "fixed") transform.multiply(new Matrix4().makeRotationAxis(new Vector3(...joint.axis), angles.get(joint.name)!));
    transforms.set(joint.child, transform);
  }
  return transforms;
}

export function forwardUrdfKinematics(robot: UrdfRobot, anglesDeg: readonly number[]): {
  toolPose: Pose | null;
  /** Row-major matrices in metres, including fixed links. */
  linkTransforms: ReadonlyMap<string, Mat4>;
} {
  const matrices = urdfLinkMatrices(robot, anglesDeg);
  let toolPose: Pose | null = null;
  const tool = robot.toolLink && matrices.get(robot.toolLink);
  if (tool) {
    const position = new Vector3().setFromMatrixPosition(tool);
    const orientation = new Quaternion().setFromRotationMatrix(tool).normalize();
    toolPose = { position: [position.x * 1000, position.y * 1000, position.z * 1000],
      orientation: [orientation.w, orientation.x, orientation.y, orientation.z] };
  }
  return { toolPose, linkTransforms: new Map([...matrices].map(([name, transform]) => [name, transform.clone().transpose().toArray()])) };
}
