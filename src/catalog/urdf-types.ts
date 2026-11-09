import type { BufferGeometry } from "three";
import type { Vec3 } from "../robot/types";

/** URDF geometry uses metres and radians; application joint controls use degrees. */
export interface UrdfOrigin { xyz: Vec3; rpy: Vec3 }
export interface UrdfVisual {
  meshPath: string;
  origin: UrdfOrigin;
  scale: Vec3;
  color?: [number, number, number, number];
}
export interface UrdfLink { name: string; visuals: UrdfVisual[] }
export interface UrdfJoint {
  name: string;
  type: "fixed" | "revolute" | "continuous";
  parent: string;
  child: string;
  origin: UrdfOrigin;
  axis: Vec3;
  minDeg: number;
  maxDeg: number;
  maxSpeedDeg: number;
}
export interface UrdfRobot {
  name: string;
  rootLink: string;
  toolLink: string | null;
  links: UrdfLink[];
  /** Parent before child, including fixed branches. */
  joints: UrdfJoint[];
  actuatedJoints: UrdfJoint[];
  homeJoints: number[];
}
/** The loader owns these geometries. Scene clones them to separate lifetimes. */
export interface PreparedUrdfModel {
  id: string;
  name: string;
  description: UrdfRobot;
  geometries: ReadonlyMap<string, BufferGeometry>;
}
