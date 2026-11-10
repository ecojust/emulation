import { composePose, identityPose, relativePose } from "../workspace/frame-graph";
import Icon from '../ui/Icon';
import { forwardRef, useEffect, useImperativeHandle, useRef, useState } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { TransformControls, type TransformControlsGizmo } from "three/addons/controls/TransformControls.js";
import { normalizeWheelDelta, smoothDistance } from "./smooth-zoom";
import { createValidatedRenderer } from "./webgl-context";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import { forwardKinematics } from "../robot/kinematics";
import type { JointAngles, Mat4, MotionSample, Pose, RobotModel, Vec3 } from "../robot/types";
import type { CollisionZone } from "../robot/collision";
import type { PreparedUrdfModel, UrdfVisual } from "../catalog/urdf-types";
import { forwardUrdfKinematics, urdfLinkMatrices, urdfOriginMatrix } from "../catalog/kinematics";
import type { ProjectCamera } from "../projects/schema";
import type { SceneSurface, SurfacePick } from '../surface/types';
import { createSurfaceVisual, updateSurfaceVisual, disposeSurfaceVisual, createSurfaceRegionVisual,
  updateSurfaceRegionVisual, pickSurface, surfaceDiagnostics,
  type SurfaceVisual, type SurfaceRegionVisual, type SceneSurfaceRegion } from './surface-visuals';
export type { SceneSurfaceRegion } from './surface-visuals';
import "./scene.css";

export interface ScenePath {
  id: string;
  name: string;
  points: { id: string; name?: string; pose: Pose }[];
  selected?: boolean;
}

export interface SceneReferenceFrame { id: string; name: string; pose: Pose; selected?: boolean }
/** Both poses are world poses, not flange or mounting offsets. */
export interface SceneTool { id: string; name: string; pose: Pose; tcpPose: Pose; radiusMm: number; selected?: boolean }

export interface RobotSceneHandle { camera(): ProjectCamera | null; capture(): Promise<{ thumbnail: string; camera: ProjectCamera }>; focusSurface(id: string): boolean }
export interface RobotSceneProps {
  robotPresent?: boolean;
  initialCamera?: ProjectCamera | null;
  /** World <- robot base. */
  basePose?: Pose;
  /** Flange <- active TCP, including tool mounting offset. */
  toolPose?: Pose;
  sceneFrames?: SceneReferenceFrame[];
  sceneTools?: SceneTool[];
  selectedJointIndex?: number | null;
  urdfModel?: PreparedUrdfModel;
  urdfJoints?: readonly number[];
  model: RobotModel;
  joints: JointAngles;
  targetPose: Pose | null;
  path: MotionSample[];
  endJoints?: JointAngles;
  showFrames?: boolean;
  view?: "iso" | "front" | "top";
  viewRevision?: number;
  jogEnabled?: boolean;
  jogMode?: "translate" | "rotate";
  jogSpace?: "world" | "local";
  jogPending?: boolean;
  jogResetRevision?: number;
  onJogStart?: () => void;
  onJogMove?: (pose: Pose) => void;
  onJogEnd?: () => void;
  dangerZones?: (CollisionZone & { name?: string })[];
  selectedDangerZoneId?: string;
  zoneManipulationEnabled?: boolean;
  onDangerZoneMove?: (id: string, centerMm: Vec3) => void;
  onDangerZoneDrag?: (dragging: boolean) => void;
  scenePaths?: ScenePath[];
  selectedPointId?: string | null;
  sceneSurfaces?: readonly SceneSurface[];
  selectedSurfaceId?: string | null;
  surfacePickingEnabled?: boolean;
  surfacePickSurfaceId?: string | null;
  surfaceRegion?: SceneSurfaceRegion | null;
  onSurfacePick?: (point: SurfacePick) => void;
  onSelectSurface?: (id: string) => void;
}

interface DangerVisual {
  root: THREE.Group;
  body: THREE.Group;
  mesh: THREE.Mesh<THREE.BoxGeometry, THREE.MeshBasicMaterial>;
  edges: THREE.LineSegments<THREE.EdgesGeometry, THREE.LineBasicMaterial>;
  caption: THREE.Sprite;
  text: string;
}

interface PathVisual {
  root: THREE.Group;
  line: THREE.Line<THREE.BufferGeometry, THREE.LineDashedMaterial>;
  points: THREE.Points<THREE.BufferGeometry, THREE.PointsMaterial>;
  signature: string;
  source: ScenePath;
}

interface FrameVisual { root: THREE.Group; name: string }
interface ToolVisual { root: THREE.Group; body: THREE.Mesh; tcp: THREE.Group; signature: string }
interface SceneState {
  scene: THREE.Scene;
  camera: THREE.PerspectiveCamera;
  controls: OrbitControls;
  renderer: THREE.WebGLRenderer;
  robot: THREE.Group;
  ghost: THREE.Group;
  frames: THREE.Group;
  tcp: THREE.Group;
  flange: THREE.Object3D;
  jointHighlight: THREE.Group;
  selectedJointIndex: number | null;
  referenceFrames: Map<string, FrameVisual>;
  tools: Map<string, ToolVisual>;
  target: THREE.Group;
  path: THREE.Line;
  start: THREE.Mesh;
  links: THREE.Mesh[];
  ghostLinks: THREE.Mesh[];
  zeroInverse: THREE.Matrix4[];
  dirty: boolean;
  zoomTarget: number;
  jogObject: THREE.Object3D;
  zoneObject: THREE.Object3D;
  manipulationTarget: "tcp" | "zone" | "none";
  manipulatedZoneId: string | null;
  transform: TransformControls;
  finishDrag: () => void;
  draggingPointer: number | null;
  dangerZones: Map<string, DangerVisual>;
  scenePaths: Map<string, PathVisual>;
  surfaces: Map<string, SurfaceVisual>;
  surfaceRegion: SurfaceRegionVisual;
  surfaceRegionSource: SceneSurfaceRegion | null;
  surfacePickingEnabled: boolean;
  selectedPoint: THREE.Group;
  selectedPointCaption: THREE.Sprite;
  selectedPointText: string;
  selectedPointId: string | null;
  genericLinks: { mesh: THREE.Mesh; link: string; visual: UrdfVisual }[];
  genericFocus: THREE.Vector3 | null;
  genericFocusLocal: THREE.Vector3 | null;
  genericRadius: number;
  genericModelId: string | null;
  genericJoints: readonly number[];
  robotJoints: readonly number[];
  grid: THREE.GridHelper;
  ground: THREE.Mesh;
  key: THREE.DirectionalLight;
}

const SCALE = 0.001;
const PARTS = ["base_link", "link_1", "link_2", "link_3", "link_4", "link_5", "link_6"];
const AXIS_COLORS = [0xe35454, 0x46a88a, 0x557bd1];
const MAX_SCENE_COORDINATE_MM = 1e6;

function usableZone(zone: CollisionZone | undefined): zone is CollisionZone {
  return Boolean(zone?.enabled && zone.centerMm.length === 3 && zone.sizeMm.length === 3
    && zone.centerMm.every(value => Number.isFinite(value) && Math.abs(value) <= MAX_SCENE_COORDINATE_MM)
    && zone.sizeMm.every(value => Number.isFinite(value) && value > 0 && value <= MAX_SCENE_COORDINATE_MM));
}

function validScenePose(pose: Pose): boolean {
  return pose.position.length === 3 && pose.position.every(value => Number.isFinite(value) && Math.abs(value) <= MAX_SCENE_COORDINATE_MM)
    && pose.orientation.length === 4 && pose.orientation.every(Number.isFinite) && Math.hypot(...pose.orientation) > 0;
}

/** Robot matrices use row-major millimetres; Three matrices are column-major. */
function matrix(value: Mat4, metres = false): THREE.Matrix4 {
  const result = new THREE.Matrix4().fromArray(value).transpose();
  if (metres) {
    result.elements[12] *= SCALE;
    result.elements[13] *= SCALE;
    result.elements[14] *= SCALE;
  }
  return result;
}

function disposeTree(root: THREE.Object3D): void {
  const geometries = new Set<THREE.BufferGeometry>();
  const materials = new Set<THREE.Material>();
  const textures = new Set<THREE.Texture>();
  root.traverse((node) => {
    const drawable = node as THREE.Mesh;
    if (drawable.geometry) geometries.add(drawable.geometry);
    if (drawable.material) {
      for (const material of Array.isArray(drawable.material) ? drawable.material : [drawable.material]) {
        materials.add(material);
        const map = (material as THREE.MeshBasicMaterial).map;
        if (map) textures.add(map);
      }
    }
  });
  geometries.forEach((item) => item.dispose());
  materials.forEach((item) => item.dispose());
  textures.forEach((item) => item.dispose());
}

function label(text: string, color: string, width = 0.12): THREE.Sprite {
  const canvas = document.createElement("canvas");
  canvas.width = 256;
  canvas.height = 64;
  const ctx = canvas.getContext("2d");
  if (ctx) {
    ctx.font = "600 40px Arial, sans-serif";
    const measuredWidth = ctx.measureText(text).width;
    if (measuredWidth > 240) ctx.font = `600 ${Math.floor(40 * 240 / measuredWidth)}px Arial, sans-serif`;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillStyle = color;
    ctx.fillText(text, 128, 32);
  }
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, transparent: true, depthWrite: false }));
  sprite.scale.set(width, width / 4, 1);
  return sprite;
}

function poseGroup(color: number, name: string): THREE.Group {
  const group = new THREE.Group();
  const isTarget = name === "TARGET";
  const marker = new THREE.Mesh(isTarget ? new THREE.OctahedronGeometry(0.013) : new THREE.SphereGeometry(0.007, 16, 12), new THREE.MeshBasicMaterial({ color, wireframe: isTarget }));
  group.add(marker);
  const axes = new THREE.AxesHelper(0.075);
  for (const material of Array.isArray(axes.material) ? axes.material : [axes.material]) material.depthTest = false;
  axes.renderOrder = 20;
  group.add(axes);
  const caption = label(name, `#${color.toString(16).padStart(6, "0")}`, 0.13);
  caption.name = "pose-caption";
  caption.position.set(isTarget ? 0 : -0.08, 0, isTarget ? 0.102 : 0.025);
  group.add(caption);
  return group;
}

function setPose(object: THREE.Object3D, pose: Pose): void {
  object.position.fromArray(pose.position).multiplyScalar(SCALE);
  const [w, x, y, z] = pose.orientation;
  object.quaternion.set(x, y, z, w).normalize();
}

function worldRobotPoses(props: RobotSceneProps): { flange: Pose; tcp: Pose } | null {
  const flangeInBase = props.urdfModel
    ? forwardUrdfKinematics(props.urdfModel.description, props.urdfJoints ?? props.urdfModel.description.homeJoints).toolPose
    : forwardKinematics(props.model, props.joints).pose;
  if (!flangeInBase) return null;
  const flange = composePose(props.basePose ?? identityPose(), flangeInBase);
  return { flange, tcp: composePose(flange, props.toolPose ?? identityPose()) };
}

function updateHierarchy(state: SceneState, props: RobotSceneProps): void {
  const frames = props.sceneFrames ?? [], tools = props.sceneTools ?? [];
  for (const [id, visual] of state.referenceFrames) if (!frames.some(frame => frame.id === id)) {
    state.scene.remove(visual.root); disposeTree(visual.root); state.referenceFrames.delete(id);
  }
  for (const frame of frames) {
    let visual = state.referenceFrames.get(frame.id);
    if (visual && visual.name !== frame.name) {
      state.scene.remove(visual.root); disposeTree(visual.root); state.referenceFrames.delete(frame.id); visual = undefined;
    }
    if (!visual) {
      visual = { root: poseGroup(0x60896f, frame.name), name: frame.name };
      state.referenceFrames.set(frame.id, visual); state.scene.add(visual.root);
    }
    setPose(visual.root, frame.pose);
    visual.root.scale.setScalar(frame.selected ? 1.15 : 1);
  }
  for (const [id, visual] of state.tools) if (!tools.some(tool => tool.id === id)) {
    state.scene.remove(visual.root); disposeTree(visual.root); state.tools.delete(id);
  }
  for (const tool of tools) {
    const localTcp = relativePose(tool.pose, tool.tcpPose);
    const end = new THREE.Vector3(...localTcp.position).multiplyScalar(SCALE), length = end.length();
    const signature = JSON.stringify([tool.name, tool.radiusMm, Number(length.toFixed(9))]);
    let visual = state.tools.get(tool.id);
    if (!visual || visual.signature !== signature) {
      if (visual) { state.scene.remove(visual.root); disposeTree(visual.root); }
      const root = new THREE.Group();
      const body = new THREE.Mesh(new THREE.CapsuleGeometry(Math.max(tool.radiusMm, .001) * SCALE, length, 6, 12),
        new THREE.MeshStandardMaterial({ color: 0x56a0b6, transparent: true, opacity: .55, roughness: .48, metalness: .25 }));
      const tcp = new THREE.Group(); tcp.add(new THREE.AxesHelper(.045));
      const caption = label(tool.name, '#386f80', .16); caption.position.set(0, 0, .05);
      root.add(body, tcp, caption); state.scene.add(root);
      visual = { root, body, tcp, signature }; state.tools.set(tool.id, visual);
    }
    setPose(visual.root, tool.pose); setPose(visual.tcp, localTcp);
    visual.body.position.copy(end).multiplyScalar(.5);
    visual.body.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), length > 1e-12 ? end.clone().normalize() : new THREE.Vector3(0, 1, 0));
    (visual.body.material as THREE.MeshStandardMaterial).opacity = tool.selected ? .8 : .55;
    (visual.body.material as THREE.MeshStandardMaterial).emissive.setHex(tool.selected ? 0x163b45 : 0);
  }
  state.dirty = true;
}

function updateJointHighlight(state: SceneState, props: RobotSceneProps): void {
  const index = props.selectedJointIndex;
  state.selectedJointIndex = typeof index === 'number' && Number.isInteger(index) && index >= 0 && index < 6 ? index : null;
  state.jointHighlight.visible = props.robotPresent !== false && state.selectedJointIndex !== null;
  if (state.selectedJointIndex === null) return;
  let position: THREE.Vector3, axis: THREE.Vector3;
  if (props.urdfModel) {
    const robot = props.urdfModel.description, joint = robot.actuatedJoints[state.selectedJointIndex];
    const transforms = urdfLinkMatrices(robot, props.urdfJoints ?? robot.homeJoints);
    const frame = transforms.get(joint.parent)!.clone().multiply(urdfOriginMatrix(joint.origin));
    position = new THREE.Vector3().setFromMatrixPosition(frame);
    axis = new THREE.Vector3(...joint.axis).transformDirection(frame);
  } else {
    const fk = forwardKinematics(props.model, props.joints);
    position = new THREE.Vector3(...fk.jointOrigins[state.selectedJointIndex]).multiplyScalar(SCALE);
    axis = new THREE.Vector3(...fk.jointAxes[state.selectedJointIndex]);
  }
  position.applyQuaternion(state.robot.quaternion).add(state.robot.position);
  axis.applyQuaternion(state.robot.quaternion).normalize();
  state.jointHighlight.position.copy(position);
  state.jointHighlight.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), axis);
}

function createDangerVisual(id: string): DangerVisual {
  const root = new THREE.Group();
  root.name = `Danger zone ${id}`;
  const body = new THREE.Group();
  const geometry = new THREE.BoxGeometry(1, 1, 1);
  const mesh = new THREE.Mesh(geometry, new THREE.MeshBasicMaterial({
    color: 0xeb3944, transparent: true, opacity: 0.1, depthWrite: false, side: THREE.DoubleSide,
  }));
  mesh.renderOrder = 1;
  const edges = new THREE.LineSegments(new THREE.EdgesGeometry(geometry), new THREE.LineBasicMaterial({ color: 0xd9343f, transparent: true, opacity: 0.65, depthWrite: false }));
  edges.renderOrder = 2;
  body.add(mesh, edges);
  const text = `DANGER ${id.slice(0, 8)}`;
  const caption = label(text, "#c9343e", 0.17);
  root.add(body, caption);
  return { root, body, mesh, edges, caption, text };
}

function updateDangerZones(state: SceneState, zones: RobotSceneProps["dangerZones"] = [], selectedId?: string): void {
  const retained = new Set(zones.map(zone => zone.id));
  for (const [id, visual] of state.dangerZones) {
    if (!retained.has(id)) {
      state.scene.remove(visual.root);
      disposeTree(visual.root);
      state.dangerZones.delete(id);
    }
  }
  for (const zone of zones) {
    let visual = state.dangerZones.get(zone.id);
    if (!visual) {
      visual = createDangerVisual(zone.id);
      state.dangerZones.set(zone.id, visual);
      state.scene.add(visual.root);
    }
    const valid = usableZone(zone);
    visual.root.visible = Boolean(valid);
    if (valid) {
      visual.root.position.fromArray(zone.centerMm).multiplyScalar(SCALE);
      visual.body.scale.fromArray(zone.sizeMm).multiplyScalar(SCALE);
      const name = typeof zone.name === "string" && zone.name.trim() ? zone.name.trim() : `DANGER ${zone.id.slice(0, 8)}`;
      const text = Array.from(name).slice(0, 28).join("");
      if (visual.text !== text) {
        visual.root.remove(visual.caption);
        disposeTree(visual.caption);
        visual.caption = label(text, "#c9343e", 0.17);
        visual.text = text;
        visual.root.add(visual.caption);
      }
      visual.caption.position.set(0, 0, zone.sizeMm[2] * SCALE / 2 + 0.04);
    }
    const selected = zone.id === selectedId;
    visual.mesh.material.opacity = selected ? 0.16 : 0.08;
    visual.edges.material.opacity = selected ? 0.95 : 0.55;
    visual.edges.material.color.setHex(selected ? 0xec2938 : 0xc75a62);
  }
  state.dirty = true;
}

function updateScenePaths(state: SceneState, paths: ScenePath[] = [], selectedPointId?: string | null): void {
  const retained = new Set(paths.map(path => path.id));
  for (const [id, visual] of state.scenePaths) if (!retained.has(id)) {
    state.scene.remove(visual.root);
    disposeTree(visual.root);
    state.scenePaths.delete(id);
  }
  let selected: ScenePath["points"][number] | undefined;
  for (const path of paths) {
    let visual = state.scenePaths.get(path.id);
    if (!visual) {
      const root = new THREE.Group();
      const geometry = new THREE.BufferGeometry();
      const line = new THREE.Line(geometry, new THREE.LineDashedMaterial({
        color: 0x507aa5, dashSize: 0.018, gapSize: 0.012, transparent: true, opacity: 0.65, depthWrite: false,
      }));
      const points = new THREE.Points(geometry, new THREE.PointsMaterial({ color: 0x507aa5, size: 6, sizeAttenuation: false, depthWrite: false }));
      line.renderOrder = 5; points.renderOrder = 6;
      root.add(line, points);
      visual = { root, line, points, signature: "", source: path };
      state.scenePaths.set(path.id, visual);
      state.scene.add(root);
    }
    visual.root.name = `Path ${path.name}`;
    visual.source = path;
    const valid = path.points.every(point => validScenePose(point.pose));
    visual.root.visible = valid && path.points.length > 0;
    const signature = JSON.stringify(path.points.map(point => point.pose.position));
    if (valid && signature !== visual.signature) {
      visual.line.geometry.dispose(); // Shared by line and point markers.
      const geometry = new THREE.BufferGeometry().setFromPoints(path.points.map(point => new THREE.Vector3(...point.pose.position).multiplyScalar(SCALE)));
      visual.line.geometry = geometry;
      visual.points.geometry = geometry;
      visual.line.computeLineDistances();
      visual.signature = signature;
    }
    visual.line.visible = path.points.length > 1;
    visual.line.material.color.setHex(path.selected ? 0x2c6fa5 : 0x7892ac);
    visual.line.material.opacity = path.selected ? 0.95 : 0.5;
    visual.points.material.color.setHex(path.selected ? 0x2c6fa5 : 0x7892ac);
    if (valid) selected ??= path.points.find(point => point.id === selectedPointId);
  }
  state.selectedPoint.visible = Boolean(selected);
  state.selectedPointId = selected?.id ?? null;
  if (selected) {
    setPose(state.selectedPoint, selected.pose);
    const text = Array.from(selected.name?.trim() || selected.id).slice(0, 28).join("");
    if (text !== state.selectedPointText) {
      state.selectedPoint.remove(state.selectedPointCaption);
      disposeTree(state.selectedPointCaption);
      const caption = label(text, "#94631d", 0.15);
      caption.position.set(0, 0, 0.066);
      state.selectedPoint.add(caption);
      state.selectedPointCaption = caption;
      state.selectedPointText = text;
    }
  }
  state.dirty = true;
}

function adjustSurfaceClipping(state: SceneState): void {
  if (!state.surfaces.size) return;
  let far = 12, radius = Infinity;
  for (const visual of state.surfaces.values()) if (visual.root.visible) {
    const bounds = new THREE.Box3().setFromObject(visual.root);
    if (bounds.isEmpty()) continue;
    const size = bounds.getSize(new THREE.Vector3()).length() / 2;
    radius = Math.min(radius, Math.max(size, .01));
    far = Math.max(far, state.camera.position.distanceTo(bounds.getCenter(new THREE.Vector3())) + size * 3);
  }
  const near = Math.min(.01, Math.max(.00001, radius / 1000));
  if (near !== state.camera.near || Math.abs(far - state.camera.far) > .01) {
    state.camera.near = near; state.camera.far = far; state.camera.updateProjectionMatrix();
  }
  const distance = state.camera.position.distanceTo(state.controls.target);
  if (state.scene.fog instanceof THREE.Fog) {
    state.scene.fog.near = Math.max(2.8, distance * 2);
    state.scene.fog.far = Math.max(6, distance * 4);
  }
}

function updateSurfaces(state: SceneState, props: RobotSceneProps): void {
  const surfaces = props.sceneSurfaces ?? [];
  const retained = new Set(surfaces.map(surface => surface.id));
  for (const [id, visual] of state.surfaces) if (!retained.has(id)) {
    disposeSurfaceVisual(visual); state.surfaces.delete(id);
  }
  for (const surface of surfaces) {
    const selected = surface.id === props.selectedSurfaceId;
    const visual = state.surfaces.get(surface.id);
    if (visual) updateSurfaceVisual(visual, surface, selected);
    else { const next = createSurfaceVisual(surface, selected); state.surfaces.set(surface.id, next); state.scene.add(next.root); }
  }
  state.surfaceRegionSource = props.surfaceRegion ?? null;
  state.surfacePickingEnabled = Boolean(props.surfacePickingEnabled);
  updateSurfaceRegionVisual(state.surfaceRegion, props.surfaceRegion, surfaces);
  state.renderer.domElement.style.cursor = props.surfacePickingEnabled ? 'crosshair' : '';
  adjustSurfaceClipping(state);
  state.renderer.shadowMap.needsUpdate = true;
  state.dirty = true;
}

function focusSurface(state: SceneState, id: string): boolean {
  const visual = state.surfaces.get(id);
  if (!visual?.root.visible) return false;
  visual.root.updateMatrixWorld(true);
  const bounds = new THREE.Box3().setFromObject(visual.root);
  if (bounds.isEmpty()) return false;
  const center = bounds.getCenter(new THREE.Vector3()), radius = Math.max(.01, bounds.getSize(new THREE.Vector3()).length() / 2);
  if (![...center.toArray(), radius].every(Number.isFinite)) return false;
  state.finishDrag();
  const distance = radius / Math.sin(THREE.MathUtils.degToRad(state.camera.fov / 2)) * 1.3 / Math.min(1, state.camera.aspect);
  const direction = state.camera.position.clone().sub(state.controls.target);
  if (direction.lengthSq() < 1e-10) direction.set(1, -1.3, .8);
  state.controls.target.copy(center);
  state.camera.position.copy(center).add(direction.normalize().multiplyScalar(distance));
  state.camera.near = Math.min(.01, Math.max(.00001, radius / 1000));
  state.camera.far = Math.max(12, distance + center.length() * 2 + radius * 8);
  state.camera.updateProjectionMatrix();
  state.controls.minDistance = Math.min(.2, Math.max(.001, radius / 8));
  state.controls.maxDistance = Math.max(state.controls.maxDistance, distance * 6);
  state.camera.lookAt(center); state.controls.update();
  state.zoomTarget = state.camera.position.distanceTo(center);
  if (state.scene.fog instanceof THREE.Fog) {
    state.scene.fog.near = Math.max(2.8, distance * 2);
    state.scene.fog.far = Math.max(6, distance * 4);
  }
  state.dirty = true;
  return true;
}

function setView(state: SceneState, view: RobotSceneProps["view"]): void {
  if (state.genericFocus) {
    const focus = state.genericFocus;
    const distance = state.genericRadius / Math.sin(THREE.MathUtils.degToRad(state.camera.fov / 2)) * 1.2 / Math.min(1, state.camera.aspect);
    const direction = view === "top" ? new THREE.Vector3(0, -0.001, 1)
      : view === "front" ? new THREE.Vector3(0, -1, 0.07) : new THREE.Vector3(1, -1.3, 0.8);
    state.controls.target.copy(focus);
    state.camera.up.set(0, 0, 1);
    state.camera.position.copy(focus).add(direction.normalize().multiplyScalar(distance));
    state.camera.lookAt(focus);
    state.controls.update();
    state.zoomTarget = state.camera.position.distanceTo(focus);
    state.dirty = true;
    return;
  }
  const focus = new THREE.Vector3(0.13, 0, 0.31).applyQuaternion(state.robot.quaternion).add(state.robot.position);
  state.controls.target.copy(focus);
  state.camera.up.set(0, 0, 1);
  if (view === "top") state.camera.position.copy(focus).add(new THREE.Vector3(0, -.001, 1.59));
  else if (view === "front") state.camera.position.copy(focus).add(new THREE.Vector3(0, -1.85, .12));
  else state.camera.position.copy(focus).add(new THREE.Vector3(.82, -1.05, .54));
  state.camera.lookAt(focus);
  state.controls.update();
  state.zoomTarget = state.camera.position.distanceTo(state.controls.target);
  state.dirty = true;
}

function updateRobot(state: SceneState, props: RobotSceneProps): void {
  state.robot.visible = props.robotPresent !== false;
  setPose(state.robot, props.basePose ?? identityPose());
  setPose(state.ghost, props.basePose ?? identityPose());
  const world = worldRobotPoses(props);
  if (world) {
    setPose(state.flange, world.flange); setPose(state.tcp, world.tcp);
    if (state.draggingPointer === null && !props.jogPending) setPose(state.jogObject, world.tcp);
  }
  updateJointHighlight(state, props);
  if (props.urdfModel) {
    const description = props.urdfModel.description;
    const joints = props.urdfJoints ?? description.homeJoints;
    const transforms = urdfLinkMatrices(description, joints);
    state.genericLinks.forEach(({ mesh, link, visual }) => {
      mesh.matrix.copy(transforms.get(link)!).multiply(urdfOriginMatrix(visual.origin))
        .scale(new THREE.Vector3(...visual.scale));
      mesh.matrixWorldNeedsUpdate = true;
    });
    state.tcp.visible = props.robotPresent !== false && world !== null;
    if (state.genericFocusLocal) state.genericFocus = state.genericFocusLocal.clone().applyQuaternion(state.robot.quaternion).add(state.robot.position);
    state.genericJoints = [...joints];
    state.robotJoints = [...joints];
    state.frames.visible = props.robotPresent !== false && Boolean(props.showFrames);
    state.frames.children.forEach((frame, index) => {
      const joint = description.actuatedJoints[index - 1];
      const local = index === 0 ? new THREE.Matrix4() : transforms.get(joint.child)!.clone();
      frame.matrix.compose(state.robot.position, state.robot.quaternion, new THREE.Vector3(1, 1, 1)).multiply(local);
      frame.matrixWorldNeedsUpdate = true;
    });
    state.ghost.visible = false;
    state.renderer.shadowMap.needsUpdate = true;
    state.dirty = true;
    return;
  }
  state.tcp.visible = props.robotPresent !== false;
  const fk = forwardKinematics(props.model, props.joints);
  state.links.forEach((link, i) => {
    link.matrix.copy(matrix(fk.frames[i]).multiply(state.zeroInverse[i]));
    link.matrixWorldNeedsUpdate = true;
  });
  state.robotJoints = [...props.joints];
  const tcpCaption = state.tcp.getObjectByName("pose-caption");
  if (tcpCaption) tcpCaption.visible = !props.targetPose || Math.hypot(...world!.tcp.position.map((value, i) => value - props.targetPose!.position[i])) > 5;
  state.frames.visible = props.robotPresent !== false && Boolean(props.showFrames);
  state.frames.children.forEach((frame, i) => {
    frame.matrix.compose(state.robot.position, state.robot.quaternion, new THREE.Vector3(1, 1, 1)).multiply(matrix(fk.frames[i], true));
    frame.matrixWorldNeedsUpdate = true;
  });
  const differs = props.endJoints?.some((angle, i) => Math.abs(angle - props.joints[i]) > 0.1);
  state.ghost.visible = props.robotPresent !== false && Boolean(differs && state.ghostLinks.length);
  if (props.endJoints && differs) {
    const end = forwardKinematics(props.model, props.endJoints);
    state.ghostLinks.forEach((link, i) => {
      link.matrix.copy(matrix(end.frames[i]).multiply(state.zeroInverse[i]));
      link.matrixWorldNeedsUpdate = true;
    });
  }
  state.renderer.shadowMap.needsUpdate = true;
  state.dirty = true;
}

/** Development-only, read-only diagnostics for real pointer regression tests. */
function updateDiagnostics(state: SceneState): void {
  const { camera, controls, renderer, transform, jogObject } = state;
  const canvas = renderer.domElement;
  const rect = canvas.getBoundingClientRect();
  state.scene.updateMatrixWorld(true);
  canvas.dataset.cameraDistance = camera.position.distanceTo(controls.target).toString();
  canvas.dataset.cameraPosition = JSON.stringify(camera.position.toArray());
  canvas.dataset.cameraTarget = JSON.stringify(controls.target.toArray());
  canvas.dataset.cameraUp = JSON.stringify(camera.up.toArray());
  canvas.dataset.cameraFov = camera.fov.toString();
  canvas.dataset.cameraAspect = camera.aspect.toString();
  canvas.dataset.zoomTarget = state.zoomTarget.toString();
  canvas.dataset.robotVisible = String(state.robot.visible);
  canvas.dataset.jointFramesVisible = String(state.frames.visible);
  canvas.dataset.urdfModel = state.genericModelId ?? "";
  canvas.dataset.urdfVisualCount = state.genericLinks.length.toString();
  canvas.dataset.urdfJoints = JSON.stringify(state.genericJoints);
  canvas.dataset.robotJoints = JSON.stringify(state.robotJoints);
  const actualPose = (object: THREE.Object3D): Pose => {
    const p = object.getWorldPosition(new THREE.Vector3()), q = object.getWorldQuaternion(new THREE.Quaternion());
    return { position: p.multiplyScalar(1000).toArray() as Vec3, orientation: [q.w, q.x, q.y, q.z] };
  };
  canvas.dataset.robotBasePose = canvas.dataset.baseWorldPose = JSON.stringify(actualPose(state.robot));
  canvas.dataset.flangePose = JSON.stringify(actualPose(state.flange));
  canvas.dataset.tcpWorldPose = state.tcp.visible ? JSON.stringify(actualPose(state.tcp)) : 'null';
  canvas.dataset.sceneFrames = JSON.stringify([...state.referenceFrames].map(([id, visual]) => ({ id, pose: actualPose(visual.root) })));
  canvas.dataset.sceneTools = JSON.stringify([...state.tools].map(([id, visual]) => ({ id, pose: actualPose(visual.root), tcpPose: actualPose(visual.tcp), radiusMm: (visual.body.geometry as THREE.CapsuleGeometry).parameters.radius * 1000 })));
  canvas.dataset.selectedJointIndex = state.selectedJointIndex === null ? '' : String(state.selectedJointIndex);
  canvas.dataset.selectedJointPose = state.jointHighlight.visible ? JSON.stringify(actualPose(state.jointHighlight)) : 'null';
  const previewPositions = state.path.geometry.getAttribute('position');
  const previewPoint = (index: number) => new THREE.Vector3().fromBufferAttribute(previewPositions, index)
    .applyMatrix4(state.path.matrixWorld).multiplyScalar(1000).toArray();
  canvas.dataset.previewPath = JSON.stringify({ visible: state.path.visible, count: previewPositions?.count ?? 0,
    startMm: previewPositions?.count ? previewPoint(0) : null,
    endMm: previewPositions?.count ? previewPoint(previewPositions.count - 1) : null });
  canvas.dataset.previewTargetPose = state.target.visible ? JSON.stringify(actualPose(state.target)) : 'null';
  const bodyPosition = state.tcp.getWorldPosition(new THREE.Vector3());
  const bodyQuaternion = state.tcp.getWorldQuaternion(new THREE.Quaternion());
  canvas.dataset.robotPose = state.tcp.visible ? JSON.stringify({
    position: bodyPosition.toArray().map(value => value / SCALE),
    orientation: [bodyQuaternion.w, bodyQuaternion.x, bodyQuaternion.y, bodyQuaternion.z],
  }) : "null";
  canvas.dataset.urdfToolPose = state.genericModelId && state.tcp.visible ? JSON.stringify({
    position: state.tcp.position.toArray().map(v => v / SCALE),
    orientation: [state.tcp.quaternion.w, state.tcp.quaternion.x, state.tcp.quaternion.y, state.tcp.quaternion.z],
  }) : "null";
  if (state.genericModelId) {
    const bounds = new THREE.Box3().setFromObject(state.robot);
    canvas.dataset.urdfBounds = JSON.stringify({ min: bounds.min.toArray(), max: bounds.max.toArray() });
  } else delete canvas.dataset.urdfBounds;
  canvas.dataset.gizmoTarget = state.manipulationTarget;
  canvas.dataset.zoneId = state.manipulatedZoneId ?? "";
  canvas.dataset.zoneDragging = String(state.draggingPointer !== null && state.manipulationTarget === "zone");
  canvas.dataset.zonePositionMm = JSON.stringify(state.zoneObject.position.toArray().map(value => value / SCALE));
  canvas.dataset.orbitEnabled = String(controls.enabled);
  canvas.dataset.jogDragging = String(state.draggingPointer !== null && state.manipulationTarget === "tcp");
  canvas.dataset.jogMode = transform.mode;
  canvas.dataset.jogSpace = transform.space;
  canvas.dataset.jogAxis = state.manipulationTarget === "tcp" ? transform.axis ?? "" : "";
  canvas.dataset.zoneAxis = state.manipulationTarget === "zone" ? transform.axis ?? "" : "";
  canvas.dataset.jogPose = JSON.stringify({ position: jogObject.position.toArray().map(v => v / SCALE), orientation: [jogObject.quaternion.w, jogObject.quaternion.x, jogObject.quaternion.y, jogObject.quaternion.z] });
  const handles: Record<string, { x: number; y: number }> = {};
  if (transform.enabled && transform.getHelper().visible) {
    const gizmo = transform.getHelper().children.find(child => (child as TransformControlsGizmo).isTransformControlsGizmo) as TransformControlsGizmo | undefined;
    const picker = gizmo?.picker[transform.mode];
    const raycaster = new THREE.Raycaster();
    const origin = (state.manipulationTarget === "zone" ? state.zoneObject : jogObject).position.clone().project(camera);
    if (picker) for (const axis of ["X", "Y", "Z"]) {
      const meshes = picker.children.filter(child => child.name === axis && child.visible) as THREE.Mesh[];
      for (const mesh of meshes) {
        const geometry = mesh.geometry;
        const attribute = geometry.getAttribute("position");
        if (!attribute) continue;
        geometry.computeBoundingBox();
        const points: THREE.Vector3[] = [geometry.boundingBox!.getCenter(new THREE.Vector3())];
        // Torus centers are not pickable. Try real triangle centers along the ring.
        const index = geometry.getIndex();
        const count = index ? index.count : attribute.count;
        const stride = Math.max(3, Math.ceil(count / 36 / 3) * 3);
        for (let face = 0; face + 2 < count; face += stride) {
          const center = new THREE.Vector3();
          for (let k = 0; k < 3; k++) center.add(new THREE.Vector3().fromBufferAttribute(attribute, index ? index.getX(face + k) : face + k));
          points.push(center.multiplyScalar(1 / 3));
        }
        for (const point of points) {
          const projected = point.applyMatrix4(mesh.matrixWorld).project(camera);
          if (Math.abs(projected.x) > 0.95 || Math.abs(projected.y) > 0.95 || projected.z < -1 || projected.z > 1) continue;
          if (Math.hypot((projected.x - origin.x) * rect.width / 2, (projected.y - origin.y) * rect.height / 2) < 24) continue;
          raycaster.setFromCamera(new THREE.Vector2(projected.x, projected.y), camera);
          const hit = raycaster.intersectObject(picker, true).find(item => item.object.visible);
          if (hit?.object.name !== axis) continue;
          handles[axis] = { x: rect.left + (projected.x + 1) * rect.width / 2, y: rect.top + (1 - projected.y) * rect.height / 2 };
          break;
        }
        if (handles[axis]) break;
      }
    }
  }
  canvas.dataset.jogHandles = JSON.stringify(state.manipulationTarget === "tcp" ? handles : {});
  canvas.dataset.zoneHandles = JSON.stringify(state.manipulationTarget === "zone" ? handles : {});
  const dangerZones = [...state.dangerZones].map(([id, visual]) => {
    if (!visual.root.visible) return { id, enabled: false };
    const bounds = new THREE.Box3().setFromObject(visual.body);
    return { id, enabled: true,
      minMm: bounds.min.toArray().map(value => value / SCALE),
      maxMm: bounds.max.toArray().map(value => value / SCALE),
    };
  });
  canvas.dataset.dangerZones = JSON.stringify(dangerZones);
  // Retain the single-zone diagnostic alias for existing read-only tooling.
  canvas.dataset.dangerZone = JSON.stringify(dangerZones[0] ?? { enabled: false });
  canvas.dataset.scenePaths = JSON.stringify([...state.scenePaths.values()].map(visual => ({
    id: visual.source.id, name: visual.source.name, selected: Boolean(visual.source.selected), visible: visual.root.visible,
    points: visual.source.points.map(point => ({ id: point.id, name: point.name ?? point.id, position: point.pose.position, orientation: point.pose.orientation })),
  })));
  canvas.dataset.sceneSurfaces = JSON.stringify(surfaceDiagnostics([...state.surfaces.values()], camera, rect));
  canvas.dataset.surfacePicking = String(state.surfacePickingEnabled);
  canvas.dataset.surfaceRegion = JSON.stringify(state.surfaceRegionSource);
  canvas.dataset.surfaceRegionVisible = String(state.surfaceRegion.root.visible);
  canvas.dataset.selectedPointId = state.selectedPointId ?? "";
  canvas.dataset.selectedPointPose = state.selectedPoint.visible ? JSON.stringify({
    position: state.selectedPoint.position.toArray().map(value => value / SCALE),
    orientation: [state.selectedPoint.quaternion.w, state.selectedPoint.quaternion.x, state.selectedPoint.quaternion.y, state.selectedPoint.quaternion.z],
  }) : "null";
}

const RobotScene = forwardRef<RobotSceneHandle, RobotSceneProps>(function RobotScene(props, ref) {
  const host = useRef<HTMLDivElement>(null);
  const state = useRef<SceneState | null>(null);
  const latest = useRef(props);
  latest.current = props;
  const [loaded, setLoaded] = useState(0);
  const [loadError, setLoadError] = useState<string | null>(null);
  const [graphicsError, setGraphicsError] = useState<string | null>(null);
  const [compatibilityGraphics, setCompatibilityGraphics] = useState(false);
  const [generation, setGeneration] = useState(0);
  const [mounted, setMounted] = useState(false);
  const selectedZone = props.dangerZones?.find(zone => zone.id === props.selectedDangerZoneId);
  const zoneTargetId = !props.surfacePickingEnabled && props.zoneManipulationEnabled && props.onDangerZoneMove && usableZone(selectedZone) ? selectedZone.id : null;
  const tcpManipulationEnabled = !props.surfacePickingEnabled && props.robotPresent !== false && !props.zoneManipulationEnabled && Boolean(props.jogEnabled);
  const gizmoMode = zoneTargetId ? "translate" : props.jogMode ?? "translate";
  const gizmoSpace = zoneTargetId ? "world" : props.jogSpace ?? "world";
  const gizmoResetRevision = zoneTargetId ? 0 : props.jogResetRevision;

  useEffect(() => {
    const element = host.current;
    if (!element) return;
    let renderer: THREE.WebGLRenderer;
    try {
      const initialized = createValidatedRenderer();
      renderer = initialized.renderer;
      setCompatibilityGraphics(initialized.compatibility);
      setGraphicsError(null);
    } catch (cause) {
      setGraphicsError(cause instanceof Error ? cause.message : "无法启动 3D 场景。请开启图形加速后重试。");
      return;
    }
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1;
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.shadowMap.autoUpdate = false;
    renderer.domElement.setAttribute("aria-label", "ABB IRB 1100 三维仿真场景");
    renderer.domElement.setAttribute("role", "img");
    element.appendChild(renderer.domElement);

    const scene = new THREE.Scene();
    scene.fog = new THREE.Fog(0xf0f3f6, 2.8, 6);
    const camera = new THREE.PerspectiveCamera(37, 1, 0.01, 12);
    camera.up.set(0, 0, 1);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.08;
    controls.minDistance = 0.2;
    // OrbitControls damps rotation, but changes wheel radius immediately.
    controls.enableZoom = false;
    controls.maxDistance = 4.5;
    controls.maxPolarAngle = Math.PI / 2 - 0.025;
    controls.enablePan = true;
    controls.screenSpacePanning = false;
    scene.add(new THREE.HemisphereLight(0xffffff, 0x8b929e, 1.6));
    const key = new THREE.DirectionalLight(0xffffff, 2.5);
    key.position.set(0.5, -0.9, 2.1);
    key.castShadow = true;
    key.shadow.mapSize.set(1024, 1024);
    Object.assign(key.shadow.camera, { left: -1.2, right: 1.2, top: 1.2, bottom: -1.2, near: 0.1, far: 5 });
    key.shadow.bias = -0.0002;
    key.shadow.normalBias = 0.002;
    key.target.position.set(0.12, 0, 0.3);
    scene.add(key, key.target);
    const fill = new THREE.DirectionalLight(0xcbdcff, 1.2);
    fill.position.set(-0.8, 1.2, 0.9);
    scene.add(fill);

    const ground = new THREE.Mesh(new THREE.PlaneGeometry(12, 12), new THREE.MeshStandardMaterial({ color: 0xe9edf2, roughness: 1 }));
    ground.position.z = -0.001;
    ground.receiveShadow = true;
    scene.add(ground);
    const grid = new THREE.GridHelper(2.4, 24, 0xc4ccd7, 0xd6dde6);
    grid.rotation.x = Math.PI / 2;
    grid.position.z = 0.0001;
    const gridMaterial = grid.material as THREE.Material;
    gridMaterial.transparent = true;
    gridMaterial.opacity = 0.72;
    scene.add(grid);

    const origin = new THREE.Group();
    origin.add(new THREE.AxesHelper(0.19));
    AXIS_COLORS.forEach((color, i) => {
      const axisLabel = label(["X", "Y", "Z"][i], `#${color.toString(16)}`, 0.065);
      axisLabel.position.setComponent(i, 0.215);
      origin.add(axisLabel);
    });
    const originLabel = label("世界原点 · 0, 0, 0", "#7a8597", 0.21);
    originLabel.position.set(-0.12, -0.15, 0.018);
    origin.add(originLabel);
    scene.add(origin);
    const robot = new THREE.Group();
    const ghost = new THREE.Group();
    robot.scale.setScalar(SCALE);
    ghost.scale.setScalar(SCALE);
    scene.add(robot, ghost);
    const frames = new THREE.Group();
    for (let i = 0; i < 7; i++) {
      const axes = new THREE.AxesHelper(0.048);
      axes.matrixAutoUpdate = false;
      frames.add(axes);
    }
    scene.add(frames);
    const tcp = poseGroup(0x426f98, "TCP");
    const target = poseGroup(0xd33840, "TARGET");
    target.visible = false;
    const flange = new THREE.Object3D();
    const jointHighlight = poseGroup(0xd19137, 'AXIS'); jointHighlight.visible = false;
    scene.add(tcp, target, flange, jointHighlight);
    const path = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineDashedMaterial({ color: 0xd64d53, dashSize: 0.014, gapSize: 0.009, transparent: true, opacity: 0.9 }));
    path.visible = false;
    path.renderOrder = 4;
    const start = new THREE.Mesh(new THREE.SphereGeometry(0.006, 14, 10), new THREE.MeshBasicMaterial({ color: 0x547899 }));
    start.visible = false;
    scene.add(path, start);
    const jogObject = new THREE.Object3D();
    jogObject.name = "Requested world TCP pose";
    scene.add(jogObject);
    const zoneObject = new THREE.Object3D();
    zoneObject.name = "Selected danger zone center";
    scene.add(zoneObject);
    const selectedPoint = new THREE.Group();
    selectedPoint.name = "Selected Path point";
    const pointAxes = new THREE.AxesHelper(0.06);
    for (const material of Array.isArray(pointAxes.material) ? pointAxes.material : [pointAxes.material]) material.depthTest = false;
    pointAxes.renderOrder = 22;
    const selectedMarker = new THREE.Points(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3()]), new THREE.PointsMaterial({
      color: 0xe1a13a, size: 11, sizeAttenuation: false, depthTest: false, depthWrite: false,
    }));
    selectedMarker.renderOrder = 23;
    const selectedPointCaption = label("POINT", "#94631d", 0.15);
    selectedPointCaption.position.set(0, 0, 0.066);
    selectedPoint.add(pointAxes, selectedMarker, selectedPointCaption);
    selectedPoint.visible = false;
    scene.add(selectedPoint);
    const transform = new TransformControls(camera, renderer.domElement);
    // Route gizmo events in capture phase so OrbitControls never starts the same drag.
    transform.disconnect();
    renderer.domElement.style.touchAction = "none";
    transform.setSize(0.8);
    transform.showXYZE = false;
    transform.showE = false;
    transform.attach(jogObject);
    transform.enabled = false;
    const helper = transform.getHelper();
    helper.visible = false;
    scene.add(helper);
    const surfaceRegion = createSurfaceRegionVisual(); scene.add(surfaceRegion.root);
    const local: SceneState = { scene, camera, controls, renderer, robot, ghost, frames, tcp, flange, jointHighlight, selectedJointIndex: null, referenceFrames: new Map(), tools: new Map(), target, path, start, links: [], ghostLinks: [], zeroInverse: [], dirty: true,
      zoomTarget: 1, jogObject, zoneObject, manipulationTarget: "none", manipulatedZoneId: null, transform, finishDrag: () => {}, draggingPointer: null, dangerZones: new Map(),
      scenePaths: new Map(), surfaces: new Map(), surfaceRegion, surfaceRegionSource: null, surfacePickingEnabled: false, selectedPoint, selectedPointCaption, selectedPointText: "POINT", selectedPointId: null,
      genericLinks: [], genericFocus: null, genericFocusLocal: null, genericRadius: 1, genericModelId: null, genericJoints: [], robotJoints: [], grid, ground, key };
    const canvas = renderer.domElement;
    const poseFromJog = (): Pose => ({ position: jogObject.position.toArray().map(v => v / SCALE) as Pose["position"], orientation: [jogObject.quaternion.w, jogObject.quaternion.x, jogObject.quaternion.y, jogObject.quaternion.z] });
    // Three's public pointer methods consume normalized x/y/button objects.
    const pointer = (event: PointerEvent, button = event.button): PointerEvent => {
      const rect = canvas.getBoundingClientRect();
      return { x: (event.clientX - rect.left) / rect.width * 2 - 1, y: -(event.clientY - rect.top) / rect.height * 2 + 1, button } as PointerEvent;
    };
    const freezeOrbit = () => {
      const position = camera.position.clone(), rotation = camera.quaternion.clone(), focus = controls.target.clone();
      controls.enableDamping = false;
      controls.update(); // Drain pending inertia, then restore the exact visible camera.
      camera.position.copy(position); camera.quaternion.copy(rotation); controls.target.copy(focus);
      controls.update(); controls.enableDamping = true;
      local.zoomTarget = camera.position.distanceTo(controls.target);
      controls.enabled = false;
    };
    const finishDrag = () => {
      const id = local.draggingPointer;
      if (id === null) return;
      const target = local.manipulationTarget;
      local.draggingPointer = null;
      transform.pointerUp({ button: 0 } as PointerEvent);
      if (canvas.hasPointerCapture(id)) canvas.releasePointerCapture(id);
      controls.enabled = true;
      if (target === "zone") latest.current.onDangerZoneDrag?.(false);
      else if (target === "tcp") {
        latest.current.onJogEnd?.();
        if (!latest.current.jogPending) { const world = worldRobotPoses(latest.current); if (world) setPose(jogObject, world.tcp); }
      }
      local.dirty = true;
    };
    local.finishDrag = finishDrag;
    const targetIsEnabled = () => {
      const current = latest.current;
      if (current.surfacePickingEnabled) return false;
      if (local.manipulationTarget === "zone") {
        return Boolean(current.zoneManipulationEnabled && current.selectedDangerZoneId === local.manipulatedZoneId
          && current.onDangerZoneMove
          && usableZone(current.dangerZones?.find(zone => zone.id === local.manipulatedZoneId)));
      }
      return local.manipulationTarget === "tcp" && !current.zoneManipulationEnabled && Boolean(current.robotPresent !== false && current.jogEnabled);
    };
    let surfacePointer: { id: number; x: number; y: number; moved: boolean; picking: boolean;
      surfaceId: string | null; camera: THREE.Vector3; rotation: THREE.Quaternion; focus: THREE.Vector3 } | null = null;
    const clearSurfacePointer = () => { surfacePointer = null; };
    const onPointerDown = (event: PointerEvent) => {
      const current = latest.current;
      surfacePointer = event.button === 0 && event.isPrimary && !event.altKey && !event.ctrlKey && !event.metaKey && !event.shiftKey
        && local.draggingPointer === null && (current.surfacePickingEnabled || current.onSelectSurface)
        ? { id: event.pointerId, x: event.clientX, y: event.clientY, moved: false, picking: Boolean(current.surfacePickingEnabled),
          surfaceId: current.surfacePickSurfaceId ?? null, camera: camera.position.clone(), rotation: camera.quaternion.clone(), focus: controls.target.clone() } : null;
      if (!targetIsEnabled() || (local.manipulationTarget === "tcp" && latest.current.jogPending) || !transform.enabled || event.button !== 0 || local.draggingPointer !== null) return;
      scene.updateMatrixWorld(true);
      transform.pointerHover(pointer(event));
      if (!transform.axis) return;
      surfacePointer = null;
      event.preventDefault(); event.stopImmediatePropagation();
      freezeOrbit();
      transform.pointerDown(pointer(event));
      if (!transform.dragging) { controls.enabled = true; return; }
      local.draggingPointer = event.pointerId;
      canvas.setPointerCapture(event.pointerId);
      if (local.manipulationTarget === "zone") latest.current.onDangerZoneDrag?.(true);
      else latest.current.onJogStart?.();
      local.dirty = true;
    };
    const onPointerMove = (event: PointerEvent) => {
      if (surfacePointer?.id === event.pointerId && Math.hypot(event.clientX - surfacePointer.x, event.clientY - surfacePointer.y) > 3) surfacePointer.moved = true;
      if (local.draggingPointer !== null) {
        if (event.pointerId !== local.draggingPointer) return;
        if (!targetIsEnabled()) { finishDrag(); return; }
        event.preventDefault(); event.stopImmediatePropagation();
        transform.pointerMove(pointer(event, -1));
      } else if (targetIsEnabled() && transform.enabled && event.buttons === 0) {
        scene.updateMatrixWorld(true);
        transform.pointerHover(pointer(event));
      }
    };
    const onPointerUp = (event: PointerEvent) => {
      const started = surfacePointer;
      if (started?.id === event.pointerId) surfacePointer = null;
      const current = latest.current;
      if (event.type !== 'pointercancel' && event.button === 0 && started?.id === event.pointerId && !started.moved
        && !event.altKey && !event.ctrlKey && !event.metaKey && !event.shiftKey && local.draggingPointer === null
        && started.picking === Boolean(current.surfacePickingEnabled) && started.surfaceId === (current.surfacePickSurfaceId ?? null)
        && Math.hypot(event.clientX - started.x, event.clientY - started.y) <= 3
        && camera.position.distanceToSquared(started.camera) < 1e-14 && controls.target.distanceToSquared(started.focus) < 1e-14
        && 1 - Math.abs(camera.quaternion.dot(started.rotation)) < 1e-12) {
        const rect = canvas.getBoundingClientRect();
        if (event.clientX >= rect.left && event.clientX <= rect.right && event.clientY >= rect.top && event.clientY <= rect.bottom) {
          scene.updateMatrixWorld(true);
          const raycaster = new THREE.Raycaster();
          raycaster.setFromCamera(new THREE.Vector2((event.clientX - rect.left) / rect.width * 2 - 1, -(event.clientY - rect.top) / rect.height * 2 + 1), camera);
          const hit = pickSurface(raycaster, [...local.surfaces.values()], current.surfacePickingEnabled ? current.surfacePickSurfaceId : null);
          if (hit) {
            if (current.surfacePickingEnabled) current.onSurfacePick?.(hit);
            else current.onSelectSurface?.(hit.surfaceId);
          }
        }
      }
      if (event.pointerId !== local.draggingPointer) return;
      event.preventDefault(); event.stopImmediatePropagation();
      finishDrag();
    };
    const onLostCapture = (event: PointerEvent) => { if (surfacePointer?.id === event.pointerId) surfacePointer = null; if (event.pointerId === local.draggingPointer) finishDrag(); };
    const onEscape = (event: KeyboardEvent) => { if (event.key === "Escape") { surfacePointer = null; if (local.draggingPointer !== null) { event.preventDefault(); finishDrag(); } } };
    const onWheel = (event: WheelEvent) => {
      event.preventDefault();
      if (!controls.enabled || local.draggingPointer !== null) return;
      const delta = normalizeWheelDelta(event.deltaY, event.deltaMode, canvas.clientHeight, event.ctrlKey);
      local.zoomTarget = THREE.MathUtils.clamp(local.zoomTarget * Math.exp(delta * 0.0015), controls.minDistance, controls.maxDistance);
      local.dirty = true;
    };
    const onTransformChange = () => { local.dirty = true; };
    const onObjectChange = () => {
      if (local.draggingPointer !== null) {
        if (!targetIsEnabled()) { finishDrag(); return; }
        if (local.manipulationTarget === "zone" && local.manipulatedZoneId) {
          const center = zoneObject.position.toArray().map(value => THREE.MathUtils.clamp(value / SCALE, -MAX_SCENE_COORDINATE_MM, MAX_SCENE_COORDINATE_MM)) as Vec3;
          if (!center.every(Number.isFinite)) { finishDrag(); return; }
          zoneObject.position.fromArray(center).multiplyScalar(SCALE);
          // Move the visual immediately; React owns the final values and the inspection request.
          local.dangerZones.get(local.manipulatedZoneId)?.root.position.copy(zoneObject.position);
          latest.current.onDangerZoneMove?.(local.manipulatedZoneId, center);
        } else latest.current.onJogMove?.(poseFromJog());
      }
      local.dirty = true;
    };
    transform.addEventListener("change", onTransformChange);
    transform.addEventListener("objectChange", onObjectChange);
    canvas.addEventListener("pointerdown", onPointerDown, true);
    canvas.addEventListener("pointermove", onPointerMove, true);
    canvas.addEventListener("pointerup", onPointerUp, true);
    canvas.addEventListener("pointercancel", onPointerUp, true);
    canvas.addEventListener("lostpointercapture", onLostCapture);
    canvas.addEventListener("wheel", onWheel, { passive: false });
    window.addEventListener("blur", finishDrag);
    window.addEventListener("blur", clearSurfacePointer);
    window.addEventListener("keydown", onEscape);
    state.current = local;
    setView(local, latest.current.view);
    const resize = () => {
      const width = Math.max(1, element.clientWidth);
      const height = Math.max(1, element.clientHeight);
      renderer.setSize(width, height);
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
      local.dirty = true;
    };
    const observer = new ResizeObserver(resize);
    observer.observe(element);
    resize();
    let disposed = false;
    let contextLost = false;
    let raf = 0;
    const lost = (event: Event) => {
      event.preventDefault();
      surfacePointer = null;
      finishDrag();
      contextLost = true;
      setGraphicsError("3D 图形上下文已中断，正在等待浏览器恢复。也可刷新页面重试。");
    };
    const restored = () => {
      contextLost = false;
      setGraphicsError(null);
      local.dirty = true;
      renderer.shadowMap.needsUpdate = true;
    };
    renderer.domElement.addEventListener("webglcontextlost", lost);
    renderer.domElement.addEventListener("webglcontextrestored", restored);
    let previousFrame = performance.now();
    let diagnosticsAt = 0;
    const animate = (now = performance.now()) => {
      if (disposed) return;
      raf = requestAnimationFrame(animate);
      const elapsed = (now - previousFrame) / 1000;
      previousFrame = now;
      let moved = false;
      if (local.draggingPointer === null) {
        const distance = camera.position.distanceTo(controls.target);
        const next = smoothDistance(distance, local.zoomTarget, elapsed);
        if (Math.abs(next - distance) > 1e-9) {
          camera.position.sub(controls.target).setLength(next).add(controls.target);
          moved = true;
        }
        moved = controls.update() || moved;
      }
      if (moved && local.surfaces.size) adjustSurfaceClipping(local);
      if (import.meta.env.DEV && (moved || local.dirty || now - diagnosticsAt > 120)) {
        diagnosticsAt = now;
        updateDiagnostics(local);
      }
      if (!contextLost && (moved || local.dirty)) {
        renderer.render(scene, camera);
        local.dirty = false;
      }
    };
    animate();
    setMounted(true);
    return () => {
      disposed = true;
      cancelAnimationFrame(raf);
      observer.disconnect();
      const captured = local.draggingPointer;
      local.draggingPointer = null; // Unmount must not schedule any new robot motion.
      if (captured !== null && canvas.hasPointerCapture(captured)) canvas.releasePointerCapture(captured);
      if (captured !== null && local.manipulationTarget === "zone") latest.current.onDangerZoneDrag?.(false);
      canvas.removeEventListener("pointerdown", onPointerDown, true);
      canvas.removeEventListener("pointermove", onPointerMove, true);
      canvas.removeEventListener("pointerup", onPointerUp, true);
      canvas.removeEventListener("pointercancel", onPointerUp, true);
      canvas.removeEventListener("lostpointercapture", onLostCapture);
      canvas.removeEventListener("wheel", onWheel);
      window.removeEventListener("blur", finishDrag);
      window.removeEventListener("blur", clearSurfacePointer);
      window.removeEventListener("keydown", onEscape);
      transform.removeEventListener("change", onTransformChange);
      transform.removeEventListener("objectChange", onObjectChange);
      scene.remove(helper);
      transform.dispose();
      renderer.domElement.removeEventListener("webglcontextlost", lost);
      renderer.domElement.removeEventListener("webglcontextrestored", restored);
      controls.dispose();
      disposeTree(scene);
      renderer.dispose();
      renderer.forceContextLoss();
      renderer.domElement.remove();
      if (state.current === local) state.current = null;
    };
  }, []);

  useEffect(() => {
    const local = state.current;
    if (!local || !mounted) return;
    let cancelled = false;
    let failed = false;
    let count = 0;
    const geometries = new Set<THREE.BufferGeometry>();
    const materials = new Set<THREE.Material>();
    setLoaded(0);
    setLoadError(null);
    local.links = [];
    local.ghostLinks = [];
    local.genericLinks = [];
    local.genericFocusLocal = null;
    local.robot.clear();
    local.ghost.clear();
    local.genericModelId = props.urdfModel?.id ?? null;
    local.renderer.domElement.setAttribute("aria-label", props.urdfModel ? `${props.urdfModel.name} 三维模型场景` : "ABB IRB 1100 三维仿真场景");
    if (props.urdfModel) {
      local.robot.scale.setScalar(1);
      const prepared = props.urdfModel;
      const copies = new Map<string, THREE.BufferGeometry>();
      for (const link of prepared.description.links) for (const visual of link.visuals) {
        let geometry = copies.get(visual.meshPath);
        if (!geometry) {
          geometry = prepared.geometries.get(visual.meshPath)!.clone();
          copies.set(visual.meshPath, geometry);
          geometries.add(geometry);
        }
        const baseColor = prepared.id.startsWith("fanuc-") ? 0xe5bc54 : prepared.id.startsWith("yaskawa-") ? 0x5e89b8 : 0xe9e9e7;
        const color = visual.color ? new THREE.Color().setRGB(...visual.color.slice(0, 3) as [number, number, number], THREE.SRGBColorSpace) : baseColor;
        const material = new THREE.MeshStandardMaterial({ color, metalness: 0.2, roughness: 0.48,
          opacity: visual.color?.[3] ?? 1, transparent: visual.color !== undefined && visual.color[3] < 1 });
        materials.add(material);
        const mesh = new THREE.Mesh(geometry, material);
        mesh.name = `${prepared.id} ${link.name}`;
        mesh.matrixAutoUpdate = false;
        mesh.castShadow = true; mesh.receiveShadow = true;
        local.robot.add(mesh);
        local.genericLinks.push({ mesh, link: link.name, visual });
      }
      updateRobot(local, props);
      local.robot.updateMatrixWorld(true);
      const bounds = new THREE.Box3().setFromObject(local.robot);
      local.genericFocus = bounds.getCenter(new THREE.Vector3());
      local.genericFocusLocal = local.genericFocus.clone().sub(local.robot.position).applyQuaternion(local.robot.quaternion.clone().invert());
      local.genericRadius = Math.max(0.15, bounds.getSize(new THREE.Vector3()).length() / 2);
      const extent = Math.max(1, local.genericRadius * 2);
      local.grid.scale.setScalar(extent);
      local.ground.scale.setScalar(extent);
      // Keep the actual URDF root at world zero; place the floor below geometry.
      local.ground.position.z = Math.min(-0.001, bounds.min.z - 0.001);
      local.grid.position.z = Math.min(0.0001, bounds.min.z + 0.0001);
      local.controls.maxDistance = Math.max(4.5, extent * 5);
      local.camera.far = Math.max(12, extent * 12);
      local.camera.updateProjectionMatrix();
      local.scene.fog = new THREE.Fog(0xf0f3f6, extent * 2.8, extent * 6);
      local.key.position.set(0.5 * extent, -0.9 * extent, 2.1 * extent);
      local.key.target.position.copy(local.genericFocus);
      Object.assign(local.key.shadow.camera, { left: -extent * 1.2, right: extent * 1.2, top: extent * 1.2, bottom: -extent * 1.2, far: extent * 5 });
      local.key.shadow.camera.updateProjectionMatrix();
      setView(local, props.view);
      setLoaded(local.genericLinks.length);
      return () => {
        geometries.forEach(geometry => geometry.dispose());
        materials.forEach(material => material.dispose());
        local.robot.clear(); local.genericLinks = [];
        local.dirty = true;
      };
    }
    local.robot.scale.setScalar(SCALE);
    local.genericFocus = null;
    local.genericJoints = [];
    local.grid.scale.setScalar(1); local.grid.position.z = 0.0001;
    local.ground.scale.setScalar(1); local.ground.position.z = -0.001;
    local.controls.maxDistance = 4.5;
    local.camera.far = 12; local.camera.updateProjectionMatrix();
    local.scene.fog = new THREE.Fog(0xf0f3f6, 2.8, 6);
    local.key.position.set(0.5, -0.9, 2.1); local.key.target.position.set(0.12, 0, 0.3);
    Object.assign(local.key.shadow.camera, { left: -1.2, right: 1.2, top: 1.2, bottom: -1.2, far: 5 });
    local.key.shadow.camera.updateProjectionMatrix();

    local.zeroInverse = forwardKinematics(props.model, [0, 0, 0, 0, 0, 0]).frames.map((frame: Mat4) => matrix(frame).invert());
    const loader = new STLLoader();
    const base = import.meta.env.BASE_URL;
    const tasks = PARTS.map(async (name) => {
      const geometry = await loader.loadAsync(`${base}models/${props.model.id}/${name}.stl`);
      if (cancelled || failed) {
        geometry.dispose();
        return null;
      }
      geometries.add(geometry);
      count++;
      // The final count means meshes are attached and the default camera is set.
      if (count < PARTS.length) setLoaded(count);
      return geometry;
    });
    void Promise.all(tasks).then((loadedGeometries) => {
      if (cancelled || failed || loadedGeometries.some((item) => !item)) return;
      loadedGeometries.forEach((geometry, i) => {
        const material = new THREE.MeshStandardMaterial({ color: i === 6 ? 0x59616c : i === 4 ? 0x7b8491 : 0xe9e9e7, metalness: i === 6 ? 0.62 : 0.24, roughness: i === 6 ? 0.36 : 0.42 });
        const ghostMaterial = new THREE.MeshBasicMaterial({ color: 0x779ab4, transparent: true, opacity: 0.11, depthWrite: false, side: THREE.FrontSide });
        materials.add(material);
        materials.add(ghostMaterial);
        const link = new THREE.Mesh(geometry!, material);
        link.name = `IRB1100 ${PARTS[i]}`;
        link.matrixAutoUpdate = false;
        link.castShadow = true;
        link.receiveShadow = true;
        const ghostLink = new THREE.Mesh(geometry!, ghostMaterial);
        ghostLink.matrixAutoUpdate = false;
        ghostLink.renderOrder = 2;
        local.robot.add(link);
        local.ghost.add(ghostLink);
        local.links.push(link);
        local.ghostLinks.push(ghostLink);
      });
      updateRobot(local, latest.current);
      setView(local, latest.current.view);
      setLoaded(PARTS.length);
    }).catch(() => {
      if (cancelled) return;
      failed = true;
      geometries.forEach((item) => item.dispose());
      setLoadError("机器人网格未能完整加载，请检查网络连接后重试。");
      local.dirty = true;
    });
    return () => {
      cancelled = true;
      geometries.forEach((item) => item.dispose());
      materials.forEach((item) => item.dispose());
      local.robot.clear();
      local.ghost.clear();
      local.links = [];
      local.ghostLinks = [];
      local.dirty = true;
    };
  }, [props.model, props.urdfModel, mounted, generation]);

  useEffect(() => {
    const local = state.current;
    if (local) updateRobot(local, props);
  }, [props.model, props.joints, props.urdfModel, props.urdfJoints, props.endJoints, props.showFrames, props.targetPose, props.basePose, props.toolPose, props.selectedJointIndex, props.robotPresent, loaded]);

  useEffect(() => {
    const local = state.current;
    if (!local) return;
    local.target.visible = props.targetPose !== null;
    if (props.targetPose) setPose(local.target, props.targetPose);
    local.path.geometry.dispose();
    local.path.geometry = new THREE.BufferGeometry().setFromPoints(props.path.map((sample) => new THREE.Vector3(...sample.pose.position).multiplyScalar(SCALE)));
    local.path.computeLineDistances();
    local.path.visible = props.path.length > 1;
    local.start.visible = props.path.length > 1;
    if (props.path[0]) local.start.position.fromArray(props.path[0].pose.position).multiplyScalar(SCALE);
    local.dirty = true;
  }, [props.targetPose, props.path, props.urdfModel, mounted]);

  useEffect(() => {
    if (state.current) updateHierarchy(state.current, props);
  }, [props.sceneFrames, props.sceneTools, mounted]);

  useEffect(() => {
    if (state.current) updateDangerZones(state.current, props.dangerZones, props.selectedDangerZoneId);
  }, [props.dangerZones, props.selectedDangerZoneId, mounted]);

  useEffect(() => {
    if (state.current) updateScenePaths(state.current, props.scenePaths, props.selectedPointId);
  }, [props.scenePaths, props.selectedPointId, mounted]);

  useEffect(() => {
    if (state.current) updateSurfaces(state.current, props);
  }, [props.sceneSurfaces, props.selectedSurfaceId, props.surfaceRegion, props.surfacePickingEnabled, mounted]);

  useEffect(() => {
    const local = state.current;
    if (!local) return;
    local.finishDrag();
    local.manipulatedZoneId = zoneTargetId;
    if (zoneTargetId) {
      const zone = props.dangerZones!.find(zone => zone.id === zoneTargetId)!;
      local.manipulationTarget = "zone";
      local.zoneObject.position.fromArray(zone.centerMm).multiplyScalar(SCALE);
      local.zoneObject.quaternion.identity();
      local.transform.attach(local.zoneObject);
      local.transform.setMode("translate");
      local.transform.setSpace("world");
    } else if (tcpManipulationEnabled) {
      local.manipulationTarget = "tcp";
      const world = worldRobotPoses(props); if (world) setPose(local.jogObject, world.tcp);
      local.transform.attach(local.jogObject);
      local.transform.setMode(gizmoMode);
      local.transform.setSpace(gizmoSpace);
    } else {
      local.manipulationTarget = "none";
      local.transform.detach();
    }
    local.transform.enabled = local.manipulationTarget !== "none";
    local.transform.getHelper().visible = local.transform.enabled;
    local.dirty = true;
  }, [props.model, props.urdfModel, tcpManipulationEnabled, gizmoMode, gizmoSpace, gizmoResetRevision, zoneTargetId, mounted]);

  useEffect(() => {
    const local = state.current;
    if (!local || local.manipulationTarget !== "zone" || local.draggingPointer !== null) return;
    const zone = props.dangerZones?.find(zone => zone.id === local.manipulatedZoneId);
    if (usableZone(zone)) local.zoneObject.position.fromArray(zone.centerMm).multiplyScalar(SCALE);
    local.dirty = true;
  }, [props.dangerZones, zoneTargetId, mounted]);

  useEffect(() => {
    const local = state.current;
    if (!local || local.draggingPointer !== null || props.jogPending) return;
    const world = worldRobotPoses(props); if (world) setPose(local.jogObject, world.tcp);
    local.dirty = true;
  }, [props.jogPending, props.joints, props.urdfJoints, props.urdfModel, props.basePose, props.toolPose]);

  useEffect(() => {
    if (state.current) { state.current.finishDrag(); setView(state.current, props.view); }
  }, [props.view, props.viewRevision, mounted]);

  const restoredCamera = useRef(false);
  useEffect(() => {
    const local = state.current;
    const expected = props.urdfModel?.description.links.reduce((sum, link) => sum + link.visuals.length, 0) ?? PARTS.length;
    if (!local || loaded !== expected || restoredCamera.current) return;
    restoredCamera.current = true;
    const saved = props.initialCamera;
    if (!saved) return;
    local.controls.target.fromArray(saved.target);
    local.camera.position.fromArray(saved.position);
    if (local.surfaces.size) {
      const distance = local.camera.position.distanceTo(local.controls.target);
      local.controls.maxDistance = Math.max(local.controls.maxDistance, distance * 6);
      local.controls.minDistance = Math.min(local.controls.minDistance, distance / 2);
      adjustSurfaceClipping(local);
    }
    local.camera.lookAt(local.controls.target); local.controls.update();
    local.zoomTarget = local.camera.position.distanceTo(local.controls.target);
    local.dirty = true;
  }, [loaded, props.initialCamera, props.urdfModel]);

  useImperativeHandle(ref, () => ({ focusSurface(id) { return state.current ? focusSurface(state.current, id) : false; }, camera() { const local = state.current; return local ? { position: local.camera.position.toArray() as Vec3, target: local.controls.target.toArray() as Vec3 } : null; }, async capture() {
    const local = state.current;
    const expected = props.urdfModel?.description.links.reduce((sum, link) => sum + link.visuals.length, 0) ?? 7;
    if (!local || graphicsError || loadError || loaded !== expected) throw new Error(loadError || graphicsError || '机器人三维模型尚未加载完成，请稍后重试。');
    local.finishDrag();
    updateRobot(local, latest.current); updateHierarchy(local, latest.current); updateSurfaces(local, latest.current);
    const canvas = local.renderer.domElement;
    if (!canvas.width || !canvas.height || local.renderer.getContext().isContextLost()) throw new Error('三维画面不可用，无法保存场景封面。');
    const helper = local.transform.getHelper(), visible = helper.visible;
    helper.visible = false;
    try {
      local.scene.updateMatrixWorld(true);
      local.renderer.render(local.scene, local.camera);
      const cover = document.createElement('canvas');
      cover.width = Math.min(720, canvas.width); cover.height = Math.max(1, Math.round(canvas.height * cover.width / canvas.width));
      const context = cover.getContext('2d');
      if (!context) throw new Error('无法读取三维画面，请重试。');
      // WebGL uses transparency over the viewport's pale background; JPEG
      // otherwise turns that transparent sky into a black strip.
      context.fillStyle = '#f1f4f7';
      context.fillRect(0, 0, cover.width, cover.height);
      context.drawImage(canvas, 0, 0, cover.width, cover.height);
      const thumbnail = cover.toDataURL('image/jpeg', .86);
      if (!thumbnail.startsWith('data:image/jpeg;base64,') || thumbnail.length < 1000) throw new Error('场景封面生成失败，请重试。');
      return { thumbnail, camera: { position: local.camera.position.toArray() as Vec3, target: local.controls.target.toArray() as Vec3 } };
    } finally { helper.visible = visible; local.dirty = true; }
  } }), [loaded, graphicsError, loadError, props.urdfModel]);

  const totalParts = props.urdfModel?.description.links.reduce((sum, link) => sum + link.visuals.length, 0) ?? 7;
  return <div className="robot-scene" aria-label="机器人三维视图">
    <div className="robot-scene-canvas" ref={host} />
    {props.scenePaths?.some(path => path.points.length > 1) && <div className="scene-path-legend" aria-label="Path 连线说明"><Icon name="route" width={23} height={14} preserveAspectRatio="none"/>Path · 目标点连线，尚未验证运动</div>}
    {!graphicsError && <div className={`scene-model-status${loadError ? " scene-load-error" : ""}`} role="status" data-loaded-parts={loaded}>
      <span>{loadError || (loaded === totalParts ? `机器人模型已加载 · ${loaded}/${totalParts}` : `正在加载机器人模型 ${loaded}/${totalParts}`)}</span>
      {compatibilityGraphics && !loadError && <span className="scene-compatibility-mode">兼容图形模式</span>}
      {loadError && !graphicsError && <button type="button" onClick={() => setGeneration((value) => value + 1)}>重新加载</button>}
    </div>}
    {graphicsError && <div className="scene-graphics-error" role="alert"><strong>3D 场景暂不可用</strong><p>{graphicsError}</p></div>}
  </div>;
});
export default RobotScene;
