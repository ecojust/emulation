import { useEffect, useRef, useState } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { TransformControls, type TransformControlsGizmo } from "three/addons/controls/TransformControls.js";
import { normalizeWheelDelta, smoothDistance } from "./smooth-zoom";
import { createValidatedRenderer } from "./webgl-context";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import { forwardKinematics } from "../robot/kinematics";
import type { JointAngles, Mat4, MotionSample, Pose, RobotModel } from "../robot/types";
import type { CollisionZone } from "../robot/collision";
import "./scene.css";

export interface RobotSceneProps {
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
}

interface DangerVisual {
  root: THREE.Group;
  body: THREE.Group;
  mesh: THREE.Mesh<THREE.BoxGeometry, THREE.MeshBasicMaterial>;
  edges: THREE.LineSegments<THREE.EdgesGeometry, THREE.LineBasicMaterial>;
  caption: THREE.Sprite;
  text: string;
}

interface SceneState {
  scene: THREE.Scene;
  camera: THREE.PerspectiveCamera;
  controls: OrbitControls;
  renderer: THREE.WebGLRenderer;
  robot: THREE.Group;
  ghost: THREE.Group;
  frames: THREE.Group;
  tcp: THREE.Group;
  target: THREE.Group;
  path: THREE.Line;
  start: THREE.Mesh;
  links: THREE.Mesh[];
  ghostLinks: THREE.Mesh[];
  zeroInverse: THREE.Matrix4[];
  dirty: boolean;
  zoomTarget: number;
  jogObject: THREE.Object3D;
  transform: TransformControls;
  finishJog: () => void;
  draggingPointer: number | null;
  dangerZones: Map<string, DangerVisual>;
}

const SCALE = 0.001;
const PARTS = ["base_link", "link_1", "link_2", "link_3", "link_4", "link_5", "link_6"];
const AXIS_COLORS = [0xe35454, 0x46a88a, 0x557bd1];

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
    const valid = zone.enabled && zone.centerMm.length === 3 && zone.sizeMm.length === 3
      && zone.centerMm.every(Number.isFinite) && zone.sizeMm.every(value => Number.isFinite(value) && value > 0);
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

function setView(state: SceneState, view: RobotSceneProps["view"]): void {
  const focus = new THREE.Vector3(0.13, 0, 0.31);
  state.controls.target.copy(focus);
  state.camera.up.set(0, 0, 1);
  if (view === "top") state.camera.position.set(0.13, -0.001, 1.9);
  else if (view === "front") state.camera.position.set(0.13, -1.85, 0.43);
  else state.camera.position.set(0.95, -1.05, 0.85);
  state.camera.lookAt(focus);
  state.controls.update();
  state.zoomTarget = state.camera.position.distanceTo(state.controls.target);
  state.dirty = true;
}

function updateRobot(state: SceneState, props: RobotSceneProps): void {
  const fk = forwardKinematics(props.model, props.joints);
  state.links.forEach((link, i) => {
    link.matrix.copy(matrix(fk.frames[i]).multiply(state.zeroInverse[i]));
    link.matrixWorldNeedsUpdate = true;
  });
  setPose(state.tcp, fk.pose);
  if (state.draggingPointer === null && !props.jogPending) setPose(state.jogObject, fk.pose);
  const tcpCaption = state.tcp.getObjectByName("pose-caption");
  if (tcpCaption) tcpCaption.visible = !props.targetPose || Math.hypot(...fk.pose.position.map((value, i) => value - props.targetPose!.position[i])) > 5;
  state.frames.visible = Boolean(props.showFrames);
  state.frames.children.forEach((frame, i) => {
    frame.matrix.copy(matrix(fk.frames[i], true));
    frame.matrixWorldNeedsUpdate = true;
  });
  const differs = props.endJoints?.some((angle, i) => Math.abs(angle - props.joints[i]) > 0.1);
  state.ghost.visible = Boolean(differs && state.ghostLinks.length);
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
  canvas.dataset.jogDragging = String(state.draggingPointer !== null);
  canvas.dataset.jogMode = transform.mode;
  canvas.dataset.jogSpace = transform.space;
  canvas.dataset.jogAxis = transform.axis ?? "";
  canvas.dataset.jogPose = JSON.stringify({ position: jogObject.position.toArray().map(v => v / SCALE), orientation: [jogObject.quaternion.w, jogObject.quaternion.x, jogObject.quaternion.y, jogObject.quaternion.z] });
  const handles: Record<string, { x: number; y: number }> = {};
  if (transform.enabled && transform.getHelper().visible) {
    const gizmo = transform.getHelper().children.find(child => (child as TransformControlsGizmo).isTransformControlsGizmo) as TransformControlsGizmo | undefined;
    const picker = gizmo?.picker[transform.mode];
    const raycaster = new THREE.Raycaster();
    const origin = jogObject.position.clone().project(camera);
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
  canvas.dataset.jogHandles = JSON.stringify(handles);
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
}

export default function RobotScene(props: RobotSceneProps) {
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
    const originLabel = label("WObj0 · 0, 0, 0", "#7a8597", 0.21);
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
    const tcp = poseGroup(0x426f98, "TCP · tool0");
    const target = poseGroup(0xd33840, "TARGET");
    target.visible = false;
    scene.add(tcp, target);
    const path = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineDashedMaterial({ color: 0xd64d53, dashSize: 0.014, gapSize: 0.009, transparent: true, opacity: 0.9 }));
    path.visible = false;
    path.renderOrder = 4;
    const start = new THREE.Mesh(new THREE.SphereGeometry(0.006, 14, 10), new THREE.MeshBasicMaterial({ color: 0x547899 }));
    start.visible = false;
    scene.add(path, start);
    const jogObject = new THREE.Object3D();
    jogObject.name = "Requested tool0 pose";
    scene.add(jogObject);
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
    const local: SceneState = { scene, camera, controls, renderer, robot, ghost, frames, tcp, target, path, start, links: [], ghostLinks: [], zeroInverse: [], dirty: true,
      zoomTarget: 1, jogObject, transform, finishJog: () => {}, draggingPointer: null, dangerZones: new Map() };
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
    const finishJog = () => {
      const id = local.draggingPointer;
      if (id === null) return;
      local.draggingPointer = null;
      transform.pointerUp({ button: 0 } as PointerEvent);
      if (canvas.hasPointerCapture(id)) canvas.releasePointerCapture(id);
      controls.enabled = true;
      latest.current.onJogEnd?.();
      if (!latest.current.jogPending) setPose(jogObject, forwardKinematics(latest.current.model, latest.current.joints).pose);
      local.dirty = true;
    };
    local.finishJog = finishJog;
    const onPointerDown = (event: PointerEvent) => {
      if (!latest.current.jogEnabled || latest.current.jogPending || !transform.enabled || event.button !== 0 || local.draggingPointer !== null) return;
      scene.updateMatrixWorld(true);
      transform.pointerHover(pointer(event));
      if (!transform.axis) return;
      event.preventDefault(); event.stopImmediatePropagation();
      freezeOrbit();
      transform.pointerDown(pointer(event));
      if (!transform.dragging) { controls.enabled = true; return; }
      local.draggingPointer = event.pointerId;
      canvas.setPointerCapture(event.pointerId);
      latest.current.onJogStart?.();
      local.dirty = true;
    };
    const onPointerMove = (event: PointerEvent) => {
      if (local.draggingPointer !== null) {
        if (event.pointerId !== local.draggingPointer) return;
        event.preventDefault(); event.stopImmediatePropagation();
        transform.pointerMove(pointer(event, -1));
      } else if (latest.current.jogEnabled && transform.enabled && event.buttons === 0) {
        scene.updateMatrixWorld(true);
        transform.pointerHover(pointer(event));
      }
    };
    const onPointerUp = (event: PointerEvent) => {
      if (event.pointerId !== local.draggingPointer) return;
      event.preventDefault(); event.stopImmediatePropagation();
      finishJog();
    };
    const onLostCapture = (event: PointerEvent) => { if (event.pointerId === local.draggingPointer) finishJog(); };
    const onWheel = (event: WheelEvent) => {
      event.preventDefault();
      if (!controls.enabled || local.draggingPointer !== null) return;
      const delta = normalizeWheelDelta(event.deltaY, event.deltaMode, canvas.clientHeight, event.ctrlKey);
      local.zoomTarget = THREE.MathUtils.clamp(local.zoomTarget * Math.exp(delta * 0.0015), controls.minDistance, controls.maxDistance);
      local.dirty = true;
    };
    const onTransformChange = () => { local.dirty = true; };
    const onObjectChange = () => {
      if (local.draggingPointer !== null) latest.current.onJogMove?.(poseFromJog());
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
    window.addEventListener("blur", finishJog);
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
      local.draggingPointer = null; // Unmount must not schedule new application work.
      if (captured !== null && canvas.hasPointerCapture(captured)) canvas.releasePointerCapture(captured);
      canvas.removeEventListener("pointerdown", onPointerDown, true);
      canvas.removeEventListener("pointermove", onPointerMove, true);
      canvas.removeEventListener("pointerup", onPointerUp, true);
      canvas.removeEventListener("pointercancel", onPointerUp, true);
      canvas.removeEventListener("lostpointercapture", onLostCapture);
      canvas.removeEventListener("wheel", onWheel);
      window.removeEventListener("blur", finishJog);
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
    local.robot.clear();
    local.ghost.clear();
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
      setLoaded(count);
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
  }, [props.model, mounted, generation]);

  useEffect(() => {
    const local = state.current;
    if (local) updateRobot(local, props);
  }, [props.model, props.joints, props.endJoints, props.showFrames, props.targetPose, loaded]);

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
  }, [props.targetPose, props.path, mounted]);

  useEffect(() => {
    if (state.current) updateDangerZones(state.current, props.dangerZones, props.selectedDangerZoneId);
  }, [props.dangerZones, props.selectedDangerZoneId, mounted]);

  useEffect(() => {
    const local = state.current;
    if (!local) return;
    local.finishJog();
    const enabled = Boolean(props.jogEnabled);
    local.transform.enabled = enabled;
    local.transform.setMode(props.jogMode ?? "translate");
    local.transform.setSpace(props.jogSpace ?? "world");
    local.transform.getHelper().visible = enabled;
    setPose(local.jogObject, forwardKinematics(props.model, props.joints).pose);
    local.dirty = true;
  }, [props.model, props.jogEnabled, props.jogMode, props.jogSpace, props.jogResetRevision, mounted]);

  useEffect(() => {
    const local = state.current;
    if (!local || local.draggingPointer !== null || props.jogPending) return;
    setPose(local.jogObject, forwardKinematics(props.model, props.joints).pose);
    local.dirty = true;
  }, [props.jogPending, props.joints]);

  useEffect(() => {
    if (state.current) { state.current.finishJog(); setView(state.current, props.view); }
  }, [props.view, props.viewRevision, mounted]);

  return <div className="robot-scene" aria-label="机器人三维视图">
    <div className="robot-scene-canvas" ref={host} />
    {!graphicsError && <div className={`scene-model-status${loadError ? " scene-load-error" : ""}`} role="status" data-loaded-parts={loaded}>
      <span>{loadError || (loaded === 7 ? "机器人模型已加载 · 7/7" : `正在加载机器人模型 ${loaded}/7`)}</span>
      {compatibilityGraphics && !loadError && <span className="scene-compatibility-mode">兼容图形模式</span>}
      {loadError && !graphicsError && <button type="button" onClick={() => setGeneration((value) => value + 1)}>重新加载</button>}
    </div>}
    {graphicsError && <div className="scene-graphics-error" role="alert"><strong>3D 场景暂不可用</strong><p>{graphicsError}</p></div>}
  </div>;
}
