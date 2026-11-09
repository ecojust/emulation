import { useEffect, useRef, useState } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import { forwardKinematics } from "../robot/kinematics";
import type { JointAngles, Mat4, MotionSample, Pose, RobotModel } from "../robot/types";
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

function setView(state: SceneState, view: RobotSceneProps["view"]): void {
  const focus = new THREE.Vector3(0.13, 0, 0.31);
  state.controls.target.copy(focus);
  state.camera.up.set(0, 0, 1);
  if (view === "top") state.camera.position.set(0.13, -0.001, 1.9);
  else if (view === "front") state.camera.position.set(0.13, -1.85, 0.43);
  else state.camera.position.set(0.95, -1.05, 0.85);
  state.camera.lookAt(focus);
  state.controls.update();
  state.dirty = true;
}

function updateRobot(state: SceneState, props: RobotSceneProps): void {
  const fk = forwardKinematics(props.model, props.joints);
  state.links.forEach((link, i) => {
    link.matrix.copy(matrix(fk.frames[i]).multiply(state.zeroInverse[i]));
    link.matrixWorldNeedsUpdate = true;
  });
  setPose(state.tcp, fk.pose);
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

export default function RobotScene(props: RobotSceneProps) {
  const host = useRef<HTMLDivElement>(null);
  const state = useRef<SceneState | null>(null);
  const latest = useRef(props);
  latest.current = props;
  const [loaded, setLoaded] = useState(0);
  const [loadError, setLoadError] = useState<string | null>(null);
  const [graphicsError, setGraphicsError] = useState<string | null>(null);
  const [generation, setGeneration] = useState(0);
  const [mounted, setMounted] = useState(false);

  useEffect(() => {
    const element = host.current;
    if (!element) return;
    let renderer: THREE.WebGLRenderer;
    try {
      renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true, powerPreference: "high-performance" });
    } catch {
      setGraphicsError("无法启动 3D 场景。请使用支持 WebGL 2 的浏览器，并开启图形加速。");
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
    controls.minDistance = 0.45;
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
    const local: SceneState = { scene, camera, controls, renderer, robot, ghost, frames, tcp, target, path, start, links: [], ghostLinks: [], zeroInverse: [], dirty: true };
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
    const animate = () => {
      if (disposed) return;
      raf = requestAnimationFrame(animate);
      const moved = controls.update();
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
    if (state.current) setView(state.current, props.view);
  }, [props.view, props.viewRevision, mounted]);

  return <div className="robot-scene" aria-label="机器人三维视图">
    <div className="robot-scene-canvas" ref={host} />
    {!graphicsError && <div className={`scene-model-status${loadError ? " scene-load-error" : ""}`} role="status" data-loaded-parts={loaded}>
      {loadError || (loaded === 7 ? "机器人模型已加载 · 7/7" : `正在加载机器人模型 ${loaded}/7`)}
      {loadError && !graphicsError && <button type="button" onClick={() => setGeneration((value) => value + 1)}>重新加载</button>}
    </div>}
    {graphicsError && <div className="scene-graphics-error" role="alert"><strong>3D 场景暂不可用</strong><p>{graphicsError}</p></div>}
  </div>;
}
