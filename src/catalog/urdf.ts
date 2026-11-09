import { STLLoader } from "three/addons/loaders/STLLoader.js";
import type { BufferGeometry } from "three";
import type { Vec3 } from "../robot/types";
import type { PreparedUrdfModel, UrdfJoint, UrdfLink, UrdfOrigin, UrdfRobot } from "./urdf-types";
export type { PreparedUrdfModel, UrdfRobot, UrdfJoint } from "./urdf-types";

function children(element: Element, name: string): Element[] {
  return Array.from(element.childNodes).filter((node): node is Element => node.nodeType === 1 && (node as Element).tagName === name);
}
function one(element: Element, name: string): Element | undefined {
  const values = children(element, name);
  if (values.length > 1) throw new Error(`URDF ${name} 节点重复。`);
  return values[0];
}
function required(element: Element | undefined, attribute: string): string {
  const value = element?.getAttribute(attribute)?.trim();
  if (!value || value.length > 256) throw new Error(`URDF 缺少有效的 ${attribute}。`);
  return value;
}
function values(text: string | null | undefined, count: number, fallback?: number[]): number[] {
  const result = text == null ? fallback : text.trim().split(/\s+/).map(Number);
  if (!result || result.length !== count || result.some(value => !Number.isFinite(value))) throw new Error("URDF 包含无效数值。");
  return [...result];
}
function origin(element?: Element): UrdfOrigin {
  return { xyz: values(element?.getAttribute("xyz"), 3, [0, 0, 0]) as Vec3,
    rpy: values(element?.getAttribute("rpy"), 3, [0, 0, 0]) as Vec3 };
}
function scalar(element: Element | undefined, attribute: string): number {
  const raw = required(element, attribute);
  const value = Number(raw);
  if (!Number.isFinite(value)) throw new Error(`URDF ${attribute} 数值无效。`);
  return value;
}
function safeMeshPath(value: string): string {
  // Only catalog-relative files are supported. Never resolve URLs, packages or traversal.
  if (!/^[a-zA-Z0-9_.\-/]+\.stl$/i.test(value) || value.startsWith("/")
    || value.split("/").some(part => !part || part === "." || part === "..")) throw new Error("URDF 网格必须为目录内的相对 STL 路径。");
  return value;
}

/** Only the bounded, catalog-supported mesh/revolute/fixed subset is accepted. */
export function parseUrdf(text: string): UrdfRobot {
  if (text.length > 1_000_000 || /<!\s*(?:DOCTYPE|ENTITY)/i.test(text)) throw new Error("URDF 不允许 DTD、实体声明或过大的文档。");
  if (typeof DOMParser === "undefined") throw new Error("当前环境缺少 XML 解析器。");
  const doc = new DOMParser().parseFromString(text, "application/xml");
  if (doc.getElementsByTagName("parsererror").length || doc.documentElement?.tagName !== "robot") throw new Error("URDF XML 格式无效。");
  const root = doc.documentElement;
  if (doc.getElementsByTagName("mimic").length) throw new Error("暂不支持 URDF mimic 关节。");
  const globalColors = new Map<string, [number, number, number, number]>();
  for (const material of children(root, "material")) {
    const color = one(material, "color");
    if (color) globalColors.set(required(material, "name"), values(required(color, "rgba"), 4) as [number, number, number, number]);
  }
  const links: UrdfLink[] = children(root, "link").map(link => ({
    name: required(link, "name"), visuals: children(link, "visual").map(visual => {
      const geometry = one(visual, "geometry");
      const mesh = geometry && one(geometry, "mesh");
      if (!mesh || Array.from(geometry!.childNodes).filter(n => n.nodeType === 1).length !== 1) throw new Error("暂只支持 STL 网格形式的 URDF visual。");
      const material = one(visual, "material"), colorNode = material && one(material, "color");
      const color = colorNode ? values(required(colorNode, "rgba"), 4) as [number, number, number, number]
        : material ? globalColors.get(required(material, "name")) : undefined;
      if (color?.some(v => v < 0 || v > 1)) throw new Error("URDF 颜色超出范围。");
      const scale = values(mesh.getAttribute("scale"), 3, [1, 1, 1]) as Vec3;
      if (scale.some(v => v <= 0 || v > 1000)) throw new Error("URDF 网格缩放无效。");
      return { meshPath: safeMeshPath(required(mesh, "filename")), origin: origin(one(visual, "origin")), scale, color };
    }),
  }));
  const names = new Set(links.map(link => link.name));
  if (!links.length || links.length > 128 || names.size !== links.length || links.reduce((sum, link) => sum + link.visuals.length, 0) > 256) throw new Error("URDF link 数量无效或名称重复。");
  const joints: UrdfJoint[] = children(root, "joint").map(joint => {
    const type = required(joint, "type");
    if (type !== "fixed" && type !== "revolute" && type !== "continuous") throw new Error(`暂不支持 URDF ${type} 关节。`);
    const axis = values(one(joint, "axis")?.getAttribute("xyz"), 3, [1, 0, 0]) as Vec3;
    const norm = Math.hypot(...axis);
    if (norm < 1e-12) throw new Error("URDF 关节轴不能为零。");
    const limit = one(joint, "limit");
    const minDeg = type === "revolute" ? scalar(limit, "lower") * 180 / Math.PI : type === "continuous" ? -360 : 0;
    const maxDeg = type === "revolute" ? scalar(limit, "upper") * 180 / Math.PI : type === "continuous" ? 360 : 0;
    const maxSpeedDeg = type === "fixed" || !limit ? 0 : scalar(limit, "velocity") * 180 / Math.PI;
    if (minDeg > maxDeg || maxSpeedDeg < 0) throw new Error("URDF 关节限制无效。");
    return { name: required(joint, "name"), type, parent: required(one(joint, "parent"), "link"), child: required(one(joint, "child"), "link"),
      origin: origin(one(joint, "origin")), axis: axis.map(value => value / norm) as Vec3, minDeg, maxDeg, maxSpeedDeg };
  });
  if (new Set(joints.map(j => j.name)).size !== joints.length || joints.length !== links.length - 1) throw new Error("URDF 关节树不完整或名称重复。");
  const parents = new Set<string>();
  for (const joint of joints) {
    if (!names.has(joint.parent) || !names.has(joint.child) || joint.parent === joint.child || parents.has(joint.child)) throw new Error("URDF link 引用无效或存在多个父关节。");
    parents.add(joint.child);
  }
  const roots = links.filter(link => !parents.has(link.name));
  if (roots.length !== 1) throw new Error("URDF 必须为单一根节点的树。");
  const ordered: UrdfJoint[] = [], visited = new Set([roots[0].name]);
  const visit = (name: string) => {
    for (const joint of joints.filter(j => j.parent === name)) {
      if (visited.has(joint.child)) throw new Error("URDF 关节图包含循环。");
      visited.add(joint.child); ordered.push(joint); visit(joint.child);
    }
  };
  visit(roots[0].name);
  if (visited.size !== links.length) throw new Error("URDF 存在断开的 link 或循环。");
  const actuatedJoints = ordered.filter(joint => joint.type !== "fixed");
  if (!actuatedJoints.length || actuatedJoints.length > 32) throw new Error("URDF 可动关节数量无效。");
  return { name: required(root, "name"), rootLink: roots[0].name, toolLink: names.has("tool0") ? "tool0" : names.has("flange") ? "flange" : null,
    links, joints: ordered, actuatedJoints, homeJoints: actuatedJoints.map(joint => Math.max(joint.minDeg, Math.min(joint.maxDeg, 0))) };
}

export async function prepareUrdfModel(input: {
  id: string; name?: string; urdfText: string; assetUrls: Readonly<Record<string, string>>; signal?: AbortSignal;
}): Promise<PreparedUrdfModel> {
  const description = parseUrdf(input.urdfText);
  const paths = new Set(description.links.flatMap(link => link.visuals.map(visual => visual.meshPath)));
  if (!paths.size) throw new Error("URDF 没有可显示的网格。");
  for (const path of paths) if (!Object.hasOwn(input.assetUrls, path)) throw new Error(`URDF 网格缺失：${path}`);
  const geometries = new Map<string, BufferGeometry>();
  const loader = new STLLoader();
  const tasks = [...paths].map(async path => {
    input.signal?.throwIfAborted();
    const response = await fetch(input.assetUrls[path], { signal: input.signal });
    if (!response.ok) throw new Error(`网格读取失败：${path}`);
    const bytes = await response.arrayBuffer();
    input.signal?.throwIfAborted();
    if (!bytes.byteLength || bytes.byteLength > 64 * 1024 * 1024) throw new Error(`网格大小无效：${path}`);
    const geometry = loader.parse(bytes);
    geometries.set(path, geometry);
    const positions = geometry.getAttribute("position");
    if (!positions || !positions.count || positions.count > 9_000_000) throw new Error(`网格顶点无效：${path}`);
    for (const value of positions.array) if (!Number.isFinite(value)) throw new Error(`网格顶点无效：${path}`);
    geometry.computeBoundingBox(); geometry.computeVertexNormals();
  });
  const results = await Promise.allSettled(tasks);
  const failure = results.find((result): result is PromiseRejectedResult => result.status === "rejected");
  if (failure || input.signal?.aborted) {
    geometries.forEach(geometry => geometry.dispose());
    if (failure) throw failure.reason;
    input.signal!.throwIfAborted();
  }
  return { id: input.id, name: input.name ?? description.name, description, geometries };
}

export function disposePreparedUrdfModel(model: PreparedUrdfModel): void {
  model.geometries.forEach(geometry => geometry.dispose());
}
