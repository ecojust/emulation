import { readFileSync } from "node:fs";
import { DOMParser as XmlDomParser } from "@xmldom/xmldom";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { BufferGeometry, Vector3 } from "three";
import { disposePreparedUrdfModel, parseUrdf, prepareUrdfModel } from "../src/catalog/urdf";
import { forwardUrdfKinematics, urdfLinkMatrices, urdfOriginMatrix } from "../src/catalog/kinematics";

beforeEach(() => vi.stubGlobal("DOMParser", XmlDomParser));
afterEach(() => { vi.unstubAllGlobals(); vi.restoreAllMocks(); });

const fixtureNames: Record<string, string> = { CRX_10iA_L: "fanuc-crx10ia-l", R_2000iB_165F: "fanuc-r2000ib-165f", GP7: "yaskawa-gp7", GP110: "yaskawa-gp110" };
const fixture = (name: string) => readFileSync(new URL(`./fixtures/catalog/${fixtureNames[name]}.urdf`, import.meta.url), "utf8");
const robot = `<robot name="test"><link name="base"/><link name="arm"><visual><geometry><mesh filename="meshes/arm.stl"/></geometry></visual></link><link name="tool0"/>
<joint name="axis" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 1"/><axis xyz="0 0 2"/><limit lower="-2" upper="2" velocity="1"/></joint>
<joint name="tool" type="fixed"><parent link="arm"/><child link="tool0"/><origin xyz="1 0 0"/></joint></robot>`;
const triangle = "solid triangle\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid triangle";
function close(actual: readonly number[], expected: readonly number[], digits = 7) {
  expect(actual).toHaveLength(expected.length);
  actual.forEach((value, index) => expect(value).toBeCloseTo(expected[index], digits));
}

describe("catalog URDF kinematics from pinned source fixtures", () => {
  it.each([
    ["CRX_10iA_L", [719.5, -150, 955]],
    ["R_2000iB_165F", [1847.5, 0, 1970]],
    ["GP7", [560, 0, 815]],
    ["GP110", [1540, 0, 1645]],
  ] as const)("%s preserves its real zero tool frame and six limits", (name, position) => {
    const model = parseUrdf(fixture(name));
    expect(model.actuatedJoints).toHaveLength(6);
    expect(model.actuatedJoints.map(j => j.name)).toEqual(["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]);
    expect(model.actuatedJoints.every(j => j.minDeg < j.maxDeg && j.maxSpeedDeg > 0)).toBe(true);
    close(forwardUrdfKinematics(model, [0, 0, 0, 0, 0, 0]).toolPose!.position, position);
    const ninety = forwardUrdfKinematics(model, [90, 0, 0, 0, 0, 0]).toolPose!;
    const sign = name === "R_2000iB_165F" ? -1 : 1;
    close(ninety.position, [-position[1] * sign, position[0] * sign, position[2]]);
  });

  it("preserves fixed balancer branches listed before their parent joint", () => {
    const model = parseUrdf(fixture("R_2000iB_165F"));
    expect(model.links.flatMap(link => link.visuals)).toHaveLength(9);
    const world = urdfLinkMatrices(model, [90, 0, 0, 0, 0, 0]);
    close(new Vector3().setFromMatrixPosition(world.get("balancer")!).toArray(), [0.351, 0.308, 0.67]);
    close(new Vector3().setFromMatrixPosition(world.get("balancer_shaft")!).toArray(), [0.189, -0.0195, 0.67]);
  });

  it("preserves common-zero millimetre visuals and local metre visuals separately", () => {
    const fanuc = parseUrdf(fixture("CRX_10iA_L"));
    const visual = fanuc.links.find(link => link.name === "link_6")!.visuals[0];
    close(visual.scale, [0.001, 0.001, 0.001]);
    const transform = urdfLinkMatrices(fanuc, [0, 0, 0, 0, 0, 0]).get("link_6")!
      .multiply(urdfOriginMatrix(visual.origin)).scale(new Vector3(...visual.scale));
    close(new Vector3(700, -150, 955).applyMatrix4(transform).toArray(), [0.7, -0.15, 0.955]);
    const yaskawa = parseUrdf(fixture("GP7"));
    const localVisual = yaskawa.links.find(link => link.name === "link_6")!.visuals[0];
    close(localVisual.scale, [1, 1, 1]);
    close(new Vector3(0.08, 0, 0).applyMatrix4(urdfLinkMatrices(yaskawa, [0, 0, 0, 0, 0, 0]).get("link_6")!).toArray(), [0.56, 0, 0.815]);
    expect(yaskawa.joints.find(j => j.name === "joint_3")!.axis).toEqual([0, -1, 0]);
    close(forwardUrdfKinematics(yaskawa, [0, 0, 0, 0, 0, 0]).toolPose!.orientation.map(Math.abs), [0, Math.SQRT1_2, 0, Math.SQRT1_2]);
  });

  it("applies fixed-axis roll/pitch/yaw in URDF order", () => {
    const transform = urdfOriginMatrix({ xyz: [1, 2, 3], rpy: [Math.PI / 2, Math.PI / 2, 0] });
    close(new Vector3(0, 1, 0).applyMatrix4(transform).toArray(), [2, 2, 3]);
    const model = parseUrdf(robot);
    close(forwardUrdfKinematics(model, [90]).toolPose!.position, [0, 1000, 1000]);
    expect(() => forwardUrdfKinematics(model, [])).toThrow();
    expect(() => forwardUrdfKinematics(model, [NaN])).toThrow();
  });
});

describe("bounded URDF import", () => {
  it.each([
    ["DTD", `<!DOCTYPE robot [<!ENTITY data SYSTEM "file:///etc/passwd">]>${robot}`],
    ["remote mesh", robot.replace("meshes/arm.stl", "https://example.com/arm.stl")],
    ["parent traversal", robot.replace("meshes/arm.stl", "../arm.stl")],
    ["encoded path", robot.replace("meshes/arm.stl", "meshes/%2e%2e/arm.stl")],
    ["mimic", robot.replace("<axis xyz=", '<mimic joint="other"/><axis xyz=')],
    ["unknown type", robot.replace('type="revolute"', 'type="prismatic"')],
    ["nonfinite origin", robot.replace('xyz="0 0 1"', 'xyz="0 NaN 1"')],
    ["zero axis", robot.replace('xyz="0 0 2"', 'xyz="0 0 0"')],
    ["reversed limits", robot.replace('lower="-2"', 'lower="3"')],
    ["duplicate link", robot.replace('<link name="base"/>', '<link name="base"/><link name="base"/>')],
    ["missing parent", robot.replace('parent link="base"', 'parent link="missing"')],
    ["cycle", robot.replace('parent link="base"', 'parent link="tool0"')],
  ])("rejects %s", (_name, xml) => expect(() => parseUrdf(xml)).toThrow());

  it("reports missing parser and missing manifest mesh without fetching", async () => {
    vi.stubGlobal("DOMParser", undefined);
    expect(() => parseUrdf(robot)).toThrow("XML");
    vi.stubGlobal("DOMParser", XmlDomParser);
    const fetcher = vi.fn(); vi.stubGlobal("fetch", fetcher);
    await expect(prepareUrdfModel({ id: "test", urdfText: robot, assetUrls: {} })).rejects.toThrow("网格缺失");
    expect(fetcher).not.toHaveBeenCalled();
  });

  it("preloads actual STL geometry and disposes partial downloads on failure", async () => {
    vi.stubGlobal("fetch", vi.fn(async () => new Response(triangle)));
    const model = await prepareUrdfModel({ id: "test", urdfText: robot, assetUrls: { "meshes/arm.stl": "blob:valid" } });
    expect(model.geometries.get("meshes/arm.stl")!.getAttribute("position").count).toBe(3);
    const dispose = vi.spyOn(BufferGeometry.prototype, "dispose");
    disposePreparedUrdfModel(model);
    expect(dispose).toHaveBeenCalledTimes(1);
    dispose.mockClear();
    const two = robot.replace('<link name="base"/>', '<link name="base"><visual><geometry><mesh filename="meshes/base.stl"/></geometry></visual></link>');
    vi.stubGlobal("fetch", vi.fn(async (url: string) => url.endsWith("bad") ? new Response("", { status: 404 }) : new Response(triangle)));
    await expect(prepareUrdfModel({ id: "test", urdfText: two, assetUrls: { "meshes/arm.stl": "blob:valid", "meshes/base.stl": "blob:bad" } })).rejects.toThrow("网格读取失败");
    expect(dispose).toHaveBeenCalledTimes(1);
  });

  it("cancellation cannot return a selectable model", async () => {
    const controller = new AbortController(); controller.abort();
    const fetcher = vi.fn(); vi.stubGlobal("fetch", fetcher);
    await expect(prepareUrdfModel({ id: "test", urdfText: robot, assetUrls: { "meshes/arm.stl": "blob:valid" }, signal: controller.signal })).rejects.toThrow();
    expect(fetcher).not.toHaveBeenCalled();
  });
});
