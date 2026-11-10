import { describe, expect, it } from 'vitest';
import { quaternionDistanceDeg } from '../src/robot/kinematics';
import { eulerZYXToQuaternion } from '../src/robot/rapid';
import type { Pose, Vec3 } from '../src/robot/types';
import {
  FLANGE_PARENT, WORLD_PARENT, activeToolTcpPose, addReferenceFrame, addToolFrame,
  baseFlangeToWorldTcp, composePose, createFrameGraph, editPathAttachment, editReferenceFrame,
  editRobotBase, editToolFrame, identityPose, inversePose, normalizePose, parentWorldPose,
  pathAttachment, pathPointToWorld, pathWorldPose, poseEulerZYX, poseFromMatrix, poseMatrix,
  referenceWorldPose, relativePose, removePathAttachment, removeReferenceFrame, removeToolFrame,
  reparentFrameObject, replaceFrameGraph, robotBaseWorldPose, selectActiveTool, toolEnvelopeWorld,
  toolTcpWorldPose, toolWorldPose, worldPointToPath, worldTcpToBaseFlange, type FrameGraph,
  type FrameParent,
} from '../src/workspace/frame-graph';

const pose = (position: Vec3 = [0, 0, 0], angles: Vec3 = [0, 0, 0]): Pose => ({ position, orientation: eulerZYXToQuaternion(angles) });
const frame = (id: string): FrameParent => ({ kind: 'frame', id });
function near(actual: Pose, expected: Pose) {
  actual.position.forEach((value, index) => expect(value).toBeCloseTo(expected.position[index], 8));
  expect(quaternionDistanceDeg(actual.orientation, expected.orientation)).toBeLessThan(1e-5);
}
function fixture() {
  let graph = createFrameGraph('irb1100-4-0.58');
  graph = addReferenceFrame(graph, { id: 'fixture-a', name: 'Fixture A', localPose: pose([200, -100, 350], [90, 25, -30]) }).graph;
  graph = addReferenceFrame(graph, { id: 'fixture-b', name: 'Fixture B', localPose: pose([-170, 240, 90], [-75, -15, 40]) }).graph;
  return graph;
}

describe('brand-neutral rigid transforms', () => {
  it('uses actual rotation when composing a translated child', () => {
    const result = composePose(pose([100, 200, 300], [90, 0, 0]), pose([40, 0, 10], [0, 30, 0]));
    near(result, pose([100, 240, 310], [90, 30, 0]));
    near(relativePose(pose([100, 200, 300], [90, 0, 0]), result), pose([40, 0, 10], [0, 30, 0]));
  });

  it('inverts arbitrary noncommuting transforms in both multiplication orders', () => {
    for (const angles of [[32, 19, -84], [-161, 89, 153], [45, 90, 0], [0, -90, 37]] as Vec3[]) {
      const value = pose([172.4, -538.7, 693.2], angles);
      near(composePose(value, inversePose(value)), identityPose());
      near(composePose(inversePose(value), value), identityPose());
      near(poseFromMatrix(poseMatrix(value)), value);
      near(pose(value.position, poseEulerZYX(value)), value);
    }
  });

  it('normalizes q and -q consistently without accepting zero, sparse or nonnumeric tuples', () => {
    near(normalizePose({ position: [1, 2, 3], orientation: [-20, 0, 0, 0] }), pose([1, 2, 3]));
    expect(() => normalizePose({ position: [1, 2, 3], orientation: [0, 0, 0, 0] })).toThrow();
    expect(() => normalizePose({ position: [NaN, 2, 3], orientation: [1, 0, 0, 0] })).toThrow();
    expect(() => normalizePose({ position: [1, 2, 3], orientation: [1, Infinity, 0, 0] })).toThrow();
    expect(() => normalizePose({ position: Array(3) as Vec3, orientation: [1, 0, 0, 0] })).toThrow();
    expect(() => normalizePose({ position: ['1', 2, 3] as unknown as Vec3, orientation: [1, 0, 0, 0] })).toThrow();
    expect(() => composePose(pose([1e308, 0, 0]), pose([1e308, 0, 0]))).toThrow();
  });

  it('rejects non-rigid matrices rather than laundering scale or reflection into a quaternion', () => {
    for (const bad of [
      [2, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
      [-1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
      [1, .5, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
      [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, .1, 0, 0, 1],
    ]) expect(() => poseFromMatrix(bad)).toThrow();
  });
});

describe('attachment graphs preserve world poses and model boundaries', () => {
  it('starts with a world-aligned base and implicit world-aligned Path without a tool offset', () => {
    const graph = createFrameGraph('yaskawa-gp7');
    expect(graph.modelId).toBe('yaskawa-gp7');
    expect(graph.frames).toEqual([]); expect(graph.tools).toEqual([]);
    near(robotBaseWorldPose(graph), identityPose());
    near(pathWorldPose(graph, 'path-1'), identityPose());
    near(activeToolTcpPose(graph), identityPose());
    near(baseFlangeToWorldTcp(graph, pose([300, 20, 700], [90, 40, 10])), pose([300, 20, 700], [90, 40, 10]));
  });

  it('reparents a robot base between rotated fixtures without moving it in world', () => {
    const before = editRobotBase(fixture(), pose([120, 40, -20], [10, -15, 17]));
    const original = robotBaseWorldPose(before);
    let next = reparentFrameObject(before, { kind: 'robot' }, frame('fixture-a'));
    near(robotBaseWorldPose(next), original);
    near(next.robotBase.localPose, relativePose(referenceWorldPose(next, 'fixture-a'), original));
    expect(next.robotBase.localPose.position).not.toEqual(original.position);
    next = reparentFrameObject(next, { kind: 'robot' }, frame('fixture-b'));
    near(robotBaseWorldPose(next), original);
    next = reparentFrameObject(next, { kind: 'robot' }, WORLD_PARENT);
    near(robotBaseWorldPose(next), original);
    near(before.robotBase.localPose, original);
  });

  it('reparents an entire Path without rewriting any local points, then follows a moved fixture', () => {
    let graph = editPathAttachment(fixture(), 'polish', pose([70, -30, 90], [17, 22, 11]));
    const points = [pose([400, 20, 300], [52, 15, -34]), pose([430, 50, 315], [52, 15, -34])];
    const before = points.map(point => pathPointToWorld(graph, 'polish', point));
    graph = reparentFrameObject(graph, { kind: 'path', id: 'polish' }, frame('fixture-a'));
    points.forEach((point, index) => near(pathPointToWorld(graph, 'polish', point), before[index]));
    graph = reparentFrameObject(graph, { kind: 'path', id: 'polish' }, frame('fixture-b'));
    points.forEach((point, index) => {
      near(pathPointToWorld(graph, 'polish', point), before[index]);
      near(worldPointToPath(graph, 'polish', before[index]), point);
    });
    const parentBefore = referenceWorldPose(graph, 'fixture-b'), attachmentBefore = pathAttachment(graph, 'polish');
    const movedParent = pose([-180, 300, 90], [45, 20, -70]);
    graph = editReferenceFrame(graph, 'fixture-b', { localPose: movedParent });
    expect(pathAttachment(graph, 'polish')).toBe(attachmentBefore);
    const delta = composePose(movedParent, inversePose(parentBefore));
    points.forEach((point, index) => near(pathPointToWorld(graph, 'polish', point), composePose(delta, before[index])));
  });

  it('attaches a nonzero mounting offset and TCP orientation to a moving flange without a jump', () => {
    let graph = reparentFrameObject(editRobotBase(fixture(), pose([50, -20, 10], [27, -17, 9])), { kind: 'robot' }, frame('fixture-a'));
    const flange = pose([420, 55, 640], [120, 35, 40]), mountWorld = pose([440, -85, 710], [85, 29, -10]);
    const tcpLocal = pose([35, -10, 120], [-20, 15, 45]);
    graph = addToolFrame(graph, { id: 'wheel', localPose: mountWorld, tcpPose: tcpLocal, radiusMm: 18 }).graph;
    const tcpWorldBefore = toolTcpWorldPose(graph, 'wheel');
    graph = reparentFrameObject(graph, { kind: 'tool', id: 'wheel' }, FLANGE_PARENT, flange);
    expect(graph.activeToolId).toBe('wheel');
    near(toolWorldPose(graph, 'wheel', flange), mountWorld);
    near(toolTcpWorldPose(graph, 'wheel', flange), tcpWorldBefore);
    near(baseFlangeToWorldTcp(graph, flange), tcpWorldBefore);
    near(worldTcpToBaseFlange(graph, tcpWorldBefore), flange);
    const mountInFlange = graph.tools[0].localPose;
    near(activeToolTcpPose(graph), composePose(mountInFlange, tcpLocal));
    const movedFlange = pose([450, 30, 580], [-30, 80, 9]);
    near(toolWorldPose(graph, 'wheel', movedFlange), composePose(composePose(robotBaseWorldPose(graph), movedFlange), mountInFlange));
    const movedTcp = baseFlangeToWorldTcp(graph, movedFlange);
    near(worldTcpToBaseFlange(graph, movedTcp), movedFlange);
    graph = reparentFrameObject(graph, { kind: 'tool', id: 'wheel' }, WORLD_PARENT, movedFlange);
    expect(graph.activeToolId).toBeNull();
    near(toolTcpWorldPose(graph, 'wheel'), movedTcp);
  });

  it('uses the actual mounting origin for a tool collision envelope', () => {
    let graph = addToolFrame(createFrameGraph('fanuc-crx10ia-l'), { id: 'offset', localPose: pose([100, 20, 300], [90, 0, 0]), tcpPose: pose([100, 0, 30]), radiusMm: 7 }).graph;
    const before = toolEnvelopeWorld(graph, 'offset');
    expect(before.startMm).toEqual([100, 20, 300]);
    before.endMm.forEach((value, index) => expect(value).toBeCloseTo([100, 120, 330][index], 8));
    graph = reparentFrameObject(graph, { kind: 'tool', id: 'offset' }, FLANGE_PARENT, pose([400, 50, 600], [40, 70, -12]));
    const after = toolEnvelopeWorld(graph, 'offset', pose([400, 50, 600], [40, 70, -12]));
    after.startMm.forEach((value, index) => expect(value).toBeCloseTo(before.startMm[index], 8));
    after.endMm.forEach((value, index) => expect(value).toBeCloseTo(before.endMm[index], 8));
    expect(after.radiusMm).toBe(7);
  });

  it('rejects invalid parents and reference cycles before changing graph state', () => {
    let graph = addReferenceFrame(fixture(), { id: 'nested', parent: frame('fixture-a'), localPose: pose([20, 30, 40], [15, 20, 25]) }).graph;
    graph = addToolFrame(graph, { id: 'tool' }).graph;
    expect(() => reparentFrameObject(graph, { kind: 'frame', id: 'fixture-a' }, frame('nested'))).toThrow(/自身或其子节点/);
    expect(() => reparentFrameObject(graph, { kind: 'frame', id: 'nested' }, frame('nested'))).toThrow();
    for (const object of [{ kind: 'robot' }, { kind: 'path', id: 'path' }, { kind: 'frame', id: 'nested' }] as const) {
      expect(() => reparentFrameObject(graph, object, FLANGE_PARENT)).toThrow(/法兰/);
      expect(() => reparentFrameObject(graph, object, frame('missing'))).toThrow();
    }
    expect(() => reparentFrameObject(graph, { kind: 'tool', id: 'tool' }, frame('fixture-a'))).toThrow();
    expect(() => reparentFrameObject(graph, { kind: 'tool', id: 'tool' }, FLANGE_PARENT)).toThrow(/有效法兰/);
    expect(() => reparentFrameObject(graph, { kind: 'path', id: 'tool' }, WORLD_PARENT)).toThrow(/重复/);
    expect(() => addReferenceFrame(graph, { id: 'tool' })).toThrow();
    expect(() => pathWorldPose(graph, '__proto__')).toThrow();
    expect(() => addReferenceFrame(graph, { id: 'constructor' })).toThrow();
    expect(() => parentWorldPose(graph, { kind: 'tool', id: 'tool' } as unknown as FrameParent)).toThrow();
    const corrupted = { ...graph, frames: graph.frames.map(f => f.id === 'fixture-a' ? { ...f, parent: frame('nested') } : f) };
    expect(() => referenceWorldPose(corrupted, 'nested')).toThrow(/循环/);
    expect(referenceWorldPose(graph, 'nested')).toBeDefined();
  });

  it('supports nested reference reparenting and removal while keeping all children in world', () => {
    let graph = addReferenceFrame(fixture(), { id: 'child', parent: frame('fixture-a'), localPose: pose([40, 50, 60], [20, -10, 30]) }).graph;
    graph = addReferenceFrame(graph, { id: 'grandchild', parent: frame('child'), localPose: pose([30, -90, 15], [-12, 55, 45]) }).graph;
    graph = reparentFrameObject(graph, { kind: 'robot' }, frame('fixture-a'));
    graph = reparentFrameObject(graph, { kind: 'path', id: 'path-a' }, frame('fixture-a'));
    graph = reparentFrameObject(graph, { kind: 'path', id: 'path-child' }, frame('child'));
    const before = [referenceWorldPose(graph, 'child'), referenceWorldPose(graph, 'grandchild'), robotBaseWorldPose(graph), pathWorldPose(graph, 'path-a'), pathWorldPose(graph, 'path-child')];
    graph = reparentFrameObject(graph, { kind: 'frame', id: 'fixture-a' }, frame('fixture-b'));
    [referenceWorldPose(graph, 'child'), referenceWorldPose(graph, 'grandchild'), robotBaseWorldPose(graph), pathWorldPose(graph, 'path-a'), pathWorldPose(graph, 'path-child')].forEach((value, index) => near(value, before[index]));
    graph = removeReferenceFrame(graph, 'fixture-a');
    expect(graph.frames.some(f => f.id === 'fixture-a')).toBe(false);
    expect(graph.frames.find(f => f.id === 'child')!.parent).toEqual(WORLD_PARENT);
    expect(graph.robotBase.parent).toEqual(WORLD_PARENT);
    expect(pathAttachment(graph, 'path-a').parent).toEqual(WORLD_PARENT);
    expect(pathAttachment(graph, 'path-child').parent).toEqual(frame('child'));
    [referenceWorldPose(graph, 'child'), referenceWorldPose(graph, 'grandchild'), robotBaseWorldPose(graph), pathWorldPose(graph, 'path-a'), pathWorldPose(graph, 'path-child')].forEach((value, index) => near(value, before[index]));
  });

  it('freezes nested input snapshots and never mutates another robot model', () => {
    const input = pose([100, 200, 300], [17, 28, 39]);
    let first = addReferenceFrame(createFrameGraph('irb1100-4-0.58'), { id: 'a', localPose: input }).graph;
    const second = createFrameGraph('yaskawa-gp7');
    const store = replaceFrameGraph(replaceFrameGraph({}, first), second);
    input.position[0] = -999; input.orientation[0] = 0;
    expect(first.frames[0].localPose.position[0]).toBe(100);
    expect(Object.isFrozen(first.frames[0].localPose.orientation)).toBe(true);
    expect(() => { first.frames[0].localPose.position[1] = -1; }).toThrow();
    first = editReferenceFrame(first, 'a', { name: 'New fixture', localPose: pose([10, 20, 30]) });
    const next = replaceFrameGraph(store, first);
    expect(next['yaskawa-gp7']).toBe(second);
    expect(store['irb1100-4-0.58'].frames[0].name).not.toBe('New fixture');
    expect(next['irb1100-4-0.58'].frames[0].name).toBe('New fixture');
  });

  it('edits tool geometry independently of attachment and clears active selection on deletion', () => {
    let graph = addToolFrame(fixture(), { id: 'tool-a' }).graph;
    graph = addToolFrame(graph, { id: 'tool-b' }).graph;
    expect(() => selectActiveTool(graph, 'tool-a')).toThrow(/挂接/);
    graph = reparentFrameObject(graph, { kind: 'tool', id: 'tool-a' }, FLANGE_PARENT, pose([0, 0, 400]));
    const previous = graph.tools[0];
    graph = editToolFrame(graph, 'tool-a', { name: 'Polisher', tcpPose: pose([10, 0, 70], [20, 30, 40]), radiusMm: 30 });
    expect(graph.tools[0].parent).toEqual(FLANGE_PARENT);
    near(graph.tools[0].localPose, previous.localPose);
    expect(previous.radiusMm).toBe(10); expect(graph.tools[0].radiusMm).toBe(30);
    for (const invalid of [0, -1, NaN, Infinity, 10001]) expect(() => editToolFrame(graph, 'tool-a', { radiusMm: invalid })).toThrow();
    graph = selectActiveTool(graph, null); expect(graph.activeToolId).toBeNull();
    graph = selectActiveTool(graph, 'tool-a');
    graph = removeToolFrame(graph, 'tool-b'); expect(graph.activeToolId).toBe('tool-a');
    graph = removeToolFrame(graph, 'tool-a'); expect(graph.activeToolId).toBeNull();
    near(activeToolTcpPose(graph), identityPose());
  });

  it('removes only the deleted Path attachment and preserves every other object', () => {
    let graph = reparentFrameObject(fixture(), { kind: 'path', id: 'first' }, frame('fixture-a'));
    graph = reparentFrameObject(graph, { kind: 'path', id: 'second' }, frame('fixture-b'));
    const second = graph.paths.second;
    const next = removePathAttachment(graph, 'first');
    expect(next.paths.first).toBeUndefined(); expect(next.paths.second).toBe(second);
    expect(next.frames).toBe(graph.frames); expect(next.tools).toBe(graph.tools);
    expect(removePathAttachment(next, 'missing')).toBe(next);
    expect(removeReferenceFrame(next, 'missing')).toBe(next);
    expect(removeToolFrame(next, 'missing')).toBe(next);
  });
});
