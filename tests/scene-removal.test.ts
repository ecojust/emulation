import { describe, expect, it } from 'vitest';
import { Matrix4, Quaternion, Vector3 } from 'three';
import { forwardKinematics, quaternionDistanceDeg } from '../src/robot/kinematics';
import { getRobotModel } from '../src/robot/model';
import { eulerZYXToQuaternion } from '../src/robot/rapid';
import type { Pose, Vec3 } from '../src/robot/types';
import {
  FLANGE_PARENT, WORLD_PARENT, activeToolTcpPose, addReferenceFrame, addToolFrame, createFrameGraph,
  editPathAttachment, editRobotBase, editToolFrame, identityPose, pathWorldPose, referenceWorldPose,
  removeReferenceFrame, removeToolFrame, reparentFrameObject, robotBaseWorldPose, selectActiveTool,
  toolTcpWorldPose, toolWorldPose,
} from '../src/workspace/frame-graph';
import { detachRobotTools } from '../src/workspace/scene-removal';

const pose = (position: Vec3 = [0, 0, 0], angles: Vec3 = [0, 0, 0]): Pose => ({ position, orientation: eulerZYXToQuaternion(angles) });
function near(actual: Pose, expected: Pose) {
  actual.position.forEach((value, index) => expect(value).toBeCloseTo(expected.position[index], 7));
  expect(quaternionDistanceDeg(actual.orientation, expected.orientation)).toBeLessThan(1e-5);
}
/** Independent matrix-chain oracle for transformed mount/TCP poses. */
function world(...chain: Pose[]): Pose {
  const product = new Matrix4();
  for (const value of chain) {
    const [w, x, y, z] = value.orientation;
    product.multiply(new Matrix4().compose(new Vector3(...value.position), new Quaternion(x, y, z, w), new Vector3(1, 1, 1)));
  }
  const p = new Vector3(), q = new Quaternion();
  product.decompose(p, q, new Vector3());
  return { position: [p.x, p.y, p.z], orientation: [q.w, q.x, q.y, q.z] };
}
function fixture() {
  const model = getRobotModel('irb1100-4-0.58');
  const flangeAtAttachment = forwardKinematics(model, model.readyJoints).pose;
  const currentFlange = forwardKinematics(model, [35, 12, -42, 18, 53, -27]).pose;
  const fixturePose = pose([250, -170, 80], [45, 20, -10]), baseLocal = pose([100, 50, 20], [10, -15, 20]);
  let graph = addReferenceFrame(createFrameGraph(model.id), { id: 'fixture', localPose: fixturePose }).graph;
  graph = reparentFrameObject(graph, { kind: 'robot' }, { kind: 'frame', id: 'fixture' });
  graph = editRobotBase(graph, baseLocal);
  graph = reparentFrameObject(graph, { kind: 'path', id: 'polish' }, { kind: 'frame', id: 'fixture' });
  graph = editPathAttachment(graph, 'polish', pose([70, 20, 600], [-20, 45, 10]));
  for (const [id, localPose, tcpPose] of [
    ['wheel', pose([25, 5, 40], [30, 20, -15]), pose([10, 0, 90], [12, -17, 35])],
    ['probe', pose([-30, 45, 10], [-60, 10, 35]), pose([0, 12, 60], [-25, 15, 10])],
  ] as const) {
    graph = addToolFrame(graph, { id, tcpPose }).graph;
    graph = reparentFrameObject(graph, { kind: 'tool', id }, FLANGE_PARENT, flangeAtAttachment);
    graph = editToolFrame(graph, id, { localPose });
  }
  graph = addToolFrame(graph, { id: 'spare', localPose: pose([-300, 50, 125], [10, 80, -40]) }).graph;
  graph = selectActiveTool(graph, 'wheel');
  return { graph, currentFlange, flangeAtAttachment, fixturePose, baseLocal };
}

describe('deleting a complete robot body', () => {
  it('detaches every mounted tool at actual current FK, preserving rotated world mounts and TCPs', () => {
    const { graph, currentFlange, flangeAtAttachment, fixturePose, baseLocal } = fixture();
    const next = detachRobotTools(graph, currentFlange);
    expect(next.tools).toHaveLength(3);
    for (const id of ['wheel', 'probe']) {
      const before = graph.tools.find(tool => tool.id === id)!;
      const after = next.tools.find(tool => tool.id === id)!;
      expect(after.parent).toEqual(WORLD_PARENT);
      near(toolWorldPose(next, id), world(fixturePose, baseLocal, currentFlange, before.localPose));
      near(toolTcpWorldPose(next, id), world(fixturePose, baseLocal, currentFlange, before.localPose, before.tcpPose));
      expect(Math.hypot(...after.localPose.position.map((value, i) => value - toolWorldPose(graph, id, flangeAtAttachment).position[i]))).toBeGreaterThan(1);
      expect(after.tcpPose).toBe(before.tcpPose);
      expect(after.radiusMm).toBe(before.radiusMm);
      expect(before.parent).toEqual(FLANGE_PARENT);
    }
    expect(next.activeToolId).toBeNull();
    near(activeToolTcpPose(next), identityPose());
    expect(graph.activeToolId).toBe('wheel');
    expect(next.revision).toBeGreaterThan(graph.revision);
    expect(next.frames).toBe(graph.frames);
    expect(next.robotBase).toBe(graph.robotBase);
    expect(next.paths).toBe(graph.paths);
    expect(next.tools.find(tool => tool.id === 'spare')).toBe(graph.tools.find(tool => tool.id === 'spare'));
    expect(detachRobotTools(next)).toBe(next);
  });

  it('does not discard mounted tools when current flange data is missing or invalid', () => {
    const { graph } = fixture();
    const before = JSON.stringify(graph);
    expect(() => detachRobotTools(graph)).toThrow(/法兰/);
    expect(() => detachRobotTools(graph, pose([NaN, 0, 0]))).toThrow();
    expect(() => detachRobotTools(graph, { position: [0, 0, 0], orientation: [0, 0, 0, 0] })).toThrow();
    expect(JSON.stringify(graph)).toBe(before);
    expect(graph.activeToolId).toBe('wheel');
    expect(graph.tools.filter(tool => tool.parent.kind === 'flange')).toHaveLength(2);
  });

  it('allows removing a body with only independent world tools without needing FK', () => {
    for (const model of ['irb1100-4-0.58', 'fanuc-crx10ia-l', 'yaskawa-gp7']) {
      const graph = addToolFrame(createFrameGraph(model), { id: 'spare', localPose: pose([13, 29, 41], [70, -20, 10]) }).graph;
      expect(detachRobotTools(graph)).toBe(graph);
    }
  });
});

describe('independent scene removals retain surviving objects', () => {
  it('deleting a parent frame preserves its child frame, robot, Path and mounted tools in world', () => {
    const { graph, currentFlange } = fixture();
    const before = addReferenceFrame(graph, { id: 'child', parent: { kind: 'frame', id: 'fixture' }, localPose: pose([10, 20, 30], [50, -10, 70]) }).graph;
    const next = removeReferenceFrame(before, 'fixture');
    expect(next.frames.map(frame => frame.id)).toEqual(['child']);
    expect(next.frames[0].parent).toEqual(WORLD_PARENT);
    expect(next.robotBase.parent).toEqual(WORLD_PARENT);
    expect(next.paths.polish.parent).toEqual(WORLD_PARENT);
    near(referenceWorldPose(next, 'child'), referenceWorldPose(before, 'child'));
    near(robotBaseWorldPose(next), robotBaseWorldPose(before));
    near(pathWorldPose(next, 'polish'), pathWorldPose(before, 'polish'));
    for (const tool of before.tools) near(toolTcpWorldPose(next, tool.id, currentFlange), toolTcpWorldPose(before, tool.id, currentFlange));
    expect(next.tools).toBe(before.tools);
    expect(next.activeToolId).toBe('wheel');
  });

  it('deleting an active tool clears the TCP without deleting other tools, frames or Path data', () => {
    const { graph } = fixture();
    const next = removeToolFrame(graph, 'wheel');
    expect(next.tools.map(tool => tool.id)).toEqual(['probe', 'spare']);
    expect(next.activeToolId).toBeNull();
    near(activeToolTcpPose(next), identityPose());
    expect(next.frames).toBe(graph.frames);
    expect(next.paths).toBe(graph.paths);
    expect(next.robotBase).toBe(graph.robotBase);
    expect(graph.activeToolId).toBe('wheel');
  });
});
