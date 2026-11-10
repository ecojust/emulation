import type { Pose } from '../robot/types';
import { reparentFrameObject, selectActiveTool, WORLD_PARENT, type FrameGraph } from './frame-graph';

/** Robot axes and flange are parts of one body, never independent scene deletions. */
export type SceneRemovalKind = 'robot' | 'frame' | 'tool' | 'zone' | 'path' | 'target';

const labels: Record<SceneRemovalKind, string> = {
  robot: '删除机器人', frame: '删除参考坐标系', tool: '删除工具', zone: '删除禁入区', path: '删除 Path', target: '删除目标点',
};
export function sceneRemovalLabel(kind: SceneRemovalKind): string { return labels[kind]; }

export function sceneRemovalMessage(kind: SceneRemovalKind, name: string, count = 0): string {
  switch (kind) {
    case 'robot': return `将删除 ${name} 的整个本体及所有关节轴。挂接的工具转到世界坐标，世界位置与姿态保持不变；参考坐标系、路径、禁入区和模块文件保留。`;
    case 'frame': return `将删除 ${name}。子对象转到世界坐标，世界位置与姿态保持不变。已生成的模块文件保持不变。`;
    case 'tool': return `将删除 ${name}。若它是当前工具，TCP 将恢复为法兰；已记录的目标点和模块文件保持不变。`;
    case 'zone': return `将删除 ${name}，此区域将不再参与干涉检查和路径规划。已生成的模块文件保持不变。`;
    case 'path': return `将删除 ${name} 及其中的 ${count} 个点。已生成的模块文件保持不变。`;
    case 'target': return `将删除 ${name}。已生成的模块文件保持不变。`;
  }
}

/** Prepare a robot-body deletion before the caller clears its presence flag.
 * The current FK flange is required for mounted tools; recorded target poses
 * cannot substitute for it. Independent frames, paths and world tools remain.
 */
export function detachRobotTools(graph: FrameGraph, flangeInBase?: Pose): FrameGraph {
  let next = graph;
  for (const tool of graph.tools) {
    if (tool.parent.kind === 'flange') next = reparentFrameObject(next, { kind: 'tool', id: tool.id }, WORLD_PARENT, flangeInBase);
  }
  return selectActiveTool(next, null);
}
