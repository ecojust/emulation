import { useMemo, useRef, useState } from 'react';
import type { Pose } from '../robot/types';
import {
  addReferenceFrame, addToolFrame, createFrameGraph, editPathAttachment, editReferenceFrame, editRobotBase,
  editToolFrame, removePathAttachment, removeReferenceFrame, removeToolFrame, reparentFrameObject,
  replaceFrameGraph, selectActiveTool, type FrameGraph, type FrameGraphStore, type FrameObjectRef, type FrameParent,
} from './frame-graph';

/** Scene frame/attachment edits survive view changes and remain isolated by robot model. */
export function useFrameGraph(modelId: string) {
  const [store, setStore] = useState<FrameGraphStore>(() => Object.freeze({ [modelId]: createFrameGraph(modelId) }));
  const current = useRef(store); current.current = store;
  const graph = useMemo(() => store[modelId] ?? createFrameGraph(modelId), [store, modelId]);
  const active = () => current.current[modelId] ?? createFrameGraph(modelId);
  function commit(next: FrameGraph) {
    if (next.modelId !== modelId) throw new Error('坐标对象不能跨机器人型号写入。');
    const updated = replaceFrameGraph(current.current, next);
    current.current = updated; setStore(updated);
  }
  return {
    graph, frames: graph.frames, tools: graph.tools, robotBase: graph.robotBase, paths: graph.paths,
    activeToolId: graph.activeToolId, revision: graph.revision,
    addFrame(options?: Parameters<typeof addReferenceFrame>[1]) {
      const changed = addReferenceFrame(active(), options); commit(changed.graph); return changed.value;
    },
    addTool(options?: Parameters<typeof addToolFrame>[1]) {
      const changed = addToolFrame(active(), options); commit(changed.graph); return changed.value;
    },
    editFrame(id: string, patch: Parameters<typeof editReferenceFrame>[2]) { commit(editReferenceFrame(active(), id, patch)); },
    editBase(localPose: Pose) { commit(editRobotBase(active(), localPose)); },
    editPath(pathId: string, localPose: Pose) { commit(editPathAttachment(active(), pathId, localPose)); },
    editTool(id: string, patch: Parameters<typeof editToolFrame>[2]) { commit(editToolFrame(active(), id, patch)); },
    reparent(object: FrameObjectRef, parent: FrameParent, flangeInBase?: Pose) { commit(reparentFrameObject(active(), object, parent, flangeInBase)); },
    removeFrame(id: string) { commit(removeReferenceFrame(active(), id)); },
    removeTool(id: string) { commit(removeToolFrame(active(), id)); },
    removePath(pathId: string) { commit(removePathAttachment(active(), pathId)); },
    setActiveTool(id: string | null) { commit(selectActiveTool(active(), id)); },
  };
}
