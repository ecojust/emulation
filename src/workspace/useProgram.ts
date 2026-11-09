import { useRef, useState } from 'react';
import { getProgramProfile } from './program-templates';
import {
  captureProgramPoint, convertProgramPointsToPath, createProgramStore, removeProgramPath,
  removeProgramPoint, renameProgramPath, replaceRobotProgram, updateProgramPoint,
  type ProgramPath, type ProgramPoint, type ProgramSnapshot, type RobotProgram,
} from './program';

const NO_POINTS: readonly ProgramPoint[] = Object.freeze([]);
const NO_PATHS: readonly ProgramPath[] = Object.freeze([]);

/** Session projects are isolated by catalog-model ID and its fixed manufacturer profile. */
export function useProgram(modelId: string | null) {
  if (modelId !== null) getProgramProfile(modelId);
  const [store, setStore] = useState(createProgramStore);
  const current = useRef(store);
  current.current = store;
  function active(): RobotProgram {
    if (modelId === null) throw new Error('请先选择已适配的机器人型号。');
    return current.current[modelId];
  }
  function commit(program: RobotProgram): void {
    const next = replaceRobotProgram(current.current, program);
    current.current = next;
    setStore(next);
  }
  const program = modelId !== null ? store[modelId] : null;
  return {
    program, points: program?.points ?? NO_POINTS, paths: program?.paths ?? NO_PATHS, revision: program?.revision ?? 0,
    capture(snapshot: ProgramSnapshot, afterPointId: string | null = null): ProgramPoint {
      const changed = captureProgramPoint(active(), snapshot, afterPointId);
      commit(changed.program); return changed.value;
    },
    updatePoint(id: string, snapshot: ProgramSnapshot): ProgramPoint {
      const changed = updateProgramPoint(active(), id, snapshot);
      commit(changed.program); return changed.value;
    },
    removePoint(id: string): void { commit(removeProgramPoint(active(), id)); },
    convertToPath(ids: readonly string[], name?: string): ProgramPath {
      const changed = convertProgramPointsToPath(active(), ids, name);
      commit(changed.program); return changed.value;
    },
    removePath(id: string): void { commit(removeProgramPath(active(), id)); },
    renamePath(id: string, name: string): void { commit(renameProgramPath(active(), id, name)); },
  };
}
