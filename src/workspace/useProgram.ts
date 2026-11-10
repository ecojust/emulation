import { useRef, useState } from 'react';
import { getProgramProfile } from './program-templates';
import {
  captureProgramPointToPath, createProgramPath, createProgramStore, getProgramPathPoints, removeProgramPath,
  removeProgramPoint, renameProgramPath, replaceRobotProgram, replaceProgramPathPoints, updateProgramPoint,
  appendSurfacePathPoints, setSurfacePathControl, removeSurfacePathPoint, clearSurfacePath, createSurfaceUvPath, resolveSurfacePath, invalidateSurfacePaths, invalidateSurfaceSolutions,
  type CreateSurfaceUvOptions,
  type PathPlanningSettings, type ProgramPath, type ProgramPoint, type ProgramSnapshot, type RobotProgram, type ProgramStore,
} from './program';
import type { SurfacePathPoint, SurfacePathSample, SurfaceUvSettings } from './surface-path';

const NO_POINTS: readonly ProgramPoint[] = Object.freeze([]);
const NO_PATHS: readonly ProgramPath[] = Object.freeze([]);

/** Session projects are isolated by catalog-model ID and its fixed manufacturer profile. */
export function useProgram(modelId: string | null, initialStore?: ProgramStore) {
  if (modelId !== null) getProgramProfile(modelId);
  const [store, setStore] = useState<ProgramStore>(() => Object.freeze({ ...createProgramStore(), ...initialStore }));
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
    store, getStore: () => current.current,
    program, points: program?.points ?? NO_POINTS, paths: program?.paths ?? NO_PATHS, revision: program?.revision ?? 0,
    createPath(name?: string): ProgramPath {
      const changed = createProgramPath(active(), name);
      commit(changed.program); return changed.value;
    },
    captureToPath(pathId: string, snapshot: ProgramSnapshot): ProgramPoint {
      const changed = captureProgramPointToPath(active(), pathId, snapshot);
      commit(changed.program); return changed.value;
    },
    pointsForPath(pathId: string): readonly ProgramPoint[] { return getProgramPathPoints(active(), pathId); },
    updatePoint(id: string, snapshot: ProgramSnapshot): ProgramPoint {
      const changed = updateProgramPoint(active(), id, snapshot);
      commit(changed.program); return changed.value;
    },
    replacePathPoints(pathId: string, snapshots: readonly ProgramSnapshot[], planning?: PathPlanningSettings): readonly ProgramPoint[] {
      const changed = replaceProgramPathPoints(active(), pathId, snapshots, planning); commit(changed.program); return changed.value;
    },
    removePoint(id: string): void { commit(removeProgramPoint(active(), id)); },
    removePath(id: string): void { commit(removeProgramPath(active(), id)); },
    renamePath(id: string, name: string): void { commit(renameProgramPath(active(), id, name)); },
    appendSurfacePoints(pathId: string, samples: readonly SurfacePathSample[], ids?: readonly string[]): readonly SurfacePathPoint[] {
      const changed = appendSurfacePathPoints(active(), pathId, samples, ids); commit(changed.program); return changed.value;
    },
    setSurfaceControl(pathId: string, pointId: string, enabled: boolean): void { commit(setSurfacePathControl(active(), pathId, pointId, enabled)); },
    removeSurfacePoint(pathId: string, pointId: string): void { commit(removeSurfacePathPoint(active(), pathId, pointId)); },
    clearSurfacePath(pathId: string): void { commit(clearSurfacePath(active(), pathId)); },
    createUvPath(sourcePathId: string, expectedRevision: number, samples: readonly SurfacePathSample[], settings: SurfaceUvSettings, options?: CreateSurfaceUvOptions): ProgramPath {
      const changed = createSurfaceUvPath(active(), sourcePathId, expectedRevision, samples, settings, options); commit(changed.program); return changed.value;
    },
    resolveSurfacePath(pathId: string, expectedRevision: number, snapshots: readonly ProgramSnapshot[]): readonly ProgramPoint[] {
      const changed = resolveSurfacePath(active(), pathId, expectedRevision, snapshots); commit(changed.program); return changed.value;
    },
    invalidateSurfacePaths(surfaceId: string): void { commit(invalidateSurfacePaths(active(), surfaceId)); },
    invalidateSurfaceSolutions(): void { commit(invalidateSurfaceSolutions(active())); },
  };
}
