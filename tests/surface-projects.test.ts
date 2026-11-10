import { describe, expect, it } from 'vitest';
import { createSimulationProject, validateSimulationProject } from '../src/projects/schema';
import { addSourceDocument, createSourceProject } from '../src/programming/documents';
import { importSurface } from '../src/surface/import';
import { evaluateSurfaceAnchor } from '../src/surface/validation';
import { createFrameGraph } from '../src/workspace/frame-graph';
import { appendSurfacePathPoints, createProgramPath, createRobotProgram, createSurfaceUvPath, setSurfacePathControl, type ProgramSnapshot } from '../src/workspace/program';
import type { SurfacePathSample } from '../src/workspace/surface-path';

const ID = 'irb1100-4-0.58';
async function projectWithSurface(resolved = false) {
  const bytes = new TextEncoder().encode('v 0 0 0\nv 10 0 0\nv 0 10 0\nf 1 2 3\n').buffer;
  const surface = await importSurface({ name: 'part.obj', size: bytes.byteLength, arrayBuffer: async () => bytes }, { id: 'surface' });
  const project = createSimulationProject('Surface project', ID);
  project.scene.surfaces = [surface];
  let program = createProgramPath(createRobotProgram(ID), '区域', 'region').program;
  const anchors = [[.8, .1, .1], [.3, .6, .1], [.2, .4, .4], [.3, .1, .6]] as [number, number, number][];
  const samples: SurfacePathSample[] = anchors.map(barycentric => {
    const anchor = { triangleIndex: 0, barycentric }, picked = evaluateSurfaceAnchor(surface, anchor);
    return { pose: { position: picked.position, orientation: [1, 0, 0, 0] }, pick: { ...anchor, ...picked, surfaceId: surface.id, surfaceRevision: surface.revision } };
  });
  program = appendSurfacePathPoints(program, 'region', samples, ['s1', 's2', 's3', 's4']).program;
  for (const id of ['s1', 's2', 's3', 's4']) program = setSurfacePathControl(program, 'region', id, true);
  const snapshots: ProgramSnapshot[] = samples.map(sample => ({ pose: sample.pose, joints: [0, 20, -30, 0, 40, 0], configuration: [0, 0, 0, 0], speedMmPerSecond: 50, motionRole: 'process' }));
  program = createSurfaceUvPath(program, 'region', program.paths[0].surface!.revision, samples, { uSpacingMm: 1, vSpacingMm: 2, reverseNormal: false }, { id: 'uv', pointIds: ['u1', 'u2', 'u3', 'u4'], ...(resolved ? { resolvedSnapshots: snapshots } : {}) }).program;
  project.scene.models[ID] = { robotPresent: true, joints: [0, 20, -30, 0, 40, 0], graph: createFrameGraph(ID), program };
  project.sources[ID] = addSourceDocument(createSourceProject(ID), 'main.mod', 'MODULE UserProgram\nENDMODULE');
  return project;
}

describe('project-owned surface assets and targets', () => {
  it('roundtrips original mesh bytes, picks, control points and both unsolved and resolved UV geometry independently of source files', async () => {
    for (const resolved of [false, true]) {
      const project = await projectWithSurface(resolved), restored = validateSimulationProject(JSON.parse(JSON.stringify(project)));
      expect(restored).toEqual(project);
      restored.scene.surfaces![0].mesh.positions[0] = 99;
      expect(project.scene.surfaces![0].mesh.positions[0]).toBe(0);
    }
  });
  it('upgrades legacy scenes with an empty asset list without changing the active robot or source', () => {
    const project = { ...createSimulationProject('old', ID), schemaVersion: 1 }; delete project.scene.surfaces;
    const restored = validateSimulationProject(project);
    expect(restored.schemaVersion).toBe(2); expect(project.schemaVersion).toBe(1);
    expect(restored.scene.surfaces).toEqual([]); expect(restored.scene.activeModelId).toBe(ID); expect(restored.sources).toEqual(project.sources);
  });
  it('retains stale picked geometry while clearing executable solutions after model deletion or revision changes', async () => {
    const project = await projectWithSurface(true), original = structuredClone(project);
    const deleted = structuredClone(project); deleted.scene.surfaces = [];
    const moved = structuredClone(project); moved.scene.surfaces![0].revision++; moved.scene.surfaces![0].pose.position[0] += 20;
    for (const changed of [deleted, moved]) {
      const restored = validateSimulationProject(changed), program = restored.scene.models[ID].program;
      expect(program.points).toEqual([]);
      for (const path of program.paths) expect(path.surface?.invalidated).toBe(true);
      expect(program.paths[1].surface?.resolvedRevision).toBeUndefined(); expect(program.paths[1].pointIds).toEqual([]);
      expect(program.paths[1].surface?.points).toEqual(project.scene.models[ID].program.paths[1].surface?.points);
      expect(restored.sources).toEqual(project.sources);
    }
    expect(project).toEqual(original);
  });
  it('invalidates derived solutions after source region changes without modifying the current region or any module', async () => {
    const project = await projectWithSurface(true), changed = structuredClone(project);
    (changed.scene.models[ID].program.paths[0].surface as any).revision++;
    const restored = validateSimulationProject(changed), paths = restored.scene.models[ID].program.paths;
    expect(paths[0]).toEqual(changed.scene.models[ID].program.paths[0]);
    expect(paths[1].surface?.invalidated).toBe(true); expect(paths[1].pointIds).toEqual([]);
    expect(restored.sources).toEqual(project.sources);
  });
  it('rejects current provenance mismatches, invalid normals and duplicate geometric identities', async () => {
    const project = await projectWithSurface();
    const forged = structuredClone(project); forged.scene.models[ID].program.paths[0].surface!.points[0].pick.position[0] += .01;
    expect(() => validateSimulationProject(forged)).toThrow(/位置/);
    const badNormal = structuredClone(project); badNormal.scene.surfaces![0].mesh.normals[2] = -1;
    expect(() => validateSimulationProject(badNormal)).toThrow(/绕序/);
    const duplicate = structuredClone(project); (duplicate.scene.models[ID].program.paths[1].surface!.points[0] as any).id = 's1';
    expect(() => validateSimulationProject(duplicate)).toThrow(/重复/);
    const movedWithoutRevision = structuredClone(project); movedWithoutRevision.scene.surfaces![0].pose.position[0] += 1;
    expect(() => validateSimulationProject(movedWithoutRevision)).toThrow(/位置/);
  });
});
