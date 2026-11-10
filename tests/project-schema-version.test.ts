import { describe, expect, it } from 'vitest';
import { addSourceDocument, createSourceProject } from '../src/programming/documents';
import type { SourceProjectRepository } from '../src/programming/storage';
import { ProjectRepository, type ProjectBackend } from '../src/projects/repository';
import { createSimulationProject, projectSummary, validateSimulationProject, type SimulationProject } from '../src/projects/schema';
import { evaluateSurfaceAnchor } from '../src/surface/validation';
import { addReferenceFrame, createFrameGraph, identityPose, reparentFrameObject } from '../src/workspace/frame-graph';
import { appendSurfacePathPoints, captureProgramPointToPath, createProgramPath, createRobotProgram, setSurfacePathControl } from '../src/workspace/program';
import type { Vec3 } from '../src/robot/types';

const MODEL = 'irb1100-4-0.58';
class MemoryBackend implements ProjectBackend {
  rows = new Map<string, unknown>();
  summaries = new Map<string, unknown>();
  list = async () => structuredClone([...this.summaries.values()]);
  read = async (id: string) => structuredClone(this.rows.get(id) ?? null);
  migrated = async () => true;
  async change(id: string, update: (current: unknown | null) => SimulationProject | null) {
    const next = update(structuredClone(this.rows.get(id) ?? null));
    if (next) { this.rows.set(id, structuredClone(next)); this.summaries.set(id, projectSummary(next)); }
    else { this.rows.delete(id); this.summaries.delete(id); }
    return next;
  }
  async importLegacyOnce(_project: SimulationProject | null) { throw new Error('Already migrated'); }
}
const legacySources: SourceProjectRepository = { load: async () => null, save: async () => { throw new Error('Must not rewrite legacy sources'); } };
function repositoryWith(raw: { id: string }) {
  const backend = new MemoryBackend(); backend.rows.set(raw.id, structuredClone(raw));
  return { backend, repository: new ProjectRepository(backend, legacySources) };
}
function ordinaryProject() {
  const project = createSimulationProject('Existing project', MODEL, 'versioned-project', '2026-01-01T00:00:00Z');
  let program = createProgramPath(createRobotProgram(MODEL), 'Existing path', 'existing-path').program;
  program = captureProgramPointToPath(program, 'existing-path', {
    pose: { position: [12, 23, 34], orientation: [1, 0, 0, 0] }, joints: [0, 20, -30, 0, 40, 0],
    configuration: [0, 0, 0, 0], speedMmPerSecond: 50,
  }, 'existing-point').program;
  let graph = addReferenceFrame(createFrameGraph(MODEL), { id: 'fixture', localPose: { position: [120, -80, 30], orientation: [Math.SQRT1_2, 0, 0, Math.SQRT1_2] } }).graph;
  graph = reparentFrameObject(graph, { kind: 'path', id: 'existing-path' }, { kind: 'frame', id: 'fixture' });
  project.scene.models[MODEL] = { robotPresent: true, joints: [0, 20, -30, 0, 40, 0], graph, program };
  project.scene.camera = { position: [2, 3, 4], target: [0, 0, .5] };
  project.scene.zones[0].center = ['', '0', '0'];
  project.sources[MODEL] = addSourceDocument(createSourceProject(MODEL), 'user.mod', 'MODULE Existing\nENDMODULE');
  return project;
}
function legacyProject() {
  const legacy = { ...ordinaryProject(), schemaVersion: 1 as const };
  delete legacy.scene.surfaces;
  return legacy;
}
function surfaceProject() {
  const project = ordinaryProject(), model = project.scene.models[MODEL];
  const surface = {
    id: 'surface', name: 'Part', revision: 0, visible: true, pose: identityPose(),
    source: { name: 'part.obj', format: 'obj' as const, unit: 'mm' as const },
    mesh: { positions: [0, 0, 0, 10, 0, 0, 0, 10, 0], normals: [0, 0, 1, 0, 0, 1, 0, 0, 1] },
  };
  let program = createProgramPath(model.program, 'Region', 'region').program;
  const weights: Vec3[] = [[.8, .1, .1], [.3, .6, .1], [.2, .4, .4], [.3, .1, .6]];
  const samples = weights.map(barycentric => {
    const anchor = { triangleIndex: 0, barycentric }, picked = evaluateSurfaceAnchor(surface, anchor);
    return { pose: { position: picked.position, orientation: [1, 0, 0, 0] as [number, number, number, number] },
      pick: { ...anchor, ...picked, surfaceId: surface.id, surfaceRevision: surface.revision } };
  });
  const ids = ['corner-1', 'corner-2', 'corner-3', 'corner-4'];
  program = appendSurfacePathPoints(program, 'region', samples, ids).program;
  for (const id of ids) program = setSurfacePathControl(program, 'region', id, true);
  project.scene.surfaces = [surface]; model.program = program;
  return validateSimulationProject(project);
}

describe('simulation project format versioning', () => {
  it('reads v1 without writing and upgrades at the next save while preserving existing scene and source data', async () => {
    const raw = legacyProject(), original = structuredClone(raw), { repository, backend } = repositoryWith(raw);
    const loaded = await repository.load(raw.id);
    expect(loaded).toEqual({ ...original, schemaVersion: 2, scene: { ...original.scene, surfaces: [] } });
    expect(backend.rows.get(raw.id)).toEqual(original);
    expect(raw).toEqual(original);
    const saved = await repository.saveScene(raw.id, loaded.scene);
    expect(saved.schemaVersion).toBe(2); expect(saved.scene).toEqual(loaded.scene); expect(saved.sources).toEqual(original.sources);
    expect(saved.id).toBe(original.id); expect(saved.createdAt).toBe(original.createdAt);
    expect(saved.revision).toBe(original.revision + 1);
    expect(backend.rows.get(raw.id)).toEqual(saved); expect(await repository.list()).toEqual([projectSummary(saved)]);
  });
  it('writes v2 on new projects and preserves mesh, corner provenance and sources through subsequent writes', async () => {
    const backend = new MemoryBackend(), repository = new ProjectRepository(backend, legacySources);
    const fresh = await repository.create('New project', MODEL);
    expect(fresh.schemaVersion).toBe(2); expect((backend.rows.get(fresh.id) as SimulationProject).schemaVersion).toBe(2);
    const project = surfaceProject(); backend.rows.set(project.id, structuredClone(project));
    const renamed = await repository.rename(project.id, 'Renamed surface');
    expect(renamed.schemaVersion).toBe(2); expect(renamed.scene).toEqual(project.scene); expect(renamed.sources).toEqual(project.sources);
    const source = addSourceDocument(project.sources[MODEL], 'another.mod', 'MODULE Added\nENDMODULE');
    await repository.sourcesFor(project.id).save(source);
    const saved = await repository.load(project.id);
    expect(saved.schemaVersion).toBe(2); expect(saved.scene).toEqual(project.scene); expect(saved.sources[MODEL]).toEqual(source);
    expect(saved.sources[MODEL].schemaVersion).toBe(1); // Separate source-document format remains unchanged.
  });
  it('refuses unsupported future versions on read, rename, scene save and source save without changing durable bytes', async () => {
    const project = surfaceProject();
    for (const schemaVersion of [0, 3, 99, '2', null, undefined]) {
      expect(() => validateSimulationProject({ ...project, schemaVersion })).toThrow(/项目版本/);
    }
    const future = { ...project, schemaVersion: 3 }, { repository, backend } = repositoryWith(future);
    const durableBytes = JSON.stringify(backend.rows.get(future.id));
    const operations = [
      () => repository.load(future.id), () => repository.rename(future.id, 'Do not overwrite'),
      () => repository.saveScene(future.id, project.scene), () => repository.sourcesFor(future.id).save(project.sources[MODEL]),
    ];
    for (const operation of operations) {
      await expect(operation()).rejects.toThrow(/项目版本/);
      expect(JSON.stringify(backend.rows.get(future.id))).toBe(durableBytes);
    }
  });
  it('keeps surface projects outside the published v0.4.1 reader contract before its lossy scene normalization', async () => {
    const project = surfaceProject(), wire = JSON.parse(JSON.stringify(project));
    // Archived compatibility boundary: v0.4.1 (e9d46d5), schema.ts,
    // validateSimulationProject requires raw.schemaVersion === 1 before
    // normalizing the scene. Retain only that guard, not a second serializer.
    const v041VersionGuard = (raw: { schemaVersion: unknown }) => {
      if (raw.schemaVersion !== 1) throw new Error('项目数据无效：项目版本');
    };
    expect(() => v041VersionGuard(legacyProject())).not.toThrow();
    expect(() => v041VersionGuard(wire)).toThrow(/项目版本/);
    const { backend } = repositoryWith(wire), before = JSON.stringify(backend.rows.get(wire.id));
    await expect(backend.change(wire.id, raw => {
      v041VersionGuard(raw as { schemaVersion: unknown });
      throw new Error('The old reader must never reach scene normalization');
    })).rejects.toThrow('项目数据无效：项目版本');
    expect(JSON.stringify(backend.rows.get(wire.id))).toBe(before);
  });
});
