import { describe, expect, it } from 'vitest';
import { addSourceDocument, createSourceProject, type SourceProject } from '../src/programming/documents';
import type { SourceProjectRepository } from '../src/programming/storage';
import { ProjectRepository, type ProjectBackend } from '../src/projects/repository';
import { createSimulationProject, projectName, projectSummary, validateProjectModelWorld, validateSimulationProject, type SimulationProject } from '../src/projects/schema';
import { addReferenceFrame, addToolFrame, createFrameGraph, reparentFrameObject, identityPose } from '../src/workspace/frame-graph';
import { captureProgramPointToPath, createProgramPath, createRobotProgram } from '../src/workspace/program';

const ID = 'irb1100-4-0.58';
const cover = 'data:image/png;base64,iVBORw0KGgo=';
class MemoryBackend implements ProjectBackend {
  rows = new Map<string, unknown>(); summaries = new Map<string, unknown>(); imported = false; tail: Promise<unknown> = Promise.resolve();
  list = async () => structuredClone([...this.summaries.values()]);
  read = async (id: string) => structuredClone(this.rows.get(id) ?? null);
  migrated = async () => this.imported;
  change(id: string, update: (current: unknown | null) => SimulationProject | null): Promise<SimulationProject | null> {
    const next = this.tail.catch(() => {}).then(() => {
      const value = update(structuredClone(this.rows.get(id) ?? null));
      if (value) { this.rows.set(id, structuredClone(value)); this.summaries.set(id, projectSummary(value)); } else { this.rows.delete(id); this.summaries.delete(id); }
      return value;
    });
    this.tail = next; return next;
  }
  async importLegacyOnce(project: SimulationProject | null) {
    const next = this.tail.catch(() => {}).then(() => { if (this.imported) return; if (project) { this.rows.set(project.id, structuredClone(project)); this.summaries.set(project.id, projectSummary(project)); } this.imported = true; });
    this.tail = next; await next;
  }
}
const emptyLegacy: SourceProjectRepository = { load: async () => null, save: async () => { throw new Error('must not overwrite legacy storage'); } };
function fixture() { const backend = new MemoryBackend(); return { backend, repository: new ProjectRepository(backend, emptyLegacy) }; }
function populated() {
  let program = createRobotProgram(ID);
  const path = createProgramPath(program, '磨削路径', 'path-1'); program = path.program;
  program = captureProgramPointToPath(program, path.value.id, { pose: { position: [12, 23, 34], orientation: [1, 0, 0, 0] }, joints: [0, 20, -30, 0, 40, 0], configuration: [0, 0, 0, 0], speedMmPerSecond: 50 }).program;
  let graph = addReferenceFrame(createFrameGraph(ID), { id: 'fixture', localPose: { position: [120, -80, 30], orientation: [Math.SQRT1_2, 0, 0, Math.SQRT1_2] } }).graph;
  graph = reparentFrameObject(graph, { kind: 'robot' }, { kind: 'frame', id: 'fixture' });
  graph = reparentFrameObject(graph, { kind: 'path', id: path.value.id }, { kind: 'frame', id: 'fixture' });
  graph = addToolFrame(graph, { id: 'tool', localPose: identityPose() }).graph;
  const project = createSimulationProject('复杂场景', ID);
  project.scene.models[ID] = { robotPresent: false, joints: [0, 20, -30, 0, 40, 0], graph, program };
  project.scene.camera = { position: [2, 3, 4], target: [0, 0, .5] };
  project.scene.zones = [{ id: 'zone', name: '禁入区', enabled: true, center: ['', '0', '0'], size: ['100', '200', '300'] }];
  return project;
}

describe('durable simulation projects', () => {
  it('roundtrips rotated frames, entire paths, removed robot, tools, camera and invalid zone drafts without changing poses', () => {
    const project = populated();
    expect(validateSimulationProject(JSON.parse(JSON.stringify(project)))).toEqual(project);
  });
  it('requires a name and a catalog robot, rejects corrupt trees and invalid source or thumbnail data', () => {
    for (const name of ['', '  ', 'a'.repeat(81), '\u0000']) expect(() => projectName(name)).toThrow();
    expect(projectName('  工件 A  ')).toBe('工件 A');
    expect(() => createSimulationProject('test', 'another-robot')).toThrow();
    const project = populated(), bad = structuredClone(project);
    (bad.scene.models[ID].graph.frames[0] as any).parent = { kind: 'frame', id: 'fixture' };
    expect(() => validateSimulationProject(bad)).toThrow(/循环/);
    for (const thumbnail of ['https://example.com/image.png', 'data:text/html;base64,PHNjcmlwdD4=', 'data:image/jpeg;base64,'.padEnd(1100000, 'A')]) {
      expect(() => validateSimulationProject({ ...project, thumbnail })).toThrow();
    }
    expect(() => validateSimulationProject({ ...project, sources: { [ID]: createSourceProject('yaskawa-gp7') } })).toThrow();
  });
  it('rejects sparse coordinates and bounded local poses whose composed world/TCP exceeds supported coordinates', async () => {
    const { repository, backend } = fixture();
    const project = populated();
    const cases: SimulationProject[] = [];
    const sparseJoints = structuredClone(project); sparseJoints.scene.models[ID].joints = new Array(6) as any; cases.push(sparseJoints);
    const sparseZone = structuredClone(project); sparseZone.scene.zones[0].center = new Array(3); cases.push(sparseZone);
    const hugeTool = structuredClone(project); hugeTool.scene.models[ID].graph.tools[0].localPose.position[0] = 1e308; hugeTool.scene.models[ID].graph.tools[0].tcpPose.position[0] = 1e308; cases.push(hugeTool);
    const composedTool = structuredClone(project); composedTool.scene.models[ID].graph.tools[0].localPose.position[0] = 600000; composedTool.scene.models[ID].graph.tools[0].tcpPose.position[0] = 600000; cases.push(composedTool);
    const composedPath = structuredClone(project); (composedPath.scene.models[ID].graph.paths as any)['path-1'] = { parent: { kind: 'world' }, localPose: { position: [1000000, 0, 0], orientation: [1, 0, 0, 0] } }; cases.push(composedPath);
    const hugeCamera = structuredClone(project); hugeCamera.scene.camera!.position[0] = 1e308; cases.push(hugeCamera);
    for (const malformed of cases) {
      backend.rows.set(project.id, malformed);
      await expect(repository.load(project.id)).rejects.toThrow();
      await expect(repository.saveScene(project.id, project.scene)).rejects.toThrow();
      expect(backend.rows.get(project.id)).toEqual(malformed);
    }
    const mounted = structuredClone(project.scene.models[ID]); mounted.robotPresent = true;
    (mounted.graph.tools[0] as any).parent = { kind: 'flange' };
    mounted.graph.tools[0].localPose.position = [999000, 0, 0];
    expect(() => validateProjectModelWorld(mounted, { position: [2000, 0, 0], orientation: [1, 0, 0, 0] })).toThrow();
    expect(() => validateProjectModelWorld(project.scene.models[ID], identityPose())).not.toThrow();
  });
  it('scopes source drafts by project and merges concurrent scene/cover saves with the latest source', async () => {
    const { repository } = fixture();
    const a = await repository.create('A', ID), b = await repository.create('B', ID);
    const sourceA = addSourceDocument(createSourceProject(ID), 'main.mod', 'MODULE A\nENDMODULE');
    const sourceB = addSourceDocument(createSourceProject(ID), 'main.mod', 'MODULE B\nENDMODULE');
    const scene = populated().scene;
    await Promise.all([repository.sourcesFor(a.id).save(sourceA), repository.saveScene(a.id, scene, cover), repository.sourcesFor(b.id).save(sourceB)]);
    expect((await repository.load(a.id)).scene).toEqual(scene);
    expect((await repository.sourcesFor(a.id).load(ID))?.files[0].text).toContain('MODULE A');
    expect((await repository.sourcesFor(b.id).load(ID))?.files[0].text).toContain('MODULE B');
    expect((await repository.load(a.id)).thumbnail).toBe(cover);
    expect((await repository.load(b.id)).thumbnail).toBeNull();
  });
  it('refreshes cover on each home save and preserves project identity, source and created time when renamed', async () => {
    const { repository } = fixture(), project = await repository.create('A', ID);
    const first = await repository.saveScene(project.id, project.scene, cover);
    const secondCover = 'data:image/jpeg;base64,/9j/AA==';
    expect(project.scene.zones).toHaveLength(1);
    const second = await repository.saveScene(project.id, { ...first.scene, view: 'top', zones: [] }, secondCover);
    const named = await repository.rename(project.id, 'B');
    expect(named.id).toBe(project.id); expect(named.createdAt).toBe(project.createdAt);
    expect(named.thumbnail).toBe(secondCover); expect(named.scene.view).toBe('top'); expect(named.scene.zones).toEqual([]);
    expect(second.revision).toBeGreaterThan(first.revision); expect(named.revision).toBeGreaterThan(second.revision);
  });
  it('lists only the small summary index and updates it with the saved scene, cover and name', async () => {
    const { repository, backend } = fixture(), project = await repository.create('A', ID);
    const saved = await repository.saveScene(project.id, populated().scene, cover);
    const renamed = await repository.rename(project.id, '打磨验证');
    backend.read = async () => { throw new Error('homepage must not read full scene or module sources'); };
    const summaries = await repository.list();
    expect(summaries).toEqual([projectSummary(renamed)]);
    expect(summaries[0].robotPresent).toBe(false);
    expect(summaries[0].thumbnail).toBe(saved.thumbnail);
    expect(summaries[0]).not.toHaveProperty('scene'); expect(summaries[0]).not.toHaveProperty('sources');
    // Even if a full row is damaged, the user can still see and explicitly remove its index entry.
    backend.rows.set(project.id, { damaged: true });
    expect(await repository.list()).toEqual(summaries);
    await repository.remove(project.id);
    expect(await repository.list()).toEqual([]); expect(backend.rows.has(project.id)).toBe(false);
  });
  it('saves scene before close without replacing the latest cover or concurrent source edits', async () => {
    const { repository } = fixture(), project = await repository.create('关闭前保存', ID);
    const first = await repository.saveScene(project.id, project.scene);
    expect(first.thumbnail).toBeNull();
    const source = addSourceDocument(createSourceProject(ID), 'main.mod', 'MODULE SavedBeforeClose\nENDMODULE');
    const scene = populated().scene;
    // A newer captured cover must survive an already-requested scene-only save as well.
    await Promise.all([
      repository.saveScene(project.id, project.scene, cover),
      repository.saveScene(project.id, scene),
      repository.sourcesFor(project.id).save(source),
    ]);
    const saved = await repository.load(project.id);
    expect(saved.scene).toEqual(scene); expect(saved.thumbnail).toBe(cover); expect(saved.sources[ID]).toEqual(source);
    expect(await repository.list()).toEqual([projectSummary(saved)]);
    await expect(repository.saveScene(project.id, scene, '')).rejects.toThrow(/封面图/);
    expect(await repository.load(project.id)).toEqual(saved);
  });
  it('never resurrects deleted projects through late source or cover saves', async () => {
    const { repository } = fixture(), project = await repository.create('A', ID), sources = repository.sourcesFor(project.id);
    await repository.remove(project.id);
    await expect(sources.save(createSourceProject(ID))).rejects.toThrow(/已删除/);
    await expect(repository.saveScene(project.id, project.scene, cover)).rejects.toThrow(/已删除/);
    expect(await repository.list()).toEqual([]);
  });
  it('rejects failed or corrupt loads/saves without replacing the durable row', async () => {
    const { repository, backend } = fixture(), project = await repository.create('A', ID);
    const corrupt = { ...project, scene: { ...project.scene, models: { [ID]: {} } } }; backend.rows.set(project.id, corrupt);
    await expect(repository.load(project.id)).rejects.toThrow();
    await expect(repository.saveScene(project.id, project.scene, cover)).rejects.toThrow();
    expect(backend.rows.get(project.id)).toEqual(corrupt);
    backend.rows.set(project.id, project);
    await expect(repository.saveScene(project.id, project.scene, 'invalid')).rejects.toThrow();
    expect(backend.rows.get(project.id)).toEqual(project);
  });
  it('imports legacy source once across concurrent initializers and never deletes or rewrites it', async () => {
    const backend = new MemoryBackend();
    const source = addSourceDocument(createSourceProject(ID), 'old.mod', 'MODULE Old\nENDMODULE');
    let writes = 0;
    const legacy: SourceProjectRepository = { load: async model => model === ID ? source : null, save: async () => { writes++; } };
    const a = new ProjectRepository(backend, legacy), b = new ProjectRepository(backend, legacy);
    await Promise.all([a.initialize(), b.initialize()]);
    const restored = await a.list(); expect(restored).toHaveLength(1); expect((await a.load(restored[0].id)).sources[ID]).toEqual(source); expect(writes).toBe(0);
    await a.remove(restored[0].id); await b.initialize(); expect(await b.list()).toEqual([]);
  });
  it('preserves migration status and rows after legacy read failure so retry can recover content', async () => {
    const backend = new MemoryBackend();
    const failing: SourceProjectRepository = { load: async () => { throw new Error('corrupt old draft'); }, save: async () => {} };
    await expect(new ProjectRepository(backend, failing).initialize()).rejects.toThrow('corrupt old draft');
    expect(backend.imported).toBe(false); expect(await backend.list()).toEqual([]);
    await new ProjectRepository(backend, emptyLegacy).initialize(); expect(backend.imported).toBe(true);
  });
});
