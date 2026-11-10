import { IndexedDbSourceRepository, type SourceProjectRepository } from '../programming/storage';
import { validateSourceProject, type SourceProject } from '../programming/documents';
import { createSimulationProject, PROJECT_LIMITS, PROJECT_MODELS, projectName, projectSummary, validateProjectSummary, validateProjectScene, validateSimulationProject, validateThumbnail, type ProjectScene, type SimulationProject, type ProjectSummary } from './schema';

export interface ProjectBackend {
  list(): Promise<unknown[]>;
  read(id: string): Promise<unknown | null>;
  /** Update runs against the latest row in one serialized read/write transaction. */
  change(id: string, update: (current: unknown | null) => SimulationProject | null): Promise<SimulationProject | null>;
  migrated(): Promise<boolean>;
  importLegacyOnce(project: SimulationProject | null): Promise<void>;
}
export class IndexedDbProjectBackend implements ProjectBackend {
  private connection: Promise<IDBDatabase> | null = null;
  private open(): Promise<IDBDatabase> {
    if (!this.connection) {
      this.connection = new Promise<IDBDatabase>((resolve, reject) => {
        const request = indexedDB.open('emulation-simulation-projects', 2);
        request.onupgradeneeded = () => {
          const db = request.result;
          if (!db.objectStoreNames.contains('projects')) db.createObjectStore('projects', { keyPath: 'id' });
          if (!db.objectStoreNames.contains('meta')) db.createObjectStore('meta');
          if (!db.objectStoreNames.contains('summaries')) {
            const summaries = db.createObjectStore('summaries', { keyPath: 'id' });
            const cursor = request.transaction!.objectStore('projects').openCursor();
            cursor.onsuccess = () => { if (!cursor.result) return; try { summaries.put(projectSummary(validateSimulationProject(cursor.result.value))); cursor.result.continue(); } catch { request.transaction!.abort(); } };
          }
        };
        let failed = false;
        const fail = () => { failed = true; reject(new Error('无法打开项目库，请保留当前窗口并重试。')); };
        request.onerror = fail; request.onblocked = fail;
        request.onsuccess = () => {
          if (failed) { request.result.close(); return; }
          request.result.onversionchange = () => { request.result.close(); this.connection = null; };
          resolve(request.result);
        };
      }).catch(error => { this.connection = null; throw error; });
    }
    return this.connection;
  }
  private async readStore(storeName: string, id?: string): Promise<any> {
    const db = await this.open();
    return new Promise((resolve, reject) => {
      const transaction = db.transaction(storeName, 'readonly'), store = transaction.objectStore(storeName);
      const request = id === undefined ? store.getAll(undefined, PROJECT_LIMITS.projects + 1) : store.get(id);
      let value: unknown;
      request.onsuccess = () => { value = request.result; };
      transaction.oncomplete = () => resolve(value ?? null);
      transaction.onabort = () => reject(transaction.error ?? new Error('项目读取未完成。'));
      transaction.onerror = () => {};
    });
  }
  list(): Promise<unknown[]> { return this.readStore('summaries'); }
  read(id: string): Promise<unknown | null> { return this.readStore('projects', id); }
  async migrated(): Promise<boolean> { return await this.readStore('meta', 'legacy-imported') === true; }
  async change(id: string, update: (current: unknown | null) => SimulationProject | null): Promise<SimulationProject | null> {
    const db = await this.open();
    return new Promise((resolve, reject) => {
      const tx = db.transaction(['projects', 'summaries'], 'readwrite'), store = tx.objectStore('projects'), summaries = tx.objectStore('summaries');
      const request = store.get(id); let result: SimulationProject | null = null, error: unknown;
      request.onsuccess = () => {
        try {
          result = update(request.result ?? null);
          if (!result) { store.delete(id); summaries.delete(id); return; }
          if (request.result === undefined) {
            const count = store.count();
            count.onsuccess = () => {
              if (count.result >= PROJECT_LIMITS.projects) { error = new Error('项目数量已达128个，请先删除不需要的项目。'); tx.abort(); }
              else { store.put(result!); summaries.put(projectSummary(result!)); }
            };
          } else { store.put(result); summaries.put(projectSummary(result)); }
        } catch (cause) { error = cause; tx.abort(); }
      };
      tx.oncomplete = () => resolve(result);
      tx.onabort = () => reject(error ?? tx.error ?? new Error('项目保存事务未完成；当前编辑内容仍保留。'));
      tx.onerror = () => {};
    });
  }
  async importLegacyOnce(project: SimulationProject | null): Promise<void> {
    const db = await this.open();
    return new Promise((resolve, reject) => {
      const tx = db.transaction(['projects', 'summaries', 'meta'], 'readwrite'), meta = tx.objectStore('meta');
      const check = meta.get('legacy-imported');
      check.onsuccess = () => {
        if (check.result === true) return;
        if (project) {
          const projects = tx.objectStore('projects'), count = projects.count();
          count.onsuccess = () => { if (count.result >= PROJECT_LIMITS.projects) tx.abort(); else { projects.add(project); tx.objectStore('summaries').add(projectSummary(project)); meta.put(true, 'legacy-imported'); } };
        } else meta.put(true, 'legacy-imported');
      };
      tx.oncomplete = () => resolve();
      tx.onabort = () => reject(tx.error ?? new Error('旧草稿迁移未完成，原草稿未修改。'));
      tx.onerror = () => {};
    });
  }
}

/** Scene/cover and source saves merge against current durable rows, never stale UI copies. */
export class ProjectRepository {
  constructor(private backend: ProjectBackend = new IndexedDbProjectBackend(), private legacy: SourceProjectRepository = new IndexedDbSourceRepository()) {}
  async initialize(): Promise<void> {
    if (await this.backend.migrated()) return;
    const entries = await Promise.all(PROJECT_MODELS.map(async model => [model.id, await this.legacy.load(model.id)] as const));
    const sources = Object.fromEntries(entries.filter((entry): entry is readonly [string, SourceProject] => Boolean(entry[1]?.files.length)));
    const ids = Object.keys(sources);
    const recovered = ids.length ? validateSimulationProject({ ...createSimulationProject('从旧版本恢复的模块', ids[0], 'legacy-workspace'), sources }) : null;
    await this.backend.importLegacyOnce(recovered);
  }
  async list(): Promise<ProjectSummary[]> {
    const rows = await this.backend.list();
    if (rows.length > PROJECT_LIMITS.projects) throw new Error('项目索引数量超出上限，原数据未修改。');
    const projects = rows.map(validateProjectSummary);
    return projects.sort((a, b) => b.updatedAt.localeCompare(a.updatedAt) || a.id.localeCompare(b.id));
  }
  async load(id: string): Promise<SimulationProject> {
    const raw = await this.backend.read(id);
    if (!raw) throw new Error('项目已删除或不存在，请返回主页刷新列表。');
    const project = validateSimulationProject(raw);
    if (project.id !== id) throw new Error('项目标识不匹配，未加载数据。');
    return project;
  }
  async create(name: string, modelId: string): Promise<SimulationProject> {
    const project = createSimulationProject(name, modelId);
    await this.backend.change(project.id, current => { if (current) throw new Error('项目标识已存在。'); return project; });
    return project;
  }
  private async update(id: string, change: (current: SimulationProject) => SimulationProject): Promise<SimulationProject> {
    return (await this.backend.change(id, raw => {
      if (!raw) throw new Error('项目已删除，未重新创建或覆盖其他项目。');
      const current = validateSimulationProject(raw);
      if (current.id !== id) throw new Error('项目标识不匹配。');
      return validateSimulationProject({ ...change(current), id, createdAt: current.createdAt, revision: current.revision + 1, updatedAt: new Date().toISOString() });
    }))!;
  }
  async rename(id: string, name: string): Promise<SimulationProject> { const valid = projectName(name); return this.update(id, current => ({ ...current, name: valid })); }
  async remove(id: string): Promise<void> { await this.backend.change(id, () => null); }
  /** Omitted thumbnail preserves the latest cover, e.g. while saving before native close. */
  async saveScene(id: string, scene: ProjectScene, thumbnail?: string): Promise<SimulationProject> {
    const valid = validateProjectScene(scene);
    if (thumbnail !== undefined) validateThumbnail(thumbnail);
    return this.update(id, current => ({ ...current, scene: valid, thumbnail: thumbnail === undefined ? current.thumbnail : thumbnail }));
  }
  sourcesFor(id: string): SourceProjectRepository {
    return {
      load: async modelId => (await this.load(id)).sources[modelId] ?? null,
      save: async source => {
        const valid = validateSourceProject(source, source.modelId);
        await this.update(id, current => ({ ...current, sources: { ...current.sources, [source.modelId]: valid } }));
      },
    };
  }
}
