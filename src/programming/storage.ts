import { createSourceProject, validateSourceProject, type SourceProject } from './documents';

export interface SourceProjectRepository {
  load(modelId: string): Promise<SourceProject | null>;
  save(project: SourceProject): Promise<void>;
}
function storageMessage(cause: unknown): string {
  if (cause instanceof DOMException && cause.name === 'QuotaExceededError') return '本地草稿存储空间不足；正文仍在内存，请导出源文件。';
  return cause instanceof Error ? cause.message : '本地草稿保存失败；正文仍在内存，请重试或导出源文件。';
}

/** Tauri's persistent WebView data and browser reference mode use separate local profiles. */
export class IndexedDbSourceRepository implements SourceProjectRepository {
  private connection: Promise<IDBDatabase> | null = null;
  private open(): Promise<IDBDatabase> {
    if (this.connection) return this.connection;
    this.connection = new Promise((resolve, reject) => {
      if (typeof indexedDB === 'undefined') { reject(new Error('本地草稿存储不可用，请导出源文件以保留修改。')); return; }
      const request = indexedDB.open('emulation-source-projects', 1);
      let settled = false;
      const fail = (error: Error) => { if (!settled) { settled = true; this.connection = null; reject(error); } };
      request.onupgradeneeded = () => { request.result.createObjectStore('projects', { keyPath: 'modelId' }); };
      request.onsuccess = () => {
        if (settled) { request.result.close(); return; }
        settled = true;
        request.result.onversionchange = () => { request.result.close(); this.connection = null; };
        resolve(request.result);
      };
      request.onerror = () => fail(new Error('无法打开本地草稿库；请保留窗口并导出源文件。'));
      request.onblocked = () => fail(new Error('草稿库升级被其他窗口阻塞，请关闭其他应用窗口后重试。'));
    });
    // Synchronous unavailability also clears the rejected connection for retries.
    this.connection = this.connection.catch(cause => { this.connection = null; throw cause; });
    return this.connection;
  }
  async load(modelId: string): Promise<SourceProject | null> {
    const db = await this.open();
    return new Promise((resolve, reject) => {
      const transaction = db.transaction('projects', 'readonly');
      const request = transaction.objectStore('projects').get(modelId);
      let value: unknown;
      request.onsuccess = () => { value = request.result; };
      transaction.oncomplete = () => {
        try { resolve(value === undefined ? null : validateSourceProject(value, modelId)); } catch (cause) { reject(cause); }
      };
      transaction.onabort = () => reject(transaction.error ?? new Error('本地草稿读取失败。'));
      transaction.onerror = () => { /* The abort handler reports the transaction error. */ };
    });
  }
  async save(project: SourceProject): Promise<void> {
    const validated = validateSourceProject(project, project.modelId), db = await this.open();
    return new Promise((resolve, reject) => {
      const transaction = db.transaction('projects', 'readwrite');
      transaction.objectStore('projects').put(validated);
      transaction.oncomplete = () => resolve();
      transaction.onabort = () => reject(transaction.error ?? new Error('本地草稿保存事务未完成。'));
      transaction.onerror = () => { /* A successful put is insufficient; wait for atomic commit. */ };
    });
  }
}

export type DraftStatus = 'loading' | 'saved' | 'pending' | 'saving' | 'error';
export interface DocumentSession {
  readonly project: SourceProject;
  readonly loading: boolean;
  readonly ready: boolean;
  readonly draftStatus: DraftStatus;
  readonly error: string | null;
}
interface Entry {
  snapshot: DocumentSession;
  operationError: string | null;
  storageError: string | null;
  epoch: number;
  loadPromise: Promise<boolean> | null;
  saveTail: Promise<boolean>;
  persistedRevision: number;
  timer: ReturnType<typeof setTimeout> | null;
}

/** One serialized save queue per model; old loads and saves never replace current text. */
export class SourceDraftStore {
  private entries = new Map<string, Entry>();
  private listeners = new Set<() => void>();
  constructor(private repository: SourceProjectRepository = new IndexedDbSourceRepository(), private debounceMs = 250) {}
  private entry(modelId: string): Entry {
    let entry = this.entries.get(modelId);
    if (!entry) {
      entry = { snapshot: Object.freeze({ project: createSourceProject(modelId), loading: true, ready: false, draftStatus: 'loading', error: null }),
        operationError: null, storageError: null, epoch: 0, loadPromise: null, saveTail: Promise.resolve(true), persistedRevision: -1, timer: null };
      this.entries.set(modelId, entry);
    }
    return entry;
  }
  private publish(entry: Entry, patch: Partial<DocumentSession>) {
    entry.snapshot = Object.freeze({ ...entry.snapshot, ...patch, error: entry.storageError ?? entry.operationError });
    for (const listener of this.listeners) listener();
  }
  readonly subscribe = (listener: () => void): (() => void) => { this.listeners.add(listener); return () => this.listeners.delete(listener); };
  getSnapshot(modelId: string): DocumentSession { return this.entry(modelId).snapshot; }
  async load(modelId: string): Promise<boolean> {
    const entry = this.entry(modelId);
    if (entry.snapshot.ready) return true;
    if (entry.loadPromise) return entry.loadPromise;
    const epoch = ++entry.epoch;
    entry.storageError = null;
    this.publish(entry, { loading: true, draftStatus: 'loading' });
    const promise = this.repository.load(modelId).then(raw => {
      if (entry.epoch !== epoch) return false;
      const project = raw ? validateSourceProject(raw, modelId) : entry.snapshot.project;
      entry.persistedRevision = project.revision;
      entry.storageError = null;
      this.publish(entry, { project, loading: false, ready: true, draftStatus: 'saved' });
      return true;
    }).catch(cause => {
      if (entry.epoch !== epoch) return false;
      entry.storageError = storageMessage(cause);
      this.publish(entry, { loading: false, ready: false, draftStatus: 'error' });
      return false;
    }).finally(() => { if (entry.loadPromise === promise) entry.loadPromise = null; });
    entry.loadPromise = promise;
    return promise;
  }
  reportError(modelId: string, cause: unknown): void {
    const entry = this.entry(modelId);
    entry.operationError = storageMessage(cause);
    this.publish(entry, {});
  }
  private schedule(modelId: string, entry: Entry) {
    if (entry.timer !== null) clearTimeout(entry.timer);
    entry.timer = setTimeout(() => { entry.timer = null; void this.flush(modelId); }, this.debounceMs);
  }
  mutate(modelId: string, change: (project: SourceProject) => SourceProject): SourceProject | null {
    const entry = this.entry(modelId);
    try {
      if (!entry.snapshot.ready) throw new Error('草稿尚未恢复；请重试加载或明确导入新的工程。');
      const next = change(entry.snapshot.project);
      entry.operationError = null;
      if (next === entry.snapshot.project) { if (entry.snapshot.error) this.publish(entry, {}); return next; }
      this.publish(entry, { project: next, draftStatus: 'pending' });
      this.schedule(modelId, entry);
      return next;
    } catch (cause) { this.reportError(modelId, cause); return null; }
  }
  /** Explicit project import/replacement invalidates an in-flight load. */
  replace(modelId: string, imported: SourceProject): boolean {
    const entry = this.entry(modelId);
    try {
      const validated = validateSourceProject(imported, modelId);
      const project = validateSourceProject({ ...validated, revision: entry.snapshot.project.revision + 1, sourceRevision: entry.snapshot.project.sourceRevision + 1 }, modelId);
      ++entry.epoch; entry.loadPromise = null; entry.operationError = null; entry.storageError = null;
      this.publish(entry, { project, loading: false, ready: true, draftStatus: 'pending' });
      this.schedule(modelId, entry);
      return true;
    } catch (cause) { this.reportError(modelId, cause); return false; }
  }
  async flush(modelId: string): Promise<boolean> {
    const entry = this.entry(modelId);
    if (entry.timer !== null) { clearTimeout(entry.timer); entry.timer = null; }
    if (!entry.snapshot.ready) return false;
    const project = entry.snapshot.project;
    if (entry.persistedRevision === project.revision && !entry.storageError) return true;
    this.publish(entry, { draftStatus: 'saving' });
    const job = entry.saveTail.then(async () => {
      try {
        await this.repository.save(project);
        entry.persistedRevision = project.revision;
        entry.storageError = null;
        this.publish(entry, { draftStatus: entry.snapshot.project.revision === project.revision ? 'saved' : 'pending' });
        return true;
      } catch (cause) {
        entry.storageError = storageMessage(cause);
        this.publish(entry, { draftStatus: 'error' });
        return false;
      }
    });
    entry.saveTail = job;
    return job;
  }
  async flushAll(): Promise<boolean> {
    const results = await Promise.all([...this.entries.keys()].map(modelId => this.flush(modelId)));
    return results.every(Boolean);
  }
  hasPendingDrafts(): boolean {
    return [...this.entries.values()].some(entry => entry.snapshot.ready && entry.persistedRevision !== entry.snapshot.project.revision);
  }
}
