import { afterEach, describe, expect, it, vi } from 'vitest';
import catalog from '../src/catalog/catalog.json';
import {
  addSourceDocument, createSourceProject, decodeSourceFile, encodeSourceFile, importSourceDocuments,
  markSourceFileSaved, parseSourceProject, removeSourceDocument, renameSourceDocument, selectSourceDocument,
  serializeSourceProject, setSourceDialect, SOURCE_LIMITS, updateSourceText, validateSourceProject,
  type SourceProject,
} from '../src/programming/documents';
import { getModelProgrammingProfile, getProgramDialect, PROGRAMMING_MODEL_IDS } from '../src/programming/profiles';
import { SourceDraftStore, type SourceProjectRepository } from '../src/programming/storage';

const abb = 'irb1100-4-0.58', fanuc = 'fanuc-crx10ia-l', yaskawa = 'yaskawa-gp7';
const source = '\ufeffMODULE Main\r\n! 未识别的指令也须原样保留\r\nVendorExtension \\Foo:=42;\r\nENDMODULE\r\n';
const saved = (modelId = abb, name = 'main.mod', text = source) => importSourceDocuments(createSourceProject(modelId), [{ name, text }], () => 'first');

describe('source files and versioned per-model projects', () => {
  it('uses dialect-specific RAPID defaults while preserving imported legacy and modern source files', () => {
    const project = createSourceProject(abb);
    expect(getProgramDialect('rapid-rw7').extensions).toEqual(['modx', 'sysx', 'mod', 'sys']);
    expect(getProgramDialect('rapid-rw6').extensions).toEqual(['mod', 'sys']);
    expect(addSourceDocument(project).files[0].name).toBe('main.modx');
    expect(addSourceDocument(setSourceDialect(project, 'rapid-rw6')).files[0].name).toBe('main.mod');
    let next = 0;
    const imported = importSourceDocuments(project, ['modx', 'sysx', 'mod', 'sys'].map(extension => ({ name: `original.${extension}`, text: source })), () => `file${next++}`);
    const rw6 = setSourceDialect(imported, 'rapid-rw6');
    expect(rw6.files).toBe(imported.files);
    expect(rw6.sourceRevision).toBe(imported.sourceRevision + 1);
    expect(parseSourceProject(serializeSourceProject(rw6), abb).files.map(file => [file.name, file.text, file.dirty])).toEqual(
      imported.files.map(file => [file.name, source, false]),
    );
  });
  it('matches the reviewed catalog and fixed dialect families without a fictitious KUKA model', () => {
    expect(PROGRAMMING_MODEL_IDS).toEqual(catalog.models.map(model => model.id));
    for (const model of catalog.models) {
      const profile = getModelProgrammingProfile(model.id);
      expect(profile.languageId).toBe(model.brand === 'ABB' ? 'rapid' : model.brand === 'FANUC' ? 'fanuc-ls' : 'inform');
    }
    expect(createSourceProject(abb).dialectId).toBe('rapid-rw7');
    expect(getProgramDialect('rapid-rw7').referenceVersion).toBe('7.21');
    expect(getProgramDialect('rapid-rw6').referenceVersion).toBe('6.16');
    expect(getProgramDialect('krl-kss8').extensions).toEqual(['src', 'dat']);
    expect(() => getModelProgrammingProfile('kuka-kr6')).toThrow('尚未接入');
  });

  it('imports and round-trips exact unknown syntax, UTF-8 BOM and CRLF without parsing code', () => {
    const bytes = new TextEncoder().encode(source), project = saved();
    expect(decodeSourceFile(bytes)).toBe(source);
    expect(encodeSourceFile(decodeSourceFile(bytes))).toEqual(bytes);
    expect(project.files[0].text).toBe(source);
    expect(project.files[0].dirty).toBe(false);
    expect(parseSourceProject(serializeSourceProject(project), abb)).toEqual(project);
    expect(project.files[0].savedSource?.text).toBe(source);
    expect(() => decodeSourceFile(Uint8Array.from([0xff, 0xfe, 0x41, 0]))).toThrow('UTF-8');
    expect(() => encodeSourceFile('\ud800')).toThrow('Unicode');
  });

  it('keeps source dirty separate from draft revisions, including a late disk save and exact undo', () => {
    const original = saved(), changed = updateSourceText(original, 'first', source + '! newest');
    expect(changed.files[0].dirty).toBe(true);
    expect(changed.sourceRevision).toBe(original.sourceRevision + 1);
    expect(changed.files[0].revision).toBe(1);
    const afterLateSave = markSourceFileSaved(changed, 'first', { name: 'main.mod', text: source });
    expect(afterLateSave.files[0].dirty).toBe(true);
    expect(afterLateSave.files[0].text).toBe(source + '! newest');
    const exported = markSourceFileSaved(changed, 'first', { name: 'main.mod', text: changed.files[0].text });
    expect(exported.files[0].dirty).toBe(false);
    expect(exported.sourceRevision).toBe(changed.sourceRevision);
    expect(exported.revision).toBe(changed.revision + 1);
    expect(updateSourceText(changed, 'first', source).files[0].dirty).toBe(false);
    expect(renameSourceDocument(original, 'first', 'renamed.mod').files[0].dirty).toBe(true);
    expect(addSourceDocument(createSourceProject(abb)).files[0].dirty).toBe(true);
  });

  it('switches dialect without rewriting source or falsely marking a source file modified', () => {
    const original = saved(), other = setSourceDialect(original, 'rapid-rw6');
    expect(other.sourceRevision).toBe(original.sourceRevision + 1);
    expect(other.files).toBe(original.files);
    expect(other.files[0].dirty).toBe(false);
    expect(other.files[0].text).toBe(source);
    expect(setSourceDialect(other, 'rapid-rw6')).toBe(other);
    expect(() => setSourceDialect(other, 'inform-yrc1000')).toThrow('品牌');
    expect(parseSourceProject(serializeSourceProject(other), abb).dialectId).toBe('rapid-rw6');
  });

  it('can mark a successful source export clean after other files were shrunk from larger disk versions', () => {
    const text = 'x'.repeat(SOURCE_LIMITS.fileBytes);
    let id = 0;
    let project = importSourceDocuments(createSourceProject(abb), Array.from({ length: 4 }, (_, index) => ({ name: `old${index}.mod`, text })), () => String(id++));
    for (const file of project.files) project = updateSourceText(project, file.id, '');
    project = addSourceDocument(project, 'new.mod', text, 'new');
    const exported = markSourceFileSaved(project, 'new', { name: 'new.mod', text });
    expect(exported.files.find(file => file.id === 'new')?.dirty).toBe(false);
    expect(exported.files.filter(file => file.id !== 'new').every(file => file.dirty)).toBe(true);
    expect(validateSourceProject(exported, abb).files).toHaveLength(5);
  });

  it('changes active tabs without invalidating source and keeps all file mutations immutable', () => {
    const original = saved(), two = addSourceDocument(original, 'tools.sys', 'SYSMODULE Tools', 'second');
    const selected = selectSourceDocument(two, 'first');
    expect(selected.sourceRevision).toBe(two.sourceRevision);
    expect(selected.revision).toBe(two.revision + 1);
    expect(selected.files).toBe(two.files);
    const removed = removeSourceDocument(selected, 'first');
    expect(removed.activeFileId).toBe('second');
    expect(removeSourceDocument(removed, 'second').activeFileId).toBeNull();
    expect(original.files).toHaveLength(1);
    expect(Object.isFrozen(two.files)).toBe(true);
    expect(Object.isFrozen(two.files[0])).toBe(true);
    expect(Object.isFrozen(two.files[0].savedSource)).toBe(true);
    expect(() => { two.files[0].text = 'mutated'; }).toThrow();
  });

  it.each(['../x.mod', 'folder/x.mod', 'folder\\x.mod', 'CON.mod', 'Lpt1.sys', 'COM¹.mod', 'x.mod ', 'x.ls', 'main.mod:evil', '中'.repeat(84) + '.mod', '\ud800.mod'])('rejects unsafe or foreign-language file name %s', name => {
    expect(() => addSourceDocument(createSourceProject(abb), name)).toThrow();
  });

  it('imports multiple files atomically and rejects cross-platform duplicate names', () => {
    const original = saved();
    let number = 0;
    expect(() => importSourceDocuments(original, [{ name: 'tools.sys', text: '!' }, { name: 'MAIN.MOD', text: 'different' }], () => 'id-' + number++)).toThrow('重复');
    expect(original.files.map(file => file.name)).toEqual(['main.mod']);
    const composed = addSourceDocument(createSourceProject(abb), 'é.mod', '', 'accent');
    expect(() => addSourceDocument(composed, 'e\u0301.mod')).toThrow('重复');
    expect(saved(fanuc, 'MAIN.LS', '/PROG UNKNOWN').files[0].text).toBe('/PROG UNKNOWN');
    expect(saved(yaskawa, 'MAIN.JBI', '/JOB\nUnknownVendorOpcode').files[0].text).toContain('UnknownVendorOpcode');
  });

  it('enforces UTF-8 byte and project/file-count bounds before changing current data', () => {
    expect(() => addSourceDocument(createSourceProject(abb), 'large.mod', '中'.repeat(Math.floor(SOURCE_LIMITS.fileBytes / 3) + 1))).toThrow('1 MiB');
    let project = createSourceProject(abb);
    for (let i = 0; i < 4; i++) project = addSourceDocument(project, `part${i}.mod`, 'x'.repeat(SOURCE_LIMITS.fileBytes), String(i));
    expect(() => addSourceDocument(project, 'extra.mod', 'x')).toThrow('4 MiB');
    expect(project.files).toHaveLength(4);
    let many = createSourceProject(abb);
    for (let i = 0; i < 32; i++) many = addSourceDocument(many, `part${i}.mod`, '', String(i));
    expect(() => addSourceDocument(many, 'overflow.mod')).toThrow('32');
  });

  it('rejects corrupt, newer or foreign model archives and recomputes untrusted dirty flags', () => {
    const original = saved(), payload = JSON.parse(serializeSourceProject(original));
    payload.files[0].text += '! edited'; payload.files[0].dirty = false;
    expect(validateSourceProject(payload, abb).files[0].dirty).toBe(true);
    expect(() => validateSourceProject({ ...payload, schemaVersion: 2 }, abb)).toThrow('版本不支持');
    expect(() => validateSourceProject(payload, fanuc)).toThrow('其他机器人');
    expect(() => validateSourceProject({ ...payload, dialectId: 'inform-yrc1000' }, abb)).toThrow('不匹配');
    expect(() => validateSourceProject({ ...payload, activeFileId: 'deleted' }, abb)).toThrow('标识');
    expect(() => parseSourceProject('{bad json', abb)).toThrow('JSON');
  });
});

class MemoryRepository implements SourceProjectRepository {
  data = new Map<string, SourceProject>();
  writes: SourceProject[] = [];
  fail = false;
  async load(modelId: string) { return this.data.get(modelId) ?? null; }
  async save(project: SourceProject) {
    if (this.fail) throw new DOMException('full', 'QuotaExceededError');
    this.writes.push(project); this.data.set(project.modelId, project);
  }
}
function deferred<T>() {
  let resolve!: (value: T) => void;
  const promise = new Promise<T>(done => { resolve = done; });
  return { promise, resolve };
}
afterEach(() => { vi.clearAllTimers(); vi.useRealTimers(); });

describe('autosaved draft state and asynchronous isolation', () => {
  it('debounces autosave while retaining independent source-file dirty state across reopening', async () => {
    vi.useFakeTimers();
    const repository = new MemoryRepository(), store = new SourceDraftStore(repository, 250);
    await store.load(abb);
    store.mutate(abb, project => addSourceDocument(project, 'main.mod', 'one', 'file'));
    store.mutate(abb, project => updateSourceText(project, 'file', 'two'));
    expect(store.getSnapshot(abb).draftStatus).toBe('pending');
    expect(repository.writes).toHaveLength(0);
    await vi.advanceTimersByTimeAsync(250);
    expect(repository.writes).toHaveLength(1);
    expect(store.getSnapshot(abb).draftStatus).toBe('saved');
    expect(store.getSnapshot(abb).project.files[0].dirty).toBe(true);
    const reopened = new SourceDraftStore(repository);
    await reopened.load(abb);
    expect(reopened.getSnapshot(abb).project.files[0].text).toBe('two');
    expect(reopened.getSnapshot(abb).project.files[0].dirty).toBe(true);
  });

  it('serializes old/new saves and never overwrites newer in-memory editing on save completion', async () => {
    vi.useFakeTimers();
    const repository = new MemoryRepository(), first = deferred<void>();
    const save = repository.save.bind(repository);
    let calls = 0;
    repository.save = async project => { if (++calls === 1) await first.promise; await save(project); };
    const store = new SourceDraftStore(repository, 250);
    await store.load(abb);
    store.mutate(abb, project => addSourceDocument(project, 'main.mod', 'old', 'file'));
    const oldSave = store.flush(abb);
    await Promise.resolve();
    store.mutate(abb, project => updateSourceText(project, 'file', 'new'));
    const newSave = store.flush(abb);
    expect(calls).toBe(1);
    first.resolve();
    await Promise.all([oldSave, newSave]);
    expect(repository.writes.map(project => project.files[0].text)).toEqual(['old', 'new']);
    expect(store.getSnapshot(abb).project.files[0].text).toBe('new');
    expect(repository.data.get(abb)?.files[0].text).toBe('new');
    expect(store.getSnapshot(abb).draftStatus).toBe('saved');
  });

  it('keeps delayed model loads isolated and ignores an old load after explicit project import', async () => {
    vi.useFakeTimers();
    const repository = new MemoryRepository(), held = deferred<SourceProject | null>();
    repository.load = async modelId => modelId === abb ? held.promise : saved(fanuc, 'MAIN.LS', '/PROG FANUC');
    const store = new SourceDraftStore(repository);
    const oldLoad = store.load(abb);
    await store.load(fanuc);
    expect(store.getSnapshot(fanuc).project.files[0].text).toBe('/PROG FANUC');
    expect(store.replace(abb, saved(abb, 'new.mod', 'new imported text'))).toBe(true);
    held.resolve(saved(abb, 'old.mod', 'old disk draft'));
    await oldLoad;
    expect(store.getSnapshot(abb).project.files[0].text).toBe('new imported text');
    expect(store.getSnapshot(fanuc).project.files[0].text).toBe('/PROG FANUC');
    await store.flushAll();
    expect(repository.data.get(abb)?.files[0].text).toBe('new imported text');
  });

  it('reports quota errors, preserves edits and dirty flags, and allows a real retry', async () => {
    vi.useFakeTimers();
    const repository = new MemoryRepository(), store = new SourceDraftStore(repository);
    await store.load(yaskawa);
    store.mutate(yaskawa, project => addSourceDocument(project, 'MAIN.JBI', '/JOB\r\n', 'file'));
    repository.fail = true;
    expect(await store.flush(yaskawa)).toBe(false);
    expect(store.getSnapshot(yaskawa).error).toContain('空间不足');
    store.reportError(yaskawa, new Error('A later file-name validation error'));
    expect(store.getSnapshot(yaskawa).error).toContain('空间不足');
    expect(store.getSnapshot(yaskawa).project.files[0].text).toBe('/JOB\r\n');
    expect(store.hasPendingDrafts()).toBe(true);
    repository.fail = false;
    expect(await store.flush(yaskawa)).toBe(true);
    expect(store.getSnapshot(yaskawa).draftStatus).toBe('saved');
    expect(store.getSnapshot(yaskawa).project.files[0].dirty).toBe(true);
    expect(store.hasPendingDrafts()).toBe(false);
  });

  it('rejects a project replacement that would overflow its monotonic revision without touching text', async () => {
    vi.useFakeTimers();
    const repository = new MemoryRepository();
    repository.data.set(abb, validateSourceProject({ ...saved(), revision: Number.MAX_SAFE_INTEGER, sourceRevision: Number.MAX_SAFE_INTEGER }, abb));
    const store = new SourceDraftStore(repository);
    await store.load(abb);
    const original = store.getSnapshot(abb).project;
    expect(store.replace(abb, saved(abb, 'new.mod', 'new text'))).toBe(false);
    expect(store.getSnapshot(abb).project).toBe(original);
    expect(store.getSnapshot(abb).error).toContain('版本编号');
    expect(repository.writes).toHaveLength(0);
  });

  it('does not replace a damaged or future-version stored draft with an empty project', async () => {
    vi.useFakeTimers();
    const repository = new MemoryRepository();
    repository.load = async () => ({ ...saved(), schemaVersion: 99 } as unknown as SourceProject);
    const store = new SourceDraftStore(repository);
    expect(await store.load(abb)).toBe(false);
    expect(store.getSnapshot(abb).ready).toBe(false);
    expect(store.getSnapshot(abb).error).toContain('版本不支持');
    expect(store.mutate(abb, project => addSourceDocument(project))).toBeNull();
    expect(await store.flush(abb)).toBe(false);
    await vi.advanceTimersByTimeAsync(1000);
    expect(repository.writes).toHaveLength(0);
  });
});
