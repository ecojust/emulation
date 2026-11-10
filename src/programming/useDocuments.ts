import { useEffect, useState, useSyncExternalStore } from 'react';
import {
  addSourceDocument, importSourceDocuments, markSourceFileSaved, parseSourceProject, removeSourceDocument,
  renameSourceDocument, selectSourceDocument, serializeSourceProject, setSourceDialect, updateSourceText,
  type SourceFileInput,
} from './documents';
import { getModelProgrammingProfile } from './profiles';
import { SourceDraftStore, type SourceProjectRepository } from './storage';

export function useDocuments(modelId: string, options: { repository?: SourceProjectRepository; debounceMs?: number } = {}) {
  getModelProgrammingProfile(modelId);
  const [store] = useState(() => new SourceDraftStore(options.repository, options.debounceMs));
  const session = useSyncExternalStore(store.subscribe, () => store.getSnapshot(modelId), () => store.getSnapshot(modelId));
  useEffect(() => { void store.load(modelId); }, [store, modelId]);
  useEffect(() => {
    const flush = () => { void store.flushAll(); };
    const guardClose = (event: BeforeUnloadEvent) => {
      if (!store.hasPendingDrafts()) return;
      flush(); event.preventDefault(); event.returnValue = '';
    };
    const visibility = () => { if (document.visibilityState === 'hidden') flush(); };
    window.addEventListener('pagehide', flush);
    window.addEventListener('beforeunload', guardClose);
    document.addEventListener('visibilitychange', visibility);
    return () => {
      window.removeEventListener('pagehide', flush); window.removeEventListener('beforeunload', guardClose);
      document.removeEventListener('visibilitychange', visibility); flush();
    };
  }, [store]);
  const mutate = (change: Parameters<SourceDraftStore['mutate']>[1]) => store.mutate(modelId, change);
  return {
    ...session, files: session.project.files, activeFile: session.project.files.find(file => file.id === session.project.activeFileId) ?? null,
    dirty: session.project.files.some(file => file.dirty), saving: session.draftStatus === 'saving',
    selectFile(id: string): boolean { return Boolean(mutate(project => selectSourceDocument(project, id))); },
    updateText(id: string, text: string): boolean { return Boolean(mutate(project => updateSourceText(project, id, text))); },
    addFile(name?: string, text = '') {
      const next = mutate(project => addSourceDocument(project, name, text));
      return next?.files.find(file => file.id === next.activeFileId) ?? null;
    },
    renameFile(id: string, name: string): boolean { return Boolean(mutate(project => renameSourceDocument(project, id, name))); },
    removeFile(id: string): boolean { return Boolean(mutate(project => removeSourceDocument(project, id))); },
    importFiles(files: readonly SourceFileInput[]): boolean { return Boolean(mutate(project => importSourceDocuments(project, files))); },
    setDialect(id: string): boolean { return Boolean(mutate(project => setSourceDialect(project, id))); },
    markFileSaved(id: string, exported: SourceFileInput): boolean { return Boolean(mutate(project => markSourceFileSaved(project, id, exported))); },
    importProject(text: string): boolean {
      try { return store.replace(modelId, parseSourceProject(text, modelId)); }
      catch (cause) { store.reportError(modelId, cause); return false; }
    },
    exportProject(): string { return serializeSourceProject(store.getSnapshot(modelId).project); },
    save(): Promise<boolean> { return store.flush(modelId); },
    flushAll(): Promise<boolean> { return store.flushAll(); },
    hasPendingDrafts(): boolean { return store.hasPendingDrafts(); },
    retry(): Promise<boolean> { return session.ready ? store.flush(modelId) : store.load(modelId); },
    reportError(cause: unknown): void { store.reportError(modelId, cause); },
  };
}

export type DocumentsController = ReturnType<typeof useDocuments>;
