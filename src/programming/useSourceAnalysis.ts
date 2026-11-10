import { useEffect, useState } from 'react';
import type { SourceProject } from './documents';
import type { ProjectAnalysis } from './analysis-protocol';
const empty = (): ProjectAnalysis => ({ byFile: Object.create(null) });
export function useSourceAnalysis(project: SourceProject, enabled: boolean) {
  const key = `${project.modelId}:${project.dialectId}:${project.sourceRevision}`;
  const [state, setState] = useState({ key: '', result: empty(), pending: false, error: '' });
  useEffect(() => {
    if (!enabled) return;
    const controller = new AbortController();
    setState({ key, result: empty(), pending: true, error: '' });
    const timer = setTimeout(() => {
      void import('./analysis-client').then(module => module.analyzeSourceProject({ modelId: project.modelId, sourceRevision: project.sourceRevision, dialectId: project.dialectId, files: project.files }, controller.signal))
        .then(result => { if (!controller.signal.aborted) setState({ key, result, pending: false, error: '' }); }, cause => {
          if (!controller.signal.aborted) setState({ key, result: empty(), pending: false, error: cause instanceof Error ? cause.message : '源码检查失败。' });
        });
    }, 180);
    return () => { clearTimeout(timer); controller.abort(); };
  }, [key, enabled]);
  return state.key === key && enabled ? state : { key, result: empty(), pending: enabled, error: '' };
}
