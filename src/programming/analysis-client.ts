import SourceWorker from './analysis.worker?worker&inline';
import { snapshotAnalysisRequest, type AnalysisMessage, type AnalysisReply, type ProjectAnalysis, type SourceAnalysisRequest } from './analysis-protocol';
import type { LanguageAnalysis } from './types';
let idle: Worker | null = null;
const object = (value: unknown): value is Record<string, unknown> => typeof value === 'object' && value !== null && !Array.isArray(value);
const symbolKinds = new Set(['instruction', 'function', 'type', 'keyword', 'variable', 'routine', 'constant', 'snippet']);
const boundedText = (value: unknown, maximum = 16_384): value is string => typeof value === 'string' && value.length <= maximum;
/** Reject malformed or stale source ranges instead of decorating a different edit. */
export function validateProjectAnalysis(value: unknown, request: SourceAnalysisRequest): ProjectAnalysis {
  if (!object(value) || !object(value.byFile)) throw new Error('语法分析没有返回有效结果。');
  const byFile: Record<string, LanguageAnalysis> = Object.create(null);
  if (Object.keys(value.byFile).length !== request.files.length) throw new Error('语法分析缺少源文件结果。');
  for (const file of request.files) {
    const analysis = Object.hasOwn(value.byFile, file.id) ? value.byFile[file.id] : null;
    if (!object(analysis) || !Array.isArray(analysis.diagnostics) || !Array.isArray(analysis.symbols) || !Array.isArray(analysis.folds)
      || analysis.diagnostics.length > 2000 || analysis.symbols.length > 100_000 || analysis.folds.length > 100_000) throw new Error('语法分析结果超出有效范围。');
    const range = (item: unknown) => object(item) && Number.isSafeInteger(item.from) && Number.isSafeInteger(item.to)
      && (item.from as number) >= 0 && (item.to as number) >= (item.from as number) && (item.to as number) <= file.text.length;
    if (analysis.diagnostics.some(item => !range(item) || !object(item) || typeof item.severity !== 'string' || !['error', 'warning', 'info'].includes(item.severity) || !boundedText(item.message)
        || item.code !== undefined && !boundedText(item.code, 256))
      || analysis.symbols.some(item => !range(item) || !object(item) || !boundedText(item.name, 1024) || !item.name || typeof item.kind !== 'string' || !symbolKinds.has(item.kind)
        || item.signature !== undefined && !boundedText(item.signature) || item.description !== undefined && !boundedText(item.description))
      || analysis.folds.some(item => !range(item) || !object(item) || item.from === item.to)) throw new Error('语法分析包含过时或无效的源码位置。');
    // Results are detached from mutable event payloads, just like the request snapshot.
    const valid = analysis as unknown as LanguageAnalysis;
    byFile[file.id] = { diagnostics: valid.diagnostics.map(item => ({ ...item })), symbols: valid.symbols.map(item => ({ ...item })), folds: valid.folds.map(item => ({ ...item })) };
  }
  return { byFile };
}
export function analyzeSourceProject(request: SourceAnalysisRequest, signal?: AbortSignal): Promise<ProjectAnalysis> {
  return new Promise((resolve, reject) => {
    if (signal?.aborted) { reject(new DOMException('语法分析已取消', 'AbortError')); return; }
    let snapshot: SourceAnalysisRequest;
    try { snapshot = snapshotAnalysisRequest(request); }
    catch (error) { reject(error); return; }
    let worker: Worker;
    try { worker = idle ?? new SourceWorker(); idle = null; }
    catch { reject(new Error('无法启动本地语法分析线程。')); return; }
    const requestId = crypto.randomUUID(); let settled = false;
    const finish = (result?: ProjectAnalysis, error?: unknown) => {
      if (settled) return; settled = true; clearTimeout(timer); signal?.removeEventListener('abort', abort);
      worker.onmessage = null; worker.onerror = null; worker.onmessageerror = null;
      if (error) { worker.terminate(); reject(error); }
      else { if (idle) idle.terminate(); idle = worker; resolve(result!); }
    };
    const abort = () => finish(undefined, new DOMException('语法分析已取消', 'AbortError'));
    const timer = setTimeout(() => finish(undefined, new Error('语法分析超时，当前源码尚未完成检查。')), 5000);
    signal?.addEventListener('abort', abort, { once: true });
    worker.onmessage = (event: MessageEvent<AnalysisReply>) => {
      if (event.data?.requestId !== requestId) return;
      try {
        const reply = event.data;
        if (reply.modelId !== snapshot.modelId || reply.sourceRevision !== snapshot.sourceRevision || reply.dialectId !== snapshot.dialectId) throw new Error('语法分析返回了其他型号或源码版本的结果。');
        if (reply.error !== undefined) {
          if (!boundedText(reply.error) || !reply.error || reply.result !== undefined) throw new Error('语法分析返回了无效错误信息。');
          throw new Error(reply.error);
        }
        finish(validateProjectAnalysis(reply.result, snapshot));
      } catch (error) { finish(undefined, error); }
    };
    worker.onerror = () => finish(undefined, new Error('本地语法分析未能完成。'));
    worker.onmessageerror = () => finish(undefined, new Error('无法读取语法分析结果。'));
    // A re-entrant abort during worker creation must not leave an uncancellable request.
    if (signal?.aborted) { abort(); return; }
    try { worker.postMessage({ ...snapshot, requestId } satisfies AnalysisMessage); }
    catch (error) { finish(undefined, error); }
  });
}
