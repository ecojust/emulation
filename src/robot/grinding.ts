import { invoke, isTauri } from '@tauri-apps/api/core';
import { validateGrindingRequest, validateGrindingResult } from './grinding-validation';
import type { GrindingRequest, GrindingResult } from './grinding-types';
export type { GrindingRequest, GrindingResult, GrindingSuccess, GrindingInput } from './grinding-types';
const TIMEOUT_MS = 60_000;

/** One bounded, cancellable job; native IPC failures never fall back to JavaScript. */
export function planGrindingPath(request: GrindingRequest, signal?: AbortSignal): Promise<GrindingResult> {
  if (signal?.aborted) return Promise.reject(new DOMException('路径规划已取消', 'AbortError'));
  let validated: GrindingRequest;
  try { validated = validateGrindingRequest(request); } catch (error) { return Promise.reject(error); }
  const native = isTauri();
  return new Promise((resolve, reject) => {
    const requestId = crypto.randomUUID();
    let settled = false, worker: Worker | null = null;
    const cleanup = () => { clearTimeout(timer); signal?.removeEventListener('abort', abort); worker?.terminate(); };
    const stop = (error: Error) => {
      if (settled) return;
      settled = true; cleanup();
      if (native) void invoke('cancel_motion', { requestId }).catch(() => {});
      reject(error);
    };
    const abort = () => stop(new DOMException('路径规划已取消', 'AbortError'));
    const timer = setTimeout(() => stop(new Error('路径规划超时，请减少目标点或调整场景后重试。')), TIMEOUT_MS);
    signal?.addEventListener('abort', abort, { once: true });
    const accept = (value: unknown) => {
      if (settled) return;
      try {
        const result = validateGrindingResult(value, validated);
        settled = true; cleanup(); resolve(result);
      } catch (error) { stop(error instanceof Error ? error : new Error('路径规划结果无效。')); }
    };
    if (signal?.aborted) { abort(); return; }
    if (native) {
      void invoke('plan_grinding_path', { requestId, request: validated }).then(accept, error => stop(new Error(typeof error === 'string' ? error : '原生路径规划失败。')));
      return;
    }
    try {
      worker = new Worker(new URL('./grinding.worker.ts', import.meta.url), { type: 'module' });
      worker.onmessage = (event: MessageEvent<{ requestId: string; result?: unknown; error?: string }>) => {
        if (settled || event.data?.requestId !== requestId) return;
        if (event.data.error) stop(new Error(event.data.error)); else accept(event.data.result);
      };
      worker.onerror = () => stop(new Error('路径规划计算未完成，请重试。'));
      worker.onmessageerror = () => stop(new Error('路径规划结果无法读取。'));
      worker.postMessage({ requestId, request: validated });
    } catch (error) { stop(error instanceof Error ? error : new Error('无法创建路径规划任务。')); }
  });
}
