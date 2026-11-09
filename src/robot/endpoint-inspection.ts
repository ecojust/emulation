import { invoke, isTauri } from '@tauri-apps/api/core';
import { validateEndpointInspectionRequest, validateEndpointSolutions } from './endpoint-inspection-validation';
import type { EndpointInspectionRequest } from './endpoint-inspection-validation';
import type { IKSolution } from './types';
export type { EndpointInspectionRequest } from './endpoint-inspection-validation';

const TIMEOUT_MS = 5000;

function nativeInspection(request: EndpointInspectionRequest, signal?: AbortSignal): Promise<IKSolution[]> {
  return new Promise((resolve, reject) => {
    const requestId = crypto.randomUUID();
    let settled = false;
    const cleanup = () => { clearTimeout(timer); signal?.removeEventListener('abort', abort); };
    const stop = (error: Error) => {
      if (settled) return;
      settled = true; cleanup();
      void invoke('cancel_motion', { requestId }).catch(() => {});
      reject(error);
    };
    const abort = () => stop(new DOMException('端点检查已取消', 'AbortError'));
    const timer = setTimeout(() => stop(new Error('原生端点检查超时，请重试')), TIMEOUT_MS);
    signal?.addEventListener('abort', abort, { once: true });
    if (signal?.aborted) { abort(); return; }
    void invoke<unknown>('inspect_endpoint', { requestId, request }).then(value => {
      if (settled) return;
      settled = true; cleanup();
      try { resolve(validateEndpointSolutions(value, request)); } catch (error) { reject(error); }
    }, cause => {
      if (settled) return;
      settled = true; cleanup();
      reject(new Error(typeof cause === 'string' ? cause : '原生端点检查失败，请重试'));
    });
  });
}

function browserInspection(request: EndpointInspectionRequest, signal?: AbortSignal): Promise<IKSolution[]> {
  return new Promise((resolve, reject) => {
    const worker = new Worker(new URL('./endpoint-inspection.worker.ts', import.meta.url), { type: 'module' });
    const requestId = crypto.randomUUID();
    let settled = false;
    const cleanup = () => { clearTimeout(timer); signal?.removeEventListener('abort', abort); worker.terminate(); };
    const fail = (error: Error) => { if (settled) return; settled = true; cleanup(); reject(error); };
    const abort = () => fail(new DOMException('端点检查已取消', 'AbortError'));
    const timer = setTimeout(() => fail(new Error('端点检查超时，请重试')), TIMEOUT_MS);
    signal?.addEventListener('abort', abort, { once: true });
    if (signal?.aborted) { abort(); return; }
    worker.onmessage = (event: MessageEvent<{ requestId?: string; solutions?: unknown; error?: string }>) => {
      if (settled || event.data?.requestId !== requestId) return;
      try {
        if (event.data.error) throw new Error(event.data.error);
        const solutions = validateEndpointSolutions(event.data.solutions, request);
        settled = true; cleanup(); resolve(solutions);
      } catch (error) { fail(error instanceof Error ? error : new Error('端点检查结果无效')); }
    };
    worker.onerror = () => fail(new Error('端点检查未完成，请重试'));
    worker.onmessageerror = () => fail(new Error('端点检查结果无法读取，请重试'));
    try { worker.postMessage({ requestId, request }); } catch { fail(new Error('端点参数无法传递')); }
  });
}

/** Endpoint alternatives only. Desktop failures never fall back to a JS solver or a motion plan. */
export function inspectEndpoint(request: EndpointInspectionRequest, signal?: AbortSignal): Promise<IKSolution[]> {
  if (signal?.aborted) return Promise.reject(new DOMException('端点检查已取消', 'AbortError'));
  try {
    const validated = validateEndpointInspectionRequest(request);
    return isTauri() ? nativeInspection(validated, signal) : browserInspection(validated, signal);
  } catch (error) { return Promise.reject(error); }
}
