import { invoke, isTauri } from '@tauri-apps/api/core';
import { validateGenericMotionRequest, validateGenericMotionResult } from './generic-motion-validation';
import type { GenericPlanRequest, GenericPlanResult } from './generic-motion-types';
export type { GenericPlanRequest, GenericPlanResult } from './generic-motion-types';

const TIMEOUT_MS = 15_000;

function nativePlan(request: GenericPlanRequest, signal?: AbortSignal): Promise<GenericPlanResult> {
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
    const abort = () => stop(new DOMException('URDF 运动计算已取消', 'AbortError'));
    const timer = setTimeout(() => stop(new Error('原生 URDF 运动计算超时，请缩短路径后重试')), TIMEOUT_MS);
    signal?.addEventListener('abort', abort, { once: true });
    if (signal?.aborted) { abort(); return; }
    void invoke<unknown>('solve_generic_motion', { requestId, request }).then(value => {
      if (settled) return;
      settled = true; cleanup();
      try { resolve(validateGenericMotionResult(value, request)); } catch (error) { reject(error); }
    }, cause => {
      if (settled) return;
      settled = true; cleanup();
      reject(new Error(typeof cause === 'string' ? cause : '原生 URDF 运动计算失败，请重试'));
    });
  });
}

function browserPlan(request: GenericPlanRequest, signal?: AbortSignal): Promise<GenericPlanResult> {
  return new Promise((resolve, reject) => {
    const worker = new Worker(new URL('./generic-motion.worker.ts', import.meta.url), { type: 'module' });
    const requestId = crypto.randomUUID();
    let settled = false;
    const cleanup = () => { clearTimeout(timer); signal?.removeEventListener('abort', abort); worker.terminate(); };
    const fail = (error: Error) => { if (settled) return; settled = true; cleanup(); reject(error); };
    const abort = () => fail(new DOMException('URDF 运动计算已取消', 'AbortError'));
    const timer = setTimeout(() => fail(new Error('URDF 运动计算超时，请缩短路径后重试')), TIMEOUT_MS);
    signal?.addEventListener('abort', abort, { once: true });
    if (signal?.aborted) { abort(); return; }
    worker.onmessage = (event: MessageEvent<{ requestId?: string; result?: unknown; error?: string }>) => {
      if (settled || event.data?.requestId !== requestId) return;
      try {
        if (event.data.error) throw new Error(event.data.error);
        const result = validateGenericMotionResult(event.data.result, request);
        settled = true; cleanup(); resolve(result);
      } catch (error) { fail(error instanceof Error ? error : new Error('URDF 运动结果无效')); }
    };
    worker.onerror = () => fail(new Error('URDF 运动计算未完成，请重试'));
    worker.onmessageerror = () => fail(new Error('URDF 运动结果无法读取，请重试'));
    try { worker.postMessage({ requestId, request }); } catch { fail(new Error('URDF 运动参数无法传递')); }
  });
}

/** Native computation is mandatory on desktop; only browsers use the reference worker. */
export function solveGenericMotion(request: GenericPlanRequest, signal?: AbortSignal): Promise<GenericPlanResult> {
  if (signal?.aborted) return Promise.reject(new DOMException('URDF 运动计算已取消', 'AbortError'));
  try {
    const validated = validateGenericMotionRequest(request);
    return isTauri() ? nativePlan(validated, signal) : browserPlan(validated, signal);
  } catch (error) { return Promise.reject(error); }
}
