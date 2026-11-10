import { invoke, isTauri } from '@tauri-apps/api/core';

/** One bounded request; native failures never fall back to browser computation. */
export function runGenericPoseTask<Request, Result>(options: {
  request: Request; signal?: AbortSignal; command: string; cancelCommand: string;
  workerUrl: URL; timeoutMs: number; validate: (value: unknown, request: Request) => Result;
}): Promise<Result> {
  const { request, signal } = options;
  return new Promise((resolve, reject) => {
    const requestId = crypto.randomUUID(); let settled = false, worker: Worker | undefined;
    const native = isTauri();
    const cleanup = () => { clearTimeout(timer); signal?.removeEventListener('abort', abort); worker?.terminate(); };
    const fail = (error: Error, stop = true) => {
      if (settled) return; settled = true; cleanup();
      if (native && stop) void invoke(options.cancelCommand, { requestId }).catch(() => {});
      reject(error);
    };
    const accept = (value: unknown) => {
      if (settled) return;
      try { const result = options.validate(value, request); settled = true; cleanup(); resolve(result); }
      catch (error) { fail(error instanceof Error ? error : new Error('URDF 计算结果无效'), false); }
    };
    const abort = () => fail(new DOMException('URDF 计算已取消', 'AbortError'));
    const timer = setTimeout(() => fail(new Error('URDF 计算超时，请调整目标后重试')), options.timeoutMs);
    signal?.addEventListener('abort', abort, { once: true });
    if (signal?.aborted) { abort(); return; }
    if (native) {
      void invoke(options.command, { requestId, request }).then(accept, error => fail(new Error(typeof error === 'string' ? error : '原生 URDF 计算失败，请重试'), false));
      return;
    }
    try {
      worker = new Worker(options.workerUrl, { type: 'module' });
      worker.onmessage = (event: MessageEvent<{ requestId?: string; result?: unknown; error?: string }>) => {
        if (settled || event.data?.requestId !== requestId) return;
        if (event.data.error) fail(new Error(event.data.error)); else accept(event.data.result);
      };
      worker.onerror = () => fail(new Error('URDF 计算未完成，请重试'));
      worker.onmessageerror = () => fail(new Error('URDF 计算结果无法读取'));
      worker.postMessage({ requestId, request });
    } catch { fail(new Error('URDF 计算请求无法启动')); }
  });
}
