import type { PlanRequest, PlanResult } from "./types";

/** Each request owns its worker, so editing or cancelling cannot publish stale results. */
export function solveMotion(request: PlanRequest, signal?: AbortSignal): Promise<PlanResult> {
  return new Promise((resolve, reject) => {
    if (signal?.aborted) {
      reject(new DOMException("求解已取消", "AbortError"));
      return;
    }
    const worker = new Worker(new URL("./planner.worker.ts", import.meta.url), { type: "module" });
    let settled = false;
    const cleanup = () => {
      clearTimeout(timer);
      signal?.removeEventListener("abort", abort);
      worker.terminate();
    };
    const abort = () => {
      if (settled) return;
      settled = true;
      cleanup();
      reject(new DOMException("求解已取消", "AbortError"));
    };
    const timer = setTimeout(() => {
      if (settled) return;
      settled = true;
      cleanup();
      reject(new Error("求解超时，请调整起始姿态或目标后重试"));
    }, 15_000);
    signal?.addEventListener("abort", abort, { once: true });
    worker.onmessage = (event: MessageEvent<PlanResult>) => {
      if (settled) return;
      settled = true;
      cleanup();
      if (event.data?.status === "success" || event.data?.status === "failure") resolve(event.data);
      else reject(new Error("未能取得有效的运动学计算结果"));
    };
    worker.onerror = () => {
      if (settled) return;
      settled = true;
      cleanup();
      reject(new Error("运动学计算未完成，请检查参数后重试"));
    };
    worker.postMessage(request);
  });
}
