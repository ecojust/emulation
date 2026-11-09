import { invoke, isTauri } from "@tauri-apps/api/core";
import type { PlanRequest, PlanResult } from "./types";

/** Each request owns its worker, so editing or cancelling cannot publish stale results. */
function solveBrowserReference(request: PlanRequest, signal?: AbortSignal): Promise<PlanResult> {
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


export const nativeDesktop = isTauri();
export interface ProgramExportRequest {
  programName: string;
  controller: "abb_rapid" | "fanuc_ls" | "kuka_krl" | "yaskawa_inform";
  motion: PlanRequest;
}

function nativeRequest<T>(command: string, request: PlanRequest | ProgramExportRequest, signal?: AbortSignal, timeoutMs = 15000): Promise<T> {
  return new Promise((resolve, reject) => {
    if (signal?.aborted) { reject(new DOMException("计算已取消", "AbortError")); return; }
    const requestId = crypto.randomUUID();
    let settled = false;
    let timer: ReturnType<typeof setTimeout> | undefined;
    const cleanup = () => { if (timer) clearTimeout(timer); signal?.removeEventListener("abort", abort); };
    const abort = () => {
      if (settled) return;
      settled = true; cleanup();
      void invoke("cancel_motion", { requestId }).catch(() => {});
      reject(new DOMException("计算已取消", "AbortError"));
    };
    signal?.addEventListener("abort", abort, { once: true });
    if (timeoutMs) timer = setTimeout(() => {
      if (settled) return;
      settled = true; cleanup();
      void invoke("cancel_motion", { requestId }).catch(() => {});
      reject(new Error("原生运动计算超时，请调整输入后重试"));
    }, timeoutMs);
    void invoke<T>(command, { requestId, request }).then(value => {
      if (settled) return;
      settled = true; cleanup(); resolve(value);
    }, cause => {
      if (settled) return;
      settled = true; cleanup(); reject(new Error(typeof cause === "string" ? cause : "原生运动引擎调用失败，请重新启动客户端"));
    });
  });
}

/** Desktop computation always uses native Rust. A failed IPC never falls back to JavaScript. */
export function solveMotion(request: PlanRequest, signal?: AbortSignal): Promise<PlanResult> {
  return nativeDesktop ? nativeRequest<PlanResult>("solve_motion", request, signal) : solveBrowserReference(request, signal);
}

export function saveProgram(request: ProgramExportRequest, signal?: AbortSignal): Promise<string | null> {
  if (!nativeDesktop) return Promise.reject(new Error("请在桌面客户端中导出机器人程序模块"));
  // A native save dialog may remain open while the user chooses a destination.
  return nativeRequest<string | null>("save_program", request, signal, 0);
}

export async function copyRapid(text: string): Promise<void> {
  if (nativeDesktop) await invoke("copy_rapid", { text });
  else await navigator.clipboard.writeText(text);
}
