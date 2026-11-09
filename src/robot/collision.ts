import { invoke, isTauri } from '@tauri-apps/api/core';
import { validateCollisionRequest } from './collision-geometry';
import type { CollisionRequest, CollisionResult } from './collision-geometry';
export * from './collision-geometry';

const TIMEOUT_MS = 5000;

function verifiedResults(value: unknown, request: CollisionRequest): CollisionResult[] {
  if (!Array.isArray(value) || value.length !== request.solutions.length) throw new Error('碰撞预检返回了不完整的结果');
  const zoneIndices = new Map(request.zones.filter(zone => zone.enabled).map((zone, index) => [zone.id, index]));
  const validLinks = (links: unknown): links is number[] => {
    if (!Array.isArray(links)) return false;
    let previous = -1;
    for (const link of links) {
      if (typeof link !== 'number' || !Number.isInteger(link) || link <= previous || link > 6) return false;
      previous = link;
    }
    return true;
  };
  for (const [index, row] of value.entries()) {
    if (!row || row.id !== request.solutions[index].id || !validLinks(row.collidingLinks) || !Array.isArray(row.zones)) throw new Error('碰撞预检返回了无效的姿态或连杆标识');
    let previousZone = -1;
    const union = new Set<number>();
    for (const zone of row.zones) {
      const zoneIndex = zone && typeof zone.zoneId === 'string' ? zoneIndices.get(zone.zoneId) : undefined;
      if (zoneIndex === undefined || zoneIndex <= previousZone || !validLinks(zone.collidingLinks) || !zone.collidingLinks.length) throw new Error('碰撞预检返回了无效的危险区标识或命中结果');
      previousZone = zoneIndex;
      zone.collidingLinks.forEach((link: number) => union.add(link));
    }
    if (union.size !== row.collidingLinks.length || row.collidingLinks.some((link: number) => !union.has(link))) throw new Error('碰撞预检返回的危险区与连杆结果不一致');
  }
  return value as CollisionResult[];
}

function nativeCheck(request: CollisionRequest, signal?: AbortSignal): Promise<CollisionResult[]> {
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
    const abort = () => stop(new DOMException('碰撞预检已取消', 'AbortError'));
    const timer = setTimeout(() => stop(new Error('原生碰撞预检超时，请重试')), TIMEOUT_MS);
    signal?.addEventListener('abort', abort, { once: true });
    void invoke<CollisionResult[]>('check_collisions', { requestId, request }).then(value => {
      if (settled) return;
      settled = true; cleanup();
      try { resolve(verifiedResults(value, request)); } catch (error) { reject(error); }
    }, cause => {
      if (settled) return;
      settled = true; cleanup();
      reject(new Error(typeof cause === 'string' ? cause : '原生碰撞预检失败，请重试'));
    });
  });
}

function browserCheck(request: CollisionRequest, signal?: AbortSignal): Promise<CollisionResult[]> {
  return new Promise((resolve, reject) => {
    const worker = new Worker(new URL('./collision.worker.ts', import.meta.url), { type: 'module' });
    let settled = false;
    const cleanup = () => { clearTimeout(timer); signal?.removeEventListener('abort', abort); worker.terminate(); };
    const fail = (error: Error) => { if (settled) return; settled = true; cleanup(); reject(error); };
    const abort = () => fail(new DOMException('碰撞预检已取消', 'AbortError'));
    const timer = setTimeout(() => fail(new Error('碰撞预检超时，请重试')), TIMEOUT_MS);
    signal?.addEventListener('abort', abort, { once: true });
    worker.onmessage = (event: MessageEvent<{ results?: CollisionResult[]; error?: string }>) => {
      if (settled) return;
      settled = true; cleanup();
      try {
        if (event.data.error) throw new Error(event.data.error);
        resolve(verifiedResults(event.data.results, request));
      } catch (error) { reject(error); }
    };
    worker.onerror = () => fail(new Error('碰撞预检未完成，请重试'));
    try { worker.postMessage(request); } catch { fail(new Error('碰撞预检参数无法传递')); }
  });
}

/** Callers cancel obsolete checks when targets, model or zone change. No desktop fallback. */
export function checkCollisions(request: CollisionRequest, signal?: AbortSignal): Promise<CollisionResult[]> {
  if (signal?.aborted) return Promise.reject(new DOMException('碰撞预检已取消', 'AbortError'));
  try { validateCollisionRequest(request); } catch (error) { return Promise.reject(error); }
  return isTauri() ? nativeCheck(request, signal) : browserCheck(request, signal);
}
