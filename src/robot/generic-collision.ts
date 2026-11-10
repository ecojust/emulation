import { invoke, isTauri } from '@tauri-apps/api/core';
import { validateGenericCollisionRequest as validateCollisionRequest, genericCollisionParts } from './generic-collision-geometry';
import type { GenericCollisionRequest as CollisionRequest } from './generic-collision-geometry';
import type { CollisionResult } from './collision-geometry';
export { genericCollisionParts } from './generic-collision-geometry';
export type { GenericCollisionRequest } from './generic-collision-geometry';

const TIMEOUT_MS = 10000;
// Keep one idle worker so the parsed STL meshes and their spatial indexes survive
// normal zone edits. An aborted/broken worker is discarded with all of its work.
let idleWorker: Worker | undefined;

export function verifiedGenericCollisionResults(value: unknown, request: CollisionRequest): CollisionResult[] {
  if (!Array.isArray(value) || value.length !== request.solutions.length) throw new Error('干涉检测返回了不完整的结果');
  const zoneIndices = new Map(request.zones.filter(zone => zone.enabled).map((zone, index) => [zone.id, index]));
  const validLinks = (links: unknown): links is number[] => {
    if (!Array.isArray(links)) return false;
    let previous = -1;
    for (const link of links) {
      if (typeof link !== 'number' || !Number.isInteger(link) || link <= previous || link >= genericCollisionParts(request.modelId).length) return false;
      previous = link;
    }
    return true;
  };
  for (const [index, row] of value.entries()) {
    if (!row || row.id !== request.solutions[index].id || !validLinks(row.collidingLinks) || !validLinks(row.uncertainLinks) || !Array.isArray(row.zones)) throw new Error('干涉检测返回了无效的姿态或连杆标识');
    let previousZone = -1;
    const union = new Set<number>();
    const uncertainUnion = new Set<number>();
    for (const zone of row.zones) {
      const zoneIndex = zone && typeof zone.zoneId === 'string' ? zoneIndices.get(zone.zoneId) : undefined;
      if (zoneIndex === undefined || zoneIndex <= previousZone || !validLinks(zone.collidingLinks) || !validLinks(zone.uncertainLinks)
        || !(zone.collidingLinks.length || zone.uncertainLinks.length)
        || zone.uncertainLinks.some((link: number) => zone.collidingLinks.includes(link))) throw new Error('干涉检测返回了无效的危险区标识或命中结果');
      previousZone = zoneIndex;
      zone.collidingLinks.forEach((link: number) => union.add(link));
      zone.uncertainLinks.forEach((link: number) => uncertainUnion.add(link));
    }
    if (union.size !== row.collidingLinks.length || row.collidingLinks.some((link: number) => !union.has(link))) throw new Error('干涉检测返回的危险区与连杆结果不一致');
    if (uncertainUnion.size !== row.uncertainLinks.length || row.uncertainLinks.some((link: number) => !uncertainUnion.has(link))) throw new Error('干涉检测返回的未确定区域与连杆结果不一致');
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
    const abort = () => stop(new DOMException('干涉检测已取消', 'AbortError'));
    const timer = setTimeout(() => stop(new Error('原生干涉检测超时，请重试')), TIMEOUT_MS);
    signal?.addEventListener('abort', abort, { once: true });
    void invoke<CollisionResult[]>('check_generic_collisions', { requestId, request }).then(value => {
      if (settled) return;
      settled = true; cleanup();
      try { resolve(verifiedGenericCollisionResults(value, request)); } catch (error) { reject(error); }
    }, cause => {
      if (settled) return;
      settled = true; cleanup();
      reject(new Error(typeof cause === 'string' ? cause : '原生干涉检测失败，请重试'));
    });
  });
}

function browserCheck(request: CollisionRequest, signal?: AbortSignal): Promise<CollisionResult[]> {
  return new Promise((resolve, reject) => {
    const worker = idleWorker ?? new Worker(new URL('./generic-collision.worker.ts', import.meta.url), { type: 'module' });
    idleWorker = undefined;
    const requestId = crypto.randomUUID();
    let settled = false;
    const cleanup = (reuse: boolean) => {
      clearTimeout(timer); signal?.removeEventListener('abort', abort);
      worker.onmessage = null; worker.onerror = null; worker.onmessageerror = null;
      if (reuse && !idleWorker) idleWorker = worker;
      else worker.terminate();
    };
    const fail = (error: Error) => { if (settled) return; settled = true; cleanup(false); reject(error); };
    const abort = () => fail(new DOMException('干涉检测已取消', 'AbortError'));
    const timer = setTimeout(() => fail(new Error('干涉检测超时，请重试')), TIMEOUT_MS);
    signal?.addEventListener('abort', abort, { once: true });
    worker.onmessage = (event: MessageEvent<{ requestId?: string; results?: CollisionResult[]; error?: string }>) => {
      if (settled || event.data?.requestId !== requestId) return;
      try {
        if (event.data.error) throw new Error(event.data.error);
        const results = verifiedGenericCollisionResults(event.data.results, request);
        settled = true; cleanup(true); resolve(results);
      } catch (error) { fail(error instanceof Error ? error : new Error('干涉检测结果无效')); }
    };
    worker.onerror = () => fail(new Error('干涉检测未完成，请重试'));
    worker.onmessageerror = () => fail(new Error('干涉检测结果无法读取，请重试'));
    try { worker.postMessage({ requestId, request }); } catch { fail(new Error('干涉检测参数无法传递')); }
  });
}

/** Callers cancel obsolete checks when targets, model or zone change. No desktop fallback. */
export function checkGenericCollisions(request: CollisionRequest, signal?: AbortSignal): Promise<CollisionResult[]> {
  if (signal?.aborted) return Promise.reject(new DOMException('干涉检测已取消', 'AbortError'));
  try { validateCollisionRequest(request); } catch (error) { return Promise.reject(error); }
  return isTauri() ? nativeCheck(request, signal) : browserCheck(request, signal);
}
