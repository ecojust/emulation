/// <reference lib="webworker" />
import { planGrindingReference } from './grinding-reference';
import type { GrindingRequest } from './grinding-types';
const scope = self as unknown as DedicatedWorkerGlobalScope;
scope.onmessage = (event: MessageEvent<{ requestId: string; request: GrindingRequest }>) => {
  const { requestId, request } = event.data;
  try { scope.postMessage({ requestId, result: planGrindingReference(request) }); }
  catch (error) { scope.postMessage({ requestId, error: error instanceof Error ? error.message : '路径规划失败。' }); }
};
