import { checkCollisionReference } from './collision-geometry';
import type { CollisionRequest } from './collision-geometry';

self.onmessage = async (event: MessageEvent<{ requestId: string; request: CollisionRequest }>) => {
  const { requestId, request } = event.data;
  try { self.postMessage({ requestId, results: await checkCollisionReference(request) }); }
  catch (error) { self.postMessage({ requestId, error: error instanceof Error ? error.message : '干涉检测无法完成' }); }
};
