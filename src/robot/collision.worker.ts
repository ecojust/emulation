import { checkCollisionReference } from './collision-geometry';
import type { CollisionRequest } from './collision-geometry';

self.onmessage = (event: MessageEvent<CollisionRequest>) => {
  try { self.postMessage({ results: checkCollisionReference(event.data) }); }
  catch (error) { self.postMessage({ error: error instanceof Error ? error.message : '碰撞预检无法完成' }); }
};
