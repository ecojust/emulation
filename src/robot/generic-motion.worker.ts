import { planGenericMoveL } from './generic-planner';
import { validateGenericMotionRequest, validateGenericMotionResult } from './generic-motion-validation';

self.onmessage = (event: MessageEvent<{ requestId: string; request: unknown }>) => {
  const requestId = event.data?.requestId;
  if (typeof requestId !== 'string') return;
  try {
    const request = validateGenericMotionRequest(event.data.request);
    const result = validateGenericMotionResult(planGenericMoveL(request), request);
    self.postMessage({ requestId, result });
  } catch (error) {
    self.postMessage({ requestId, error: error instanceof Error ? error.message : 'URDF 运动计算失败' });
  }
};
