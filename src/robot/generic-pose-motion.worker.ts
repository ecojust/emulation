import { planGenericPoseMoveL } from './generic-pose-algorithms';
import { validateGenericPoseRequest, validateGenericPoseResult } from './generic-pose-validation';
self.onmessage = (event: MessageEvent<{ requestId: string; request: unknown }>) => {
  const requestId = event.data?.requestId; if (typeof requestId !== 'string') return;
  try { const request = validateGenericPoseRequest(event.data.request); self.postMessage({ requestId, result: validateGenericPoseResult(planGenericPoseMoveL(request), request) }); }
  catch (error) { self.postMessage({ requestId, error: error instanceof Error ? error.message : 'TCP 运动计算失败' }); }
};
