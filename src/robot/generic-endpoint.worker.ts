import { inspectGenericEndpointReference } from './generic-pose-algorithms';
import { validateGenericEndpointRequest, validateGenericEndpointResult } from './generic-pose-validation';
self.onmessage = (event: MessageEvent<{ requestId: string; request: unknown }>) => {
  const requestId = event.data?.requestId; if (typeof requestId !== 'string') return;
  try { const request = validateGenericEndpointRequest(event.data.request); self.postMessage({ requestId, result: validateGenericEndpointResult(inspectGenericEndpointReference(request), request) }); }
  catch (error) { self.postMessage({ requestId, error: error instanceof Error ? error.message : '端点数值逆解失败' }); }
};
