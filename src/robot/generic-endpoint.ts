import { runGenericPoseTask } from './generic-pose-task';
import { validateGenericEndpointRequest, validateGenericEndpointResult } from './generic-pose-validation';
import type { GenericEndpointRequest, GenericIkSolution } from './generic-motion-types';
export type { GenericEndpointRequest, GenericIkSolution } from './generic-motion-types';
export function inspectGenericEndpoint(request: GenericEndpointRequest, signal?: AbortSignal): Promise<GenericIkSolution[]> {
  if (signal?.aborted) return Promise.reject(new DOMException('端点数值逆解已取消', 'AbortError'));
  try { return runGenericPoseTask({ request: validateGenericEndpointRequest(request), signal, command: 'inspect_generic_endpoint', cancelCommand: 'cancel_motion', workerUrl: new URL('./generic-endpoint.worker.ts', import.meta.url), timeoutMs: 5_000, validate: validateGenericEndpointResult }); }
  catch (error) { return Promise.reject(error); }
}
