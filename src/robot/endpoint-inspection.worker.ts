import { inverseKinematics } from './kinematics';
import { ROBOT_MODELS } from './model';
import { validateEndpointInspectionRequest, validateEndpointSolutions } from './endpoint-inspection-validation';

self.onmessage = (event: MessageEvent<{ requestId: string; request: unknown }>) => {
  const requestId = event.data?.requestId;
  if (typeof requestId !== 'string') return;
  try {
    const request = validateEndpointInspectionRequest(event.data.request);
    const solutions = inverseKinematics(ROBOT_MODELS[request.modelId], request.pose, request.seedJoints);
    self.postMessage({ requestId, solutions: validateEndpointSolutions(solutions, request) });
  } catch (error) {
    self.postMessage({ requestId, error: error instanceof Error ? error.message : '端点逆解检查失败' });
  }
};
