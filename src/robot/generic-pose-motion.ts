import { runGenericPoseTask } from './generic-pose-task';
import { validateGenericPoseRequest, validateGenericPoseResult } from './generic-pose-validation';
import type { GenericPosePlanRequest, GenericPosePlanResult } from './generic-motion-types';
export type { GenericPosePlanRequest, GenericPosePlanResult, GenericPosePlanSuccess } from './generic-motion-types';
export function solveGenericPoseMotion(request: GenericPosePlanRequest, signal?: AbortSignal): Promise<GenericPosePlanResult> {
  if (signal?.aborted) return Promise.reject(new DOMException('TCP 运动计算已取消', 'AbortError'));
  try { return runGenericPoseTask({ request: validateGenericPoseRequest(request), signal, command: 'solve_generic_pose_motion', cancelCommand: 'cancel_motion', workerUrl: new URL('./generic-pose-motion.worker.ts', import.meta.url), timeoutMs: 15_000, validate: validateGenericPoseResult }); }
  catch (error) { return Promise.reject(error); }
}
