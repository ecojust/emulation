import { afterEach, describe, expect, it, vi } from 'vitest';
import { createJogController } from '../src/robot/jog-controller';
import { forwardKinematics } from '../src/robot/kinematics';
import { ROBOT_MODELS } from '../src/robot/model';
import { planMoveL } from '../src/robot/planner';
import type { PlanRequest, PlanResult, PlanSuccess, Pose, RobotId } from '../src/robot/types';

const initial = (modelId: RobotId = 'irb1100-4-0.58') => {
  const model = ROBOT_MODELS[modelId];
  return { modelId, joints: model.readyJoints, pose: forwardKinematics(model, model.readyJoints).pose, speedMmPerSecond: 100 };
};
const target = (millimetres: number, modelId: RobotId = 'irb1100-4-0.58'): Pose => {
  const pose = initial(modelId).pose;
  return { position: [pose.position[0], millimetres, pose.position[2]], orientation: pose.orientation };
};
function successful(request: PlanRequest): PlanSuccess {
  const result = planMoveL(request);
  expect(result.status).toBe('success');
  if (result.status !== 'success') throw new Error(result.message);
  return result;
}
function harness(minimumIntervalMs = 0) {
  const calls: { request: PlanRequest; signal: AbortSignal; resolve: (result: PlanResult) => void; reject: (reason: Error) => void }[] = [];
  const accept = vi.fn();
  const status = vi.fn();
  const controller = createJogController({
    initial: initial(), minimumIntervalMs,
    solve: (request, signal) => new Promise<PlanResult>((resolve, reject) => { calls.push({ request, signal, resolve, reject }); }),
    onAccept: accept, onStatus: status,
  });
  return { controller, calls, accept, status };
}
async function flush() { for (let i = 0; i < 8; i++) await Promise.resolve(); }
afterEach(() => vi.useRealTimers());

describe('incremental TCP jog scheduling', () => {
  it('keeps one active solve and only the latest queued cursor position, continuing from accepted joints', async () => {
    const { controller, calls, accept, status } = harness();
    controller.update(target(5));
    controller.update(target(10));
    controller.update(target(15));
    expect(calls).toHaveLength(1);
    const first = successful(calls[0].request);
    calls[0].resolve(first);
    await flush();
    expect(accept).toHaveBeenCalledTimes(1);
    expect(calls).toHaveLength(2);
    expect(calls[1].request.target).toEqual(target(15));
    expect(calls[1].request.startJoints).toEqual(first.selected.joints);
    expect(calls[0].signal.aborted).toBe(false);
    calls[1].resolve(successful(calls[1].request));
    await flush();
    expect(accept).toHaveBeenCalledTimes(2);
    expect(status.mock.calls.at(-1)?.[0]).toMatchObject({ pending: false, error: null });
    controller.dispose();
  });

  it('retains the last accepted robot pose when a target fails and recovers on the next valid target', async () => {
    const { controller, calls, accept, status } = harness();
    controller.update(target(5));
    const accepted = successful(calls[0].request);
    calls[0].resolve(accepted);
    await flush();
    controller.update({ position: [3000, 0, 3000], orientation: target(0).orientation });
    calls[1].resolve({ status: 'failure', code: 'unreachable', message: '目标不可达，保留最后可达姿态' });
    await flush();
    expect(accept).toHaveBeenCalledTimes(1);
    expect(status.mock.calls.at(-1)?.[0]).toMatchObject({ pending: false, error: '目标不可达，保留最后可达姿态' });
    controller.update(target(10));
    expect(calls[2].request.startJoints).toEqual(accepted.selected.joints);
    calls[2].resolve(successful(calls[2].request));
    await flush();
    expect(accept).toHaveBeenCalledTimes(2);
    expect(status.mock.calls.at(-1)?.[0]).toMatchObject({ pending: false, error: null });
    controller.dispose();
  });

  it('aborts a replaced model generation and rejects its late success without clearing a new request status', async () => {
    const { controller, calls, accept, status } = harness();
    controller.update(target(5));
    controller.update(target(10));
    const stale = successful(calls[0].request);
    controller.reset(initial('irb1100-4-0.475'));
    expect(calls[0].signal.aborted).toBe(true);
    controller.update(target(5, 'irb1100-4-0.475'));
    expect(calls).toHaveLength(2);
    expect(calls[1].request.modelId).toBe('irb1100-4-0.475');
    calls[0].resolve(stale);
    await flush();
    expect(accept).not.toHaveBeenCalled();
    expect(calls).toHaveLength(2);
    expect(status.mock.calls.at(-1)?.[0]).toMatchObject({ pending: true });
    calls[1].resolve(successful(calls[1].request));
    await flush();
    expect(accept).toHaveBeenCalledTimes(1);
    expect(accept.mock.calls[0][1].modelId).toBe('irb1100-4-0.475');
    controller.dispose();
  });

  it('cancels queued motion while preserving accepted state and ignores late errors after disposal', async () => {
    const { controller, calls, accept, status } = harness();
    controller.update(target(5));
    const accepted = successful(calls[0].request);
    calls[0].resolve(accepted);
    await flush();
    controller.update(target(10));
    controller.update(target(15));
    controller.cancel();
    expect(calls[1].signal.aborted).toBe(true);
    calls[1].resolve(successful(calls[1].request));
    await flush();
    expect(calls).toHaveLength(2);
    expect(accept).toHaveBeenCalledTimes(1);
    controller.update(target(20));
    expect(calls[2].request.startJoints).toEqual(accepted.selected.joints);
    controller.dispose();
    const statusesAtDisposal = status.mock.calls.length;
    expect(calls[2].signal.aborted).toBe(true);
    calls[2].reject(new Error('Late worker failure'));
    await flush();
    expect(accept).toHaveBeenCalledTimes(1);
    expect(status).toHaveBeenCalledTimes(statusesAtDisposal);
  });

  it('rate limits completed bursts and drops a pending timer when the scene is reset', async () => {
    vi.useFakeTimers({ toFake: ['setTimeout', 'clearTimeout', 'Date', 'performance'] });
    const { controller, calls, accept } = harness(34);
    controller.update(target(5));
    expect(calls).toHaveLength(1);
    calls[0].resolve(successful(calls[0].request));
    await flush();
    controller.update(target(10));
    controller.update(target(15));
    expect(calls).toHaveLength(1);
    await vi.advanceTimersByTimeAsync(33);
    expect(calls).toHaveLength(1);
    await vi.advanceTimersByTimeAsync(1);
    expect(calls).toHaveLength(2);
    expect(calls[1].request.target).toEqual(target(15));
    calls[1].resolve(successful(calls[1].request));
    await flush();
    controller.update(target(20));
    controller.reset(initial());
    await vi.advanceTimersByTimeAsync(100);
    expect(calls).toHaveLength(2);
    expect(accept).toHaveBeenCalledTimes(2);
    controller.dispose();
  });
});
