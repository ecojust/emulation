import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import chainsJson from './fixtures/catalog/generic-chains.json';
import { forwardGenericKinematics } from '../src/robot/generic-kinematics';
import { planGenericMoveL } from '../src/robot/generic-planner';
import { validateGenericMotionRequest, validateGenericMotionResult } from '../src/robot/generic-motion-validation';
import type { GenericChain, GenericPlanRequest, GenericPlanSuccess } from '../src/robot/generic-motion-types';

const tauri = vi.hoisted(() => ({ invoke: vi.fn(), isTauri: vi.fn(() => true) }));
vi.mock('@tauri-apps/api/core', () => tauri);
import { solveGenericMotion } from '../src/robot/generic-motion';

function request(): GenericPlanRequest {
  return { chain: structuredClone(chainsJson[0]) as GenericChain, startJoints: [20, 15, -25, 25, 35, 10],
    targetJoints: [21, 14.5, -24.25, 25.5, 34.25, 11], speedMmPerSecond: 100 };
}
function result(input = request()): GenericPlanSuccess {
  const value = planGenericMoveL(input);
  if (value.status !== 'success') throw new Error(value.message);
  return value;
}

describe('generic Cartesian transport verification', () => {
  it('copies the complete chain and recorded points before asynchronous work', () => {
    const input = request(), privateCopy = validateGenericMotionRequest(input), original = structuredClone(privateCopy);
    input.chain.steps[0].origin[3] = 1234; input.chain.limits[0].maxSpeedDeg = 999;
    input.startJoints[0] = 50; input.targetJoints[5] = 90;
    expect(privateCopy).toEqual(original);
    expect(validateGenericMotionResult(result(privateCopy), privateCopy).status).toBe('success');
  });

  it('rejects sparse and out-of-range inputs before either transport runs', () => {
    for (const mutate of [
      (input: GenericPlanRequest) => { delete (input.startJoints as number[])[1]; },
      (input: GenericPlanRequest) => { delete input.chain.steps[0].origin[2]; },
      (input: GenericPlanRequest) => { input.targetJoints[0] = Infinity; },
      (input: GenericPlanRequest) => { input.targetJoints[0] = input.chain.limits[0].maxDeg + 1; },
      (input: GenericPlanRequest) => { input.speedMmPerSecond = 2001; },
    ]) { const input = request(); mutate(input); expect(() => validateGenericMotionRequest(input)).toThrow(); }
  });

  it('rejects claimed success with false FK, a different endpoint, omitted samples or fabricated timing', () => {
    const input = request();
    for (const mutate of [
      (value: GenericPlanSuccess) => { value.path[1].pose.position[0] += 1; },
      (value: GenericPlanSuccess) => { value.path[1].joints[0] += 1; value.path[1].pose = forwardGenericKinematics(input.chain, value.path[1].joints).pose; },
      (value: GenericPlanSuccess) => { delete (value.path[1].joints as number[])[0]; },
      (value: GenericPlanSuccess) => { value.path.at(-1)!.joints[5] += 0.1; },
      (value: GenericPlanSuccess) => { value.path = [value.path[0], value.path.at(-1)!]; },
      (value: GenericPlanSuccess) => { value.path[1].t = 0; },
      (value: GenericPlanSuccess) => { value.lengthMm += 1; },
      (value: GenericPlanSuccess) => { value.durationSeconds = 0; },
      (value: GenericPlanSuccess) => { value.warnings = new Array(2); },
    ]) { const value = result(input); mutate(value); expect(() => validateGenericMotionResult(value, input)).toThrow(); }
    expect(() => validateGenericMotionResult({ status: 'failure', code: 'unrecognized', message: 'bad' }, input)).toThrow();
    expect(() => validateGenericMotionResult({ status: 'failure', code: 'cancelled', message: 'cancelled', path: [] }, input)).toThrow();
  });
});

describe('generic native/browser isolation and cancellation', () => {
  let workerConstructor: ReturnType<typeof vi.fn>;
  beforeEach(() => { tauri.invoke.mockReset(); tauri.isTauri.mockReturnValue(true); workerConstructor = vi.fn(); vi.stubGlobal('Worker', workerConstructor); });
  afterEach(() => { vi.useRealTimers(); vi.unstubAllGlobals(); });

  it('calls the native generic foreground planner and never falls back after rejection', async () => {
    tauri.invoke.mockResolvedValue(result());
    expect((await solveGenericMotion(request())).status).toBe('success');
    expect(tauri.invoke).toHaveBeenCalledWith('solve_generic_motion', expect.objectContaining({ requestId: expect.any(String), request: request() }));
    tauri.invoke.mockRejectedValue('native unavailable');
    await expect(solveGenericMotion(request())).rejects.toThrow('native unavailable');
    expect(workerConstructor).not.toHaveBeenCalled();
    const corrupt = result(); corrupt.path[1].pose.position[0] += 1;
    tauri.invoke.mockResolvedValue(corrupt);
    await expect(solveGenericMotion(request())).rejects.toThrow('复核');
  });

  it('preserves the original request despite caller edits while native work is pending', async () => {
    let finish: (value: unknown) => void = () => {};
    tauri.invoke.mockImplementation(() => new Promise(resolve => { finish = resolve; }));
    const input = request(), expected = result(input), pending = solveGenericMotion(input);
    input.chain.steps[0].origin[3] += 100; input.targetJoints[0] += 20;
    finish(expected);
    expect(await pending).toEqual(expected);
  });

  it('cancels only its own native request and ignores a late success', async () => {
    let finish: (value: unknown) => void = () => {};
    tauri.invoke.mockImplementation(command => command === 'solve_generic_motion' ? new Promise(resolve => { finish = resolve; }) : Promise.resolve());
    const controller = new AbortController(), pending = solveGenericMotion(request(), controller.signal);
    const rejected = expect(pending).rejects.toMatchObject({ name: 'AbortError' });
    const { requestId } = tauri.invoke.mock.calls[0][1];
    controller.abort(); await rejected;
    expect(tauri.invoke).toHaveBeenCalledWith('cancel_motion', { requestId });
    finish(result()); await Promise.resolve();
    expect(workerConstructor).not.toHaveBeenCalled();
  });

  it('bounds work to fifteen seconds and does not submit already-aborted requests', async () => {
    const controller = new AbortController(); controller.abort();
    await expect(solveGenericMotion(request(), controller.signal)).rejects.toMatchObject({ name: 'AbortError' });
    expect(tauri.invoke).not.toHaveBeenCalled();
    vi.useFakeTimers();
    tauri.invoke.mockImplementation(command => command === 'solve_generic_motion' ? new Promise(() => {}) : Promise.resolve());
    const rejected = expect(solveGenericMotion(request())).rejects.toThrow('超时');
    await vi.advanceTimersByTimeAsync(15_000); await rejected;
    expect(tauri.invoke).toHaveBeenCalledWith('cancel_motion', expect.objectContaining({ requestId: expect.any(String) }));
  });

  it('uses a dedicated browser worker with request identity and terminates it on success or abort', async () => {
    tauri.isTauri.mockReturnValue(false);
    class FakeWorker {
      static instances: FakeWorker[] = [];
      onmessage: ((event: MessageEvent) => void) | null = null;
      onerror: (() => void) | null = null;
      onmessageerror: (() => void) | null = null;
      postMessage = vi.fn(); terminate = vi.fn();
      constructor() { FakeWorker.instances.push(this); }
    }
    vi.stubGlobal('Worker', FakeWorker);
    let settled = false;
    const pending = solveGenericMotion(request()).then(value => { settled = true; return value; });
    const worker = FakeWorker.instances[0], { requestId } = worker.postMessage.mock.calls[0][0];
    worker.onmessage?.({ data: { requestId: 'stale', result: result() } } as MessageEvent);
    await Promise.resolve(); expect(settled).toBe(false);
    worker.onmessage?.({ data: { requestId, result: result() } } as MessageEvent);
    expect((await pending).status).toBe('success'); expect(worker.terminate).toHaveBeenCalledOnce();
    const controller = new AbortController();
    const rejected = expect(solveGenericMotion(request(), controller.signal)).rejects.toMatchObject({ name: 'AbortError' });
    controller.abort(); await rejected;
    expect(FakeWorker.instances[1].terminate).toHaveBeenCalledOnce(); expect(tauri.invoke).not.toHaveBeenCalled();
  });
});
