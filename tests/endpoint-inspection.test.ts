import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { forwardKinematics, inverseKinematics, quaternionDistanceDeg } from '../src/robot/kinematics';
import { ROBOT_MODELS } from '../src/robot/model';
import { validateEndpointInspectionRequest, validateEndpointSolutions } from '../src/robot/endpoint-inspection-validation';
import type { EndpointInspectionRequest } from '../src/robot/endpoint-inspection-validation';
import type { IKSolution } from '../src/robot/types';

const tauri = vi.hoisted(() => ({ invoke: vi.fn(), isTauri: vi.fn(() => true) }));
vi.mock('@tauri-apps/api/core', () => tauri);
import { inspectEndpoint } from '../src/robot/endpoint-inspection';

const model = ROBOT_MODELS['irb1100-4-0.58'];
function request(): EndpointInspectionRequest {
  return { modelId: model.id, pose: forwardKinematics(model, model.readyJoints).pose, seedJoints: [...model.readyJoints] };
}
function solutions(input = request()): IKSolution[] { return inverseKinematics(model, input.pose, input.seedJoints); }

describe('independent endpoint result verification', () => {
  it('keeps all stable IK alternatives without constructing a motion path', () => {
    const input = validateEndpointInspectionRequest(request());
    const result = validateEndpointSolutions(solutions(input), input);
    expect(result.length).toBeGreaterThan(8);
    expect(result[0]).not.toHaveProperty('path');
    expect(validateEndpointSolutions([], input)).toEqual([]);
  });

  it('bounds input, accepts only supported models and normalizes without mutating the point', () => {
    const input = request();
    input.pose.orientation = input.pose.orientation.map(component => component * 1.01) as typeof input.pose.orientation;
    const original = structuredClone(input);
    expect(validateEndpointInspectionRequest(input).pose.orientation).not.toEqual(input.pose.orientation);
    expect(input).toEqual(original);
    for (const invalid of [
      { ...request(), modelId: 'yaskawa-gp7' },
      { ...request(), pose: { ...request().pose, position: [Infinity, 0, 0] } },
      { ...request(), pose: { ...request().pose, position: [1_000_000.01, 0, 0] } },
      { ...request(), pose: { ...request().pose, orientation: [2, 0, 0, 0] } },
      { ...request(), seedJoints: [231, 20, -30, 0, 40, 0] },
      { ...request(), seedJoints: new Array(6) },
      { ...request(), pose: { ...request().pose, position: new Array(3) } },
      { ...request(), success: true },
    ]) expect(() => validateEndpointInspectionRequest(invalid)).toThrow();
  });

  it('rejects false FK, configuration, errors, score, IDs and duplicate alternatives', () => {
    const input = request();
    for (const corrupt of [
      (value: IKSolution[]) => { value[0].joints[0] += 1; },
      (value: IKSolution[]) => { delete (value[0].joints as number[])[1]; },
      (value: IKSolution[]) => { value[0].pose.position[0] += 0.001; },
      (value: IKSolution[]) => { value[0].configuration[3] ^= 1; },
      (value: IKSolution[]) => { value[0].positionErrorMm = 0.009; },
      (value: IKSolution[]) => { value[0].orientationErrorDeg = NaN; },
      (value: IKSolution[]) => { value[0].score += 10; },
      (value: IKSolution[]) => { value[0].id = ' '; },
      (value: IKSolution[]) => { value[1].id = value[0].id; },
      (value: IKSolution[]) => { value[1] = { ...structuredClone(value[0]), id: 'duplicated-joints' }; },
    ]) {
      const value = solutions(input);
      corrupt(value);
      expect(() => validateEndpointSolutions(value, input)).toThrow();
    }
    expect(() => validateEndpointSolutions(Array.from({ length: 257 }, () => solutions()[0]), input)).toThrow();
  });
});

describe('endpoint transport cancellation and native isolation', () => {
  let workerConstructor: ReturnType<typeof vi.fn>;
  beforeEach(() => { tauri.invoke.mockReset(); tauri.isTauri.mockReturnValue(true); workerConstructor = vi.fn(); vi.stubGlobal('Worker', workerConstructor); });
  afterEach(() => { vi.useRealTimers(); vi.unstubAllGlobals(); });

  it('uses native inspection and verifies candidates without a browser fallback', async () => {
    tauri.invoke.mockResolvedValue(solutions());
    expect((await inspectEndpoint(request())).length).toBeGreaterThan(8);
    expect(tauri.invoke).toHaveBeenCalledWith('inspect_endpoint', expect.objectContaining({ requestId: expect.any(String), request: expect.objectContaining({ modelId: model.id, seedJoints: model.readyJoints }) }));
    expect(quaternionDistanceDeg(tauri.invoke.mock.calls[0][1].request.pose.orientation, request().pose.orientation)).toBeLessThan(1e-5);
    expect(workerConstructor).not.toHaveBeenCalled();
    tauri.invoke.mockRejectedValue('native unavailable');
    await expect(inspectEndpoint(request())).rejects.toThrow('native unavailable');
    expect(workerConstructor).not.toHaveBeenCalled();
  });

  it('rejects native candidates that claim the wrong pose', async () => {
    const value = solutions(); value[0].pose.position[0] += 1;
    tauri.invoke.mockResolvedValue(value);
    await expect(inspectEndpoint(request())).rejects.toThrow('复核');
  });

  it('aborts only its own native request and ignores a late result', async () => {
    let finish: (value: unknown) => void = () => {};
    tauri.invoke.mockImplementation(command => command === 'inspect_endpoint' ? new Promise(resolve => { finish = resolve; }) : Promise.resolve());
    const controller = new AbortController();
    const pending = inspectEndpoint(request(), controller.signal);
    const rejection = expect(pending).rejects.toMatchObject({ name: 'AbortError' });
    const { requestId } = tauri.invoke.mock.calls[0][1];
    controller.abort();
    await rejection;
    expect(tauri.invoke).toHaveBeenCalledWith('cancel_motion', { requestId });
    finish(solutions());
    await Promise.resolve();
    expect(workerConstructor).not.toHaveBeenCalled();
  });

  it('times out bounded background work and never submits an already aborted request', async () => {
    const controller = new AbortController(); controller.abort();
    await expect(inspectEndpoint(request(), controller.signal)).rejects.toMatchObject({ name: 'AbortError' });
    expect(tauri.invoke).not.toHaveBeenCalled();
    vi.useFakeTimers();
    tauri.invoke.mockImplementation(command => command === 'inspect_endpoint' ? new Promise(() => {}) : Promise.resolve());
    const pending = inspectEndpoint(request());
    const rejection = expect(pending).rejects.toThrow('超时');
    await vi.advanceTimersByTimeAsync(5000);
    await rejection;
    expect(tauri.invoke).toHaveBeenCalledWith('cancel_motion', expect.objectContaining({ requestId: expect.any(String) }));
    expect(workerConstructor).not.toHaveBeenCalled();
  });

  it('uses a dedicated browser worker, ignores another request ID and terminates cancelled work', async () => {
    tauri.isTauri.mockReturnValue(false);
    class FakeWorker {
      static instances: FakeWorker[] = [];
      onmessage: ((event: MessageEvent) => void) | null = null;
      onerror: (() => void) | null = null;
      onmessageerror: (() => void) | null = null;
      postMessage = vi.fn();
      terminate = vi.fn();
      constructor() { FakeWorker.instances.push(this); }
    }
    vi.stubGlobal('Worker', FakeWorker);
    let settled = false;
    const pending = inspectEndpoint(request()).then(value => { settled = true; return value; });
    const worker = FakeWorker.instances[0];
    const { requestId } = worker.postMessage.mock.calls[0][0];
    worker.onmessage?.({ data: { requestId: 'stale', solutions: solutions() } } as MessageEvent);
    await Promise.resolve();
    expect(settled).toBe(false);
    worker.onmessage?.({ data: { requestId, solutions: solutions() } } as MessageEvent);
    expect((await pending).length).toBeGreaterThan(8);
    expect(worker.terminate).toHaveBeenCalledOnce();
    const controller = new AbortController();
    const cancelled = inspectEndpoint(request(), controller.signal);
    const rejection = expect(cancelled).rejects.toMatchObject({ name: 'AbortError' });
    controller.abort();
    await rejection;
    expect(FakeWorker.instances[1].terminate).toHaveBeenCalledOnce();
    expect(tauri.invoke).not.toHaveBeenCalled();
  });
});
