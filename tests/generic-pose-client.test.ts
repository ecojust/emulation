import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import chainsJson from './fixtures/catalog/generic-chains.json';
import { forwardGenericKinematics } from '../src/robot/generic-kinematics';
import { genericSolution, planGenericPoseMoveL } from '../src/robot/generic-pose-algorithms';
import type { GenericChain, GenericEndpointRequest, GenericPosePlanRequest } from '../src/robot/generic-motion-types';
const tauri = vi.hoisted(() => ({ invoke: vi.fn(), isTauri: vi.fn(() => true) }));
vi.mock('@tauri-apps/api/core', () => tauri);
import { inspectGenericEndpoint } from '../src/robot/generic-endpoint';
import { solveGenericPoseMotion } from '../src/robot/generic-pose-motion';
function poseInput(): GenericPosePlanRequest {
  const chain = structuredClone(chainsJson[0]) as GenericChain, startJoints: GenericPosePlanRequest['startJoints'] = [20, 15, -25, 25, 35, 10];
  const start = forwardGenericKinematics(chain, startJoints).pose;
  return { chain, startJoints, target: { position: [start.position[0] + 2, start.position[1], start.position[2]], orientation: start.orientation }, speedMmPerSecond: 100 };
}
function endpointInput(): GenericEndpointRequest { const request = poseInput(); return { chain: request.chain, seedJoints: request.startJoints, pose: forwardGenericKinematics(request.chain, request.startJoints).pose }; }
function endpointResult() { const request = endpointInput(); return [genericSolution(request.seedJoints, request.pose, request.pose, request.seedJoints)]; }

describe.each([
  { command: 'solve_generic_pose_motion', request: poseInput, invoke: (input: unknown, signal?: AbortSignal) => solveGenericPoseMotion(input as GenericPosePlanRequest, signal), result: () => planGenericPoseMoveL(poseInput()), timeout: 15000 },
  { command: 'inspect_generic_endpoint', request: endpointInput, invoke: (input: unknown, signal?: AbortSignal) => inspectGenericEndpoint(input as GenericEndpointRequest, signal), result: endpointResult, timeout: 5000 },
])('$command bounded native/browser isolation', api => {
  let workerConstructor: ReturnType<typeof vi.fn>;
  beforeEach(() => { tauri.invoke.mockReset(); tauri.isTauri.mockReturnValue(true); workerConstructor = vi.fn(); vi.stubGlobal('Worker', workerConstructor); });
  afterEach(() => { vi.useRealTimers(); vi.unstubAllGlobals(); });
  it('uses real IPC command and rejects native failures without worker fallback', async () => {
    tauri.invoke.mockResolvedValue(api.result()); expect(await api.invoke(api.request())).toEqual(api.result());
    expect(tauri.invoke).toHaveBeenCalledWith(api.command, expect.objectContaining({ requestId: expect.any(String) }));
    tauri.invoke.mockRejectedValue('native unavailable'); await expect(api.invoke(api.request())).rejects.toThrow('native unavailable'); expect(workerConstructor).not.toHaveBeenCalled();
  });
  it('owns an immutable request copy and cancels only its request with late responses ignored', async () => {
    let finish: (value: unknown) => void = () => {};
    tauri.invoke.mockImplementation(command => command === api.command ? new Promise(resolve => { finish = resolve; }) : Promise.resolve());
    const input = api.request(), snapshot = structuredClone(input), controller = new AbortController(), pending = api.invoke(input, controller.signal);
    const rejection = expect(pending).rejects.toMatchObject({ name: 'AbortError' });
    input.chain.steps[0].origin[3] += 100;
    expect(tauri.invoke.mock.calls[0][1].request.chain).toEqual(snapshot.chain);
    const requestId = tauri.invoke.mock.calls[0][1].requestId; controller.abort(); await rejection;
    expect(tauri.invoke).toHaveBeenCalledWith('cancel_motion', { requestId }); finish(api.result()); await Promise.resolve();
    expect(workerConstructor).not.toHaveBeenCalled();
  });
  it('bounds the native wait and cancels after timeout', async () => {
    vi.useFakeTimers(); tauri.invoke.mockImplementation(command => command === api.command ? new Promise(() => {}) : Promise.resolve());
    const pending = api.invoke(api.request()), rejection = expect(pending).rejects.toThrow('超时');
    await vi.advanceTimersByTimeAsync(api.timeout); await rejection; expect(tauri.invoke).toHaveBeenCalledWith('cancel_motion', expect.anything());
  });
  it('rejects malformed input before native or browser transport', async () => {
    const request = api.request(); request.chain.steps[0].origin[0] = 2;
    await expect(api.invoke(request)).rejects.toThrow(); expect(tauri.invoke).not.toHaveBeenCalled(); expect(workerConstructor).not.toHaveBeenCalled();
  });
  it('terminates the browser worker and ignores stale request IDs', async () => {
    tauri.isTauri.mockReturnValue(false);
    const worker = { postMessage: vi.fn(), terminate: vi.fn(), onmessage: (_event: unknown) => {}, onerror: () => {}, onmessageerror: () => {} };
    workerConstructor.mockImplementation(() => worker);
    const pending = api.invoke(api.request()), requestId = worker.postMessage.mock.calls[0][0].requestId;
    worker.onmessage({ data: { requestId: 'stale', result: api.result() } }); expect(worker.terminate).not.toHaveBeenCalled();
    worker.onmessage({ data: { requestId, result: api.result() } }); expect(await pending).toEqual(api.result()); expect(worker.terminate).toHaveBeenCalledOnce(); expect(tauri.invoke).not.toHaveBeenCalled();
  });
  it('aborts a browser worker and rejects malformed reported success', async () => {
    tauri.isTauri.mockReturnValue(false);
    const worker = { postMessage: vi.fn(), terminate: vi.fn(), onmessage: (_event: unknown) => {}, onerror: () => {}, onmessageerror: () => {} };
    workerConstructor.mockImplementation(() => worker);
    const controller = new AbortController(), pending = api.invoke(api.request(), controller.signal), rejection = expect(pending).rejects.toMatchObject({ name: 'AbortError' });
    controller.abort(); await rejection; expect(worker.terminate).toHaveBeenCalledOnce();
    const next = api.invoke(api.request()), failed = expect(next).rejects.toThrow();
    worker.onmessage({ data: { requestId: worker.postMessage.mock.calls[1][0].requestId, result: { status: 'success', path: [] } } }); await failed;
  });
});
