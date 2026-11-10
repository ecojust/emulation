import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { forwardKinematics, quaternionDistanceDeg, quaternionToMatrix, multiplyMatrices, matrixToQuaternion } from '../src/robot/kinematics';
import { getRobotModel } from '../src/robot/model';
import genericChains from './fixtures/catalog/generic-chains.json';
import { forwardGenericKinematics } from '../src/robot/generic-kinematics';
import type { GenericChain } from '../src/robot/generic-motion-types';
import { planGrindingReference } from '../src/robot/grinding-reference';
import { validateGrindingRequest, validateGrindingResult } from '../src/robot/grinding-validation';
import { parseGrindingTargets, formatGrindingTargets } from '../src/workspace/PathPlanningDialog';
import type { GrindingRequest, GrindingSuccess } from '../src/robot/grinding-types';
import type { Pose, Vec3 } from '../src/robot/types';
const transport = vi.hoisted(() => ({ invoke: vi.fn(), isTauri: vi.fn(() => true) }));
vi.mock('@tauri-apps/api/core', () => transport);
import { planGrindingPath } from '../src/robot/grinding';
const identity: Pose = { position: [0, 0, 0], orientation: [1, 0, 0, 0] };
function input(): GrindingRequest {
  const model = getRobotModel('irb1100-4-0.58'), start = forwardKinematics(model, model.readyJoints).pose;
  return { robot: { kind: 'abb', modelId: model.id }, basePose: structuredClone(identity), toolPose: structuredClone(identity), toolRadiusMm: 3,
    startJoints: [...model.readyJoints], targets: [{ position: [start.position[0], 5, start.position[2]], orientation: start.orientation }, { position: [start.position[0], 15, start.position[2]], orientation: start.orientation }], orientation: start.orientation, clearanceMm: 2, speedMmPerSecond: 100, zones: [] };
}
function success(request = input()): GrindingSuccess { const result = planGrindingReference(request); if (result.status !== 'success') throw new Error(result.message); return result; }
function compose(a: Pose, b: Pose): Pose {
  const mat = (pose: Pose) => { const m = quaternionToMatrix(pose.orientation); pose.position.forEach((v, i) => { m[i * 4 + 3] = v; }); return m; };
  const m = multiplyMatrices(mat(a), mat(b)); return { position: [m[3], m[7], m[11]], orientation: matrixToQuaternion(m) };
}
describe('grinding reference geometry and immutable inputs', () => {
  it('keeps all processing targets and one actual TCP orientation without mutating their input poses', () => {
    const request = input(), original = structuredClone(request), result = success(request);
    expect(request).toEqual(original); expect(result.waypoints.map(p => p.targetIndex)).toEqual([0, 1]);
    for (const point of result.waypoints) expect(quaternionDistanceDeg(point.pose.orientation, request.orientation!)).toBeLessThan(0.002);
    expect(result.checkedIntervals).toBeGreaterThan(0); expect(validateGrindingResult(result, request)).toEqual(result);
  });
  it('includes both translated rotated base and nonzero rotated tool offset in actual world TCP results', () => {
    const request = input(), model = getRobotModel('irb1100-4-0.58');
    request.basePose = { position: [1200, -700, 100], orientation: [Math.SQRT1_2, 0, 0, Math.SQRT1_2] };
    request.toolPose = { position: [20, 0, 40], orientation: [Math.SQRT1_2, 0, 0, Math.SQRT1_2] };
    const start = compose(compose(request.basePose, forwardKinematics(model, request.startJoints).pose), request.toolPose);
    request.orientation = start.orientation;
    request.targets = [5, 10].map(offset => ({ position: start.position.map((v, i) => i === 0 ? v - offset : v) as Vec3, orientation: start.orientation }));
    const result = success(request); expect(validateGrindingResult(result, request).status).toBe('success');
    expect(Math.hypot(...result.path[0].pose.position.map((v, i) => v - start.position[i]))).toBeLessThan(1e-6);
  });
  it('rejects a blocked robot or tool and never returns partial trajectories', () => {
    const request = input(); request.zones = [{ id: 'blocked', enabled: true, centerMm: [0, 0, 400], sizeMm: [4000, 4000, 4000] }];
    const result = planGrindingReference(request); expect(result.status).toBe('failure'); expect(result).not.toHaveProperty('path');
    request.zones[0].enabled = false; expect(planGrindingReference(request).status).toBe('success');
  });
  it('stops cancelled or invalid jobs before returning a purported safe path', () => {
    expect(planGrindingReference(input(), () => true)).toMatchObject({ status: 'failure', code: 'cancelled' });
    for (const patch of [{ targets: [] }, { clearanceMm: -1 }, { speedMmPerSecond: Infinity }, { toolRadiusMm: -1 }]) expect(planGrindingReference({ ...input(), ...patch }).status).toBe('failure');
  });
  it('plans each real six-axis URDF chain with complete declared rigid-body envelopes', () => {
    for (const raw of genericChains) {
      const chain = structuredClone(raw) as GenericChain, request = input();
      request.startJoints = [20, 15, -25, 25, 35, 10];
      const bodies = [0, ...chain.steps.flatMap((step, index) => step.jointIndex === null ? [] : [index + 1])].map(stepIndex => ({ stepIndex, centerMm: [0, 0, 0] as Vec3, halfSizeMm: [1, 1, 1] as Vec3 }));
      request.robot = { kind: 'generic', chain, bodies };
      const start = forwardGenericKinematics(chain, request.startJoints).pose;
      request.orientation = start.orientation; request.targets = [0.5, 1].map(offset => ({ position: [start.position[0], start.position[1] + offset, start.position[2]], orientation: start.orientation }));
      const result = success(request); expect(result.waypoints.every(point => point.configuration === null)).toBe(true);
      expect(validateGrindingResult(result, request).status).toBe('success');
      request.robot.bodies.pop(); expect(planGrindingReference(request).status).toBe('failure');
    }
  });
  it('rejects non-finite duration caused by a finite but subnormal joint speed', () => {
    const request = input(), chain = structuredClone(genericChains[0]) as GenericChain;
    request.startJoints = [10, 20, -25, 15, 30, -10];
    const bodies = [0, ...chain.steps.flatMap((step, index) => step.jointIndex === null ? [] : [index + 1])].map(stepIndex => ({ stepIndex, centerMm: [0, 0, 0] as Vec3, halfSizeMm: [1, 1, 1] as Vec3 }));
    request.robot = { kind: 'generic', chain, bodies };
    const start = forwardGenericKinematics(chain, request.startJoints).pose;
    request.orientation = start.orientation; request.targets = [{ position: [start.position[0], start.position[1] + 2, start.position[2]], orientation: start.orientation }];
    for (const limit of chain.limits) limit.maxSpeedDeg = 1e-320;
    const result = planGrindingReference(request);
    expect(result).toMatchObject({ status: 'failure', code: 'invalid_input' }); expect(result).not.toHaveProperty('path');
  });
  it('validates stationary and repeated end targets in order while retaining the strict endpoint check', () => {
    const request = input(), start = forwardKinematics(getRobotModel('irb1100-4-0.58'), request.startJoints).pose;
    for (const targets of [[start], [start, start], [request.targets[0], request.targets[0]], [start, request.targets[0], request.targets[0]]]) {
      const revised = { ...request, targets }, result = success(revised);
      expect(validateGrindingResult(result, revised).status).toBe('success');
      expect(result.waypoints[0].kind).toBe('transfer');
      expect(result.waypoints.slice(1).every(point => point.kind === 'process')).toBe(true);
      const corrupt = structuredClone(result); corrupt.path.at(-1)!.pose.position[0] += 1;
      expect(() => validateGrindingResult(corrupt, revised)).toThrow();
      const omitted = structuredClone(result); omitted.path.pop();
      expect(() => validateGrindingResult(omitted, revised)).toThrow();
    }
  });
  it('parses only complete finite pose rows and preserves world coordinates', () => {
    const targets = input().targets; expect(parseGrindingTargets(formatGrindingTargets(targets))).toHaveLength(2);
    expect(() => parseGrindingTargets('0 0 0 1 0 0 0')).toThrow();
    expect(() => parseGrindingTargets('0 0 0 1 0 0 0\n1 2 3 0 0 0 0')).toThrow();
    expect(() => parseGrindingTargets('0 0 0 1 0 0 0\n1 2 Infinity 1 0 0 0')).toThrow();
    expect(() => parseGrindingTargets('0 0 0 1 0 0 0\n1,2,3,1,0,0,')).toThrow();
  });
  it('rejects a forged successful response with wrong actual TCP, missing target or joint jump', () => {
    const request = input();
    for (const mutate of [(r: GrindingSuccess) => { r.path[1].pose.position[0] += 1; }, (r: GrindingSuccess) => { r.waypoints.pop(); }, (r: GrindingSuccess) => { r.path[1].joints[0] += 30; }, (r: GrindingSuccess) => { r.orientation = [1, 0, 0, 0]; }]) {
      const result = success(request); mutate(result); expect(() => validateGrindingResult(result, request)).toThrow();
    }
    const requestCopy = validateGrindingRequest(request); request.targets[0].position[0] = 999; expect(requestCopy.targets[0].position[0]).not.toBe(999);
  });
});
describe('continuous tool envelope proof and detours', () => {
  function thinObstacle(): GrindingRequest {
    const request = input(), model = getRobotModel('irb1100-4-0.58');
    request.toolPose.position = [0, 0, 200]; request.toolRadiusMm = 0.02; request.clearanceMm = 0;
    const start = compose(forwardKinematics(model, request.startJoints).pose, request.toolPose);
    request.orientation = start.orientation; request.targets = [{ position: [start.position[0], 8, start.position[2]], orientation: start.orientation }];
    request.zones = [{ id: 'thin', enabled: true, centerMm: [start.position[0], 1, start.position[2]], sizeMm: [0.01, 0.01, 0.01] }];
    return request;
  }
  it('rejects a thin obstacle between individually clear endpoints, including the full tool body', () => {
    const request = thinObstacle(); request.allowDetours = false;
    const free = success({ ...request, zones: [] });
    expect(planGrindingReference({ ...request, targets: [free.path[0].pose] }).status).toBe('success');
    expect(planGrindingReference({ ...request, startJoints: free.path.at(-1)!.joints }).status).toBe('success');
    expect(planGrindingReference(request)).toMatchObject({ status: 'failure', code: 'collision' });
  });
  it('adds explicit transfer points while retaining process targets and fixed tool direction', () => {
    const request = thinObstacle(), result = success(request);
    expect(result.waypoints.some(point => point.kind === 'transfer' && point.targetIndex === null)).toBe(true);
    expect(result.waypoints.at(-1)!.targetIndex).toBe(0);
    expect(result.waypoints.every(point => point.kind === 'transfer')).toBe(true);
    expect(validateGrindingResult(result, request).status).toBe('success');
    const model = getRobotModel('irb1100-4-0.58');
    for (let index = 1; index < result.path.length; index++) {
      const before = result.path[index - 1].joints, after = result.path[index].joints;
      const midpoint = before.map((value, axis) => (value + after[axis]) / 2) as typeof before;
      const pose = compose(forwardKinematics(model, midpoint).pose, request.toolPose);
      expect(quaternionDistanceDeg(pose.orientation, request.orientation!)).toBeLessThanOrEqual(0.1 + 1e-8);
    }
  });
});
describe('grinding transport cancellation and native isolation' , () => {
  let worker: ReturnType<typeof vi.fn>;
  beforeEach(() => { transport.invoke.mockReset(); transport.isTauri.mockReturnValue(true); worker = vi.fn(); vi.stubGlobal('Worker', worker); });
  afterEach(() => { vi.unstubAllGlobals(); vi.useRealTimers(); });
  it('uses native jobs and rejects native failures without browser fallback', async () => {
    transport.invoke.mockResolvedValue(success()); expect((await planGrindingPath(input())).status).toBe('success');
    expect(transport.invoke).toHaveBeenCalledWith('plan_grinding_path', expect.objectContaining({ requestId: expect.any(String) }));
    transport.invoke.mockRejectedValue('engine offline'); await expect(planGrindingPath(input())).rejects.toThrow('engine offline'); expect(worker).not.toHaveBeenCalled();
  });
  it('cancels only its own request and ignores late success', async () => {
    let finish: (value: unknown) => void = () => {};
    transport.invoke.mockImplementation(command => command === 'plan_grinding_path' ? new Promise(resolve => { finish = resolve; }) : Promise.resolve());
    const controller = new AbortController(), pending = planGrindingPath(input(), controller.signal), check = expect(pending).rejects.toMatchObject({ name: 'AbortError' });
    const { requestId } = transport.invoke.mock.calls[0][1]; controller.abort(); await check;
    expect(transport.invoke).toHaveBeenCalledWith('cancel_motion', { requestId }); finish(success());
  });
  it('does not submit invalid or already-cancelled requests', async () => {
    await expect(planGrindingPath({ ...input(), targets: [] })).rejects.toThrow();
    const controller = new AbortController(); controller.abort(); await expect(planGrindingPath(input(), controller.signal)).rejects.toMatchObject({ name: 'AbortError' });
    expect(transport.invoke).not.toHaveBeenCalled(); expect(worker).not.toHaveBeenCalled();
  });
  it('times out and cancels a native task without accepting a later reply', async () => {
    vi.useFakeTimers(); transport.invoke.mockImplementation(() => new Promise(() => {}));
    const pending = planGrindingPath(input()), rejected = expect(pending).rejects.toThrow('超时');
    const requestId = transport.invoke.mock.calls[0][1].requestId;
    await vi.advanceTimersByTimeAsync(60001); await rejected;
    expect(transport.invoke).toHaveBeenCalledWith('cancel_motion', { requestId }); expect(worker).not.toHaveBeenCalled();
  });
  it('isolates browser worker responses by ID and terminates after cancellation', async () => {
    transport.isTauri.mockReturnValue(false);
    const instance = { onmessage: null as null | ((event: MessageEvent) => void), onerror: null, onmessageerror: null, postMessage: vi.fn(), terminate: vi.fn() };
    worker.mockImplementation(function () { return instance; });
    const controller = new AbortController(), pending = planGrindingPath(input(), controller.signal), rejected = expect(pending).rejects.toMatchObject({ name: 'AbortError' });
    const sent = instance.postMessage.mock.calls[0][0];
    instance.onmessage!({ data: { requestId: 'stale', result: success() } } as MessageEvent);
    expect(instance.terminate).not.toHaveBeenCalled(); controller.abort(); await rejected;
    expect(instance.terminate).toHaveBeenCalledTimes(1);
    instance.onmessage!({ data: { requestId: sent.requestId, result: success() } } as MessageEvent);
    expect(transport.invoke).not.toHaveBeenCalled();
  });

});
