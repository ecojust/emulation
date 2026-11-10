import { execFileSync } from 'node:child_process';
import { resolve } from 'node:path';
import { describe, expect, it } from 'vitest';
import chainsJson from './fixtures/catalog/generic-chains.json';
import { composePose, identityPose } from '../src/workspace/frame-graph';
import { forwardGenericKinematics, GenericMotionCancelled } from '../src/robot/generic-kinematics';
import { inspectGenericEndpointReference, planGenericPoseMoveL } from '../src/robot/generic-pose-algorithms';
import { validateGenericEndpointRequest, validateGenericEndpointResult, validateGenericPoseRequest, validateGenericPoseResult } from '../src/robot/generic-pose-validation';
import { quaternionDistanceDeg } from '../src/robot/kinematics';
import type { GenericChain, GenericEndpointRequest, GenericIkSolution, GenericPosePlanRequest, GenericPosePlanResult, GenericPosePlanSuccess } from '../src/robot/generic-motion-types';
import type { JointAngles, Pose } from '../src/robot/types';
const chains = chainsJson as GenericChain[], seed: JointAngles = [20, 15, -25, 25, 35, 10];
const base: Pose = { position: [350, -180, 220], orientation: [Math.cos(0.4), 0, 0, Math.sin(0.4)] };
const tool: Pose = { position: [120, 35, 170], orientation: [Math.cos(0.15), Math.sin(0.15), 0, 0] };
function input(chain: GenericChain): GenericPosePlanRequest {
  const end = seed.map((v, i) => v + [0.4, -0.3, 0.5, 0.4, -0.2, 0.6][i]) as JointAngles;
  return { chain, startJoints: seed, target: composePose(composePose(base, forwardGenericKinematics(chain, end).pose), tool), basePose: base, toolPose: tool, speedMmPerSecond: 100 };
}
function success(request: GenericPosePlanRequest, result = planGenericPoseMoveL(request)): GenericPosePlanSuccess {
  expect(validateGenericPoseResult(result, request)).toBe(result);
  expect(result.status, JSON.stringify(result)).toBe('success'); if (result.status !== 'success') throw new Error(result.message); return result;
}
const nativePath = process.env.EMULATION_NATIVE_CORE || resolve(process.env.CARGO_TARGET_DIR || 'target', 'debug', 'core-cli');
function native<T>(op: string, request: unknown): T {
  const output = execFileSync(nativePath, { input: `${JSON.stringify({ op, request })}\n`, encoding: 'utf8', timeout: 30_000, maxBuffer: 4 * 1024 * 1024 });
  const response = JSON.parse(output); expect(response.error).toBeUndefined(); return response.result;
}

describe.each(chains)('$modelId pose-based motion and endpoints', chain => {
  it('keeps world TCP straight with rotated base and offset/rotated tool, including native parity', () => {
    const request = input(chain), reference = success(request), actual = success(request, native<GenericPosePlanResult>('generic_pose_plan', request));
    expect(actual.path).toHaveLength(reference.path.length);
    actual.path.forEach((sample, i) => sample.joints.forEach((v, axis) => expect(Math.abs(v - reference.path[i].joints[axis])).toBeLessThan(1e-6)));
    expect(actual.selected.pose.position).toEqual(actual.path.at(-1)!.pose.position);
  });
  it('rotates at fixed TCP without treating the offset flange as a straight stationary point', () => {
    const request = input(chain), start = composePose(composePose(base, forwardGenericKinematics(chain, seed).pose), tool);
    request.target = { position: start.position, orientation: composePose(start, { position: [0, 0, 0], orientation: [Math.cos(0.025), 0, 0, Math.sin(0.025)] }).orientation };
    const result = success(request); expect(result.lengthMm).toBeLessThan(1e-8); expect(result.orientationTravelDeg).toBeGreaterThan(2);
    for (const sample of result.path) expect(Math.hypot(...sample.pose.position.map((v, i) => v - start.position[i]))).toBeLessThanOrEqual(0.001001);
    expect(Math.hypot(...forwardGenericKinematics(chain, result.selected.joints).pose.position.map((v, i) => v - forwardGenericKinematics(chain, seed).pose.position[i]))).toBeGreaterThan(1);
    success(request, native<GenericPosePlanResult>('generic_pose_plan', request));
  });
  it('accepts XYZ-only position mode and identity frame defaults', () => {
    const start = forwardGenericKinematics(chain, seed).pose;
    const request: GenericPosePlanRequest = { chain, startJoints: seed, target: { position: [start.position[0] + 1, start.position[1], start.position[2]], orientation: null }, speedMmPerSecond: 100 };
    const result = success(request); expect(result.automaticOrientation).toBe(true);
    expect(quaternionDistanceDeg(result.selected.pose.orientation, start.orientation)).toBeLessThan(0.001);
    success(request, native<GenericPosePlanResult>('generic_pose_plan', request));
  });
  it('relaxes the direction continuously for a reachable XYZ move from mechanical zero', () => {
    const zero: JointAngles = [0, 0, 0, 0, 0, 0], start = forwardGenericKinematics(chain, zero).pose;
    const request: GenericPosePlanRequest = { chain, startJoints: zero, target: { position: [start.position[0], start.position[1] + 1, start.position[2]], orientation: null }, speedMmPerSecond: 100 };
    const result = success(request); expect(result.automaticOrientation).toBe(true);
    expect(result.orientationTravelDeg).toBeGreaterThan(0.01); expect(result.orientationTravelDeg).toBeLessThan(0.2);
    success(request, native<GenericPosePlanResult>('generic_pose_plan', request));
  });
  it('finds independent FK-checked endpoint branches without ABB configuration fields', () => {
    const request: GenericEndpointRequest = { chain, seedJoints: seed, pose: forwardGenericKinematics(chain, seed).pose };
    const result = inspectGenericEndpointReference(request), actual = native<GenericIkSolution[]>('generic_inspect', request);
    expect(validateGenericEndpointResult(result, request)).toBe(result); expect(validateGenericEndpointResult(actual, request)).toBe(actual);
    expect(result.length).toBeGreaterThan(1); expect(result.length).toBeLessThanOrEqual(32); expect(actual.length).toBe(result.length);
    expect(result[0].joints).toEqual(seed); expect(actual[0].joints).toEqual(seed);
    for (const candidate of result) { expect(candidate).not.toHaveProperty('configuration'); expect(candidate).not.toHaveProperty('robconf'); }
    expect(inspectGenericEndpointReference(request)).toEqual(result);
  });
});

it('rejects malformed frames, sparse coordinates, corrupt candidates, fabricated paths and stale frame results', () => {
  const request = input(chains[0]), result = success(request);
  expect(() => validateGenericPoseRequest({ ...request, basePose: { position: [0, 0, 0], orientation: [2, 0, 0, 0] } })).toThrow();
  const sparse = structuredClone(request); delete (sparse.target.position as number[])[1]; expect(() => validateGenericPoseRequest(sparse)).toThrow();
  const copy = validateGenericPoseRequest(request); request.basePose = identityPose(); expect(() => validateGenericPoseResult(result, request)).toThrow();
  expect(validateGenericPoseResult(result, copy)).toBe(result);
  expect(() => validateGenericPoseResult({ status: 'failure', code: 'cancelled', message: 'cancelled', selected: result.selected }, copy)).toThrow();
  for (const mutate of [(value: GenericPosePlanSuccess) => { value.path[1].pose.position[0] += 1; }, (value: GenericPosePlanSuccess) => { value.selected.score = 999; }, (value: GenericPosePlanSuccess) => { value.durationSeconds = 0; }, (value: GenericPosePlanSuccess) => { value.automaticOrientation = true; }]) {
    const bad = structuredClone(result); mutate(bad); expect(() => validateGenericPoseResult(bad, copy)).toThrow();
  }
  const endpoint = validateGenericEndpointRequest({ chain: chains[0], pose: forwardGenericKinematics(chains[0], seed).pose, seedJoints: seed });
  const candidates = inspectGenericEndpointReference(endpoint); candidates[0].joints[0] += 1; expect(() => validateGenericEndpointResult(candidates, endpoint)).toThrow();
});
it('bounds requests and returns no partial path or endpoint list on cancellation/nonconvergence', () => {
  const request = input(chains[0]); let polls = 0;
  const result = planGenericPoseMoveL(request, () => ++polls > 50); expect(result).toMatchObject({ status: 'failure', code: 'cancelled' }); expect(result).not.toHaveProperty('path');
  const endpoint = { chain: chains[0], pose: forwardGenericKinematics(chains[0], seed).pose, seedJoints: seed }; polls = 0;
  expect(() => inspectGenericEndpointReference(endpoint, () => ++polls > 100)).toThrow(GenericMotionCancelled);
  request.target.position = [900000, 0, 0]; const far = planGenericPoseMoveL(request); expect(far.status).toBe('failure'); expect(far).not.toHaveProperty('path');
  expect(inspectGenericEndpointReference({ ...endpoint, pose: { position: [900000, 0, 0], orientation: [1, 0, 0, 0] } })).toEqual([]);
});
