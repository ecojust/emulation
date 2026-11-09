import { execFileSync } from 'node:child_process';
import { existsSync, readFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { DOMParser } from '@xmldom/xmldom';
import { afterAll, beforeAll, describe, expect, it, vi } from 'vitest';
import chainsJson from './fixtures/catalog/generic-chains.json';
import { forwardUrdfKinematics } from '../src/catalog/kinematics';
import { parseUrdf } from '../src/catalog/urdf';
import { buildGenericChain } from '../src/robot/generic-motion-chain';
import { validateGenericMotionResult } from '../src/robot/generic-motion-validation';
import { forwardGenericKinematics, genericJointsWithinLimits, inverseGenericKinematics, validateGenericChain } from '../src/robot/generic-kinematics';
import { planGenericMoveL } from '../src/robot/generic-planner';
import { quaternionDistanceDeg, slerpQuaternion } from '../src/robot/kinematics';
import type { GenericChain, GenericFkResult, GenericPlanRequest, GenericPlanResult, GenericPlanSuccess } from '../src/robot/generic-motion-types';
import type { JointAngles, Pose } from '../src/robot/types';

const chains = chainsJson as GenericChain[];
const seed: JointAngles = [20, 15, -25, 25, 35, 10];
const delta = [1, -0.5, 0.75, 0.5, -0.75, 1];
const nativePath = process.env.EMULATION_NATIVE_CORE || resolve(process.env.CARGO_TARGET_DIR || 'target', 'debug', process.platform === 'win32' ? 'core-cli.exe' : 'core-cli');
beforeAll(() => vi.stubGlobal('DOMParser', DOMParser));
afterAll(() => vi.unstubAllGlobals());

function request(chain: GenericChain): GenericPlanRequest {
  return { chain, startJoints: [...seed], targetJoints: seed.map((value, i) => value + delta[i]) as JointAngles, speedMmPerSecond: 100 };
}
function closeVector(actual: number[], expected: number[], tolerance = 1e-7) {
  expect(actual).toHaveLength(expected.length);
  actual.forEach((value, i) => expect(Math.abs(value - expected[i])).toBeLessThanOrEqual(tolerance));
}
function closePose(actual: Pose, expected: Pose, tolerance = 1e-7) {
  closeVector(actual.position, expected.position, tolerance);
  expect(quaternionDistanceDeg(actual.orientation, expected.orientation)).toBeLessThan(1e-4);
}
function assertCartesian(result: GenericPlanResult, input: GenericPlanRequest): asserts result is GenericPlanSuccess {
  expect(validateGenericMotionResult(result, input)).toBe(result);
  expect(result.status, JSON.stringify(result)).toBe('success');
  if (result.status !== 'success') throw new Error(result.message);
  const start = forwardGenericKinematics(input.chain, input.startJoints).pose;
  const end = forwardGenericKinematics(input.chain, input.targetJoints).pose;
  expect(result.path[0].joints).toEqual(input.startJoints);
  expect(result.path.at(-1)!.joints).toEqual(input.targetJoints);
  for (const [index, sample] of result.path.entries()) {
    closePose(sample.pose, forwardGenericKinematics(input.chain, sample.joints).pose);
    closeVector(sample.pose.position, start.position.map((value, axis) => value + (end.position[axis] - value) * sample.t), 0.001);
    expect(quaternionDistanceDeg(sample.pose.orientation, slerpQuaternion(start.orientation, end.orientation, sample.t))).toBeLessThan(0.0006);
    expect(genericJointsWithinLimits(input.chain, sample.joints)).toBe(true);
    if (index) {
      const previous = result.path[index - 1];
      sample.joints.forEach((value, axis) => {
        const change = Math.abs(value - previous.joints[axis]);
        expect(change).toBeLessThanOrEqual(15 + 1e-8);
        if (result.durationSeconds > 0) expect(change / ((sample.t - previous.t) * result.durationSeconds)).toBeLessThanOrEqual(input.chain.limits[axis].maxSpeedDeg + 1e-7);
      });
    }
  }
}
function native<T>(commands: unknown[]): T[] {
  if (!existsSync(nativePath)) throw new Error(`Native generic parity needs core-cli: ${nativePath}`);
  const output = execFileSync(nativePath, { input: commands.map(command => JSON.stringify(command)).join('\n') + '\n', encoding: 'utf8', maxBuffer: 24 * 1024 * 1024, timeout: 30_000 });
  const responses = output.trim().split('\n').map(line => JSON.parse(line));
  expect(responses).toHaveLength(commands.length);
  return responses.map(response => { expect(response.error).toBeUndefined(); return response.result as T; });
}

describe.each(chains)('$modelId actual URDF Cartesian motion', chain => {
  it('preserves real URDF fixed tool transforms and matches independent Three.js FK', () => {
    const description = parseUrdf(readFileSync(resolve('tests/fixtures/catalog', `${chain.modelId}.urdf`), 'utf8'));
    // JSON fixtures canonicalize -0; preserve the exact persisted adapter contract.
    expect(JSON.parse(JSON.stringify(buildGenericChain({ id: chain.modelId, name: description.name, description, geometries: new Map() })))).toEqual(chain);
    let state = 572983;
    const random = () => ((state = (Math.imul(state, 1664525) + 1013904223) >>> 0) / 2 ** 32);
    const joints = [seed, [0, 0, 0, 0, 0, 0] as JointAngles,
      ...Array.from({ length: 12 }, () => chain.limits.map(limit => limit.minDeg + random() * (limit.maxDeg - limit.minDeg)) as JointAngles)];
    for (const joint of joints) {
      const independentlyRendered = forwardUrdfKinematics(description, joint).toolPose;
      expect(independentlyRendered).not.toBeNull();
      closePose(forwardGenericKinematics(chain, joint).pose, independentlyRendered!);
    }
  });

  it('returns actual FK along a Cartesian straight segment and preserves exact target joints', () => {
    const input = request(chain), result = planGenericMoveL(input);
    assertCartesian(result, input);
    expect(result.path.length).toBeGreaterThan(2);
    expect(result.lengthMm).toBeGreaterThan(5);
    expect(result.warnings.join('')).toContain('未验证完整路径碰撞');
  });

  it('continues a pure wrist rotation without moving the tool position', () => {
    const input = request(chain);
    input.targetJoints = [...seed]; input.targetJoints[5] += 6.4;
    const result = planGenericMoveL(input);
    assertCartesian(result, input);
    expect(result.lengthMm).toBeLessThan(1e-8);
    expect(result.orientationTravelDeg).toBeCloseTo(6.4, 6);
    expect(result.durationSeconds).toBeGreaterThan(0);
  });

  it('handles stationary targets without inventing motion or unwinding a complete revolution', () => {
    const stationary = { ...request(chain), targetJoints: [...seed] as JointAngles };
    const result = planGenericMoveL(stationary);
    assertCartesian(result, stationary);
    expect(result.durationSeconds).toBe(0);
    const fullTurn = request(chain);
    fullTurn.startJoints[5] = -180; fullTurn.targetJoints = [...fullTurn.startJoints]; fullTurn.targetJoints[5] = 180;
    expect(planGenericMoveL(fullTurn)).toMatchObject({ status: 'failure', code: 'configuration_change' });
  });

  it('solves a fixed-orientation translation with the analytic Jacobian', () => {
    const start = forwardGenericKinematics(chain, seed).pose;
    const target: Pose = { position: [start.position[0] + 1, start.position[1] - 0.5, start.position[2] + 0.75], orientation: start.orientation };
    const solved = inverseGenericKinematics(chain, target, seed);
    expect(solved).not.toBeNull();
    closePose(forwardGenericKinematics(chain, solved!).pose, target, 0.001);
  });
});

describe('generic motion rejection and cancellation boundaries', () => {
  it('rejects malformed rigid transforms, axes, six-joint mappings and engineering bounds', () => {
    const mutations: ((chain: GenericChain) => void)[] = [
      value => { value.modelId = '机'.repeat(43); },
      value => { value.steps = Array(33).fill(value.steps[0]); },
      value => { value.steps[0].origin[15] = 0; },
      value => { value.steps[0].origin[0] *= -1; value.steps[0].origin[4] *= -1; value.steps[0].origin[8] *= -1; },
      value => { value.steps[0].origin[3] = 1e308; },
      value => { delete value.steps[0].origin[3]; },
      value => { value.steps[0].axis = [0, 0, 2]; },
      value => { value.steps[1].jointIndex = value.steps[0].jointIndex; },
      value => { value.limits[0].minDeg = value.limits[0].maxDeg; },
      value => { value.limits[0].maxSpeedDeg = 0; },
    ];
    for (const mutate of mutations) {
      const chain = structuredClone(chains[0]); mutate(chain);
      expect(() => validateGenericChain(chain)).toThrow();
      expect(planGenericMoveL(request(chain))).toMatchObject({ status: 'failure', code: 'invalid_input' });
    }
    expect(planGenericMoveL(null as unknown as GenericPlanRequest)).toMatchObject({ status: 'failure', code: 'invalid_input' });
  });

  it('rejects nonfinite/out-of-limit joints and invalid speeds without partial motion', () => {
    for (const speed of [0, 2001, Infinity, NaN]) expect(planGenericMoveL({ ...request(chains[0]), speedMmPerSecond: speed })).toMatchObject({ status: 'failure', code: 'invalid_input' });
    for (const value of [NaN, Infinity, 1e308]) {
      const input = request(chains[0]); input.targetJoints[0] = value;
      expect(planGenericMoveL(input)).toMatchObject({ status: 'failure', code: 'joint_limit' });
    }
    const input = request(chains[0]); delete (input.startJoints as number[])[0];
    expect(planGenericMoveL(input)).toMatchObject({ status: 'failure', code: 'joint_limit' });
  });

  it('rejects endpoint-reachable motion whose Cartesian interior is unreachable', () => {
    const identity = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
    const chain: GenericChain = { modelId: 'circular-six-axis-fixture',
      steps: Array.from({ length: 6 }, (_, jointIndex) => ({ origin: [...identity], axis: [0, 0, 1], jointIndex })),
      limits: Array.from({ length: 6 }, () => ({ minDeg: -360, maxDeg: 360, maxSpeedDeg: 90 })) as GenericChain['limits'] };
    chain.steps.push({ origin: [...identity], axis: [0, 0, 1], jointIndex: null }); chain.steps[6].origin[3] = 100;
    const result = planGenericMoveL({ chain, startJoints: [0, 0, 0, 0, 0, 0], targetJoints: [90, 0, 0, 0, 0, 0], speedMmPerSecond: 100 });
    expect(result).toMatchObject({ status: 'failure', code: 'path_unreachable', failedAt: expect.any(Number) });
    expect(result).not.toHaveProperty('path');
  });

  it('bounds sample counts before attempting a very long finite chain motion', () => {
    const chain = structuredClone(chains[0]);
    for (const step of chain.steps) { step.origin[3] = 100_000; step.origin[7] = 0; step.origin[11] = 0; }
    const input = request(chain); input.startJoints = [0, 0, 0, 0, 0, 0]; input.targetJoints = [90, 0, 0, 0, 0, 0];
    expect(planGenericMoveL(input)).toMatchObject({ status: 'failure', code: 'invalid_input' });
  });

  it('discards all samples when cancelled initially or during an IK iteration', () => {
    expect(planGenericMoveL(request(chains[0]), () => true)).toMatchObject({ status: 'failure', code: 'cancelled' });
    let checks = 0;
    const result = planGenericMoveL(request(chains[0]), () => ++checks >= 50);
    expect(checks).toBe(50);
    expect(result).toMatchObject({ status: 'failure', code: 'cancelled' });
    expect(result).not.toHaveProperty('path');
  });
});

describe('generic Rust/browser parity with real catalog chains', () => {
  it('matches FK, Cartesian samples, endpoint branches and velocity limits in the actual native binary', () => {
    const requests = chains.flatMap(chain => {
      const moving = request(chain);
      const rotation = request(chain); rotation.targetJoints = [...seed]; rotation.targetJoints[5] += 6.4;
      const wound = request(chain); wound.startJoints[5] = -180; wound.targetJoints = [...wound.startJoints]; wound.targetJoints[5] = 180;
      return [moving, rotation, wound];
    });
    const forward = native<GenericFkResult>(chains.map(chain => ({ op: 'generic_fk', chain, joints: seed })));
    forward.forEach((result, index) => {
      const expected = forwardGenericKinematics(chains[index], seed);
      closePose(result.pose, expected.pose);
      closeVector(result.transform, expected.transform);
      result.jointOrigins.forEach((origin, axis) => closeVector(origin, expected.jointOrigins[axis]));
      result.jointAxes.forEach((axis, index) => closeVector(axis, expected.jointAxes[index]));
    });
    const results = native<GenericPlanResult>(requests.map(input => ({ op: 'generic_plan', request: input })));
    results.forEach((actual, index) => {
      const input = requests[index], expected = planGenericMoveL(input);
      expect(validateGenericMotionResult(actual, input)).toBe(actual);
      expect(actual.status).toBe(expected.status);
      if (actual.status === 'failure' && expected.status === 'failure') {
        expect(actual.code).toBe(expected.code); expect(actual.failedAt).toBe(expected.failedAt); return;
      }
      assertCartesian(actual, input); assertCartesian(expected, input);
      expect(actual.path).toHaveLength(expected.path.length);
      expect(actual.durationSeconds).toBeCloseTo(expected.durationSeconds, 6);
      actual.path.forEach((sample, i) => {
        closeVector(sample.joints, expected.path[i].joints, 1e-5);
        closePose(sample.pose, expected.path[i].pose, 1e-5);
      });
    });
  });
});
