import { execFileSync } from 'node:child_process';
import { existsSync } from 'node:fs';
import { resolve } from 'node:path';
import { describe, expect, it } from 'vitest';
import { validateEndpointInspectionRequest, validateEndpointSolutions } from '../src/robot/endpoint-inspection-validation';
import { forwardKinematics, inverseKinematics } from '../src/robot/kinematics';
import { ROBOT_MODELS } from '../src/robot/model';
import type { JointAngles } from '../src/robot/types';

const nativePath = process.env.EMULATION_NATIVE_CORE || resolve(process.env.CARGO_TARGET_DIR || 'target', 'debug', process.platform === 'win32' ? 'core-cli.exe' : 'core-cli');
function native(commands: unknown[]): { result?: unknown; error?: string }[] {
  if (!existsSync(nativePath)) throw new Error(`Run cargo build -p emulation-core --bin core-cli before native endpoint parity: ${nativePath}`);
  const output = execFileSync(nativePath, { input: commands.map(command => JSON.stringify(command)).join('\n') + '\n', encoding: 'utf8', maxBuffer: 4 * 1024 * 1024, timeout: 10_000 });
  const responses = output.trim().split('\n').map(line => JSON.parse(line));
  expect(responses).toHaveLength(commands.length);
  return responses;
}

describe.each(Object.values(ROBOT_MODELS))('$name actual native endpoint inspection', model => {
  it('passes browser FK/config verification and preserves all branch and turn alternatives', () => {
    const poses: JointAngles[] = [model.readyJoints, [20, 20, -40, 25, 40, 370], [-100, 45, -75, 40, -60, -370], [180, 20, -30, 0, 40, 0]];
    const requests = poses.map(seedJoints => validateEndpointInspectionRequest({ modelId: model.id, seedJoints, pose: forwardKinematics(model, seedJoints).pose }));
    const responses = native(requests.map(request => ({ op: 'inspect', request })));
    responses.forEach((response, index) => {
      expect(response.error).toBeUndefined();
      const request = requests[index];
      const actual = validateEndpointSolutions(response.result, request);
      const expected = inverseKinematics(model, request.pose, request.seedJoints);
      expect(actual.length).toBe(expected.length);
      for (const solution of actual) {
        expect(expected.some(candidate => candidate.configuration.every((part, axis) => part === solution.configuration[axis])
          && candidate.joints.every((angle, axis) => Math.abs(angle - solution.joints[axis]) < 1e-5))).toBe(true);
        expect(solution).not.toHaveProperty('path');
      }
    });
  });

  it('distinguishes unreachable endpoints from invalid native inspection requests', () => {
    const base = { modelId: model.id, pose: forwardKinematics(model, model.readyJoints).pose, seedJoints: model.readyJoints };
    const requests = [
      { ...base, pose: { ...base.pose, position: [99999, 0, 0] } },
      { ...base, pose: { ...base.pose, orientation: [2, 0, 0, 0] } },
      { ...base, seedJoints: [10000, 20, -30, 0, 40, 0] },
      { ...base, modelId: 'yaskawa-gp7' },
    ];
    const responses = native(requests.map(request => ({ op: 'inspect', request })));
    expect(responses[0].result).toEqual([]);
    expect(responses[0].error).toBeUndefined();
    responses.slice(1).forEach(response => { expect(response.error).toBeTypeOf('string'); expect(response.result).toBeUndefined(); });
  });
});
