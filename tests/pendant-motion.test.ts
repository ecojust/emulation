import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { forwardKinematics } from '../src/robot/kinematics';
import { getRobotModel } from '../src/robot/model';
import type { JointAngles, Pose } from '../src/robot/types';
const transport = vi.hoisted(() => ({ invoke: vi.fn(), isTauri: vi.fn(() => true) }));
vi.mock('@tauri-apps/api/core', () => transport);
import { createPendantMotionController, jogPendant, validateJogRequest, validateJogResult, type JogContext, type JogRequest, type JogSuccess } from '../src/pendant/motion';
import type { PendantInput } from '../src/pendant/types';
const id: Pose = { position: [0, 0, 0], orientation: [1, 0, 0, 0] };
const initial: JointAngles = [0, 20, -30, 0, 40, 0];
const context: JogContext = { robot: { kind: 'abb', modelId: 'irb1100-4-0.58' }, basePose: id, toolPose: id, toolRadiusMm: 0, zones: [] };
const request = (): JogRequest => ({ ...structuredClone(context), startJoints: [...initial], mode: 'joint', space: 'world', axes: [1, 0, 0, 0, 0, 0], speedScale: 1, stepSeconds: .05 });
function result(input: JogRequest): JogSuccess {
  const magnitude = Math.max(1, Math.hypot(...input.axes));
  const joints = input.startJoints.map((value, index) => value + input.axes[index] / magnitude * input.speedScale * input.stepSeconds * 30) as JointAngles;
  return { status: 'success', joints, pose: forwardKinematics(getRobotModel('irb1100-4-0.58'), joints).pose };
}
function packet(sequence: number, active = true, patch: Partial<PendantInput> = {}): PendantInput {
  return { sessionId: 'session', epoch: 1, sequence, receivedAtMs: Date.now(), mode: 'joint', space: 'world', axes: active ? [1, 0, 0, 0, 0, 0] : [0, 0, 0, 0, 0, 0], speed: 1, active, ...patch };
}
afterEach(() => { vi.useRealTimers(); transport.invoke.mockReset(); transport.isTauri.mockReturnValue(true); });

describe('native pendant motion boundary', () => {
  it('validates bounded input and immutable context snapshots', () => {
    const input = request(), checked = validateJogRequest(input); input.basePose.position[0] = 999;
    expect(checked.basePose.position[0]).toBe(0);
    for (const mutate of [(r: JogRequest) => { r.stepSeconds = .051; }, (r: JogRequest) => { r.speedScale = NaN; }, (r: JogRequest) => { r.axes[0] = Infinity; }, (r: JogRequest) => { r.mode = 'linear'; r.axes[4] = .1; }, (r: JogRequest) => { r.startJoints[0] = 999; }]) {
      const bad = request(); mutate(bad); expect(() => validateJogRequest(bad)).toThrow();
    }
  });
  it('independently rejects forged joints, mismatched FK, non-unit quaternions and failure payloads containing a usable pose', () => {
    const input = request(); expect(validateJogResult(result(input), input)).toEqual(result(input));
    for (const mutate of [(r: JogSuccess) => { r.joints[2] += .1; }, (r: JogSuccess) => { r.pose.position[0] += 10; }, (r: JogSuccess) => { r.pose.orientation[0] *= 2; }]) {
      const bad = result(input); mutate(bad); expect(() => validateJogResult(bad, input)).toThrow();
    }
    expect(() => validateJogResult({ status: 'failure', code: 'x', message: 'x', joints: initial }, input)).toThrow();
  });
  it('uses the actual command contract and never falls back when native transport is absent or fails', async () => {
    transport.invoke.mockImplementation(async (command, args) => command === 'pendant_jog' ? result(args.request) : undefined);
    await expect(jogPendant(request())).resolves.toMatchObject({ status: 'success' });
    expect(transport.invoke).toHaveBeenCalledWith('pendant_jog', expect.objectContaining({ requestId: expect.any(String), request: request() }));
    transport.invoke.mockRejectedValue('offline'); await expect(jogPendant(request())).rejects.toThrow('offline');
    transport.isTauri.mockReturnValue(false); transport.invoke.mockClear(); await expect(jogPendant(request())).rejects.toThrow('桌面客户端'); expect(transport.invoke).not.toHaveBeenCalled();
  });
  it('cancels only its own native work and discards a late valid result', async () => {
    let finish: (value: unknown) => void = () => {};
    transport.invoke.mockImplementation(command => command === 'pendant_jog' ? new Promise(resolve => { finish = resolve; }) : Promise.resolve());
    const abort = new AbortController(), pending = jogPendant(request(), abort.signal), rejected = expect(pending).rejects.toMatchObject({ name: 'AbortError' });
    const requestId = transport.invoke.mock.calls[0][1].requestId;
    abort.abort(); await rejected; expect(transport.invoke).toHaveBeenCalledWith('cancel_motion', { requestId }); finish(result(request()));
  });
  it('times out a native increment rather than accumulating stalled work', async () => {
    vi.useFakeTimers(); transport.invoke.mockImplementation(command => command === 'pendant_jog' ? new Promise(() => {}) : Promise.resolve());
    const pending = jogPendant(request()), rejected = expect(pending).rejects.toThrow('超时');
    await vi.advanceTimersByTimeAsync(1501); await rejected;
    expect(transport.invoke.mock.calls.map(call => call[0])).toEqual(['pendant_jog', 'cancel_motion']);
  });
});

describe('pendant velocity scheduler', () => {
  beforeEach(() => { vi.useFakeTimers(); vi.setSystemTime(1_000_000); });
  function setup(solve = vi.fn(async (input: JogRequest) => result(input))) {
    const onAccept = vi.fn(), onStatus = vi.fn();
    const controller = createPendantMotionController({ sessionId: 'session', context, initialJoints: initial, solve, onAccept, onStatus });
    return { controller, solve, onAccept, onStatus };
  }
  it('requires release before the first gesture and retains exact accepted seeds across bounded increments', async () => {
    const { controller, solve, onAccept } = setup();
    controller.input(packet(0)); expect(solve).not.toHaveBeenCalled();
    controller.input(packet(1, false)); controller.input(packet(2, true, { speed: .123456789 }));
    await vi.advanceTimersByTimeAsync(125);
    expect(onAccept).toHaveBeenCalledTimes(3);
    expect(solve.mock.calls.every(([r]) => r.stepSeconds === .05)).toBe(true);
    expect(solve.mock.calls[1][0].startJoints).toEqual(onAccept.mock.calls[0][0].joints);
    expect(solve.mock.calls[2][0].startJoints).toEqual(onAccept.mock.calls[1][0].joints);
    controller.dispose();
  });
  it('a centered stick or zero speed does not solve or claim that the robot is moving', async () => {
    const { controller, solve, onStatus } = setup(); controller.input(packet(0, false));
    controller.input(packet(1, true, { axes: [0, 0, 0, 0, 0, 0] })); await vi.advanceTimersByTimeAsync(50);
    controller.input(packet(2, true, { speed: 0 })); await vi.advanceTimersByTimeAsync(50);
    expect(solve).not.toHaveBeenCalled(); expect(onStatus.mock.calls.some(([status]) => status.moving)).toBe(false); controller.dispose();
  });
  it('keeps one solve in flight without catch-up displacement and release invalidates its late result', async () => {
    let finish: (value: JogSuccess) => void = () => {};
    const solve = vi.fn((_: JogRequest, _signal?: AbortSignal) => new Promise<JogSuccess>(resolve => { finish = resolve; }));
    const { controller, onAccept } = setup(solve);
    controller.input(packet(0, false)); controller.input(packet(1));
    for (let sequence = 2; sequence <= 6; sequence++) { await vi.advanceTimersByTimeAsync(50); controller.input(packet(sequence)); }
    expect(solve).toHaveBeenCalledTimes(1);
    controller.input(packet(7, false)); expect(solve.mock.calls[0][1]?.aborted).toBe(true);
    finish(result(solve.mock.calls[0][0])); await vi.advanceTimersByTimeAsync(100);
    expect(onAccept).not.toHaveBeenCalled();
    controller.input(packet(8)); expect(solve).toHaveBeenCalledTimes(2); expect(solve.mock.calls[1][0].stepSeconds).toBe(.05);
    controller.dispose();
  });
  it('stops on stale input and cannot resume a held gesture until a fresh release', async () => {
    const { controller, solve, onAccept, onStatus } = setup(); controller.input(packet(0, false)); controller.input(packet(1));
    await vi.advanceTimersByTimeAsync(400); const count = onAccept.mock.calls.length;
    expect(onStatus).toHaveBeenLastCalledWith({ moving: false, error: expect.stringContaining('超时') });
    controller.input(packet(2)); await vi.advanceTimersByTimeAsync(100); expect(onAccept).toHaveBeenCalledTimes(count);
    controller.input(packet(3, false)); controller.input(packet(4)); await vi.advanceTimersByTimeAsync(1); expect(onAccept).toHaveBeenCalledTimes(count + 1);
    expect(solve.mock.calls.every(([r]) => r.stepSeconds === .05)).toBe(true); controller.dispose();
  });
  it('rejects delayed epochs and repeats; changing mode needs a release', async () => {
    const { controller, solve } = setup(); controller.input(packet(0, false)); controller.input(packet(1)); await vi.advanceTimersByTimeAsync(1);
    controller.input(packet(2, true, { mode: 'linear' })); await vi.advanceTimersByTimeAsync(60); expect(solve).toHaveBeenCalledTimes(1);
    controller.input(packet(0, false, { epoch: 2 })); controller.input(packet(1, true, { epoch: 2 })); await vi.advanceTimersByTimeAsync(1); expect(solve).toHaveBeenCalledTimes(2);
    controller.input(packet(99, false, { epoch: 1 })); controller.input(packet(1, true, { epoch: 2, axes: [-1, 0, 0, 0, 0, 0] }));
    await vi.advanceTimersByTimeAsync(50); expect(solve.mock.lastCall![0].axes[0]).toBe(1); controller.dispose();
  });
  it('aborts pending work on context reset and starts only from the new externally confirmed joint state', async () => {
    let finish: (value: JogSuccess) => void = () => {};
    const solve = vi.fn((_: JogRequest, _signal?: AbortSignal) => new Promise<JogSuccess>(resolve => { finish = resolve; }));
    const { controller, onAccept } = setup(solve); controller.input(packet(0, false)); controller.input(packet(1));
    const next: JointAngles = [5, 20, -30, 0, 40, 0]; controller.reset(context, next);
    expect(solve.mock.calls[0][1]?.aborted).toBe(true); finish(result(solve.mock.calls[0][0])); await vi.advanceTimersByTimeAsync(1); expect(onAccept).not.toHaveBeenCalled();
    controller.input(packet(2)); expect(solve).toHaveBeenCalledTimes(1);
    controller.input(packet(3, false)); controller.input(packet(4)); expect(solve.mock.calls[1][0].startJoints).toEqual(next); controller.dispose();
  });
  it('rejects forged or already stale messages, ignores other sessions and disarms after solver failure', async () => {
    const solve = vi.fn(async (_: JogRequest) => ({ status: 'failure' as const, code: 'collision', message: '碰撞' }));
    const onAccept = vi.fn(), onStatus = vi.fn();
    const controller = createPendantMotionController({ sessionId: 'session', context, initialJoints: initial, solve, onAccept, onStatus });
    controller.input(packet(0, false)); controller.input(packet(1, true, { sessionId: 'other' })); expect(solve).not.toHaveBeenCalled();
    controller.input(packet(1, true, { receivedAtMs: Date.now() - 351 })); expect(solve).not.toHaveBeenCalled();
    controller.input(packet(2, false)); controller.input(packet(3)); await vi.advanceTimersByTimeAsync(1); expect(onStatus).toHaveBeenLastCalledWith({ moving: false, error: '碰撞' });
    controller.input(packet(4)); expect(solve).toHaveBeenCalledTimes(1);
    controller.input(packet(5, false)); controller.input(packet(6, true, { axes: [NaN, 0, 0, 0, 0, 0] })); expect(solve).toHaveBeenCalledTimes(1); expect(onAccept).not.toHaveBeenCalled(); controller.dispose();
  });
});
