import { describe, expect, it, vi } from 'vitest';
import { createPendantController, joystickAxes, joystickVector, validRobotState, INPUT_INTERVAL_MS, STATE_TIMEOUT_MS,
  type MobileInput, type MobileRobotState } from '../src/pendant/mobile/controller.js';

const robot = (): MobileRobotState => ({ robotName: 'ABB IRB1100', modelId: 'irb1100-4-0.58', joints: [0, 20, -30, 0, 40, 0],
  tcp: { position: [364, 0, 617], orientation: [1, 0, 0, 0] }, available: true, reason: null, moving: false });
function harness() {
  let clock = 0, tick = () => {};
  const packets: MobileInput[] = [], clear = vi.fn(), send = vi.fn((packet: MobileInput) => { packets.push(packet); });
  const control = createPendantController({ send, now: () => clock,
    setInterval(callback, delay) { expect(delay).toBe(INPUT_INTERVAL_MS); tick = callback; return 17; }, clearInterval: clear });
  const connect = () => { control.connecting(); expect(control.receive({ type: 'ready', sessionId: 'session-one', epoch: 1 })).toBe(true); control.receive({ type: 'state', state: robot() }); };
  return { control, packets, send, clear, connect, tick(dt = INPUT_INTERVAL_MS) { clock += dt; tick(); } };
}

describe('mobile joystick geometry', () => {
  it('has a true dead zone, positive screen-up Y, radial speed and bounded diagonals', () => {
    expect(joystickVector(3, 4, 100)).toEqual([0, 0]);
    expect(joystickVector(100, 0, 100)).toEqual([1, -0]);
    expect(joystickVector(0, -100, 100)).toEqual([0, 1]);
    expect(joystickVector(0, 100, 100)).toEqual([0, -1]);
    expect(joystickVector(56, 0, 100)[0]).toBeCloseTo(.5);
    const diagonal = joystickVector(300, -300, 100);
    expect(Math.hypot(...diagonal)).toBeCloseTo(1); expect(diagonal[0]).toBeCloseTo(diagonal[1]);
    for (const input of [[NaN, 0, 100], [0, Infinity, 100], [0, 0, 0]]) expect(joystickVector(...input as [number, number, number])).toEqual([0, 0]);
  });
  it('maps every linear plane and all six joints without an unintended axis', () => {
    expect(joystickAxes('linear', 0, [.4, -.6])).toEqual([.4, -.6, 0, 0, 0, 0]);
    expect(joystickAxes('linear', 1, [.4, -.6])).toEqual([.4, 0, -.6, 0, 0, 0]);
    expect(joystickAxes('linear', 2, [.4, -.6])).toEqual([0, .4, -.6, 0, 0, 0]);
    expect(joystickAxes('joint', 0, [.4, -.6])).toEqual([.4, -.6, 0, 0, 0, 0]);
    expect(joystickAxes('joint', 1, [.4, -.6])).toEqual([0, 0, .4, -.6, 0, 0]);
    expect(joystickAxes('joint', 2, [.4, -.6])).toEqual([0, 0, 0, 0, .4, -.6]);
    expect(Math.hypot(...joystickAxes('joint', 2, [1, 1]))).toBeCloseTo(1);
    expect(joystickAxes('linear', 8, [1, 1])).toEqual([0, 0, 0, 0, 0, 0]);
  });
});

describe('mobile hold, authentication and rearming', () => {
  it('requires ready, an initial zero frame, fresh state and a deliberate hold', () => {
    const h = harness(); expect(h.control.start([1, 0])).toBe(false); h.control.connecting();
    expect(h.control.receive({ type: 'ready', sessionId: 'one', epoch: 4 })).toBe(true);
    expect(h.packets).toEqual([{ type: 'input', epoch: 4, sequence: 1, mode: 'linear', space: 'world', speed: .2, axes: [0, 0, 0, 0, 0, 0], active: false }]);
    expect(h.control.start([1, 0])).toBe(false); h.control.receive({ type: 'state', state: robot() });
    expect(h.control.start([1, 0])).toBe(true); expect(h.packets.at(-1)).toMatchObject({ sequence: 2, epoch: 4, active: true, axes: [1, 0, 0, 0, 0, 0] });
  });
  it('samples only the latest direction at 20Hz and never queues released input', () => {
    const h = harness(); h.connect(); h.control.start([.1, 0]); const count = h.packets.length;
    h.control.move([.4, 0]); h.control.move([0, 1]); expect(h.packets).toHaveLength(count);
    h.tick(); expect(h.packets.at(-1)?.axes).toEqual([0, 1, 0, 0, 0, 0]);
    h.control.end(); expect(h.packets.at(-1)).toMatchObject({ active: false, axes: [0, 0, 0, 0, 0, 0] });
    const stoppedCount = h.packets.length; h.control.move([1, 0]); h.tick(); expect(h.packets).toHaveLength(stoppedCount);
  });
  it('sends an exact zero vector for a sub-threshold held direction', () => {
    const h = harness(); h.connect(); h.control.start([1e-10, 0]);
    expect(h.packets.at(-1)).toMatchObject({ active: false, axes: [0, 0, 0, 0, 0, 0] });
    h.tick(); expect(h.packets.at(-1)).toMatchObject({ active: false, axes: [0, 0, 0, 0, 0, 0] });
  });
  it('stops before mode/pair/space/speed changes and requires a new hold', () => {
    const h = harness(); h.connect(); h.control.start([1, 0]); h.control.configure({ mode: 'joint', pair: 2, space: 'tool', speed: .6 });
    expect(h.packets.at(-1)).toMatchObject({ active: false, mode: 'linear', space: 'world', speed: .2 });
    expect(h.control.snapshot().holding).toBe(false); h.tick(); expect(h.packets.at(-1)?.active).toBe(false);
    h.control.start([0, -1]); expect(h.packets.at(-1)).toMatchObject({ active: true, mode: 'joint', space: 'tool', speed: .6, axes: [0, 0, 0, 0, 0, -1] });
  });
  it('consumes an old gesture on server stop and acknowledges the new epoch with zero', () => {
    const h = harness(); h.connect(); h.control.start([1, 0]);
    expect(h.control.receive({ type: 'stop', epoch: 2, message: '信号超时' })).toBe(true);
    expect(h.packets.at(-1)).toMatchObject({ epoch: 2, active: false }); expect(h.control.snapshot().holding).toBe(false);
    h.control.move([1, 0]); h.tick(); expect(h.packets.at(-1)?.active).toBe(false);
    expect(h.control.receive({ type: 'stop', epoch: 1, message: '旧消息' })).toBe(true);
    expect(h.control.snapshot().epoch).toBe(2); h.control.start([0, 1]); expect(h.packets.at(-1)).toMatchObject({ epoch: 2, active: true });
    expect(h.packets.map(packet => packet.sequence)).toEqual(h.packets.map((_, index) => index + 1));
  });
  it('stops when desktop state becomes unavailable and does not resume from its later recovery', () => {
    const h = harness(); h.connect(); h.control.start([1, 0]); h.control.receive({ type: 'state', state: { ...robot(), available: false, reason: '正在运行程序' } });
    expect(h.packets.at(-1)?.active).toBe(false); expect(h.control.snapshot().feedback).toBe('正在运行程序');
    h.control.receive({ type: 'state', state: robot() }); h.tick(); expect(h.control.snapshot().holding).toBe(false);
    expect(h.control.start([1, 0])).toBe(true);
  });
  it('consumes a rejected motion once while keeping a deliberate retry available', () => {
    const h = harness(); h.connect(); h.control.start([1, 0]);
    const failure = { ...robot(), available: true, reason: '目标不可达，请调整方向。' };
    h.control.receive({ type: 'state', state: failure });
    expect(h.control.snapshot()).toMatchObject({ available: true, holding: false, feedback: failure.reason });
    expect(h.packets.at(-1)).toMatchObject({ active: false, axes: [0, 0, 0, 0, 0, 0] });
    const count = h.packets.length;
    for (let i = 0; i < 4; i++) { h.control.receive({ type: 'state', state: failure }); h.tick(); }
    expect(h.packets).toHaveLength(count); h.control.move([1, 0]); expect(h.control.snapshot().holding).toBe(false);
    expect(h.control.start([-1, 0])).toBe(true);
    h.control.receive({ type: 'state', state: failure });
    expect(h.control.snapshot().holding).toBe(false); // An identical immediate failure may arrive before a cleared state was ever published.
    expect(h.packets.at(-1)?.active).toBe(false);
    const retryCount = h.packets.length; h.control.receive({ type: 'state', state: failure });
    expect(h.packets).toHaveLength(retryCount);
    expect(h.control.start([-1, 0])).toBe(true);
    h.control.receive({ type: 'state', state: robot() });
    h.control.receive({ type: 'state', state: failure });
    expect(h.control.snapshot().holding).toBe(false); expect(h.packets.at(-1)?.active).toBe(false);
  });
  it('drops a held input when live desktop state stops arriving even if the socket stays open', () => {
    const h = harness(); h.connect(); h.control.start([1, 0]); h.tick(STATE_TIMEOUT_MS + 1);
    expect(h.packets.at(-1)?.active).toBe(false); expect(h.control.snapshot().available).toBe(false);
    expect(h.control.start([1, 0])).toBe(false); h.control.receive({ type: 'state', state: robot() });
    expect(h.control.snapshot().holding).toBe(false); h.tick(); expect(h.packets.at(-1)?.active).toBe(false);
  });
  it('starts a reconnect with zero and a fresh sequence, never with the previous held command', () => {
    const h = harness(); h.connect(); h.control.start([1, 0]); h.control.disconnect(); const length = h.packets.length;
    h.control.move([0, 1]); h.tick(); expect(h.packets).toHaveLength(length); h.connect();
    expect(h.packets.at(-1)).toMatchObject({ active: false, sequence: 1 }); expect(h.control.snapshot().holding).toBe(false);
  });
  it('fails closed on a transport error instead of retaining a direction to retry later', () => {
    const h = harness(); h.connect(); h.send.mockImplementationOnce(() => { throw new Error('not writable'); });
    expect(h.control.start([1, 0])).toBe(false); expect(h.control.snapshot()).toMatchObject({ connected: false, available: false, holding: false });
    const calls = h.send.mock.calls.length; h.tick(); expect(h.send).toHaveBeenCalledTimes(calls);
  });
  it('rejects malformed state, invalid settings and invalid epochs', () => {
    const h = harness(); h.connect();
    expect(h.control.receive({ type: 'state', state: { ...robot(), joints: [NaN] } })).toBe(false);
    expect(h.control.receive({ type: 'stop', epoch: Infinity, message: 'bad' })).toBe(false);
    expect(h.control.receive({ type: 'ready', sessionId: 'second', epoch: 2 })).toBe(false);
    h.control.configure({ speed: -1 }); h.control.configure({ pair: 7 });
    expect(h.control.snapshot()).toMatchObject({ speed: .2, pair: 0 });
    expect(validRobotState({ ...robot(), tcp: { position: [0, 0, Infinity], orientation: [1, 0, 0, 0] } })).toBe(false);
  });
  it('disposes the timer and releases any held command exactly once', () => {
    const h = harness(); h.connect(); h.control.start([1, 0]); h.control.dispose(); const length = h.packets.length;
    expect(h.packets.at(-1)?.active).toBe(false); expect(h.clear).toHaveBeenCalledWith(17);
    h.control.dispose(); h.tick(); expect(h.packets).toHaveLength(length); expect(h.control.start([1, 0])).toBe(false);
  });
});
