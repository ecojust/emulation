/** The phone has no robot solver or persistent credentials. It sends only a
 * bounded, current joystick direction; the desktop owns every accepted pose. */
export const INPUT_INTERVAL_MS = 50;
export const STATE_TIMEOUT_MS = 1200;
const PAIRS = { linear: [[0, 1], [0, 2], [1, 2]], joint: [[0, 1], [2, 3], [4, 5]] };
const zeroAxes = () => [0, 0, 0, 0, 0, 0];
const finiteArray = (value, length) => Array.isArray(value) && value.length === length && value.every(Number.isFinite);
const normalizedVector = vector => { const length = Math.max(1, Math.hypot(...vector)); return vector.map(value => value / length); };

/** Convert screen displacement to a circular, rescaled dead zone; +Y is up. */
export function joystickVector(dx, dy, radius, deadZone = 0.12) {
  if (![dx, dy, radius, deadZone].every(Number.isFinite) || radius <= 0 || deadZone < 0 || deadZone >= 1) return [0, 0];
  const distance = Math.hypot(dx, dy) / radius;
  if (distance <= deadZone) return [0, 0];
  const scale = (Math.min(distance, 1) - deadZone) / (1 - deadZone);
  const length = Math.hypot(dx, dy);
  return [dx / length * scale, -dy / length * scale];
}

export function joystickAxes(mode, pair, vector) {
  const axes = zeroAxes();
  if (!PAIRS[mode] || !Number.isInteger(pair) || !PAIRS[mode][pair] || !finiteArray(vector, 2)) return axes;
  const length = Math.max(1, Math.hypot(...vector));
  for (let index = 0; index < 2; index++) axes[PAIRS[mode][pair][index]] = vector[index] / length;
  return axes;
}

export function validRobotState(value) {
  return value !== null && typeof value === 'object'
    && typeof value.robotName === 'string' && value.robotName.length <= 200
    && typeof value.modelId === 'string' && value.modelId.length <= 200
    && finiteArray(value.joints, 6) && typeof value.available === 'boolean'
    && (value.reason === null || typeof value.reason === 'string' && value.reason.length <= 500)
    && typeof value.moving === 'boolean'
    && (value.tcp === null || value.tcp && finiteArray(value.tcp.position, 3) && finiteArray(value.tcp.orientation, 4));
}

/** Transport-independent hold/deadman state. There is no command queue and an
 * epoch change always consumes the current gesture before accepting another. */
export function createPendantController(options) {
  const now = options.now ?? (() => performance.now());
  const schedule = options.setInterval ?? ((callback, delay) => setInterval(callback, delay));
  const unschedule = options.clearInterval ?? (handle => clearInterval(handle));
  let disposed = false, sequence = 0, lastState = -Infinity;
  const state = { phase: 'disconnected', connected: false, holding: false, epoch: null, sessionId: null,
    mode: 'linear', pair: 0, space: 'world', speed: 0.2, vector: [0, 0], robot: null,
    available: false, feedback: '请扫描电脑端的二维码连接。' };
  const snapshot = () => ({ ...state, vector: [...state.vector], robot: state.robot ? structuredClone(state.robot) : null });
  const notify = () => { if (!disposed) options.onChange?.(snapshot()); };
  function packet(active) {
    if (!state.connected || state.epoch === null || disposed) return false;
    const axes = active ? joystickAxes(state.mode, state.pair, state.vector) : zeroAxes();
    const moving = active && axes.some(value => Math.abs(value) > 1e-9);
    try {
      options.send({ type: 'input', epoch: state.epoch, sequence: ++sequence,
        mode: state.mode, space: state.space, axes: moving ? axes : zeroAxes(), speed: state.speed,
        active: moving });
      return true;
    } catch {
      state.connected = false; state.phase = 'disconnected'; state.available = false;
      state.holding = false; state.vector = [0, 0]; state.feedback = '连接已中断，请重新连接。';
      notify(); return false;
    }
  }
  function end(message, alwaysSend = false) {
    const wasHolding = state.holding;
    state.holding = false; state.vector = [0, 0];
    if (wasHolding || alwaysSend) packet(false);
    if (message) state.feedback = message;
    else if (wasHolding && state.available) state.feedback = '已停止，拖动摇杆可继续。';
    notify();
  }
  const timer = schedule(() => {
    if (!state.connected || disposed) return;
    if (now() - lastState > STATE_TIMEOUT_MS && state.available) {
      state.available = false; end('电脑状态更新中断，已停止 Jog。', true); return;
    }
    if (state.holding) packet(true);
  }, INPUT_INTERVAL_MS);
  return {
    snapshot,
    connecting() {
      end(); sequence = 0; lastState = -Infinity;
      Object.assign(state, { phase: 'connecting', connected: false, available: false, epoch: null, sessionId: null,
        feedback: '正在与电脑配对…' }); notify();
    },
    receive(message) {
      if (disposed || !message || typeof message !== 'object') return false;
      if (message.type === 'ready') {
        if (state.connected || !Number.isSafeInteger(message.epoch) || message.epoch < 1
          || typeof message.sessionId !== 'string' || !message.sessionId.length || message.sessionId.length > 200) return false;
        Object.assign(state, { connected: true, phase: 'connected', epoch: message.epoch,
          sessionId: message.sessionId, available: false, holding: false, vector: [0, 0], feedback: '等待电脑的机器人状态…' });
        lastState = now(); packet(false); notify(); return true;
      }
      if (message.type === 'stop') {
        if (!state.connected || !Number.isSafeInteger(message.epoch) || message.epoch < 1
          || typeof message.message !== 'string' || message.message.length > 500) return false;
        if (message.epoch < state.epoch) return true;
        state.epoch = message.epoch;
        end(message.message || 'Jog 已停止，请松手后重新拖动。', true); return true;
      }
      if (message.type === 'state') {
        if (!state.connected || !validRobotState(message.state)) return false;
        const previousAvailable = state.available, reportedReason = message.state.reason;
        state.robot = structuredClone(message.state); state.available = message.state.available;
        lastState = now();
        if (!state.available) end(message.state.reason || '电脑暂时无法接受 Jog。');
        else if (reportedReason !== null && (state.holding || reportedReason !== state.feedback)) {
          // A solver rejection consumes this gesture but leaves controls ready
          // for a deliberate retry. Repeated telemetry after release sends no more zero frames.
          end(reportedReason || 'Jog 未完成，请重新拖动。', true);
        }
        else if (!previousAvailable && reportedReason === null) { state.feedback = '拖动摇杆控制机器人，松手停止。'; notify(); }
        else notify();
        return true;
      }
      if (message.type === 'error' && typeof message.message === 'string' && message.message.length <= 500) {
        this.disconnect(message.message || '配对失败，请重新扫描二维码。', true); return true;
      }
      return false;
    },
    start(vector = [0, 0]) {
      if (disposed || !state.connected || !state.available || state.holding || now() - lastState > STATE_TIMEOUT_MS || !finiteArray(vector, 2)) return false;
      state.holding = true; state.vector = normalizedVector(vector); state.feedback = '正在 Jog，松手停止。';
      packet(true); notify(); return state.holding;
    },
    move(vector) {
      if (!state.holding || !finiteArray(vector, 2)) return;
      state.vector = normalizedVector(vector); notify();
    },
    end(message) { end(message); },
    stop(message = 'Jog 已停止，请重新拖动。') { end(message, true); },
    configure(patch) {
      if (disposed) return;
      const mode = patch.mode ?? state.mode, pair = patch.pair ?? (patch.mode ? 0 : state.pair);
      const space = patch.space ?? state.space, speed = patch.speed ?? state.speed;
      if (!PAIRS[mode] || !PAIRS[mode][pair] || !Number.isInteger(pair) || !['world', 'tool'].includes(space)
        || !Number.isFinite(speed) || speed < 0.05 || speed > 1) return;
      end(state.holding ? '设置已切换，请重新拖动摇杆。' : undefined);
      Object.assign(state, { mode, pair, space, speed }); notify();
    },
    disconnect(message = '连接已断开，请重新连接。', failed = false) {
      end(); Object.assign(state, { connected: false, available: false, phase: failed ? 'error' : 'disconnected', epoch: null,
        sessionId: null, holding: false, vector: [0, 0], feedback: message }); notify();
    },
    dispose() { if (disposed) return; end(undefined, true); disposed = true; unschedule(timer); },
  };
}

export function mountPendant(document, window) {
  const element = id => document.getElementById(id);
  const joystick = element('joystick');
  if (!joystick) return () => {};
  let socket = null, connectionGeneration = 0, authTimer = null, pointer = null, keyboard = new Set();
  let token = new URLSearchParams(window.location.hash.slice(1)).get('token') ?? '';
  // The secret lives only in this page. Reloading requires a new scan.
  if (window.location.hash) window.history.replaceState(null, '', window.location.pathname + window.location.search);
  if (!/^[A-Za-z0-9_-]{32,256}$/.test(token)) token = '';
  let renderedMode = '', renderedPair = -1;
  const controls = createPendantController({
    send(packet) {
      if (!socket || socket.readyState !== 1 || socket.bufferedAmount > 4096) {
        socket?.close(); throw new Error('Connection is not writable');
      }
      socket.send(JSON.stringify(packet));
    },
    onChange(state) {
      if (!state.holding) {
        keyboard.clear();
        if (pointer !== null) {
          const previous = pointer; pointer = null;
          if (joystick.hasPointerCapture(previous)) joystick.releasePointerCapture(previous);
        }
      }
      element('connection-dot').dataset.connected = String(state.connected);
      element('connection-status').textContent = state.connected ? '已连接电脑' : state.phase === 'connecting' ? '连接中…' : state.phase === 'error' ? '连接不可用' : '未连接';
      element('reconnect').hidden = state.connected || state.phase === 'connecting' || !token;
      if (element('feedback').textContent !== state.feedback) element('feedback').textContent = state.feedback;
      element('robot-name').textContent = state.robot?.robotName || '等待机器人';
      element('motion-status').textContent = state.robot?.moving && state.connected ? '运动中' : '未运动';
      element('motion-status').dataset.moving = String(Boolean(state.robot?.moving && state.connected));
      joystick.setAttribute('aria-disabled', String(!state.available)); joystick.dataset.held = String(state.holding);
      element('stick-knob').setAttribute('transform', `translate(${state.vector[0] * 62} ${-state.vector[1] * 62})`);
      element('speed-value').textContent = `${Math.round(state.speed * 100)}%`;
      element('space-label').hidden = state.mode !== 'linear';
      element('control-title').textContent = state.mode === 'linear' ? 'TCP 直线移动' : '关节轴移动';
      for (const button of document.querySelectorAll('[data-mode]')) button.setAttribute('aria-pressed', String(button.dataset.mode === state.mode));
      if (renderedMode !== state.mode) {
        renderedMode = state.mode; renderedPair = -1;
        element('axis-pairs').replaceChildren(...PAIRS[state.mode].map((pair, index) => {
          const button = document.createElement('button'); button.type = 'button'; button.dataset.pair = String(index);
          button.textContent = state.mode === 'linear' ? pair.map(axis => 'XYZ'[axis]).join(' / ') : pair.map(axis => 'J' + (axis + 1)).join(' / ');
          button.addEventListener('click', () => controls.configure({ pair: index })); return button;
        }));
      }
      if (renderedPair !== state.pair) {
        renderedPair = state.pair;
        for (const button of element('axis-pairs').children) button.setAttribute('aria-pressed', String(Number(button.dataset.pair) === state.pair));
        const labels = PAIRS[state.mode][state.pair].map(axis => state.mode === 'linear' ? 'XYZ'[axis] : 'J' + (axis + 1));
        element('axis-left').textContent = '−' + labels[0]; element('axis-right').textContent = '+' + labels[0];
        element('axis-bottom').textContent = '−' + labels[1]; element('axis-top').textContent = '+' + labels[1];
      }
      for (const [index, axis] of ['x', 'y', 'z'].entries()) element('tcp-' + axis).textContent = state.robot?.tcp ? state.robot.tcp.position[index].toFixed(2) : '—';
      for (const [index, output] of [...element('joint-values').querySelectorAll('output')].entries()) output.textContent = state.robot ? state.robot.joints[index].toFixed(2) + '°' : '—';
    },
  });
  function disconnect(message) {
    connectionGeneration++; clearTimeout(authTimer); controls.disconnect(message);
    const previous = socket; socket = null; previous?.close();
  }
  function connect() {
    disconnect('');
    if (!token) { controls.disconnect('配对信息已失效，请重新扫描电脑端二维码。', true); return; }
    controls.connecting();
    const generation = connectionGeneration, protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    let next;
    try { next = new window.WebSocket(`${protocol}//${window.location.host}/ws`); } catch { controls.disconnect('无法连接电脑，请确认手机和电脑处于同一网络。', true); return; }
    socket = next;
    authTimer = window.setTimeout(() => { if (generation === connectionGeneration && !controls.snapshot().connected) disconnect('连接超时，请确认电脑端示教器仍已开启。'); }, 6000);
    next.addEventListener('open', () => {
      if (generation !== connectionGeneration) { next.close(); return; }
      next.send(JSON.stringify({ type: 'auth', token }));
    });
    next.addEventListener('message', event => {
      if (generation !== connectionGeneration || next !== socket) return;
      try {
        if (typeof event.data !== 'string' || event.data.length > 16_384) throw new Error('Invalid message');
        const message = JSON.parse(event.data);
        if (!controls.receive(message)) throw new Error('Invalid protocol');
        if (message.type === 'ready') clearTimeout(authTimer);
        if (message.type === 'error') { clearTimeout(authTimer); next.close(); }
      } catch { disconnect('收到无效的电脑状态，请重新连接。'); }
    });
    next.addEventListener('close', () => {
      if (generation !== connectionGeneration || next !== socket) return;
      clearTimeout(authTimer); socket = null;
      if (controls.snapshot().phase !== 'error') controls.disconnect('连接已断开，请重新连接。');
    });
    next.addEventListener('error', () => {
      if (generation !== connectionGeneration || next !== socket) return;
      controls.disconnect('无法连接电脑，请确认网络和电脑端示教器状态。', true); next.close();
    });
  }
  const vector = event => {
    const rect = joystick.getBoundingClientRect();
    return joystickVector(event.clientX - (rect.left + rect.width / 2), event.clientY - (rect.top + rect.height / 2), Math.min(rect.width, rect.height) * .31);
  };
  joystick.addEventListener('pointerdown', event => {
    if (pointer !== null || !event.isPrimary || event.button !== 0 || keyboard.size || !controls.start(vector(event))) return;
    event.preventDefault(); pointer = event.pointerId;
    try { joystick.setPointerCapture(pointer); } catch { pointer = null; controls.stop('触摸控制中断，请重新拖动。'); }
  });
  joystick.addEventListener('pointermove', event => { if (pointer !== event.pointerId) return; event.preventDefault(); controls.move(vector(event)); });
  const pointerEnd = event => { if (pointer !== event.pointerId) return; event.preventDefault(); controls.end(); };
  joystick.addEventListener('pointerup', pointerEnd); joystick.addEventListener('pointercancel', pointerEnd);
  joystick.addEventListener('lostpointercapture', () => { if (pointer !== null) { pointer = null; controls.end(); } });
  joystick.addEventListener('contextmenu', event => event.preventDefault());
  const keyVector = () => [Number(keyboard.has('ArrowRight')) - Number(keyboard.has('ArrowLeft')), Number(keyboard.has('ArrowUp')) - Number(keyboard.has('ArrowDown'))];
  joystick.addEventListener('keydown', event => {
    if (event.key === 'Escape' || event.key === ' ') { event.preventDefault(); controls.stop(); return; }
    if (!['ArrowLeft', 'ArrowRight', 'ArrowUp', 'ArrowDown'].includes(event.key) || pointer !== null || event.repeat) return;
    event.preventDefault(); const wasHolding = controls.snapshot().holding;
    keyboard.add(event.key);
    if (wasHolding) controls.move(keyVector()); else if (!controls.start(keyVector())) keyboard.clear();
  });
  joystick.addEventListener('keyup', event => {
    if (!keyboard.has(event.key)) return; event.preventDefault(); keyboard.delete(event.key);
    if (keyboard.size) controls.move(keyVector()); else controls.end();
  });
  joystick.addEventListener('blur', () => controls.end());
  for (const button of document.querySelectorAll('[data-mode]')) button.addEventListener('click', () => controls.configure({ mode: button.dataset.mode }));
  element('space').addEventListener('change', event => controls.configure({ space: event.target.value }));
  element('speed').addEventListener('input', event => controls.configure({ speed: Number(event.target.value) / 100 }));
  element('stop').addEventListener('click', () => controls.stop()); element('reconnect').addEventListener('click', connect);
  const blur = () => controls.end('Jog 已停止，请重新拖动。');
  const resize = () => controls.end('屏幕尺寸已改变，请重新拖动。');
  const hidden = () => { if (document.visibilityState === 'hidden') disconnect('页面已离开，重新连接后可继续示教。'); };
  const pagehide = () => disconnect('页面已离开，已停止 Jog。');
  window.addEventListener('blur', blur); window.addEventListener('pagehide', pagehide); window.addEventListener('resize', resize); window.addEventListener('orientationchange', resize); document.addEventListener('visibilitychange', hidden);
  connect();
  return () => { disconnect(); controls.dispose(); token = ''; window.removeEventListener('blur', blur); window.removeEventListener('pagehide', pagehide); window.removeEventListener('resize', resize); window.removeEventListener('orientationchange', resize); document.removeEventListener('visibilitychange', hidden); };
}

if (typeof document !== 'undefined' && typeof window !== 'undefined' && document.getElementById('joystick')) mountPendant(document, window);
