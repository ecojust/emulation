import { useEffect, useLayoutEffect, useRef, useState } from 'react';
import type { JointAngles } from '../robot/types';
import { createPendantMotionController, type JogContext } from './motion';
import { pendantTransport } from './transport';
import type { PendantSession, PendantState, PendantTransport } from './types';

interface Options {
  state: Omit<PendantState, 'moving'>;
  context: JogContext | null;
  contextKey: string;
  onAccept(joints: JointAngles): void;
  beforeStart(): void;
  transport?: PendantTransport;
}
const sameJoints = (a: JointAngles, b: JointAngles) => a.every((value, index) => value === b[index]);
/** Owns one project-scoped connection. The server never receives mutable project
 * documents, and a late network/solver result cannot migrate to another project. */
export function usePendant(options: Options) {
  const latest = useRef(options); latest.current = options;
  const transport = options.transport ?? pendantTransport;
  const [session, setSession] = useState<PendantSession | null>(null);
  const sessionRef = useRef<PendantSession | null>(null);
  const [dialogOpen, setDialogOpen] = useState(false);
  const [busy, setBusy] = useState(false), [error, setError] = useState('');
  const [motionStatus, setMotionStatus] = useState({ moving: false, error: null as string | null });
  const motion = useRef<ReturnType<typeof createPendantMotionController> | null>(null);
  const acceptedContext = useRef(options.contextKey);
  const expected = useRef<JointAngles>([...options.state.joints]);
  const alive = useRef(true), generation = useRef(0);
  const queue = useRef<Promise<void>>(Promise.resolve());
  const listener = useRef<Promise<() => void> | null>(null);
  const unavailable = useRef(!options.state.available);
  const motionStatusRef = useRef(motionStatus); motionStatusRef.current = motionStatus;
  function stopMotion() {
    motion.current?.dispose(); motion.current = null;
    if (alive.current) setMotionStatus({ moving: false, error: null });
  }
  function createMotion(current: PendantSession) {
    stopMotion();
    const config = latest.current;
    if (!config.context) return;
    expected.current = [...config.state.joints]; acceptedContext.current = config.contextKey;
    motion.current = createPendantMotionController({ sessionId: current.sessionId, context: config.context, initialJoints: [...config.state.joints],
      onAccept(result) {
        if (!alive.current || sessionRef.current?.sessionId !== current.sessionId) return;
        if (!latest.current.state.available || document.visibilityState === 'hidden'
          || acceptedContext.current !== latest.current.contextKey || !sameJoints(expected.current, latest.current.state.joints)) {
          if (latest.current.context) { motion.current?.reset(latest.current.context, latest.current.state.joints); acceptedContext.current = latest.current.contextKey; expected.current = [...latest.current.state.joints]; }
          else stopMotion();
          return;
        }
        expected.current = [...result.joints]; latest.current.onAccept(result.joints);
      },
      onStatus(status) { if (alive.current && sessionRef.current?.sessionId === current.sessionId) setMotionStatus(status); },
    });
  }
  function enqueue(work: () => Promise<void>) {
    const next = queue.current.catch(() => {}).then(work);
    queue.current = next.catch(() => {}); return next;
  }
  async function disconnect() {
    generation.current++; stopMotion(); sessionRef.current = null;
    if (alive.current) { setSession(null); setBusy(true); }
    try { await enqueue(() => transport.stop()); }
    catch (cause) { if (alive.current) setError(cause instanceof Error ? cause.message : '示教器连接未能关闭。'); }
    finally { if (alive.current) setBusy(false); }
  }
  function ensureListener() {
    if (!listener.current) listener.current = transport.listen(input => {
      if (!alive.current || sessionRef.current?.sessionId !== input.sessionId) return;
      if (!latest.current.state.available || !latest.current.context || document.visibilityState === 'hidden') { motion.current?.stop(); return; }
      motion.current?.input(input);
    }).catch(cause => { listener.current = null; throw cause; });
    return listener.current;
  }
  async function start() {
    const operation = ++generation.current;
    stopMotion(); sessionRef.current = null; setSession(null); setBusy(true); setError('');
    latest.current.beforeStart();
    try {
      await enqueue(async () => {
        await transport.stop(); await ensureListener();
        if (!alive.current || generation.current !== operation) return;
        const next = await transport.start();
        if (!alive.current || generation.current !== operation) { await transport.stop(); return; }
        sessionRef.current = next; setSession(next); createMotion(next);
        await transport.publish(next.sessionId, { ...latest.current.state, moving: false });
      });
    } catch (cause) {
      if (alive.current && generation.current === operation) {
        stopMotion(); sessionRef.current = null; setSession(null);
        setError(cause instanceof Error ? cause.message : typeof cause === 'string' ? cause : '无法启动示教器，请检查本机网络后重试。');
        await enqueue(() => transport.stop()).catch(() => {});
      }
    } finally { if (alive.current && generation.current === operation) setBusy(false); }
  }
  function open() { setDialogOpen(true); if (!sessionRef.current && !busy) void start(); }
  // Rebase only on an external edit. Accepted f64 output must not reset the
  // continuous hold loop or be re-read from the rounded inspector controls.
  useLayoutEffect(() => {
    const current = sessionRef.current;
    if (!current) return;
    if (!options.context) { stopMotion(); return; }
    if (!motion.current) createMotion(current);
    else { motion.current.reset(options.context, options.state.joints); acceptedContext.current = options.contextKey; expected.current = [...options.state.joints]; }
  }, [options.contextKey]);
  useLayoutEffect(() => {
    if (motion.current && options.context && !sameJoints(expected.current, options.state.joints)) {
      motion.current.reset(options.context, options.state.joints); expected.current = [...options.state.joints];
    }
  }, [options.state.joints]);
  useLayoutEffect(() => {
    if (unavailable.current !== !options.state.available) {
      if (options.context) motion.current?.reset(options.context, options.state.joints);
      else stopMotion();
      expected.current = [...options.state.joints];
    }
    unavailable.current = !options.state.available;
  }, [options.state.available]);
  useEffect(() => {
    alive.current = true;
    let pollPending = false;
    const tick = async () => {
      const current = sessionRef.current;
      if (!current || pollPending) return;
      pollPending = true;
      try {
        const state = latest.current.state;
        const visible = document.visibilityState !== 'hidden';
        await transport.publish(current.sessionId, { ...state, available: state.available && visible, reason: visible ? motionStatusRef.current.error ?? state.reason : '电脑端已暂停显示，请返回应用。', moving: motionStatusRef.current.moving });
        const next = await transport.status();
        if (!alive.current || sessionRef.current?.sessionId !== current.sessionId) return;
        if (!next || next.sessionId !== current.sessionId) { stopMotion(); sessionRef.current = null; setSession(null); setError('示教器会话已结束，请重新连接。'); return; }
        if (next.connected !== current.connected) {
          sessionRef.current = next; setSession(next);
          if (!next.connected) motion.current?.stop();
        }
      } catch (cause) {
        if (alive.current && sessionRef.current?.sessionId === current.sessionId) {
          motion.current?.stop(); setError(cause instanceof Error ? cause.message : '示教器通信中断，请重新连接。');
        }
      } finally { pollPending = false; }
    };
    const timer = setInterval(() => void tick(), 100);
    const hidden = () => { if (document.visibilityState === 'hidden') motion.current?.stop(); void tick(); };
    document.addEventListener('visibilitychange', hidden);
    return () => {
      alive.current = false; generation.current++; clearInterval(timer); document.removeEventListener('visibilitychange', hidden);
      stopMotion(); sessionRef.current = null;
      const pending = listener.current; listener.current = null;
      void pending?.then(unlisten => unlisten()).catch(() => {});
      void enqueue(() => transport.stop());
    };
  }, [transport]);
  return { session, connected: Boolean(session?.connected), dialogOpen, busy, error, motionStatus,
    open, closeDialog: () => setDialogOpen(false), start, disconnect, pause: () => motion.current?.stop() };
}
