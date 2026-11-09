import { quaternionDistanceDeg } from './kinematics';
import type { JointAngles, PlanRequest, PlanResult, PlanSuccess, Pose, RobotId } from './types';

export interface JogInitial { modelId: RobotId; joints: JointAngles; pose: Pose; speedMmPerSecond: number }
export interface JogStatus { pending: boolean; error: string | null }
export interface JogOptions {
  initial: JogInitial;
  solve: (request: PlanRequest, signal: AbortSignal) => Promise<PlanResult>;
  onAccept: (result: PlanSuccess, request: PlanRequest) => void;
  onStatus: (status: JogStatus) => void;
  minimumIntervalMs?: number;
}
export interface JogController { update(target: Pose): void; reset(initial: JogInitial): void; cancel(): void; dispose(): void }

const clonePose = (pose: Pose): Pose => ({ position: [...pose.position], orientation: [...pose.orientation] });
const cloneInitial = (initial: JogInitial): JogInitial => ({ ...initial, joints: [...initial.joints], pose: clonePose(initial.pose) });
function samePose(a: Pose, b: Pose): boolean {
  try { return Math.hypot(...a.position.map((value, i) => value - b.position[i])) < 1e-5 && quaternionDistanceDeg(a.orientation, b.orientation) < 1e-5; }
  catch { return false; }
}

/** Keep one in-flight solve and one latest absolute target. Accepted f64 joints
 * remain private here; UI formatting never becomes the next segment's seed. */
export function createJogController(options: JogOptions): JogController {
  let accepted = cloneInitial(options.initial);
  let desired: Pose | null = null;
  let active: AbortController | null = null;
  let timer: ReturnType<typeof setTimeout> | null = null;
  let generation = 0;
  let disposed = false;
  let error: string | null = null;
  let lastDispatch = -Infinity;
  let lastStatus: JogStatus = { pending: false, error: null };
  const interval = Math.max(0, options.minimumIntervalMs ?? 34);

  function publish() {
    if (disposed) return;
    const next = { pending: Boolean(active || desired || timer), error };
    if (next.pending !== lastStatus.pending || next.error !== lastStatus.error) {
      lastStatus = next;
      options.onStatus(next);
    }
  }
  function schedule() {
    if (disposed || active || timer || !desired) { publish(); return; }
    if (samePose(accepted.pose, desired)) { desired = null; error = null; publish(); return; }
    const delay = interval - (performance.now() - lastDispatch);
    if (delay > 0) {
      const expected = generation;
      timer = setTimeout(() => { if (expected !== generation || disposed) return; timer = null; schedule(); }, delay);
      publish(); return;
    }
    const target = desired;
    desired = null;
    const controller = new AbortController();
    active = controller;
    const expected = generation;
    const request: PlanRequest = { modelId: accepted.modelId, startJoints: [...accepted.joints], target: clonePose(target), speedMmPerSecond: accepted.speedMmPerSecond };
    lastDispatch = performance.now();
    publish();
    void (async () => {
      try {
        const result = await options.solve(request, controller.signal);
        if (disposed || generation !== expected || controller.signal.aborted) return;
        if (result.status === 'success') {
          accepted = { ...accepted, joints: [...result.selected.joints], pose: clonePose(result.selected.pose) };
          error = null;
          options.onAccept(result, request);
        } else { error = result.message; }
      } catch (cause) {
        if (!disposed && generation === expected && !controller.signal.aborted) error = cause instanceof Error ? cause.message : '拖动求解未完成';
      } finally {
        if (!disposed && generation === expected && active === controller) {
          active = null;
          schedule();
          publish();
        }
      }
    })();
  }
  function stop(notify: boolean) {
    generation++;
    active?.abort(); active = null;
    if (timer) clearTimeout(timer);
    timer = null; desired = null; error = null; lastDispatch = -Infinity;
    if (notify) publish();
  }
  return {
    update(target) { if (disposed) return; desired = clonePose(target); schedule(); },
    reset(initial) { if (disposed) return; stop(false); accepted = cloneInitial(initial); publish(); },
    cancel() { if (!disposed) stop(true); },
    dispose() { if (disposed) return; disposed = true; stop(false); },
  };
}
