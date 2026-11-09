/** Convert wheel/trackpad units to bounded CSS pixels before changing zoom intent. */
export function normalizeWheelDelta(deltaY: number, deltaMode: number, viewportHeight: number, ctrlKey = false): number {
  if (!Number.isFinite(deltaY)) return 0;
  const pixels = deltaY * (deltaMode === 1 ? 16 : deltaMode === 2 ? (Number.isFinite(viewportHeight) ? Math.max(1, viewportHeight) : 1) : 1);
  // Pinch gestures arrive as small ctrl+wheel deltas on desktop browsers.
  return Math.max(-240, Math.min(240, pixels * (ctrlKey ? 3 : 1)));
}

/** Exponential, elapsed-time interpolation: monotonic, no overshoot at any frame rate. */
export function smoothDistance(current: number, target: number, deltaSeconds: number): number {
  if (!Number.isFinite(current) || !Number.isFinite(target) || !Number.isFinite(deltaSeconds)) return current;
  if (deltaSeconds <= 0) return current;
  const next = target + (current - target) * Math.exp(-14 * deltaSeconds);
  return Math.abs(next - target) < Math.max(1e-6, Math.abs(target) * 1e-5) ? target : next;
}
