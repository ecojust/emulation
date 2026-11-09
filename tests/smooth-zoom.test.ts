import { describe, expect, it } from 'vitest';
import { normalizeWheelDelta, smoothDistance } from '../src/scene/smooth-zoom';

describe('wheel and trackpad zoom', () => {
  it('normalizes pixel, line and page units and preserves small trackpad movements', () => {
    expect(normalizeWheelDelta(32, 0, 800)).toBe(normalizeWheelDelta(2, 1, 800));
    expect(normalizeWheelDelta(80, 0, 800)).toBe(normalizeWheelDelta(.1, 2, 800));
    expect(normalizeWheelDelta(.25, 0, 800)).toBe(.25);
    expect(normalizeWheelDelta(-.25, 0, 800)).toBe(-.25);
    expect(normalizeWheelDelta(0, 0, 800)).toBe(0);
    expect(normalizeWheelDelta(NaN, 0, 800)).toBe(0);
    expect(normalizeWheelDelta(Infinity, 0, 800)).toBe(0);
    expect(normalizeWheelDelta(1e8, 0, 800)).toBeLessThanOrEqual(240);
    expect(normalizeWheelDelta(-1e8, 0, 800)).toBeGreaterThanOrEqual(-240);
    expect(normalizeWheelDelta(1, 0, 800, true)).toBeGreaterThan(0);
  });

  it('converges progressively with no overshoot in either direction and settles exactly', () => {
    for (const [start, target] of [[1.8, .6], [.6, 4.5]]) {
      let current = start;
      const first = smoothDistance(current, target, 1 / 60);
      expect(first).not.toBe(start);
      expect(first).not.toBe(target);
      for (let frame = 0; frame < 120; frame++) {
        const next = smoothDistance(current, target, 1 / 60);
        expect(next).toBeGreaterThanOrEqual(Math.min(current, target));
        expect(next).toBeLessThanOrEqual(Math.max(current, target));
        current = next;
      }
      expect(current).toBe(target);
    }
    expect(smoothDistance(2, 1, 0)).toBe(2);
    expect(smoothDistance(2, 1, -1)).toBe(2);
    expect(smoothDistance(2, 1, NaN)).toBe(2);
  });

  it('uses elapsed time rather than frame count, giving equivalent 30/60/120Hz motion', () => {
    const distanceAfter = (fps: number) => {
      let distance = 2;
      for (let frame = 0; frame < fps / 5; frame++) distance = smoothDistance(distance, .5, 1 / fps);
      return distance;
    };
    expect(distanceAfter(30)).toBeCloseTo(distanceAfter(60), 10);
    expect(distanceAfter(60)).toBeCloseTo(distanceAfter(120), 10);
  });
});
