import { describe, expect, it } from 'vitest';
import { parseGrindingTargets, formatGrindingTargets } from '../src/workspace/PathPlanningDialog';
import type { Pose } from '../src/robot/types';

function samePhysicalOrientation(a: readonly number[], b: readonly number[]) {
  const dot = a.reduce((sum, value, index) => sum + value * b[index], 0) / (Math.hypot(...a) * Math.hypot(...b));
  return Math.abs(1 - Math.abs(dot));
}

describe('path planning target poses', () => {
  it('preserves each distinct pose while normalizing quaternion scale and accepting equivalent signs', () => {
    const targets = parseGrindingTargets('# world positions and independent tool orientations\n10 20 30 2 2 0 0\n40,50,60,3,0,3,0\n70；80；90；-4；0；0；-4');
    expect(targets.map(target => target.position)).toEqual([[10, 20, 30], [40, 50, 60], [70, 80, 90]]);
    const expected = [[1, 1, 0, 0], [1, 0, 1, 0], [1, 0, 0, 1]];
    for (const [index, target] of targets.entries()) {
      expect(Math.hypot(...target.orientation)).toBeCloseTo(1, 14);
      expect(samePhysicalOrientation(target.orientation, expected[index])).toBeLessThan(1e-14);
      if (index > 0) expect(samePhysicalOrientation(target.orientation, targets[0].orientation)).toBeGreaterThan(.1);
    }
  });

  it('reopening and formatting a path retains target position precision and physical orientation', () => {
    const targets: Pose[] = [
      { position: [123.123456789123, 1e-11, -2e-12], orientation: [Math.cos(.12345), Math.sin(.12345), 0, 0] },
      { position: [-999999.123456789, 987.987654321987, 4e-12], orientation: [-Math.cos(.54321), 0, -Math.sin(.54321), 0] },
    ];
    const original = structuredClone(targets);
    const restored = parseGrindingTargets(formatGrindingTargets(targets));
    expect(targets).toEqual(original);
    for (const [index, target] of restored.entries()) {
      expect(target.position).toEqual(original[index].position);
      expect(samePhysicalOrientation(target.orientation, original[index].orientation)).toBeLessThan(1e-14);
    }
  });
});
