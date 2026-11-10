import { describe, expect, it } from 'vitest';
import { analyzeRapid } from '../src/programming/languages/rapid-parser';
import { quaternionDistanceDeg } from '../src/robot/kinematics';
import { eulerZYXToQuaternion, parseRobtarget } from '../src/robot/rapid';
import type { Pose, Vec3 } from '../src/robot/types';
import { createPathModule } from '../src/workspace/path-module';
import type { ProgramPath, ProgramPoint } from '../src/workspace/program';
import { composePose, identityPose, relativePose } from '../src/workspace/frame-graph';
import { renderProgramTemplate, type ProgramTemplateContext } from '../src/workspace/program-templates';

const pose = (position: Vec3, angles: Vec3): Pose => ({ position, orientation: eulerZYXToQuaternion(angles) });
const context = (): ProgramTemplateContext => ({ toolPose: pose([15, -7, 120], [12, 31, -10]), referencePose: pose([140, -30, 220], [90, 15, 30]) });
const path: ProgramPath = { id: 'surface', name: 'Surface', pointIds: ['point-1', 'point-2'] };
const points = (abb = true): ProgramPoint[] => [1, 2].map(n => ({ id: `point-${n}`, pathId: path.id, pointName: `P00${n}`, kind: 'MoveL', revision: 0,
  pose: pose([20 * n, 30, 40], [12, 18, 24]), joints: [2 * n, 20, -30, 4, 40, 6], configuration: abb ? [0, 0, 0, 0] : null, speedMmPerSecond: 50 }));
const near = (a: Pose, b: Pose) => {
  a.position.forEach((value, i) => expect(value).toBeCloseTo(b.position[i], 3));
  expect(quaternionDistanceDeg(a.orientation, b.orientation)).toBeLessThan(1e-4);
};
const literal = (code: string, name: string): unknown[] => JSON.parse(code.match(new RegExp(`${name} := (\\[[^\\n]+\\]);`))![1].replace(/\bTRUE\b/g, 'true').replace(/\bFALSE\b/g, 'false'));
const readPose = (value: unknown): Pose => { const [position, orientation] = value as [Pose['position'], Pose['orientation']]; return { position, orientation }; };

describe('Path-local TCP source conversion', () => {
  it.each(['irb1100-4-0.58', 'fanuc-crx10ia-l', 'yaskawa-gp7'])('preserves the previous default source byte-for-byte for %s when frames are identities', modelId => {
    const input = points(modelId.startsWith('irb'));
    const baseline = renderProgramTemplate(modelId, input, 'SURFACE');
    const explicit = renderProgramTemplate(modelId, input, 'SURFACE', { referencePose: identityPose(), toolPose: identityPose() });
    expect(explicit.code).toBe(baseline.code);
    expect(explicit.unresolvedFields).toEqual(baseline.unresolvedFields);
  });

  it.each(['6.16', '7.21'] as const)('emits complete local targets, tool and reference declarations for RAPID %s', version => {
    const input = points(), frames = context(), before = JSON.stringify({ input, frames });
    const result = createPathModule('irb1100-4-0.58', path, input, version === '6.16' ? 'rapid-rw6' : 'rapid-rw7', [], frames);
    expect(analyzeRapid(result.text, version).diagnostics.filter(d => d.severity === 'error')).toEqual([]);
    expect(result.text).toContain('PERS tooldata SimTool');
    expect(result.text).toContain('PERS wobjdata SimFrame');
    expect(result.text.match(/fine, SimTool\\WObj:=SimFrame;/g)).toHaveLength(2);
    expect(result.text).not.toContain('fine, tool0\\WObj:=wobj0');
    const tool = literal(result.text, 'SimTool'), reference = literal(result.text, 'SimFrame');
    expect(tool[0]).toBe(true); expect(reference.slice(0, 3)).toEqual([false, true, '']);
    near(readPose(tool[1]), frames.toolPose); near(readPose(reference[3]), frames.referencePose); near(readPose(reference[4]), identityPose());
    for (const point of input) {
      const declaration = result.text.match(new RegExp(`CONST robtarget ${point.pointName} := [^\\n]+;`))![0];
      const parsed = parseRobtarget(declaration);
      near(parsed.pose, point.pose); expect(parsed.configuration).toEqual(point.configuration);
      const baseTcp = composePose(readPose(reference[3]), parsed.pose);
      near(baseTcp, composePose(frames.referencePose, point.pose));
    }
    expect(result.text).toContain('uncalibrated simulation placeholder');
    expect(JSON.stringify({ input, frames })).toBe(before);
  });

  it('does not apply a nonzero robot world placement twice in the reference declaration', () => {
    const baseWorld = pose([450, 250, 100], [55, -20, 30]), pathWorld = pose([100, 650, 200], [-35, 14, 22]);
    const frames = { toolPose: context().toolPose, referencePose: relativePose(baseWorld, pathWorld) };
    const result = renderProgramTemplate('irb1100-4-0.475', points(), 'SURFACE', frames);
    const reference = readPose(literal(result.code, 'SimFrame')[3]);
    near(composePose(baseWorld, composePose(reference, points()[0].pose)), composePose(pathWorld, points()[0].pose));
    expect(result.exportable).toBe(false);
    expect(result.unresolvedFields).toContain('工具负载与坐标系现场标定');
  });

  it('retains FANUC solved joints and unresolved UF/UT while documenting transformed TCP coordinates', () => {
    const input = points(false), frames = context();
    const result = createPathModule('fanuc-crx10ia-l', path, input, 'fanuc-r30ib', [], frames);
    expect(result.text).toContain('UF : {{UF}}, UT : {{UT}}');
    expect(result.text).toContain('REFERENCE_BASE_XYZ 140,-30,220');
    expect(result.text).toContain('TOOL_FLANGE_XYZ 15,-7,120');
    expect(result.text).toContain('P001 BASE_TCP_XYZ');
    expect(result.text).toContain('J1=2 deg'); expect(result.text).toContain('J1=4 deg');
    expect([...result.text.matchAll(/\d+:L P\[(\d+)\]/g)].map(match => match[1])).toEqual(['1', '2']);
    expect(result.text).not.toContain('CONFIG'); expect(result.text).not.toContain('robconf');
  });

  it('writes Yaskawa BASE TCP positions and orientations after transforming Path-local coordinates', () => {
    const input = points(false), frames = context(), before = JSON.stringify(input);
    const result = createPathModule('yaskawa-gp7', path, input, 'inform-yrc1000', [], frames);
    expect(result.text).toContain('///POSTYPE BASE'); expect(result.text).toContain('///TOOL {{TOOL}}');
    expect(result.text).toContain('///RCONF {{RCONF_P001}}');
    expect(result.text).toContain("'TOOL_FLANGE_XYZ 15,-7,120");
    for (const [index, point] of input.entries()) {
      const values = result.text.match(new RegExp(`^C${String(index).padStart(5, '0')}=(.+)$`, 'm'))![1].split(',').map(Number);
      const resolved = { position: values.slice(0, 3) as Vec3, orientation: eulerZYXToQuaternion([values[5], values[4], values[3]]) };
      near(resolved, composePose(frames.referencePose, point.pose));
      expect(resolved.position).not.toEqual(point.pose.position);
    }
    expect(JSON.stringify(input)).toBe(before);
  });

  it('does not use coordinate transformation to normalize invalid original target data', () => {
    const bad = points(false); bad[0].pose.orientation = [2, 0, 0, 0];
    expect(() => renderProgramTemplate('yaskawa-gp7', bad, 'SURFACE', context())).toThrow('四元数');
    expect(() => renderProgramTemplate('irb1100-4-0.58', points(), 'SURFACE', { ...context(), toolPose: { position: [0, Infinity, 0], orientation: [1, 0, 0, 0] } })).toThrow();
    expect(() => renderProgramTemplate('irb1100-4-0.58', points(), 'SURFACE', { ...context(), referencePose: { position: [0, 0, 0], orientation: [0, 0, 0, 0] } })).toThrow();
  });
});
