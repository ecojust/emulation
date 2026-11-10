import { describe, expect, it } from 'vitest';
import type { ProgramDialectId } from '../src/programming/profiles';
import { createPathModule } from '../src/workspace/path-module';
import type { ProgramPath, ProgramPoint } from '../src/workspace/program';

const path = (name = 'Path 1', pointIds: readonly string[] = ['point-1']): ProgramPath =>
  Object.freeze({ id: 'path-1', name, pointIds: Object.freeze([...pointIds]) });

function point(ordinal = 1, abb = true, pathId = 'path-1'): ProgramPoint {
  const value: ProgramPoint = {
    id: `point-${ordinal}`, pointName: `P${String(ordinal).padStart(3, '0')}`,
    kind: 'MoveL', pathId, revision: 0,
    pose: { position: [100 + ordinal, 20, 300], orientation: [1, 0, 0, 0] },
    joints: [ordinal, 20, 30, 40, 50, 60], configuration: abb ? [0, 0, 0, 0] : null,
    speedMmPerSecond: 100,
  };
  Object.freeze(value.pose.position); Object.freeze(value.pose.orientation); Object.freeze(value.pose);
  Object.freeze(value.joints); if (value.configuration) Object.freeze(value.configuration);
  return Object.freeze(value);
}

describe('Path source modules', () => {
  it.each([
    ['irb1100-4-0.475', 'rapid-rw7', 'modx', 'MODULE PATH1', true],
    ['irb1100-4-0.58', 'rapid-rw6', 'mod', 'MODULE PATH1', true],
    ['fanuc-crx10ia-l', 'fanuc-r30ib', 'ls', '/PROG PATH1', false],
    ['fanuc-r2000ib-165f', 'fanuc-r30ib', 'ls', '/PROG PATH1', false],
    ['yaskawa-gp7', 'inform-yrc1000', 'jbi', '//NAME PATH1', false],
    ['yaskawa-gp110', 'inform-yrc1000', 'jbi', '//NAME PATH1', false],
  ] as const)('creates the %s %s module with a matching header and extension', (modelId, dialect, extension, header, abb) => {
    const result = createPathModule(modelId, path(), [point(1, abb)], dialect, []);
    expect(result.name).toBe(`PATH1.${extension}`);
    expect(result.text).toContain(header);
    expect(result.text).toContain(abb ? 'Simulation template' : 'SIMULATION TEMPLATE');
    if (modelId.startsWith('fanuc')) expect(result.text).toContain('UF : {{UF}}, UT : {{UT}}');
    if (modelId.startsWith('yaskawa')) expect(result.text).toContain('///RCONF {{RCONF_P001}}');
  });

  it.each([
    ['irb1100-4-0.475', 'rapid-rw7', true, /MoveL (P\d+),/g, ['P004', 'P001']],
    ['fanuc-crx10ia-l', 'fanuc-r30ib', false, /\d+:L (P\[\d+\])/g, ['P[4]', 'P[1]']],
    ['yaskawa-gp7', 'inform-yrc1000', false, /^MOVL (C\d+) /gm, ['C00003', 'C00000']],
  ] as const)('keeps %s Path order and stable point numbers without borrowing another Path', (modelId, dialect, abb, pattern, expected) => {
    const selectedPath = path('Path 2', ['point-4', 'point-1']);
    const inputs = Object.freeze([point(1, abb), point(2, abb, 'path-other'), point(4, abb)]);
    const before = JSON.stringify({ selectedPath, inputs });
    const result = createPathModule(modelId, selectedPath, inputs, dialect, []);
    expect([...result.text.matchAll(pattern)].map(match => match[1])).toEqual(expected);
    expect(result.text).not.toMatch(/P002|P\[2\]|C00001/);
    expect(JSON.stringify({ selectedPath, inputs })).toBe(before);
    expect(selectedPath.pointIds).toEqual(['point-4', 'point-1']);
  });

  it('creates independent names across case, extensions and successive generations', () => {
    const sourcePath = path(), points = [point()];
    const existing = ['path1.MOD', 'PATH1_2.sysx', 'other.modx'];
    const first = createPathModule('irb1100-4-0.475', sourcePath, points, 'rapid-rw7', existing);
    const second = createPathModule('irb1100-4-0.475', sourcePath, points, 'rapid-rw6', [...existing, first.name]);
    expect(first.name).toBe('PATH1_3.modx');
    expect(first.text).toContain('MODULE PATH1_3\n');
    expect(second.name).toBe('PATH1_4.mod');
    expect(second.text).toContain('MODULE PATH1_4\n');
    expect(existing).toEqual(['path1.MOD', 'PATH1_2.sysx', 'other.modx']);
    expect(sourcePath.name).toBe('Path 1');
  });

  it.each([
    ['上料轨迹', 'P_PATH'], ['MODULE', 'P_MODULE'], ['CON', 'P_CON'],
    ['a very long path title', 'P_A_VERY'], ['../../bad\\name\n/MN', 'P_BAD_NA'],
    ['Path 123456789', 'PATH1234'], ['ébauche', 'P_EBAUCH'],
  ])('makes a safe short identifier from %s', (title, expectedStem) => {
    const result = createPathModule('fanuc-crx10ia-l', path(title), [point(1, false)], 'fanuc-r30ib', []);
    expect(result.name).toBe(`${expectedStem}.ls`);
    expect(result.name).toMatch(/^[A-Z][A-Z0-9_]{0,7}\.ls$/);
    expect(result.text).toContain(`/PROG ${expectedStem}\n`);
  });

  it('keeps collision suffixes within the eight-character controller limit', () => {
    const existing = ['P_LONG_N.ls', ...Array.from({ length: 10 }, (_, index) => {
      const suffix = `_${index + 2}`;
      return `${'P_LONG_N'.slice(0, 8 - suffix.length)}${suffix}.ls`;
    })];
    const result = createPathModule('fanuc-crx10ia-l', path('long name'), [point(1, false)], 'fanuc-r30ib', existing);
    expect(result.name).toBe('P_LON_12.ls');
    expect(result.text).toContain('/PROG P_LON_12\n');
  });

  it('rejects empty paths, missing points, duplicated references and wrong ownership', () => {
    const generate = (input: ProgramPath, points: readonly ProgramPoint[]) =>
      createPathModule('irb1100-4-0.475', input, points, 'rapid-rw7', []);
    expect(() => generate(path('Path 1', []), [])).toThrow('Path 为空');
    expect(() => generate(path(), [])).toThrow('已删除');
    expect(() => generate(path('Path 1', ['point-1', 'point-1']), [point()])).toThrow('重复');
    expect(() => generate(path(), [point(), point()])).toThrow('重复');
    expect(() => generate(path(), [point(1, true, 'path-other')])).toThrow('不属于此 Path');
  });

  it('rejects unsupported models, unknown dialects and mismatched manufacturers', () => {
    expect(() => createPathModule('kuka-kr6', path(), [point()], 'krl-kss8', [])).toThrow('型号');
    expect(() => createPathModule('irb1100-4-0.475', path(), [point()], 'unknown' as ProgramDialectId, [])).toThrow('未知');
    expect(() => createPathModule('irb1100-4-0.475', path(), [point()], 'fanuc-r30ib', [])).toThrow('不适用');
    expect(() => createPathModule('fanuc-crx10ia-l', path(), [point(1, false)], 'rapid-rw7', [])).toThrow('不适用');
  });
});
