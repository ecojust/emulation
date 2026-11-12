import { describe, expect, it } from 'vitest';
import { fanucLanguage } from '../src/programming/languages/fanuc';
import { parseFanucLs } from '../src/programming/languages/fanuc-parser';
import { renderProgramTemplate, type ProgramTemplatePoint } from '../src/workspace/program-templates';

const group = (frames = 'UF:{{UF}}, UT:{{UT}}', number = 1) => `GP${number}:
 ${frames},
 J1=-10deg, J2=20deg, J3=-30deg, J4=40deg, J5=-50deg, J6=60deg`;
const point = (index = 1, groups = group()) => `P[${index}]{
 ${groups}
};`;
const program = (positions = point(), main = ' : L P[1] 100mm/sec FINE;', application = '') => `/PROG TEMPLATE_TEST
/ATTR
COMMENT="Template test";
${application ? `/APPL\n${application}\n` : ''}/MN
${main}
/POS
${positions}
/END
`;
const template = (source: string) => parseFanucLs(source, { templateParameters: true });

describe('FANUC generated-template parameters', () => {
  it('keeps ordinary LS parsing strict unless template parameters are explicitly enabled', () => {
    const source = program();
    const strict = parseFanucLs(source);
    expect(strict.diagnostics.some(item => item.severity === 'error' && item.code === 'fanuc.frame-index')).toBe(true);
    expect(strict.diagnostics.some(item => item.code === 'template-parameter')).toBe(false);
    expect(strict.positions).toEqual([]);
    expect(parseFanucLs(source, { templateParameters: false })).toEqual(strict);
    expect(parseFanucLs(source, {})).toEqual(strict);
    expect(fanucLanguage.analyze(source).diagnostics).toEqual(strict.diagnostics);
  });

  it('preserves unresolved UF/UT and complete position records without missing-position cascades', () => {
    const source = program(`${point()}\n${point(2)}`, ' : L P[1] 100mm/sec FINE;\n : L P[2] 80mm/sec FINE;');
    const result = template(source);
    expect(result.source).toBe(source);
    expect(result.diagnostics).toHaveLength(4);
    expect(result.diagnostics.every(item => item.code === 'template-parameter' && item.severity === 'warning' && item.message.includes('不能直接用于控制器'))).toBe(true);
    expect(result.diagnostics.map(item => source.slice(item.from, item.to))).toEqual(['{{UF}}', '{{UT}}', '{{UF}}', '{{UT}}']);
    expect(result.positions.map(position => position.index)).toEqual([1, 2]);
    expect(result.positions.map(position => position.groups[0].unresolvedFrames)).toEqual([['UF', 'UT'], ['UF', 'UT']]);
    expect(result.positions[0].groups[0].axes.J1).toEqual({ value: -10, unit: 'deg' });
    expect(result.symbols.map(symbol => symbol.name)).toEqual(['TEMPLATE_TEST', 'P[1]', 'P[2]']);
    expect(fanucLanguage.analyzeTemplate!(source)).toEqual({ diagnostics: result.diagnostics, symbols: result.symbols, folds: result.folds });
  });

  it('tracks each unresolved frame only on its own motion group', () => {
    const result = template(program(point(1, `${group('UF:2, UT:{{UT}}')}\n${group('UF:{{UF}}, UT:4', 2)}\n${group('UF:3, UT:5', 3)}`)));
    expect(result.diagnostics.map(item => item.code)).toEqual(['template-parameter', 'template-parameter']);
    expect(result.positions[0].groups.map(item => item.unresolvedFrames)).toEqual([['UT'], ['UF'], undefined]);
    expect(result.positions[0].groups.map(item => item.number)).toEqual([1, 2, 3]);
  });

  it('leaves fully configured numeric and floating-frame source analysis unchanged', () => {
    const source = program(point(1, `${group('UF:2, UT:3')}\n${group('UF:F, UT:F', 2)}`));
    const strict = parseFanucLs(source);
    expect(strict.diagnostics).toEqual([]);
    expect(template(source)).toEqual(strict);
    expect(strict.positions[0].groups.every(item => item.unresolvedFrames === undefined)).toBe(true);
  });

  it('reports full UTF-16 placeholder ranges after Unicode text without rewriting the source', () => {
    const source = `\uFEFF${program(point(), ' : ! 模板 🦾;\n : L P[1] 100mm/sec FINE;')}`;
    const result = template(source);
    expect(result.source).toBe(source);
    expect(result.diagnostics.map(item => [item.from, item.to])).toEqual([
      [source.indexOf('{{UF}}'), source.indexOf('{{UF}}') + 6],
      [source.indexOf('{{UT}}'), source.indexOf('{{UT}}') + 6],
    ]);
  });

  it.each([
    ['wrong UF parameter', 'UF:{{UT}}, UT:3'],
    ['wrong UT parameter', 'UF:2, UT:{{UF}}'],
    ['unknown parameter', 'UF:{{USER_FRAME}}, UT:3'],
    ['different case', 'UF:{{uf}}, UT:3'],
    ['split opening braces', 'UF:{ {UF}}, UT:3'],
    ['split closing braces', 'UF:{{UF} }, UT:3'],
    ['space before name', 'UF:{{ UF}}, UT:3'],
    ['space after name', 'UF:{{UF }}, UT:3'],
    ['newline in parameter', 'UF:{{\nUF}}, UT:3'],
    ['missing brace', 'UF:{{UF}, UT:3'],
    ['extra brace', 'UF:{{UF}}}, UT:3'],
    ['numeric suffix', 'UF:{{UF}}2, UT:3'],
    ['parameter expression', 'UF:{{UF}}+1, UT:3'],
    ['quoted parameter', 'UF:"{{UF}}", UT:3'],
  ])('keeps %s as an error', (_name, frames) => {
    const result = template(program(point(1, group(frames))));
    expect(result.diagnostics.some(item => item.severity === 'error')).toBe(true);
  });

  it.each([
    ['joint value', program().replace('J1=-10deg', 'J1={{UF}}deg')],
    ['position index', program().replace('P[1]{', 'P[{{UF}}]{')],
    ['motion group', program().replace('GP1:', 'GP{{UF}}:')],
    ['motion speed', program().replace('100mm/sec', '{{UF}}mm/sec')],
    ['frame selection instruction', program(point(), ' : UFRAME_NUM={{UF}};')],
    ['CONFIG', program(point(1, "GP1: UF:2, UT:3, CONFIG:{{UF}}, X=0mm,Y=0mm,Z=0mm,W=0deg,P=0deg,R=0deg"))],
  ])('does not accept a frame parameter in %s', (_name, source) => {
    expect(template(source).diagnostics.some(item => item.severity === 'error')).toBe(true);
  });

  it.each([
    ['invalid speed', program().replace('100mm/sec', '100mm')],
    ['invalid unit', program().replace('J1=-10deg', 'J1=-10invalid')],
    ['duplicate axis', program().replace('J2=20deg', 'J1=20deg')],
    ['missing separator', program().replace('UF:{{UF}}, UT', 'UF:{{UF}} UT')],
    ['missing frame', program().replace('UF:{{UF}}, UT:{{UT}},', 'UF:{{UF}},')],
    ['invalid frame number', program().replace('UT:{{UT}}', 'UT:1.5')],
    ['duplicate group', program(point(1, `${group()}\n${group()}`))],
  ])('retains the actual %s error alongside template warnings', (_name, source) => {
    const result = template(source);
    expect(result.diagnostics.some(item => item.severity === 'error')).toBe(true);
    expect(result.diagnostics.some(item => item.code === 'template-parameter')).toBe(true);
  });

  it('keeps shared /APPL pallet groups strict even while /POS templates are enabled', () => {
    const application = `Palletizing[1];
CONFIG:
 PALLETIZING-B_1 "Fixture";
 TYPE=PALLET;
 INCR=1;
 PAL REG=1;
 ORDER=RCL;
 ROWS=[1];
 COLUMNS=[1];
 LAYERS=[1];
 AUXILIARY POS=[NO];
 APPR=1;
 RTRT=1;
BOTTOM:
 P[1,1,1]{${group('UF:2, UT:3')}};`;
    const baseline = program('', ' : END;', application);
    expect(template(baseline).diagnostics).toEqual([]);
    const source = baseline.replace('UF:2', 'UF:{{UF}}');
    const result = template(source);
    expect(result.diagnostics.some(item => item.code === 'fanuc.frame-index' && item.severity === 'error')).toBe(true);
    expect(result.diagnostics.some(item => item.code === 'template-parameter')).toBe(false);
  });

  const generated = (count: number) => renderProgramTemplate('fanuc-crx10ia-l', Array.from({ length: count }, (_, index): ProgramTemplatePoint => ({
    pointName: `P${String(index + 1).padStart(3, '0')}`, pose: { position: [100, 200, 300], orientation: [1, 0, 0, 0] },
    joints: [-10, 20, -30, 40, -50, 60], configuration: null, speedMmPerSecond: 100,
  }))).code;

  it.each([256, 512])('bounds pending-parameter warnings separately for %i generated positions', count => {
    const result = template(generated(count));
    expect(result.positions).toHaveLength(count);
    expect(result.diagnostics).toHaveLength(33);
    expect(result.diagnostics.every(item => item.code === 'template-parameter' && item.severity === 'warning')).toBe(true);
    expect(result.diagnostics.at(-1)!.message).toContain(`另有 ${count * 2 - 32} 处模板参数待配置`);
    expect(result.positions.at(-1)!.groups[0].unresolvedFrames).toEqual(['UF', 'UT']);
  });

  it('still reports an invalid final coordinate after hundreds of unresolved template parameters', () => {
    const source = generated(512), at = source.lastIndexOf('J6=60 deg');
    expect(at).toBeGreaterThan(0);
    const invalid = source.slice(0, at) + source.slice(at).replace('J6=60 deg', 'J6=invalid deg');
    const result = template(invalid);
    expect(result.diagnostics.some(item => item.code === 'fanuc.limit')).toBe(false);
    expect(result.diagnostics.some(item => item.severity === 'error' && invalid.slice(item.from, item.to) === 'invalid')).toBe(true);
    expect(result.diagnostics.filter(item => item.code === 'template-parameter')).toHaveLength(33);
  });

  it('retains the 500 normal-diagnostic budget without overwriting interleaved template warnings', () => {
    const positions = Array.from({ length: 501 }, (_, index) => point(index + 1, group().replace('J2=20deg', 'J1=20deg'))).join('\n');
    const source = program(positions, ' : END;');
    const result = template(source);
    const normal = result.diagnostics.filter(item => item.code !== 'template-parameter');
    expect(normal).toHaveLength(500);
    expect(normal.at(-1)).toMatchObject({ code: 'fanuc.limit', severity: 'error' });
    const pending = result.diagnostics.filter(item => item.code === 'template-parameter');
    expect(pending).toHaveLength(33);
    expect(pending.at(-1)!.message).toContain('另有 970 处模板参数待配置');
    const configured = source.replaceAll('{{UF}}', '2').replaceAll('{{UT}}', '3');
    const strict = parseFanucLs(configured);
    expect(strict.diagnostics).toHaveLength(500);
    expect(strict.diagnostics.at(-1)).toMatchObject({ code: 'fanuc.limit', severity: 'error' });
    expect(template(configured)).toEqual(strict);
  });
});
