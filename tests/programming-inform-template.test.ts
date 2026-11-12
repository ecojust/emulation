import { describe, expect, it } from 'vitest';
import { informLanguage } from '../src/programming/languages/inform';
import { analyzeInform } from '../src/programming/languages/inform-parser';

function job(): string {
  return `/JOB\n//NAME TEMPLATE\n//POS\n///NPOS 1,0,0,0,0,0\n///TOOL {{TOOL}}\n///POSTYPE BASE\n///RECTAN\n///RCONF {{RCONF_P001}}\nC00000=410,30,520,180,0,0\n//INST\n///ATTR SC,RW,RJ\n///GROUP1 RB1\nNOP\nMOVL C00000 V=100\nEND\n`;
}
function manyPointJob(count: number): string {
  const positions = Array.from({ length: count }, (_, index) => ({ parameter: `P${String(index + 1).padStart(3, '0')}`, reference: `C${String(index).padStart(5, '0')}` }));
  return job().replace('///NPOS 1,0,0,0,0,0', `///NPOS ${count},0,0,0,0,0`)
    .replace('///RCONF {{RCONF_P001}}\nC00000=410,30,520,180,0,0', positions.map(({ parameter, reference }) => `///RCONF {{RCONF_${parameter}}}\n${reference}=410,30,520,180,0,0`).join('\n'))
    .replace('MOVL C00000 V=100', positions.map(({ reference }) => `MOVL ${reference} V=100`).join('\n'));
}
const template = (source: string) => informLanguage.analyzeTemplate!(source, 'TEMPLATE.JBI');
const errors = (source: string) => template(source).diagnostics.filter(item => item.severity === 'error');

describe('explicit INFORM simulation-template analysis', () => {
  it('keeps external JBI analysis strict unless template analysis is requested', () => {
    for (const analysis of [informLanguage.analyze(job()), analyzeInform(job()), analyzeInform(job(), undefined, { templateParameters: false })]) {
      expect(analysis.diagnostics.filter(item => item.severity === 'error')).toHaveLength(2);
      expect(analysis.diagnostics.some(item => item.code === 'template-parameter')).toBe(false);
    }
    const result = template(job());
    expect(result.diagnostics).toHaveLength(2);
    for (const item of result.diagnostics) {
      expect(item).toMatchObject({ code: 'template-parameter', severity: 'warning' });
      expect(item.message).toContain('待配置');
      expect(item.message).toContain('不能直接用于控制器');
    }
    expect(result.symbols.some(item => item.name === 'C00000')).toBe(true);
  });

  it('locates only pending parameter text with BOM, CRLF and preceding Unicode', () => {
    const source = ('\ufeff' + job().replace('//NAME TEMPLATE', '//NAME 工件👷').replace('///TOOL', '  ///TOOL')).replaceAll('\n', '\r\n');
    const pending = template(source).diagnostics;
    expect(pending.map(item => source.slice(item.from, item.to))).toEqual(['{{TOOL}}', '{{RCONF_P001}}']);
    expect(pending.every(item => item.code === 'template-parameter')).toBe(true);
  });

  it.each(['001', '999', '1000', '9998'])('recognizes the generated RCONF label P%s without assigning a configuration', number => {
    const source = job().replace('RCONF_P001', 'RCONF_P' + number);
    expect(errors(source)).toEqual([]);
    expect(template(source).diagnostics.some(item => source.slice(item.from, item.to) === `{{RCONF_P${number}}}`)).toBe(true);
  });

  it.each([
    ['{{TOOL}}', '{{USER}}'],
    ['{{TOOL}}', '{{TOOL}'],
    ['{{TOOL}}', '{{ TOOL }}'],
    ['{{TOOL}}', '{{TOOL}}0'],
    ['{{TOOL}}', '64'],
    ['{{TOOL}}', '-1'],
    ['{{RCONF_P001}}', '{{TOOL}}'],
    ['{{RCONF_P001}}', '{{RCONF_P001}'],
    ['{{RCONF_P001}}', '{{RCONF_P000}}'],
    ['{{RCONF_P001}}', '{{RCONF_P0001}}'],
    ['{{RCONF_P001}}', '{{RCONF_P9999}}'],
    ['{{RCONF_P001}}', '{{RCONF_P10000}}'],
    ['{{RCONF_P001}}', '{{RCONF_P001}},0'],
    ['{{RCONF_P001}}', '0,0,0'],
  ])('keeps malformed or out-of-range field %s → %s as an error', (before, after) => {
    expect(errors(job().replace(before, after)).length).toBeGreaterThan(0);
  });

  it.each([
    ['///TOOL {{TOOL}}', '///USER {{TOOL}}'],
    ['///TOOL {{TOOL}}', '//TOOL {{TOOL}}'],
    ['///NPOS 1,0,0,0,0,0', '///NPOS {{TOOL}}'],
    ['C00000=410,30,520,180,0,0', 'C00000={{TOOL}},30,520,180,0,0'],
    ['MOVL C00000 V=100', 'MOVL C00000 V={{TOOL}}'],
  ])('does not admit a parameter in another grammar position: %s', (before, after) => {
    expect(errors(job().replace(before, after)).length).toBeGreaterThan(0);
  });

  it.each(['///TOOL {{TOOL}}', '///RCONF {{RCONF_P001}}'])('keeps %s invalid outside the position section', header => {
    const source = job().replace(header + '\n', '').replace('//INST\n', '//INST\n' + header + '\n');
    const diagnostics = template(source).diagnostics;
    expect(diagnostics.some(item => item.severity === 'error' && item.message.includes('//POS'))).toBe(true);
    const parameter = header.slice(header.indexOf('{{'));
    expect(diagnostics.filter(item => item.code === 'template-parameter').some(item => source.slice(item.from, item.to) === parameter)).toBe(false);
  });

  it.each([
    ['///NPOS 1,0,0,0,0,0', '///NPOS 2,0,0,0,0,0'],
    ['MOVL C00000 V=100', 'MOVL C00001 V=100'],
    ['MOVL C00000 V=100', 'MOVL C00000 V=-1'],
    ['MOVL C00000 V=100', 'IFTHENEXP I000=1\nMOVL C00000 V=100'],
    ['NOP', "'" + 'x'.repeat(33) + '\nNOP'],
  ])('continues checking real source errors alongside pending fields: %s', (before, after) => {
    const result = template(job().replace(before, after));
    expect(result.diagnostics.filter(item => item.code === 'template-parameter')).toHaveLength(2);
    expect(result.diagnostics.some(item => item.severity === 'error')).toBe(true);
  });

  it('reports no template parameters after actual syntactic values have been supplied', () => {
    // Parser fixture only: these values are not a configuration for a real robot.
    const source = job().replace('{{TOOL}}', '12').replace('{{RCONF_P001}}', Array(24).fill(0).join(','));
    expect(template(source).diagnostics).toEqual([]);
    expect(informLanguage.analyze(source).diagnostics).toEqual([]);
  });

  it.each([256, 299, 512])('bounds pending fields separately while analyzing all %i positions', count => {
    const source = manyPointJob(count);
    const result = template(source);
    const pending = result.diagnostics.filter(item => item.code === 'template-parameter');
    expect(result.diagnostics.filter(item => item.severity === 'error')).toEqual([]);
    expect(pending).toHaveLength(33);
    expect(pending.at(-1)?.message).toContain(`另有 ${count + 1 - 32} 处`);
    expect(pending.every(item => item.severity === 'warning')).toBe(true);
    expect(result.symbols.filter(item => item.kind === 'variable')).toHaveLength(count);
    expect(result.symbols.some(item => item.name === `C${String(count - 1).padStart(5, '0')}`)).toBe(true);
  });

  it.each([256, 512])('still reports invalid final coordinates and motion after %i pending positions', count => {
    const reference = `C${String(count - 1).padStart(5, '0')}`;
    const source = manyPointJob(count).replace(`${reference}=410,`, `${reference}=NaN,`)
      .replace(`MOVL ${reference} V=100`, `MOVL ${reference} V=-1`);
    const diagnostics = template(source).diagnostics;
    expect(diagnostics.filter(item => item.code === 'template-parameter')).toHaveLength(33);
    expect(diagnostics.some(item => item.code === 'analysis-limit')).toBe(false);
    expect(diagnostics.some(item => item.severity === 'error' && item.message.includes('位置分量必须为有限数值'))).toBe(true);
    expect(diagnostics.some(item => item.severity === 'error' && item.message.includes('速度超出有效范围'))).toBe(true);
  });

  it('keeps the normal 300-diagnostic budget independent of pending field summaries', () => {
    const source = manyPointJob(512).replace('\nEND\n', '\n' + 'MOVL C00000 V=-1\n'.repeat(310) + 'END\n');
    const diagnostics = template(source).diagnostics;
    expect(diagnostics.filter(item => item.code === 'template-parameter')).toHaveLength(33);
    const syntax = diagnostics.filter(item => item.code !== 'template-parameter');
    expect(syntax).toHaveLength(300);
    expect(syntax.filter(item => item.code === 'analysis-limit')).toHaveLength(1);
    expect(syntax.at(-1)).toMatchObject({ severity: 'error', code: 'analysis-limit' });
  });

  it('retains the strict external JBI limit when unresolved fields are syntax errors', () => {
    const diagnostics = informLanguage.analyze(manyPointJob(512)).diagnostics;
    expect(diagnostics).toHaveLength(300);
    expect(diagnostics.some(item => item.code === 'template-parameter')).toBe(false);
    expect(diagnostics.filter(item => item.code === 'analysis-limit')).toHaveLength(1);
    expect(diagnostics.at(-1)).toMatchObject({ severity: 'error', code: 'analysis-limit' });
  });
});
