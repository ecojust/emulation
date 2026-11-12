import { describe, expect, it } from 'vitest';
import { insertCurrentSourcePose } from '../src/programming/source-insertion';
import { rapidLanguage } from '../src/programming/languages/rapid';
import { fanucLanguage } from '../src/programming/languages/fanuc';
import { renderProgramTemplate, type ProgramTemplatePoint } from '../src/workspace/program-templates';

const pose = (abb = true): Omit<ProgramTemplatePoint, 'pointName'> => ({
  pose: { position: [350, -15, 620], orientation: [1, 0, 0, 0] },
  joints: [10, 20, -30, 40, -50, 60], configuration: abb ? [0, 0, 0, 1] : null,
  speedMmPerSecond: 125,
});
function insert(model: string, source: string, at = source.length, to = at) {
  const result = insertCurrentSourcePose(model, source, at, to, pose(model.startsWith('irb')));
  let output = source;
  for (const change of [...result.changes].reverse()) output = output.slice(0, change.from) + change.insert + output.slice(change.to);
  expect(result.cursor).toBeLessThanOrEqual(output.length);
  return output;
}
const abb = 'irb1100-4-0.58';
describe('current simulation pose source insertion', () => {
  it('inserts into one-line RAPID without rewriting user source', () => {
    const source = 'MODULE User PROC main() TPWrite "keep this"; ENDPROC ENDMODULE';
    const output = insert(abb, source);
    expect(output).toContain('TPWrite "keep this";');
    expect(output).toContain('CONST robtarget P001 := [[350,-15,620]');
    expect(output).toContain('MoveL P001, [125,500,5000,1000]');
    expect(output.indexOf('CONST')).toBeLessThan(output.indexOf('PROC'));
    expect(rapidLanguage.analyze(output).diagnostics.filter(d => d.severity === 'error')).toEqual([]);
  });
  it('finds a multiline signature and selected module/routine through strings and comments', () => {
    const source = 'MODULE A\nPROC main()\nTPWrite "MODULE Foo PROC nope() ENDPROC ENDMODULE";\nENDPROC\nENDMODULE\nMODULE B\n! PROC fake()\nLOCAL PROC second(\n num arg)\n TPWrite "B";\nENDPROC\nENDMODULE';
    const at = source.indexOf(' TPWrite "B";');
    const output = insert(abb, source, at);
    expect(output.slice(0, output.indexOf('MODULE B'))).toBe(source.slice(0, source.indexOf('MODULE B')));
    expect(output.indexOf('CONST robtarget')).toBeGreaterThan(output.indexOf('MODULE B'));
    expect(output.indexOf('MoveL')).toBeGreaterThan(output.indexOf('LOCAL PROC second'));
    expect(rapidLanguage.analyze(output).diagnostics.filter(d => d.severity === 'error')).toEqual([]);
  });
  it('reserves referenced point identifiers but ignores them in comments and strings; keeps BOM and CRLF', () => {
    const source = '\ufeffMODULE A\r\n! P002 ENDMODULE\r\nPROC main()\r\nMoveL P001,v100,fine,tool0;\r\nTPWrite "P003";\r\nENDPROC\r\nENDMODULE';
    const output = insert(abb, source);
    expect(output.startsWith('\ufeff')).toBe(true);
    expect(output).toContain('CONST robtarget P002');
    expect(output.replaceAll('\r\n', '')).not.toContain('\n');
    expect(output).toContain('TPWrite "P003";');
  });
  it('keeps declarations ahead of TRAP routines and inserts into main by default', () => {
    const source = 'MODULE A\nTRAP alarm\nStop;\nENDTRAP\nPROC main()\nENDPROC\nENDMODULE';
    const output = insert(abb, source);
    expect(output.indexOf('CONST robtarget')).toBeLessThan(output.indexOf('TRAP alarm'));
    expect(output.indexOf('MoveL')).toBeGreaterThan(output.indexOf('PROC main()'));
    expect(rapidLanguage.analyze(output).diagnostics.filter(d => d.severity === 'error')).toEqual([]);
  });
  it('never splits local declaration prefixes or appends motion to an error handler', () => {
    const source = 'MODULE A\nPROC main()\nVAR num a;\nVAR num b;\nTPWrite "normal";\nERROR\nRETRY;\nENDPROC\nENDMODULE';
    for (const at of [source.indexOf('VAR num a;'), source.length]) {
      const output = insert(abb, source, at);
      expect(output.indexOf('MoveL')).toBeGreaterThan(output.indexOf('VAR num b;'));
      expect(output.indexOf('MoveL')).toBeLessThan(output.indexOf('ERROR'));
      expect(rapidLanguage.analyze(output).diagnostics.filter(d => d.severity === 'error')).toEqual([]);
    }
  });
  it('creates a routine in an empty module, but refuses a broken existing routine', () => {
    const output = insert(abb, 'MODULE New ENDMODULE');
    expect(output).toContain('PROC main()');
    expect(rapidLanguage.analyze(output).diagnostics.filter(d => d.severity === 'error')).toEqual([]);
    expect(() => insert(abb, 'MODULE New PROC main( ENDMODULE')).toThrow('不完整');
    expect(() => insert(abb, 'MODULE New')).toThrow('MODULE');
  });
  it('places a FANUC L instruction in MN and its actual joint point in POS without renumbering lines', () => {
    const source = '/PROG TEST\r\n/MN\r\n  10:! P[2] is a comment ;\r\n  20:IF (!DI[1]), JMP LBL[1] ;\r\n  30:L P[1] 100mm/sec FINE ;\r\n/POS\r\n/END';
    const output = insert('fanuc-crx10ia-l', source);
    expect(output).toContain('\r\n: L P[2] 125mm/sec FINE ;\r\n');
    expect(output).toContain('J1=10 deg, J2=20 deg, J3=-30 deg');
    expect(output).toContain('J4=40 deg, J5=-50 deg, J6=60 deg');
    expect(output.indexOf('L P[2]')).toBeLessThan(output.indexOf('/POS'));
    expect(output.indexOf('P[2]{')).toBeGreaterThan(output.indexOf('/POS'));
    expect(output).toContain('  30:L P[1]');
    expect(output).toContain('UF : {{UF}}, UT : {{UT}}');
    expect(output.replaceAll('\r\n', '')).not.toContain('\n');
    expect(fanucLanguage.analyze(output).diagnostics.some(item => item.code === 'fanuc.line-prefix')).toBe(false);
    expect(fanucLanguage.analyze(output).diagnostics.some(item => item.code === 'fanuc.frame-index' && item.severity === 'error')).toBe(true);
    expect(output.replace(': L P[2] 125mm/sec FINE ;\r\n', '').replace(/P\[2\]\{[\s\S]*?\};\r\n/, '')).toBe(source);
  });
  it('adds a Yaskawa Cartesian target and increases count by one for sparse existing indexes', () => {
    const source = renderProgramTemplate('yaskawa-gp7', [{ ...pose(false), pointName: 'P005' }]).code;
    const output = insert('yaskawa-gp7', source);
    expect(output).toContain('///NPOS 2,0,0,0,0,0');
    expect(output).toContain('C00000=350,-15,620,0,0,0');
    expect(output).toContain('MOVL C00000 V=125');
    expect(output.indexOf('C00000=')).toBeLessThan(output.indexOf('//INST'));
    expect(output.indexOf('MOVL C00000')).toBeGreaterThan(output.indexOf('//INST'));
    expect(output).toContain('///RCONF {{RCONF_P001}}');
    expect(output).toContain('MOVL C00004');
  });
  it('chooses the final valid INFORM identifier and refuses exhaustion without adding C09998', () => {
    const header = renderProgramTemplate('yaskawa-gp7', []).code;
    const references = Array.from({ length: 9997 }, (_, index) => `MOVL C${String(index).padStart(5, '0')} V=125`).join('\n');
    const source = header.replace('\nEND', `\n${references}\nEND`);
    const output = insert('yaskawa-gp7', source);
    expect(output).toContain('///RCONF {{RCONF_P9998}}\nC09997=');
    expect(output).toContain('MOVL C09997 V=125');
    expect(output).not.toContain('C09998');
    expect(() => insert('yaskawa-gp7', output)).toThrow('编号已用完');
    const sparse = source.replace('MOVL C00003 V=125\n', '').replace('\nEND', '\nMOVL C09997 V=125\nEND');
    expect(insert('yaskawa-gp7', sparse)).toContain('///RCONF {{RCONF_P004}}\nC00003=');
  });
  it('refuses to write Cartesian coordinates into pulse or user coordinate jobs', () => {
    const source = renderProgramTemplate('yaskawa-gp7', []).code;
    expect(() => insert('yaskawa-gp7', source.replace('POSTYPE BASE', 'POSTYPE PULSE'))).toThrow('标定');
    expect(() => insert('yaskawa-gp7', source.replace('//INST', '///POSTYPE PULSE\n///PULSE\n//INST'))).toThrow('标定');
    expect(() => insert('yaskawa-gp7', source.replace('//INST', '///PULSE\n//INST'))).toThrow('标定');
    expect(() => insert('yaskawa-gp7', source.replace('///NPOS 0,0,0,0,0,0', ''))).toThrow('NPOS');
  });
  it.each([abb, 'fanuc-r2000ib-165f', 'yaskawa-gp110'])('creates an empty source program using the correct brand: %s', model => {
    const output = insert(model, '');
    const expected = renderProgramTemplate(model, [{ ...pose(model === abb), pointName: 'P001' }]).code;
    expect(output).toBe(expected);
  });
  it('retains CR-only line endings and a BOM-only empty document', () => {
    const source = renderProgramTemplate('fanuc-crx10ia-l', []).code.replaceAll('\n', '\r');
    const output = insert('fanuc-crx10ia-l', source, source.indexOf('/MN') + 4);
    expect(output).not.toContain('\n');
    expect(output.indexOf('L P[1]')).toBeGreaterThan(output.indexOf('/MN'));
    expect(output.indexOf('L P[1]')).toBeLessThan(output.indexOf('/POS'));
    expect(insert(abb, '\ufeff')).toMatch(/^\ufeffMODULE/);
    expect(insert(abb, 'MODULE Ärm PROC main() ENDPROC ENDMODULE')).toContain('CONST robtarget');
  });
  it('rejects invalid cursor ranges and non-ABB robconf rather than misusing a stored pose', () => {
    expect(() => insertCurrentSourcePose(abb, '', 1, 1, pose())).toThrow('光标');
    expect(() => insertCurrentSourcePose('fanuc-crx10ia-l', '', 0, 0, pose())).toThrow('非 ABB');
  });
});
