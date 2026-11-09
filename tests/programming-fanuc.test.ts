import { describe, expect, it } from 'vitest';
import { fanucLanguage } from '../src/programming/languages/fanuc';
import { parseFanucLs } from '../src/programming/languages/fanuc-parser';

const position = (index: number) => `P[${index}: Authored fixture] {
 GP1:
 UF : 0, UT : 1, CONFIG : 'N U T, 0, 0, 0',
 X = 100.0 mm, Y = -20.0 mm, Z = 300.0 mm,
 W = 180.0 deg, P = 0.0 deg, R = 90.0 deg,
 E1 = 250.0 mm
 GP2:
 UF : 0, UT : 0, J1 = 45.0 deg
};`;
const program = (main: string, positions = '', attributes = 'COMMENT = "Authored tests";') => `/PROG DEMO
/ATTR
${attributes}
/MN
${main}
/POS
${positions}
/END
`;
const errors = (source: string) => fanucLanguage.analyze(source).diagnostics.filter(item => item.severity === 'error');

describe('FANUC TP / LS structural grammar', () => {
  it('parses repeated motion groups and external axes without collapsing them into one six-axis robot', () => {
    const source = program('   1:J P[1] 20% FINE;\n   2:L P[2] 100mm/sec CNT50 Offset,PR[1];', position(1) + '\n' + position(2));
    const document = parseFanucLs(source);
    expect(errors(source)).toEqual([]);
    expect(document.source).toBe(source);
    expect(document.positions).toHaveLength(2);
    expect(document.positions[0].groups.map(group => [group.number, group.representation])).toEqual([[1, 'cartesian'], [2, 'joint']]);
    expect(document.positions[0].groups[0].axes.E1).toEqual({ value: 250, unit: 'mm' });
    expect(document.positions[0].groups[1].axes.J1).toEqual({ value: 45, unit: 'deg' });
    expect(document.symbols.map(symbol => symbol.name)).toEqual(['DEMO', 'P[1]', 'P[2]']);
    expect(document.folds.length).toBeGreaterThan(3);
  });

  it('supports unnumbered instructions and physical continuation lines for circular moves and SELECT', () => {
    const source = program(` : C P[1]
 : P[2] 200mm/sec FINE;
 : SELECT R[1]=1,JMP LBL[10]
 : =2,CALL TASK(1,'two')
 : ELSE,JMP LBL[20];
 : LBL[10];
 : LBL[20];`, position(1) + position(2));
    expect(errors(source)).toEqual([]);
    expect(parseFanucLs(source).statements[0].kind).toBe('C');
  });

  it('parses nested expressions, indirect indexes, group-qualified PR components and system fields', () => {
    const source = program(` : R[1]=(ABS[R[2]]+R[R[2]]*2);
 : PR[GP1:1,2:offset]=R[1];
 : DO[1:Ready ! marker]=(DI[1] AND !F[2]);
 : SR[1]='hello';
 : $MOR_GRP[1].$ROB_MOVE=0;
 : UFRAME_NUM[GP1]=AR[1];
 : TIMER[1]=START;
 : WAIT (DI[1]=ON OR R[1]>=2) TIMEOUT,LBL[99];
 : LBL[99];`);
    expect(errors(source)).toEqual([]);
  });

  it('matches nested IF/FOR blocks and both SELECT continuation representations', () => {
    const source = program(` : FOR R[1]=1 TO 3;
 : IF (R[1]>1) THEN;
 : CALL DO_WORK(R[1],SR[1]);
 : ELSE;
 : PAUSE;
 : ENDIF;
 : ENDFOR;
 : SELECT R[1]=1,JMP LBL[1];
 : =2,CALL OTHER;
 : ELSE,JMP LBL[2];
 : LBL[1];
 : LBL[2];
 : END;`);
    expect(errors(source)).toEqual([]);
    expect(fanucLanguage.analyze(source).folds.filter(fold => source.slice(fold.from, fold.to).includes('CALL DO_WORK')).length).toBeGreaterThanOrEqual(2);
  });

  it('checks timing, offsets, skip labels and parameterless RUN without executing them', () => {
    const source = program(` : SKIP CONDITION DI[1]=ON;
 : OFFSET CONDITION PR[1],UFRAME[2];
 : L P[1] R[1]mm/sec FINE ACC100 TB 0.1sec,CALL OPEN_HAND Skip,LBL[9],PR[3]=LPOS;
 : RUN BACKGROUND;
 : DO[1]=PULSE,0.1(sec);
 : WAIT 0.2(sec);
 : LBL[9];`, position(1));
    expect(errors(source)).toEqual([]);
    expect(fanucLanguage.entries.find(entry => entry.name === 'RUN')!.description).toContain('异步');
  });

  it('parses distinct TP string-operation productions and rejects missing parameters', () => {
    expect(errors(program(` : R[1]=STRLEN SR[2];
 : R[2]=FINDSTR SR[3],AR[1];
 : SR[4]=SUBSTR SR[3],R[1],5;
 : R[5]=ATAN2[R[1],R[2]];
 : ERROR_PROG=FAULT_HANDLER;`))).toEqual([]);
    for (const instruction of ['R[1]=STRLEN', 'R[1]=FINDSTR SR[1]', 'SR[1]=SUBSTR SR[2],1', 'R[1]=ATAN2[R[2]]', 'R[1]=SIN[R[2],R[3]]', 'R[1,2]=0', 'PR[1,2,3]=0']) {
      expect(errors(program(` : ${instruction};`)), instruction).not.toHaveLength(0);
    }
  });

  it('reports wrong motion arity, speed, unit and termination syntax rather than accepting keywords alone', () => {
    for (const statement of [
      'C P[1] 100mm/sec FINE', 'L P[1] 100 FINE', 'J P[1] 101% FINE',
      'L P[1] AR[1]mm/sec FINE', 'L P[1] 100mm/sec CNT101', 'J P[1] 10% FINE ACC501',
      'L P[1] 100mm/sec FINE Offset,R[1]',
    ]) expect(errors(program(` : ${statement};`, position(1))), statement).not.toHaveLength(0);
  });

  it('rejects malformed call/control-flow/expression forms while recovering later symbols', () => {
    const source = program(` : RUN TASK(1);
 : IF (R[1]>0) THEN;
 : ENDFOR;
 : R[(AR[1]+1)]=2;
 : DO[1]=(DI[1] AND);
 : JMP LBL[404];
 : LBL[9];
 : LBL[9];`);
    const result = fanucLanguage.analyze(source);
    expect(result.diagnostics.some(item => item.message.includes('不接受参数列表'))).toBe(true);
    expect(result.diagnostics.some(item => item.code === 'fanuc.unclosed-block')).toBe(true);
    expect(result.diagnostics.some(item => item.code === 'fanuc.missing-label')).toBe(true);
    expect(result.diagnostics.some(item => item.code === 'fanuc.duplicate-label')).toBe(true);
    expect(result.symbols.map(symbol => symbol.name)).toContain('LBL[9]');
  });

  it('validates position representations, configuration, duplicate groups, units and initialization', () => {
    for (const corrupt of [
      position(1).replace('GP2:', 'GP1:'), position(1).replace('X = 100.0 mm', 'X = 100.0 deg'),
      position(1).replace("'N U T, 0, 0, 0'", "'invented'"), position(1).replace('R = 90.0 deg,', ''),
      position(1).replace('GP2:', 'GP9:'),
    ]) expect(errors(program(' : END;', corrupt))).not.toHaveLength(0);
    const result = fanucLanguage.analyze(program(' : END;', position(1).replace('X = 100.0 mm', 'X = ***** mm')));
    expect(result.diagnostics.some(item => item.code === 'fanuc.uninitialized-position')).toBe(true);
  });

  it('keeps unknown application statements and macro text with explicit unsupported diagnostics', () => {
    const source = program(' : VISION RUN_FIND CAMERA;\n : USER_MACRO;\n : L P[1] 50mm/sec FINE FUTURE_OPTION[1];', position(1));
    const result = parseFanucLs(source);
    expect(result.source).toBe(source);
    expect(result.statements.filter(statement => statement.kind === 'unsupported')).toHaveLength(2);
    expect(result.diagnostics.filter(item => item.code?.startsWith('fanuc.unsupported'))).toHaveLength(3);
    expect(result.diagnostics.every(item => item.code !== 'fanuc.profile-unverified')).toBe(true);
  });

  it('has explicit v9.40 local-register metadata and checks declaration capacity', () => {
    const source = program(' : R[10001]=AR[1];', '', 'LOCAL_REGISTERS = 2,1,0;');
    expect(errors(source)).toEqual([]);
    expect(fanucLanguage.analyze(source).diagnostics.some(item => item.code === 'fanuc.version')).toBe(true);
    expect(fanucLanguage.entries.find(entry => entry.name === 'LOCAL_REGISTERS')!.availableSince).toContain('9.40');
    expect(errors(program(' : END;', '', 'LOCAL_REGISTERS = 101,31,21;'))).not.toHaveLength(0);
  });

  it('preserves UTF-16 offsets for non-ASCII comments and does not inspect quoted/comment instruction names', () => {
    const source = program(' : ! 🤖 IF THEN JMP LBL[999];\n : MESSAGE[机器人];\n : LBL[10];\n : JMP LBL[10];');
    const analysis = fanucLanguage.analyze(source);
    expect(errors(source)).toEqual([]);
    const label = analysis.symbols.find(symbol => symbol.name === 'LBL[10]')!;
    expect(source.slice(label.from, label.to)).toBe('LBL[10]');
    expect(label.from).toBe(source.indexOf('LBL[10]'));
    for (const item of analysis.diagnostics) { expect(item.from).toBeGreaterThanOrEqual(0); expect(item.to).toBeLessThanOrEqual(source.length); }
  });

  it('bounds nested syntax, file size and diagnostics without claiming unchecked input is valid', () => {
    const nested = program(` : R[1]=${'('.repeat(70)}1${')'.repeat(70)};`);
    expect(errors(nested).some(item => item.message.includes('64'))).toBe(true);
    expect(fanucLanguage.analyze(' '.repeat(2_000_001)).diagnostics).toMatchObject([{ code: 'fanuc.limit', severity: 'error' }]);
    const many = program(Array.from({ length: 600 }, () => ' : R[1]=;').join('\n'));
    expect(fanucLanguage.analyze(many).diagnostics.length).toBeLessThanOrEqual(500);
  });

  it('provides signatures, parameter help, option/version distinctions and insertion snippets', () => {
    expect(fanucLanguage.entries.length).toBeGreaterThan(70);
    expect(fanucLanguage.entries.find(entry => entry.name === 'L')!.parameters).toHaveLength(4);
    expect(fanucLanguage.entries.find(entry => entry.name === 'L')!.insertText).toContain('${index}');
    expect(fanucLanguage.entries.find(entry => entry.name === 'S')!.requiresOption).toBeTruthy();
    expect(fanucLanguage.sources.some(source => source.revision?.length === 40)).toBe(true);
    expect(fanucLanguage.limitations.join('')).toContain('非空 POINT_LOGIC');
    for (const mode of ['B', 'BX', 'E', 'EX', 'END']) {
      const entry = fanucLanguage.entries.find(item => item.name === `PALLETIZING-${mode}`);
      expect(entry?.insertText).toBe(`PALLETIZING-${mode}_\${number}`);
      expect(entry?.parameters?.[0].type).toBe('integer 1..16');
    }
  });
});
