import { describe, expect, it } from 'vitest';
import { krlLanguage } from '../src/programming/languages/krl';
import { informLanguage } from '../src/programming/languages/inform';
import { analyzeRapid } from '../src/programming/languages/rapid-parser';
import { parseFanucLs } from '../src/programming/languages/fanuc-parser';

// Independently authored near-neighbour programs: changing a target or argument
// into an expression must not silently broaden the language's grammar.
const krl = (body: string) => `DEF Review()\n${body}\nEND\n`;
const jbi = (body: string) => `/JOB\n//NAME REVIEW\n//POS\n///NPOS 1,0,0,0,0,0\n///POSTYPE PULSE\n///PULSE\nC00000=0,0,0,0,0,0\n//INST\nNOP\n${body}\nEND\n`;
const ls = (body: string, position = '') => `/PROG REVIEW\n/ATTR\nOWNER=MNEDITOR;\n/MN\n1: ${body};\n/POS\n${position}\n/END`;

describe('independent language grammar boundary review', () => {
  it.each([
    'DECL INT f()',
    'DECL INT value+1',
    'EXT Worker(,,)',
    'EXTFCT INT Func(REAL:OUT,,)',
    'value+1=2',
    'Function()=2',
  ])('rejects malformed KRL declaration or assignment: %s', body => {
    const diagnostics = krlLanguage.analyze(krl(body), 'Review.src').diagnostics;
    expect(diagnostics.some(item => item.severity === 'error')).toBe(true);
  });

  it('retains legal KRL declarations, external signatures and indexed field assignment', () => {
    const body = 'DECL INT value[2,3]\nEXT Worker(INT:IN,REAL:OUT)\nEXTFCT INT Func(REAL:IN)\nvalue[1,2]=Func(2.0)\npoint.X=2';
    expect(krlLanguage.analyze(krl(body), 'Review.src').diagnostics).toEqual([]);
  });

  it.each([
    'MOVL C00000 V',
    'MOVL C00000 V=100 25',
    'MOVL C00000 V=100+',
    'CALL JOB:WORK(1,)',
    'CALL JOB:WORK EXTRA',
    '*DONE\nJUMP *DONE EXTRA',
  ])('rejects malformed INFORM operand framing: %s', body => {
    const diagnostics = informLanguage.analyze(jbi(body), 'REVIEW.JBI').diagnostics;
    expect(diagnostics.some(item => item.severity === 'error')).toBe(true);
  });

  it('retains legal INFORM motion, call arguments and conditional local jump', () => {
    const body = '*DONE\nMOVL C00000 V=100 PL=0\nCALL JOB:WORK(1,2)\nJUMP *DONE IF I000<5';
    expect(informLanguage.analyze(jbi(body), 'REVIEW.JBI').diagnostics).toEqual([]);
  });

  // YRC1000 178649-1CD rev.10 §§2.2 CALL (2-36), LABEL (2-50),
  // RET (2-54): these are legal core operand forms, not controller options.
  it.each([
    'CALL B002 IF IN#(2)=ON',
    'RET LI002',
    '*42\nJUMP *42 IF I002<4',
  ])('accepts documented INFORM dynamic/return/label forms: %s', body => {
    expect(informLanguage.analyze(jbi(body), 'REVIEW.JBI').diagnostics.filter(item => item.severity === 'error')).toEqual([]);
  });

  // RW7 kernel 3HAC065039-001 rev.K §§1.6 / 2.5 explicitly classify
  // an out-of-range exponent as a lexical error, before type checking.
  it.each(['2E52786', '0d2E52786'])('rejects overflowing RAPID numeric literal %s', literal => {
    const source = `MODULE Review\nPROC main()\nvalue:=${literal};\nENDPROC\nENDMODULE`;
    expect(analyzeRapid(source).diagnostics.some(item => item.severity === 'error')).toBe(true);
  });

  it.each(['6.16', '7.21'] as const)('accepts independently combined RAPID kernel forms in %s', version => {
    const source = `MODULE Review(NOSTEPIN)\nVAR robtarget targets{2,3,4};\nCONST dnum limit:=0d25E2;\nPROC main(\\num value)\nVAR num n:=0xFA;\n%"review"+NumToStr(n,0)% \\value?value,targets{1,2,3}.trans.x;\nIF n>0 THEN\nTEST n\nCASE 1,2: n:=n+1;\nDEFAULT: RETURN;\nENDTEST\nENDIF\nENDPROC\nENDMODULE ! EOF comment says MODULE ENDPROC`;
    expect(analyzeRapid(source, version).diagnostics).toEqual([]);
  });

  // FANUC Basic B-83284EN/09 §4.15.8 permits reading/writing a system
  // variable. An arithmetic expression is not a writable variable path.
  it('rejects arithmetic on the left of a FANUC system-variable assignment', () => {
    const document = parseFanucLs(ls('$SCR.$MAXNUMTOOL+1=10'));
    expect(document.diagnostics.some(item => item.severity === 'error')).toBe(true);
  });

  it('accepts FANUC multidimensional system-variable paths for reading and writing', () => {
    const document = parseFanucLs(ls('$MNUFRAME[1,2].$X=10;\n2: R[3]=$MNUFRAME[1,2].$X'));
    expect(document.diagnostics.filter(item => item.severity === 'error')).toEqual([]);
  });

  it.each(['UF:0 UT:1', 'UF:0,,UT:1'])('rejects a malformed FANUC position field separator: %s', fields => {
    const document = parseFanucLs(ls('J P[1] 10% FINE', `P[1]{GP1: ${fields},J1=0deg,J2=0deg,J3=0deg,J4=0deg,J5=0deg,J6=0deg};`));
    expect(document.diagnostics.some(item => item.severity === 'error')).toBe(true);
  });

  it('retains separate FANUC Cartesian and joint motion groups in one position', () => {
    const document = parseFanucLs(ls('J P[1] 10% FINE', `P[1]{\nGP1: UF:0,UT:1,CONFIG:'N U T,0,0,0',X=400mm,Y=20mm,Z=500mm,W=0deg,P=90deg,R=0deg\nGP2: UF:0,UT:0,J1=40mm\n};`));
    expect(document.diagnostics.filter(item => item.severity === 'error')).toEqual([]);
    expect(document.positions[0].groups.map(group => [group.number, group.representation])).toEqual([[1, 'cartesian'], [2, 'joint']]);
    expect(document.positions[0].groups[1].axes.J1).toEqual({ value: 40, unit: 'mm' });
  });
});
