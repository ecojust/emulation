import { describe, expect, it } from 'vitest';
import { krlLanguage } from '../src/programming/languages/krl';
const src = (body: string) => `&ACCESS RVP\n&REL 1\nDEF Main()\nDECL INT i\n${body}\nEND\n`;
const errors = (source: string, file = 'Main.src') => krlLanguage.analyze(source, file).diagnostics.filter(item => item.severity === 'error');
describe('KRL core parser with explicit uncovered capabilities', () => {
  it.each([
    ['expressions', 'i=(1+2)*3\n$OUT[1]=TRUE\nBAS(#INITMOV,0)'],
    ['typed structure and array', 'DECL REAL values[3]\nDECL E6POS point\npoint={E6POS:X 400,Y 20,Z 500,A 0,B 90,C 0,S 6,T 0}\nvalues[1]=point.X'],
    ['condition', 'IF i==1 THEN\nWAIT FOR $IN[1]\nELSE\nWAIT SEC 1.0\nENDIF'],
    ['loop families', 'FOR i=1 TO 3 STEP 1\nWHILE i<2\nEXIT\nENDWHILE\nENDFOR\nREPEAT\ni=i+1\nUNTIL i>3\nLOOP\nEXIT\nENDLOOP'],
    ['switch', 'SWITCH i\nCASE 1,2\ni=3\nDEFAULT\ni=0\nENDSWITCH'],
    ['geometry and motion', 'PTP {A1 0,A2 -90,A3 90,A4 0,A5 30,A6 0}\nLIN point C_DIS\nCIRC {X 300,Y 0,Z 400},{X 400,Y 20,Z 400} CA=90 C_DIS'],
    ['spline', 'SPLINE\nSLIN {X 10,Y 20,Z 30} WITH $VEL.CP=0.1\nSCIRC point,point\nENDSPLINE\nPTP_SPLINE\nSPTP point\nENDSPLINE'],
    ['interrupt', 'GLOBAL INTERRUPT DECL 3 WHEN $IN[1] DO StopRoutine()\nINTERRUPT ON 3\nINTERRUPT OFF 3'],
    ['labels', 'again:\ni=i+1\nGOTO again'],
  ])('parses %s without errors', (_name, body) => expect(errors(src(body))).toEqual([]));
  it('parses DAT data, ENUM and typed routines', () => {
    const data = 'DEFDAT Main PUBLIC\nENUM State Ready,Busy,Done\nDECL E6POS point={X 400,Y 20,Z 500,A 0,B 0,C 0}\nDECL INT count=0\nENDDAT\n';
    expect(errors(data, 'Main.dat')).toEqual([]);
    expect(errors('DEFFCT REAL Calc(value:IN)\nDECL REAL value\nRETURN value*2\nENDFCT\n')).toEqual([]);
  });
  it.each([
    ['IF lacks THEN', 'IF i==0\ni=2\nENDIF'],
    ['wrong closer', 'IF i==0 THEN\nENDWHILE'],
    ['expression missing RHS', 'i=1+'],
    ['bad assignment equality', 'IF i=0 THEN\nENDIF'],
    ['unclosed structure', 'LIN {X 1,Y 2'],
    ['CIRC needs two points', 'CIRC point'],
    ['undefined label', 'GOTO missing'],
    ['EXIT outside loop', 'EXIT'],
    ['empty WAIT', 'WAIT SEC'],
    ['missing FOR bound', 'FOR i=1\nENDFOR'],
  ])('rejects %s', (_name, body) => expect(errors(src(body)).length).toBeGreaterThan(0));
  it('marks technology statements as unchecked and never invents a KUKA model', () => {
    const analysis = krlLanguage.analyze(src('TRIGGER WHEN PATH=10 DELAY=0 DO Handler()\nCUSTOM_TECH 4'));
    expect(analysis.diagnostics.filter(item => item.code === 'krl-unchecked')).toHaveLength(2);
    expect(krlLanguage.limitations.join('')).toContain('没有 KUKA 机器人模型');
  });
  it('returns symbols, UTF-16 positions and matched source folds', () => {
    const source = src('; 🤖中文\nIF i<1 THEN\ni=2\nENDIF\nGOTO missing');
    const analysis = krlLanguage.analyze(source);
    const missing = analysis.diagnostics.find(item => item.message.includes('未定义标签'))!;
    expect(source.slice(missing.from, missing.to)).toBe('missing');
    expect(analysis.symbols.map(item => item.name)).toEqual(expect.arrayContaining(['MAIN', 'I']));
    expect(analysis.folds.some(item => source.slice(item.from, item.to).includes('i=2'))).toBe(true);
  });
  it('bounds input size, recursive expressions and diagnostics', () => {
    expect(errors(src(`i=${'('.repeat(140)}1${')'.repeat(140)}`)).some(item => item.message.includes('128'))).toBe(true);
    expect(krlLanguage.analyze(' '.repeat(1_048_577)).diagnostics[0].severity).toBe('error');
    const analysis = krlLanguage.analyze(src('i=\n'.repeat(500)));
    expect(analysis.diagnostics).toHaveLength(300);
    expect(analysis.diagnostics.at(-1)?.code).toBe('analysis-limit');
  });
});
