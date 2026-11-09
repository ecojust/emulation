import { describe, expect, it } from 'vitest';
import { analyzeRapid, tokenizeRapid } from '../src/programming/languages/rapid-parser';

const module = (body: string) => `MODULE Demo\n${body}\nENDMODULE`;
const routine = (body: string) => module(`PROC main()\n${body}\nENDPROC`);
const errors = (source: string, version: '6.16' | '7.21' = '7.21') => analyzeRapid(source, version).diagnostics.filter(d => d.severity === 'error');

describe('RAPID lexical and versioned kernel grammar', () => {
  it.each([
    'VAR num radix{3}:=[0xFE,0b101,0o17];\nCONST dnum n:=0d13E2;\nPERS string text:="中文🙂, \\07";',
    'LOCAL RECORD pair\nnum left;\nnum right;\nENDRECORD\nALIAS pair other;\nTASK PERS num counter:=0;\nLOCAL VAR pair item:=[1,2];',
    'LOCAL FUNC num f(VAR num a{*,*,*} \\switch enabled | num value, INOUT num result)\nVAR num n:=0;\nRETURN a{1,2,3};\nERROR(ERR_DIVZERO)\nRETURN 0;\nUNDO\nTPWrite "cleanup";\nENDFUNC',
    'LOCAL TRAP stopTrap\nVAR num state;\nstate:=1;\nERROR\nRAISE;\nUNDO\nStop;\nENDTRAP',
    'PROC optional(\\num a \\num b, num c)\nENDPROC',
  ])('accepts declarations, records, arrays, parameters and handlers: %s', body => expect(errors(module(body))).toEqual([]));

  it.each([
    'MoveL [[1,-2,3],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]],v100,fine,tool0\\WObj:=wobj0;',
    'x:=-(a+2)*3 DIV 2 MOD 4;\ny:=NOT a=1 AND b<2 OR c<>3 XOR FALSE;\nz:=Offs(p1,1,2,3);',
    'IF a>0 THEN\nx:=1;\nELSEIF a<0 THEN\nx:=-1;\nELSE\nx:=0;\nENDIF\nIF a=0 TPWrite "zero";',
    'FOR i FROM 1 TO 5 STEP 2 DO\nWHILE ready DO\nBREAK;\nENDWHILE\nCONTINUE;\nENDFOR',
    'TEST choice\nCASE 1,2: TPWrite "one or two";\nCASE 3: RETURN;\nDEFAULT: EXIT;\nENDTEST',
    'CONNECT interrupts{1} WITH stopTrap;\nGOTO retryPoint;\nretryPoint:\nmyProc \\answer?answer, count:=3;\n%"dynamic"+routineName% 1,2;',
    'RAISE ERR_DIVZERO;\nRETRY;\nTRYNEXT;\nRETURN;',
  ])('accepts syntax-complete statement forms: %s', body => expect(errors(routine(body))).toEqual([]));

  it('parses module attributes and both return paths with procedure-only BACKWARD', () => {
    const source = 'MODULE App(SYSMODULE,NOSTEPIN,VIEWONLY,READONLY,NOVIEW)\nPROC main()\nMoveJ p,v100,fine,tool0;\nBACKWARD\nMoveJ start,v100,fine,tool0;\nERROR(1,ERR_DIVZERO)\nRETRY;\nUNDO\nStop;\nENDPROC\nENDMODULE';
    expect(errors(source)).toEqual([]);
    expect(errors(module('FUNC num f()\nRETURN 1;\nBACKWARD\nStop;\nENDFUNC')).length).toBeGreaterThan(0);
  });

  it('preserves Unicode UTF-16 spans, quoted punctuation and EOF comments', () => {
    const source = routine('! 🙂 comment says ENDPROC\nTPWrite "a;!""b\\22\\\\";\n') + '! comment at EOF';
    expect(errors(source)).toEqual([]);
    const { tokens } = tokenizeRapid(source);
    const string = tokens.find(t => t.kind === 'string')!;
    expect(source.slice(string.from, string.to)).toBe('"a;!""b\\22\\\\"');
    const symbols = analyzeRapid(source).symbols;
    expect(symbols.map(s => source.slice(s.from, s.to))).toEqual(['Demo', 'main']);
  });

  it('recognizes all editing placeholders and warns rather than treating them as executable data', () => {
    const source = module('<TDN>\n<DDN>\n<RDN>\nPROC main(<PAR>,\\<ALT>,num x{<DIM>})\n<DDN>\n<VAR>:=<EXP>;\n<SMT>\nIF TRUE THEN\n<EIT>\nENDIF\nTEST x\n<CSE>\nENDTEST\n< ID >;\nENDPROC').replace('< ID >;', 'routine <ARG>;');
    const result = analyzeRapid(source);
    expect(result.diagnostics.filter(d => d.severity === 'error')).toEqual([]);
    expect(result.diagnostics.filter(d => d.code === 'rapid.placeholder').length).toBeGreaterThanOrEqual(12);
  });

  it('distinguishes RW7 reserved loop-control words from RW6 identifiers', () => {
    const source = routine('WHILE TRUE DO\nBREAK;\nCONTINUE;\nENDWHILE');
    expect(errors(source, '7.21')).toEqual([]);
    // In RW6 these spellings can name user procedures; syntax alone cannot reject a call.
    expect(errors(module('VAR num Break:=1;'), '6.16')).toEqual([]);
    expect(errors(module('VAR num Break:=1;'), '7.21').length).toBeGreaterThan(0);
    expect(errors(module('CONST num mask:=0xFF;'), '6.16')).toEqual([]);
  });

  it.each([
    ['CONST missing initial value', module('CONST num value;')],
    ['TASK CONST', module('TASK CONST num value:=1;')],
    ['module statement', module('MoveL p,v100,fine,tool0;')],
    ['declaration after routine', module('PROC main()\nENDPROC\nVAR num late;')],
    ['type after data', module('VAR num first;\nALIAS num tooLate;')],
    ['routine nested in routine', routine('PROC nested()\nENDPROC')],
    ['missing semicolon', routine('MoveL p,v100,fine,tool0\nStop;')],
    ['wrong end block', routine('IF TRUE THEN\nStop;\nENDWHILE')],
    ['malformed expression', routine('value:=1+;')],
    ['empty aggregate', routine('value:=[];')],
    ['invalid lhs', routine('1:=value;')],
    ['array access needs assignment', routine('items{1} p1;')],
    ['missing optional value', routine('MoveL p,v100,fine,tool0\\WObj:=;')],
    ['missing parameter comma', module('PROC p(num x num y)\nENDPROC')],
    ['parameter uses array size', module('PROC p(num x{2})\nENDPROC')],
    ['required arg lacks comma', routine('p 1 2;')],
    ['reserved identifier', module('VAR num IF:=1;')],
    ['leading underscore identifier', module('VAR num _bad:=1;')],
    ['long identifier', module(`VAR num ${'a'.repeat(33)}:=1;`)],
    ['bad radix', routine('x:=0b102;')],
    ['bad exponent', routine('x:=12e+;')],
    ['unfinished string', routine('TPWrite "unterminated;')],
    ['invalid string escape', routine('TPWrite "bad\\q";')],
  ])('rejects %s', (_name, source) => expect(errors(source).length).toBeGreaterThan(0));

  // Each pair exercises a kernel production and a minimal invalid alteration,
  // independently from the manually curated examples above.
  it.each([
    ['module attributes', 'MODULE M(SYSMODULE,NOVIEW)\nENDMODULE', 'MODULE M(SYSMODULE NOVIEW)\nENDMODULE'],
    ['record member', module('RECORD pair\nnum value;\nENDRECORD'), module('RECORD pair\nnum;\nENDRECORD')],
    ['alias', module('ALIAS num count;'), module('ALIAS num;')],
    ['array dimensions', module('VAR num a{2,3,4};'), module('VAR num a{2,3,};')],
    ['function return type', module('FUNC num f()\nRETURN 1;\nENDFUNC'), module('FUNC f()\nRETURN 1;\nENDFUNC')],
    ['trap name', module('TRAP interrupt\nStop;\nENDTRAP'), module('TRAP\nStop;\nENDTRAP')],
    ['error list', module('PROC p()\nERROR(1,ERR_DIVZERO)\nRETRY;\nENDPROC'), module('PROC p()\nERROR(1,)\nRETRY;\nENDPROC')],
    ['handler order', module('PROC p()\nBACKWARD\nERROR\nUNDO\nENDPROC'), module('PROC p()\nUNDO\nERROR\nENDPROC')],
    ['optional alternatives', module('PROC p(\\num a|switch b)\nENDPROC'), module('PROC p(\\num a|)\nENDPROC')],
    ['record component', routine('x:=targets{1}.trans.x;'), routine('x:=targets{1}.;')],
    ['named required argument', routine('SomeProc count:=3;'), routine('SomeProc count:=;')],
    ['conditional argument', routine('SomeProc \\count?parameter;'), routine('SomeProc \\count?;')],
    ['function arguments', routine('x:=Function(1,\\count:=2);'), routine('x:=Function(1,);')],
    ['dynamic procedure', routine('%"job"+name% 1;'), routine('%"job"+name 1;')],
    ['goto target', routine('GOTO done;\ndone:'), routine('GOTO;')],
    ['connect', routine('CONNECT interrupt WITH trap1;'), routine('CONNECT interrupt trap1;')],
    ['return expression', routine('RETURN 1;'), routine('RETURN 1 2;')],
    ['raise expression', routine('RAISE ERR_DIVZERO;'), routine('RAISE +;')],
    ['simple no-arg statement', routine('EXIT;\nRETRY;\nTRYNEXT;'), routine('RETRY 1;')],
    ['compact if', routine('IF TRUE Stop;'), routine('IF TRUE WHILE TRUE DO Stop; ENDWHILE')],
    ['elseif', routine('IF TRUE THEN ELSEIF FALSE THEN ENDIF'), routine('IF TRUE THEN ELSEIF FALSE ENDIF')],
    ['for bounds', routine('FOR i FROM 1 TO 2 DO ENDFOR'), routine('FOR i FROM 1 2 DO ENDFOR')],
    ['while condition', routine('WHILE TRUE DO ENDWHILE'), routine('WHILE DO ENDWHILE')],
    ['test case', routine('TEST 1 CASE 1: DEFAULT: ENDTEST'), routine('TEST 1 CASE 1 DEFAULT: ENDTEST')],
    ['relational expression', routine('x:=a<b;'), routine('x:=a<b<c;')],
  ])('checks legal and invalid %s productions', (_name, valid, invalid) => {
    expect(errors(valid)).toEqual([]);
    expect(errors(invalid).length).toBeGreaterThan(0);
  });

  it('reports multiple local errors with folds and declarations still available', () => {
    const source = module('VAR num first:=0;\nPROC run()\nfirst:=;\nsecond:=;\nWHILE TRUE DO\nStop;\nENDWHILE\nENDPROC');
    const result = analyzeRapid(source);
    expect(result.diagnostics.filter(d => d.severity === 'error')).toHaveLength(2);
    expect(result.symbols.map(s => s.name)).toEqual(['Demo', 'first', 'run']);
    expect(result.folds).toHaveLength(3);
    for (const d of result.diagnostics) expect(d.from <= d.to && d.to <= source.length).toBe(true);
  });

  it('bounds hostile nesting and source size without throwing or infinite recovery', () => {
    const result = analyzeRapid(routine('x:=' + '('.repeat(200) + '0' + ')'.repeat(200) + ';'));
    expect(result.diagnostics.some(d => d.message.includes('128'))).toBe(true);
    expect(analyzeRapid(' '.repeat(1_000_001)).diagnostics[0].code).toBe('rapid.size');
    expect(analyzeRapid('@'.repeat(1000)).diagnostics.length).toBeLessThanOrEqual(200);
  });
});
