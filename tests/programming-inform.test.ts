import { describe, expect, it } from 'vitest';
import { informLanguage } from '../src/programming/languages/inform';
import { informInstructionEntries, informIndexSource } from '../src/programming/languages/inform-catalog';

function job(body: string, header = ''): string {
  return `/JOB\n//NAME EDITOR_TEST\n//POS\n///NPOS 1,0,0,0,0,0\n///TOOL 0\n///POSTYPE BASE\n///RECTAN\n///RCONF ${Array(24).fill(0).join(',')}\nC00000=400,20,500,180,0,0\n//INST\n///ATTR SC,RW,RJ\n///GROUP1 RB1\n${header}NOP\n${body}\nEND\n`;
}
const errors = (source: string) => informLanguage.analyze(source, 'EDITOR_TEST.JBI').diagnostics.filter(item => item.severity === 'error');
describe('INFORM YRC1000 expanded external file syntax', () => {
  it('indexes all 117 official manual entries with original descriptions and page links', () => {
    expect(informInstructionEntries).toHaveLength(117);
    expect(new Set(informInstructionEntries.map(item => item.name)).size).toBe(117);
    expect(informInstructionEntries.map(item => item.name)).toEqual(informIndexSource.entries.map(item => item.name));
    for (const entry of informInstructionEntries) {
      expect(entry.description.trim().length).toBeGreaterThan(0);
      expect(entry.signature).toContain(entry.name.startsWith('’') ? "'" : entry.name === '*(LABEL)' ? '*' : ['WHILE', 'IFTHEN', 'ELSEIF'].includes(entry.name) ? entry.name : entry.name === 'LOGICEXP' ? 'ANDEXP' : entry.name === 'RELEXPR' ? 'comparison' : entry.name);
      expect(entry.documentationUrl).toMatch(/#page=\d+$/);
    }
  });
  it.each([
    ['assignment and arrays', 'SET I000 1+2*3\nSET LI[0] (I000+2)/3\nADD I000 1\nINC I000'],
    ['digital I/O and waiting', 'DOUT OT#(1) ON\nDIN B000 IN#(1)\nWAIT IN#(1)=ON T=2.0\nTIMER T=0.5'],
    ['conditional blocks', 'IFTHENEXP IN#(1)=ON ANDEXP I000<3\nSET I000 1\nELSEIFEXP I000=4\nSET I000 2\nELSE\nSET I000 3\nENDIF'],
    ['nested loops', 'FOR I000=0 TO 4 STEP 1\nWHILEEXP I000<2\nINC I000\nENDWHILE\nNEXT I000'],
    ['selection', 'SWITCH I000 CASE 0\nSET I001 1\nCASE 1\nSET I001 2\nDEFAULT\nSET I001 3\nENDSWITCH'],
    ['labels calls and motion', '*AGAIN\nMOVJ C00000 VJ=10\nMOVL C00000 V=100 PL=0\nCALL JOB:HELPER\nJUMP *AGAIN IF I000<10'],
    ['strings comments CRLF', 'MSG "你好 world"\n\'ignore "unterminated\nRET IF I000=0'],
  ])('accepts the %s production', (_name, body) => {
    expect(errors(job(body).replaceAll('\n', '\r\n'))).toEqual([]);
  });
  it.each([
    ['unmatched control', 'IFTHENEXP I000=1\nSET I000 0'],
    ['misplaced branch', 'ELSE'],
    ['wrong loop variable', 'FOR I000=0 TO 2\nNEXT I001'],
    ['missing expression', 'SET I000'],
    ['missing arithmetic operand', 'ADD I000 (1+)'],
    ['bad parentheses', 'WAIT IN#(1=ON'],
    ['bad string', 'MSG "unfinished'],
    ['undefined target', 'MOVL C00007 V=100'],
    ['undefined jump', 'JUMP *MISSING'],
    ['bad speed', 'MOVJ C00000 VJ=101'],
    ['negative speed', 'MOVL C00000 V=-1'],
    ['negative wait', 'TIMER T=-1'],
    ['missing destination', 'DIN IN#(1)'],
    ['pendant syntax', 'IF (I000=1) THEN\nENDIF'],
    ['display WHILE syntax', 'WHILE I000<1\nENDWHILE'],
  ])('rejects %s', (_name, body) => expect(errors(job(body)).length).toBeGreaterThan(0));
  it('validates position counts, records and headers', () => {
    expect(errors(job('').replace('///NPOS 1,0,0,0,0,0', '///NPOS 2,0,0,0,0,0')).some(item => item.message.includes('NPOS'))).toBe(true);
    expect(errors(job('').replace('400,20,500,180,0,0', 'NaN,20,500,180,0,0')).length).toBeGreaterThan(0);
    expect(errors(job('').replace('///TOOL 0', '///TOOL 99')).length).toBeGreaterThan(0);
  });
  it('reports unverified technical instructions and unknown tags instead of silently passing', () => {
    const analysis = informLanguage.analyze(job('ARCON ASF#(1) EXTRA\nMOVL C00000 V=100 MADEUP=2\nOPTIONAL_MACRO 1'));
    expect(analysis.diagnostics.filter(item => item.severity === 'warning')).toHaveLength(2);
  });
  const schemaCases = [
    'INPUT I001', 'PRINT "n=%d" I001', 'CLEAR I001 4', 'SETTM TM001 TSTART',
    'VAL I001 "123" RADIX=10', 'VAL2STR S001 I001', 'MID$ S001 "abcdef" 2 3',
    'CAT$ S001 S002 "z"', 'STRSTR B001 S001 "a" B003', 'MULMAT P001 P002 P003', 'INVMAT P001 P002',
    'SETE P001 (2) 150', 'GETE D001 P001 (2)', 'GETS D001 $D001',
    'GETARG LI001 IARG#(2)', 'GETNAME S001 I001', 'GETPOS PX001 STEP#(2) READCTP D001',
    'SETFILE WEV#(2) (3) 1000', 'GETFILE D001 WEV#(2) (3)',
    'SETREG MREG#(2) I001 REGNUM=1', 'GETREG I001 MREG#(2) LOW',
    'GETPRM D001 KIND=5 PRMNO=33', 'SETPRM KIND=5 PRMNO=33 D001',
    'PULSE OT#(2) T=0.2', 'AOUT AO#(2) 2.5', 'ARATION AO#(2) BV=5 V=100 OFV=0',
    'ARATIOF AO#(2)', 'ANTOUT AT#(2) OFF', 'HAND 1 OFF', 'HSEN 1 ON',
    'SPEED VJ=10 V=100 VR=20', 'SFTON P001 UF#(2)', 'SFTON3D RB1 P001',
    'MSHIFT PX001 BF PX002 PX003', 'CNVRT PX001 PX002 BF', 'MFRAME UF#(2) PX001 PX002 PX003', 'REFP 1 C00000',
  ];
  it.each(schemaCases)('validates the operand schema for %s', body => {
    expect(informLanguage.analyze(job(body)).diagnostics).toEqual([]);
    if (!body.startsWith('PRINT ')) expect(errors(job(body.split(' ')[0])).length).toBeGreaterThan(0);
    expect(errors(job(body + ' B999')).length).toBeGreaterThan(0);
  });
  it('accepts whole-expression assignment, binary NOT and external CALL argument forms', () => {
    expect(errors(job('SET I001 (1+2)*3\nNOT B001 B002\nCALL JOB:HELPER IF IN#(2)=ON (1,2)\nRET LI001'))).toEqual([]);
  });
  const manualCases = [
    ['alarm with optional name and stop mode', 'SETUALM 8000 "ALM" 0 SMODE=1', 'SETUALM 8000 "ALM"'],
    ['dialog with one button', 'DIALOG B000 DIALSB B001 B002 S000 B003 B004 S001', 'DIALOG B000 DIALSB B001 B002 S000'],
    ['dialog with second message only', 'DIALOG B000 DIALSB B001 B002 S000 B003 B004 S001 DIALSB LB001 LB002 LS000', 'DIALOG B000 DIALSB B001 B002 S000 B003 B004 S001 DIALSB LB001'],
    ['arc condition file', 'ARCON WELD1 ASF#(1)', 'ARCON ASF#(1) AC=100'],
    ['arc scalar settings', 'ARCON AC=220 AVP=100 T=0.5 V=100 RETRY REPLAY', 'ARCON AC=220'],
    ['arc end', 'ARCOF AC=160 AVP=90 T=0.5 ANTSTK', 'ARCOF AVP=90'],
    ['arc voltage', 'VWELD WELD2 -3.5', 'VWELD WELD2'],
    ['arc current', 'AWELD 2.5', 'AWELD 2.5 3.5'],
    ['arc modification', 'ARCSET AC=250', 'ARCSET AV=10 AVP=100'],
    ['weaving file', 'WVON RB2 WEV#(1) DIR=1', 'WVON WEV#(1) FREQ=3'],
    ['weaving numeric settings', 'WVON AMP=5.0 FREQ=3.0 ANGL=40 DIR=1', 'WVON AMP=5'],
    ['weaving stop', 'WVOF RB2', 'WVOF RB9'],
    ['arc start ramp', 'ARCCTS AC=150 AV=16 AN3=7.5 AN4=6.5 DIS=100', 'ARCCTS AC=150 DIS='],
    ['arc end ramp', 'ARCCTE AC=150 AV=16 DIS=100', 'ARCCTE AV=16 AVP=100'],
    ['gun close', 'GUNCL GUN#(1) T=2.0 ATT=0.5', 'GUNCL GUN#(1) ATT=0.5'],
    ['two gun spot welding', 'SPOT GUN#(1) MODE=2 WTM=5 GUN#(2) MODE=3 WTM=6 ATT=0.5', 'SPOT GUN#(1) MODE=2'],
    ['gun stroke', 'STROKE GUN#(1) LONG', 'STROKE GUN#(1) MIDDLE'],
    ['gun waiting', 'STRWAIT GUN#(1) SHORT', 'STRWAIT GUN#(1)'],
    ['early instruction', 'MOVL C00000 V=100 ENWAIT DIS=2', 'MOVL C00000 V=100 ENWAIT'],
    ['limit trap', 'MOVL C00000 V=100 TRAP *ERR\n*ERR', 'MOVL C00000 V=100 TRAP ERR'],
    ['position capture', 'MOVL C00000 V=100 UNTIL IN#(1)=ON CAPTURE PX001', 'MOVL C00000 V=100 UNTIL IN#(1)=ON CAPTURE P001'],
    ['parallel inline shift', 'MOVL C00000 V=100 +SFT RB1 P001 BP001 EX001 UF#(2)', 'MOVL C00000 V=100 +SFT'],
    ['3D inline shift', 'MOVL C00000 V=100 +SFTON3D RB1 P001', 'MOVL C00000 V=100 +SFTON3D BP001'],
    ['multi group shift', 'SFTON RB1 P001 BP001 EX001 UF#(2)', 'SFTON P001 P002'],
    ['shift end for station', 'SFTOF ST', 'SFTOF ST EXTRA'],
    ['3D shift end', 'SFTOF3D RB1', 'SFTOF3D ST'],
    ['indirect job jump', 'JUMP JET#(1) ENTRY=2 UF#(1)', 'JUMP JET#(1)'],
    ['indirect label jump', 'JUMP LABEL:B001 LBLST=0 LBLDF=1', 'JUMP LABEL:I001'],
    ['local jump search', '*1\nJUMP *1 LBLST=0 LBLDF=1', '*1\nJUMP *1 LBLST'],
    ['user-group I/O', 'DOUT OGU#(1) 123\nDIN D000 IGU#(1)', 'DIN B000 IGU#(1)'],
    ['string array message', 'MSG LS[1]', 'MSG I001'],
    ['logic assignment', 'SET FL0000 LOGICEXP IN#(1)=ON ANDEXP IN#(2)=ON', 'SET FL0000 LOGICEXP IN#(1)'],
    ['compound wait', 'WAIT FOREXPRESS IN#(1)=ON OREXP B000=1 T=2', 'WAIT FOREXPRESS IN#(1)'],
    ['external first switch branch', 'SWITCH I000 CASE 1\nCASE 2\nDEFAULT\nENDSWITCH', 'SWITCH I000\nCASE 1\nENDSWITCH'],
    ['unchanged external for', 'FOR LI000=1 TO 10 STEP 1\nNEXT LI000', 'FOR LI000=1 TO\nNEXT LI000'],
  ];
  it.each(manualCases)('matches official syntax for %s and rejects its malformed neighbor', (_name, valid, invalid) => {
    expect(informLanguage.analyze(job(valid)).diagnostics).toEqual([]);
    expect(errors(job(invalid)).length).toBeGreaterThan(0);
  });
  it('returns UTF-16 ranges, definitions and folds without treating comments as program text', () => {
    const source = job('MSG "🤖你好"\nIFTHENEXP I000=0\nMOVL C00009 V=20\nENDIF');
    const analysis = informLanguage.analyze(source);
    const ref = analysis.diagnostics.find(item => item.code === 'inform-reference')!;
    expect(source.slice(ref.from, ref.to)).toBe('C00009');
    expect(analysis.symbols.map(item => item.name)).toEqual(expect.arrayContaining(['EDITOR_TEST', 'C00000']));
    expect(analysis.folds.some(fold => source.slice(fold.from, fold.to).includes('MOVL'))).toBe(true);
  });
  it('bounds adversarial depth, source size and error counts', () => {
    expect(errors(job(`SET I000 ${'('.repeat(140)}1${')'.repeat(140)}`)).some(item => item.message.includes('128'))).toBe(true);
    expect(informLanguage.analyze(' '.repeat(1_048_577)).diagnostics[0].severity).toBe('error');
    const analysis = informLanguage.analyze(job('SET\n'.repeat(500)));
    expect(analysis.diagnostics).toHaveLength(300);
    expect(analysis.diagnostics.at(-1)?.code).toBe('analysis-limit');
  });
});
