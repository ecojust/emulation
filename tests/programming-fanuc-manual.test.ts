import { describe, expect, it } from 'vitest';
import { fanucLanguage } from '../src/programming/languages/fanuc';
import { parseFanucLs } from '../src/programming/languages/fanuc-parser';

// Independently authored productions, cross-checked against Basic B-83284EN/09
// §§4, 8.10.5, 9.5 and Appendix A.3; Optional B-83284EN-2/09 §§14,31.
const source = (body: string) => `/PROG MANUAL_CASE\n/ATTR\nCOMMENT="Grammar test";\n/MN\n${body.split('\n').map(line => ` : ${line};`).join('\n')}\n/POS\n/END`;
const errors = (body: string) => parseFanucLs(source(body)).diagnostics.filter(item => item.severity === 'error');
const checked = (body: string) => parseFanucLs(source(body)).diagnostics.filter(item => item.severity === 'error' || item.code?.startsWith('fanuc.unsupported'));

describe('FANUC Basic /09 manual-derived production matrix', () => {
  it.each([
    'A P[2] 80mm/sec FINE',
    'J PR[2] 250msec FINE',
    'L PR[2] 100mm/sec CNT R[5] EV 30%',
    'L PR[2] 100mm/sec FINE Ind.EV 40%',
    'C P[1] P[2] 20cm/min CNT40 ORNT_BASE UF[0,Z]',
    'L PR[2] 100mm/sec FINE TA .2sec,DO[3]=ON',
    'L PR[2] 100mm/sec FINE TB 0sec,CALL ACTION(AR[1])',
    'L PR[2] 100mm/sec FINE DB 10mm,AO[2]=R[3]',
    'OFFSET CONDITION PR[1],UFRAME[2]',
    'TOOL_OFFSET CONDITION PR[2],UTOOL[3]',
    'RSR[2]=ENABLE\nRSR[2]=DISABLE',
    'JOINT_MAX_SPEED[2]=R[3]\nLINEAR_MAX_SPEED=300',
    'R[1]=TIMER_OVERFLOW[2]\nTIMER[2]=RESET',
    'WAIT DI[2]=ON+ TIMEOUT,LBL[8]\nLBL[8]',
    'WAIT DI[2]=OFF-,TIMEOUT LBL[8]\nLBL[8]',
    'WAIT ERR_NUM=2\nWAIT .5(sec)\nWAIT .5sec',
    'DO[2]=PULSE,.2sec\nRO[3]=PULSE,.3(sec)',
    'DO[2]=(DI[3] AND !DI[4])\nWAIT (!DI[5] OR DI[6])',
    'DIAG_REC[2,0,100]\nDIAG_REC_SEC(3,2.5)',
    'R[1]=STRLEN AR[2]\nSR[2]=SUBSTR AR[3],1,8',
    'PL[1]=PL[2]+[1,R[3],*]',
    'L PR[1] 30mm/sec FINE DO[1]=ON GO[2]=R[3]',
    'TOOL ATTACH[2]\nTOOL DETACH[AR[1]]',
    'J PR[1] 30% FINE MROT\nL PR[2] 30mm/sec FINE Wjnt MROT',
    'J PR[1] 30% FINE VIA(DEFAULT)',
    'L PR[1] 30mm/sec FINE VIA(CNT,Z=200,RT_LD=10,AP_LD=20)',
    'R[2]=(TCP_SPD[1])',
    'PALLETIZING-B_2\nJ PAL_2[A_1] 40% CNT30\nL PAL_2[BTM] 90mm/sec FINE\nPALLETIZING-END_2',
    'Independent GP\nGP1 J PR[1] 20% FINE\nGP2 L PR[1] 50mm/sec CNT30',
    'Simultaneous GP\nGP1 L PR[1] 50mm/sec CNT30\nGP2 J PR[1] 20% FINE',
  ])('recognizes the production %s', body => {
    // Group blocks are one multiline statement, not separate semicolon statements.
    if (/^(Independent|Simultaneous)/.test(body)) {
      const result = parseFanucLs(source(body).replace(/;\n(?= : GP[1-8] )/g, '\n'));
      expect(result.diagnostics.filter(item => item.severity === 'error' || item.code?.startsWith('fanuc.unsupported'))).toEqual([]);
    } else expect(checked(body)).toEqual([]);
  });

  it.each([
    'A P[1] P[2] 30mm/sec FINE',
    'RSR[1]=ON', 'RSR[1]=ENABLE EXTRA',
    'TIMER_OVERFLOW[1]=0',
    'OFFSET CONDITION PR[1],PR[2]', 'TOOL_OFFSET CONDITION PR[1],UFRAME[2]',
    'L PR[1] 20mm/sec FINE EV 101%', 'L PR[1] 20mm/sec FINE Ind.EV 50',
    'J PR[1] 20% FINE ORNT_BASE',
    'L PR[1] 20mm/sec FINE TA .2sec,R[1]=1',
    'L PR[1] 20mm/sec FINE TA .2,CALL ACT',
    'GO[1]=PULSE,.2sec', 'DO[1]=PULSE,.2',
    'DIAG_REC[0,0,10]', 'DIAG_REC[1,0]', 'DIAG_REC_SEC[1,2,3]',
    'R[1]=SIN[1]', 'R[1]=SIN[R[2]+1]', 'R[1]=ATAN2[R[1]]',
    'SR[1]=SUBSTR SR[2],0,2',
    'PALLETIZING-B_17', 'J PAL_1[BAD] 30% FINE', 'PL[1]=[1,2]',
    'TOOL ATTACH[5]', 'TOOL DETACH[]', 'TOOL OPEN[1]',
    'L PR[1] 30mm/sec FINE MROT', 'C P[1] P[2] 30mm/sec FINE VIA(DEFAULT)',
    'J PR[1] 30% FINE VIA(CNT,Z=100,RT_LD=10,AP_LD=10)',
    'L PR[1] 30mm/sec FINE VIA(CNT,Z=100,AP_LD=10)',
    'L PR[1] 30mm/sec FINE DO[1]=ON DO[2]=ON DO[3]=ON DO[4]=ON DO[5]=ON DO[6]=ON',
    'TCP_SPD[1]=0',
    'PR[GP9:1]=LPOS', 'R[GP1:1]=0',
  ])('rejects a minimal malformed production %s', body => expect(errors(body)).not.toHaveLength(0));

  it('requires register arguments for all J593 math functions and documents degree units', () => {
    for (const name of ['SIN', 'COS', 'TAN', 'ASIN', 'ACOS', 'ATAN', 'SQRT', 'ABS', 'TRUNC', 'ROUND', 'LN', 'EXP']) {
      expect(errors(`R[1]=${name}[AR[1]]`)).toEqual([]);
      expect(errors(`R[1]=${name}[1]`)).not.toHaveLength(0);
      expect(fanucLanguage.entries.find(entry => entry.name === name)?.requiresOption).toContain('J593');
    }
    expect(errors('R[1]=ATAN2[R[2],AR[3]]')).toEqual([]);
  });

  it('distinguishes single-point A from two-point C and marks unimplemented POINT_LOGIC payload', () => {
    expect(checked('A P[1] 50mm/sec FINE')).toEqual([]);
    expect(errors('C P[1] 50mm/sec FINE')).not.toHaveLength(0);
    const doc = parseFanucLs(source('L P[1] 50mm/sec FINE DB 2mm,POINT_LOGIC'));
    expect(doc.diagnostics.some(item => item.code === 'fanuc.unsupported-point-logic')).toBe(true);
    expect(doc.source).toContain('POINT_LOGIC');
  });

  it('retains the documented ACC range disagreement instead of rejecting all values above 100', () => {
    const result = parseFanucLs(source('J PR[1] 30% FINE ACC200'));
    expect(result.diagnostics.some(item => item.code === 'fanuc.manual-range-difference')).toBe(true);
    expect(errors('J PR[1] 30% FINE ACC500')).toEqual([]);
    expect(errors('J PR[1] 30% FINE ACC501')).not.toHaveLength(0);
  });

  it('checks /09 name length and reserved names, and accepts Job/Process subtypes', () => {
    for (const name of ['CON', 'com1', 'NUL', '9START', 'A'.repeat(37)]) expect(parseFanucLs(source('END').replace('MANUAL_CASE', name)).diagnostics.some(item => item.code === 'fanuc.program-name')).toBe(true);
    for (const name of ['_START', 'A'.repeat(36), 'TASK Job', 'TASK Process']) expect(parseFanucLs(source('END').replace('MANUAL_CASE', name)).diagnostics.filter(item => item.code === 'fanuc.program-name' || item.code === 'fanuc.unsupported-subtype')).toEqual([]);
  });

  it('does not mistake mixed-logic ! or arithmetic subtraction for comments', () => {
    const doc = parseFanucLs(source('-- first-language comment\n! plain comment\nDO[1]=(!DI[2])\nR[1]=2--1'));
    expect(doc.diagnostics.filter(item => item.severity === 'error')).toEqual([]);
    expect(doc.statements.map(statement => statement.kind)).toEqual(['comment', 'comment', 'DO', 'R']);
  });

  it('parses documented /ATTR fields and rejects malformed field productions', () => {
    const attrs = `OWNER=MNEDITOR;
COMMENT="Source test";
PROG_SIZE=120;
CREATE=DATE 24-05-07 TIME 12:30:59;
MODIFIED=DATE xx-05-07 TIME 12:30:59;
FILE_NAME=;
VERSION=0;
LINE_COUNT=1;
MEMORY_SIZE=1024;
PROTECT=READ_WRITE;
TCD: STACK_SIZE=0,TASK_PRIORITY=50,TIME_SLICE=0,BUSY_LAMP_OFF=0,ABORT_REQUEST=0,PAUSE_REQUEST=0;
DEFAULT_GROUP=1,*,*,*,*,*,*,*;
CONTROL_CODE=00000000 00000000;`;
    const document = source('END').replace('COMMENT="Grammar test";', attrs);
    expect(parseFanucLs(document).diagnostics.filter(item => item.severity === 'error' || item.code === 'fanuc.unsupported-attribute')).toEqual([]);
    for (const [before, after] of [
      ['PROG_SIZE=120', 'PROG_SIZE=abc'], ['COMMENT="Source test"', 'COMMENT=unquoted'],
      ['FILE_NAME=', 'FILE_NAME=A B'], ['DEFAULT_GROUP=1,*,*,*,*,*,*,*', 'DEFAULT_GROUP=1,,*'],
      ['CONTROL_CODE=00000000 00000000', 'CONTROL_CODE=00000002'],
      ['TASK_PRIORITY=50', 'TASK_PRIORITY='], ['TIME 12:30:59', 'TIME 12:30'],
    ]) expect(parseFanucLs(document.replace(before, after)).diagnostics.some(item => item.code === 'fanuc.attribute'), after).toBe(true);
  });

  it('reports missing pallet configuration instead of generic unsupported syntax', () => {
    const doc = parseFanucLs(source('END').replace('/MN', '/APPL\nPalletizing[1];\n/APPL\n/MN'));
    expect(doc.diagnostics.some(item => item.code === 'fanuc.pallet-config')).toBe(true);
    expect(doc.diagnostics.some(item => item.code === 'fanuc.duplicate-section')).toBe(false);
  });
});
