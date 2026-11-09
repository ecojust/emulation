import { describe, expect, it } from 'vitest';
import { parseFanucLs } from '../src/programming/languages/fanuc-parser';
const source = (body: string) => `/PROG TYPE_TEST\n/ATTR\nCOMMENT="Type test";\n/MN\n${body.split('\n').map(line => ` : ${line};`).join('\n')}\n/POS\n/END`;
const errors = (body: string) => parseFanucLs(source(body)).diagnostics.filter(item => item.severity === 'error');

describe('FANUC statically decidable operand productions', () => {
  it.each([
    'PR[1]=PR[2]+LPOS', 'PR[1]=UTOOL[2]', 'PR[1,2]=R[3]+4', 'PR[1]=$MNUFRAME[1,2]',
    'PL[1]=PL[2]+[1,2,*]', 'R[1]=DI[2]+3', 'SR[1]=SR[2]+R[3]', 'R[1]=SR[2]',
    'DO[1]=(DI[2] AND !DI[3])', 'R[1]=(R[2]+3*4)', 'R[1]=(GI[2]>R[3])',
    'CALL TASK(1,2.5,\'hello\',R[1],SR[2],AR[3])',
    'C P[1] Offset,PR[1] P[2] 100mm/sec FINE',
    'C P[1] INC P[2] 100mm/sec FINE',
    'R[1]=1+2-3+4-5+6', 'R[1]=1*2/3*4/5',
    'SELECT R[1]=R[2],CALL ONE\n=3,CALL TWO\nELSE,CALL OTHER',
    'SELECT R[1]=AR[1],CALL TASK', 'SELECT R[1]=SR[2],CALL TASK',
    'PAYLOAD[GP1,3:2]', 'PAYLOAD[GP2,3:R[4]]',
    'UFRAME[2]=PR[1]', 'UTOOL[1]=PR[1]',
    'IF PL[1]<>[*,R[2],2-0],CALL TASK',
    'L PR[1] 100mm/sec FINE MROT Wjnt',
    'S PR[1] 100mm/sec FINE Wjnt MROT PSPD50',
    'A PR[1] 100mm/sec CR50 CTV20',
    `CALL TASK("Area Name"='CELL_A',"Wait Time"=(-1),"Mode"=R[2],"Ready"=1)`,
    'M[1]=()', 'M[1]=(DI[2] AND DI[3])',
    'TC_ONLINE (DI[1] AND !DI[2])', 'TC_ONLINE ENABLE', 'TC_ONLINE DISABLE',
    'L PR[1] 100mm/sec CR12.5', 'L PR[1] 100mm/sec CR 12.5',
    'L P[15:STOCK:3] 23cm/min CNT100 TB 0sec,CALL PROCESS(7) Offset Tool_Offset',
  ])('accepts %s', body => expect(errors(body)).toEqual([]));

  it.each([
    'PR[1]=3', 'PR[1]=R[2]', 'PR[1,2]=LPOS', 'R[1]=LPOS', 'R[1]=PL[2]',
    'PL[1]=R[2]', 'PL[1]=PL[2]*PL[3]', 'PR[1]=PR[2]*PR[3]', 'SR[1]=PR[2]',
    'R[1]=(DI[2]+1)', 'DO[1]=(R[1] AND DI[2])', 'R[1]=(!R[2])',
    'R[1]=(PR[2])', 'PR[1]=(LPOS)', 'WAIT (DI[1]=ON+)', 'WAIT (ERR_NUM=2)',
    'IF (R[1]) THEN\nENDIF', 'R[1]=START', 'R[1]=STRLEN PR[2]', 'SR[1]=STRLEN SR[2]',
    'R[1]=SUBSTR SR[2],1,2', 'CALL TASK(PR[1])', 'CALL TASK(R[1]+1)',
    'CALL TASK(DI[1])', 'CALL TASK('+Array(31).fill('1').join(',')+')',
    'CALL TASK(\''+'x'.repeat(35)+'\')',
    'FOR R[1]=1+2 TO 3\nENDFOR',
    'C P[1] INC Offset,PR[2] P[2] 100mm/sec FINE',
    'R[1]=1+2+3+4+5+6+7', 'R[1]=1+2*3',
    'SELECT R[1]=1,R[2]=3', 'SELECT R[1]=PR[1,2],CALL TASK',
    'SELECT R[1]=1+2,CALL TASK', 'SELECT R[1]=DI[2],CALL TASK',
    'PAYLOAD[GP1,1:2]', 'PAYLOAD[GP9:2]', 'PAYLOAD[AR[1]]',
    'UFRAME[2]=LPOS', 'UFRAME_NUM=PR[1]',
    'PL[1]=[1,2,2-0]', 'PL[1]=[1,2,AR[1]]', 'IF PL[1]=[1,2,2-128],CALL TASK',
    'DO[1]=(NOT DI[2])', 'DO[1]=(DI[2] XOR DI[3])', 'DO[1]=TRUE',
    'J PR[1] 50% FINE Wjnt', 'L PR[1] 100mm/sec FINE MROT',
    'A PR[1] 100mm/sec FINE INC', 'A PR[1] 100mm/sec FINE Search[X]',
    'S PR[1] 100mm/sec CR50', 'S PR[1] 100mm/sec FINE Skip,LBL[1]',
    'S PR[1] 100mm/sec FINE EV 40%', 'S PR[1] 100mm/sec FINE DO[1]=ON',
    'L PR[1] 100mm/sec FINE TB 31sec,CALL TASK',
    'L PR[1] 100mm/sec FINE TA .6sec,CALL TASK',
    'L PR[1] 100mm/sec FINE DB 1000mm,CALL TASK',
    'L PR[1] R[2] FINE', 'J PR[1] R[2] FINE',
    'R[0]=1', 'PL[33]=[1,2,3]', 'LBL[32767]',
    'CALL TASK("Label"=)', 'CALL TASK("Label"=(1+2))', 'CALL TASK("Label"=PR[1])',
    'M[1]=ON', 'R[1]=()', 'TC_ONLINE ()', 'TC_ONLINE (5)', 'TC_ONLINE DI[1]',
    'L PR[1] 100mm/sec CR1000.1', 'L PR[1] 100mm/sec CR 1001', 'L PR[1] 100mm/sec CNT12.5',
    'L PAL_1[A_9] 100mm/sec FINE', 'J PAL_1[R_10] 20% FINE',
  ])('rejects malformed operand combination %s', body => expect(errors(body)).not.toHaveLength(0));

  it('bounds IF/FOR nesting using the /09 ten-level production limit', () => {
    const nested = (count: number) => [...Array(count).fill('IF (DI[1]) THEN'), 'R[1]=1', ...Array(count).fill('ENDIF')].join('\n');
    expect(errors(nested(10))).toEqual([]);
    expect(errors(nested(11)).some(item => item.message.includes('10'))).toBe(true);
  });

  it('checks Cond WHEN clauses, operand families and fixed no-argument monitor calls', () => {
    const cond = (body: string) => source(body).replace('/PROG TYPE_TEST', '/PROG TYPE_TEST Cond');
    for (const body of [
      'WHEN R[1]>R[2],CALL ALERT', 'WHEN DI[1]=ON+,CALL ALERT',
      'WHEN $MNUFRAME[1,2].$X>10 AND GI[2]<=R[3],CALL ALERT',
      'WHEN ERR_NUM=11006,CALL ALERT', 'WHEN UI[1]<>OFF OR RI[2]=DI[3],CALL ALERT',
    ]) expect(parseFanucLs(cond(body)).diagnostics.filter(item => item.severity === 'error')).toEqual([]);
    for (const body of [
      'END', 'R[1]=2', 'WHEN (DI[1]),CALL ALERT', 'WHEN DI[1]>ON,CALL ALERT',
      'WHEN R[1]>PR[2,3],CALL ALERT', 'WHEN DI[1]=ON,CALL ALERT(1)',
      'WHEN ERR_NUM<>11006,CALL ALERT', 'WHEN DI[1]=ON,CALL SR[1]',
      'WHEN R[1]=1 AND R[2]=2 OR R[3]=3,CALL ALERT',
      'WHEN '+Array(6).fill('DI[1]=ON').join(' AND ')+',CALL ALERT',
    ]) expect(parseFanucLs(cond(body)).diagnostics.some(item => item.severity === 'error')).toBe(true);
    expect(errors('WHEN DI[1]=ON,CALL ALERT')).not.toHaveLength(0);
    expect(errors('MONITOR WATCH\nMONITOR END WATCH')).toEqual([]);
    expect(errors('MONITOR WATCH(1)')).not.toHaveLength(0);
  });

  it('requires an instruction section and rejects application data after instruction content', () => {
    expect(parseFanucLs('/PROG EMPTY\n/ATTR\n/END').diagnostics.some(item => item.code === 'fanuc.missing-main')).toBe(true);
    expect(parseFanucLs(source('END').replace('/POS', '/APPL\nHANDLING:TRUE;\n/POS')).diagnostics.some(item => item.code === 'fanuc.section-order')).toBe(true);
  });

  it('checks macro argument productions while leaving controller-specific name resolution unverified', () => {
    for (const body of ['HAND1 OPEN', 'MY_MACRO(1,SR[2],AR[3])', 'Prompt Box Msg(2,3)', 'MY_MACRO("Label"=2)']) {
      const result = parseFanucLs(source(body));
      expect(result.source).toBe(source(body));
      expect(result.diagnostics.filter(item => item.severity === 'error')).toEqual([]);
      expect(result.diagnostics.some(item => item.code === 'fanuc.unsupported-macro')).toBe(true);
    }
    for (const body of ['MY_MACRO(PR[1])', 'MY_MACRO(R[1]+1)', 'MY_MACRO(1,)', 'MY_MACRO(1) unexpected', 'M'.repeat(37)+'(1)']) expect(errors(body)).not.toHaveLength(0);
  });

  it('does not mistake abstract four-placement configuration diagrams for verified ASCII serialization', () => {
    const text = source('END').replace('/POS', `/POS\nP[1]{GP1: UF:0,UT:0,CONFIG:'N L U T,0,0,0',X=0mm,Y=0mm,Z=0mm,W=0deg,P=0deg,R=0deg};`);
    const result = parseFanucLs(text);
    expect(result.source).toBe(text);
    expect(result.diagnostics.filter(item => item.severity === 'error')).toEqual([]);
    expect(result.diagnostics.some(item => item.code === 'fanuc.unverified-configuration')).toBe(true);
  });
});
