import { describe, expect, it } from 'vitest';
import { parseFanucLs } from '../src/programming/languages/fanuc-parser';
import { fanucLanguage } from '../src/programming/languages/fanuc';
import { FANUC_OPTIONAL_AUDIT } from '../src/programming/languages/fanuc-optional';

const program = (statement: string) => '/PROG OPTIONAL\n/ATTR\nCOMMENT = "Optional grammar";\nDEFAULT_GROUP=1,*,*,*,*;\n/MN\n : ' + statement + ';\n/POS\n/END';
const result = (statement: string) => parseFanucLs(program(statement));
const checked = (statement: string) => result(statement).diagnostics.filter(item => item.severity === 'error' || item.code?.startsWith('fanuc.unsupported'));

describe('FANUC Optional /09 actual grammar, not keyword-only recognition', () => {
  it.each([
    'SOFTFLOAT[1]', 'SOFTFLOAT[10]', 'SOFTFLOAT END', 'FOLLOW UP',
    'COL GUARD ADJUST', 'COL GUARD ADJUST 80', 'COL GUARD ADJUST R[1:Collision sensitivity]',
    'COL GUARD ADJUST GP1,2: 200', 'COL GUARD ADJUST GP8: R[R[1]]',
    'RESUME_PROG[1]=WIRE_CUT', 'RESUME_PROG[5:WIRE CUT]=WIRE_CUT',
    'MAINT_PROG[2:Maintenance]=CLEAN', 'CLEAR_RESUME_PROG', 'RETURN_PATH_DSBL',
    'Sample Start[1]', 'Sample Start[5]', 'Sample End', 'TORQ_LIMIT 20.5%',
    'Search Start[32] PR[1]', 'Search Start[R[3]] PR[R[5]]', 'Search End',
    'Touch Offset PR[3]', 'Touch Offset PR[R[5]]', 'Touch Offset End',
    'STITCH[1]', 'STITCH[20]', 'STITCH END',
  ])('validates documented standalone production: %s', statement => {
    expect(checked(statement)).toEqual([]);
    expect(result(statement).diagnostics.some(item => item.code === 'fanuc.option')).toBe(true);
  });

  it.each([
    'SOFTFLOAT', 'SOFTFLOAT[0]', 'SOFTFLOAT[1.2]', 'SOFTFLOAT[R[1]]', 'SOFTFLOAT END 1',
    'FOLLOW UP 1', 'COL GUARD', 'COL GUARD ADJUST 0', 'COL GUARD ADJUST 201', 'COL GUARD ADJUST 1.5',
    'COL GUARD ADJUST GP0: 100', 'COL GUARD ADJUST GP1,1: 100', 'COL GUARD ADJUST GP1:',
    'COL GUARD ADJUST AR[1]', 'COL GUARD ADJUST R[1] + 1',
    'RESUME_PROG[0]=WIRE_CUT', 'RESUME_PROG[6]=WIRE_CUT', 'RESUME_PROG[R[1]]=WIRE_CUT',
    'RESUME_PROG[1]=R[2]', 'MAINT_PROG[1]=CLEAN(1)', 'CLEAR_RESUME_PROG 1',
    'Sample Start[6]', 'Sample Start[R[1]]', 'Sample End 1', 'TORQ_LIMIT 101%', 'TORQ_LIMIT 20',
    'Search Start[33] PR[1]', 'Search Start[1] PR[1,2]', 'Search Start[1] R[1]', 'Search End PR[1]',
    'Touch Offset PR[1,2]', 'Touch Offset End 1',
    'STITCH', 'STITCH[0]', 'STITCH[21]', 'STITCH[1.5]', 'STITCH[R[1]]',
    'STITCH END 1', 'STITCH[1] END',
  ])('rejects wrong parameter grammar: %s', statement => {
    expect(result(statement).diagnostics.filter(item => item.severity === 'error')).not.toHaveLength(0);
  });

  it.each([
    'J PR[1] 100% CNT100 CTV100', 'J PR[1] 100% FINE CTV-100',
    'L PR[1] 100mm/sec FINE CTVR[1]', 'J PR[1] 100% FINE CTV R[1]',
    'L PR[1] 100mm/sec FINE AP_LD100.5 RT_LD 20.25',
    'L PR[1] 100mm/sec FINE AP_LDR[1] RT_LD R[R[2]]',
    'J PR[1] 100% FINE PSPD110', 'L PR[1] 100mm/sec FINE PSPD 50',
    'S PR[1] 100mm/sec FINE PSPD50', 'L PR[1] max_speed CNT100',
    'L PR[1] 100mm/sec FINE SOFTFLOAT[1]',
    'J PR[1] 50% FINE Search[-X]', 'L PR[1] 100mm/sec FINE Search[Z]',
  ])('parses attached/indirect motion fields with their actual arity: %s', statement => {
    expect(checked(statement)).toEqual([]);
  });

  it.each([
    'J PR[1] 100% FINE CTV101', 'J PR[1] 100% FINE CTV-101',
    'J PR[1] 100% FINE CTV1.5', 'J PR[1] 100% FINE CTV AR[1]',
    'S PR[1] 100mm/sec FINE CTV100', 'J PR[1] 100% FINE AP_LD10',
    'L PR[1] 100mm/sec FINE AP_LD-1', 'L PR[1] 100mm/sec FINE AP_LD',
    'L PR[1] 100mm/sec FINE AP_LD10 CTV0', 'L PR[1] max_speed FINE RT_LD10',
    'J PR[1] max_speed FINE', 'L PR[1] max_speed mm/sec FINE',
    'L PR[1] 100mm/sec FINE PSPD1.5', 'L PR[1] 100mm/sec FINE PSPDR[1]',
    'L PR[1] 100mm/sec FINE SOFTFLOAT END',
    'J PR[1] 50% FINE Search[R[1]]', 'J PR[1] 50% FINE Search[W]',
    'J PR[1] 50% FINE Search[X] ACC50', 'S PR[1] 50mm/sec FINE Search[X]',
    'L PR[1] 50mm/sec FINE STITCH[1]', 'J PR[1] 50% FINE STITCH END',
  ])('rejects illegal motion operand/type combinations: %s', statement => {
    expect(result(statement).diagnostics.filter(item => item.severity === 'error')).not.toHaveLength(0);
  });

  it.each([
    'PS -100mm +0.2sec,DO[1]=(ON)',
    'PS -150mm -0.2sec,IF(DI[1]),DO[2]=(DI[2] AND !DI[3])',
    'PS +150mm +0sec,R[1]=(R[2]+PR[1,2])',
    'PS -10mm +0sec,PR[1,2]=(R[3]*2)',
    'PS -10mm +0sec,M[1]=(DI[1])',
    'PS -10mm +0sec,TC_ONLINE(!DI[1])',
    'PS -10mm +0sec,$MY_VAR.$START=(R[1])',
    'PS -10mm +0sec,DO[1:AR is a comment]=(ON)',
  ])('checks §16 trigger syntax and delegates typed mixed logic: %s', statement => {
    expect(checked(statement)).toEqual([]);
  });

  it.each([
    'PS -10 +0.2sec,DO[1]=(ON)', 'PS -10mm +0.2,DO[1]=(ON)',
    'PS -10mm +0sec,IF DI[1],DO[1]=(ON)', 'PS -10mm +0sec,IF(DI[1]),',
    'PS -10mm +0sec,CALL OTHER', 'PS -10mm +0sec,DI[1]=(ON)',
    'PS -10mm +0sec,DO[1]=(AR[1])', 'PS -10mm +0sec,R[1]=SIN[R[2]]',
    'PS -10mm +0sec,DO[1]=PULSE', 'PS -10mm +0sec,PR[1]=LPOS',
    'PS -10mm +0sec,$MY_VAR=PR[1]', 'PS -10mm +0sec,TC_ONLINE(5)',
    'PS -10mm +0sec,DO[1]=(DI[1] XOR DI[2])',
  ])('rejects unsupported data kinds and malformed PS operands: %s', statement => {
    expect(result(statement).diagnostics.filter(item => item.severity === 'error')).not.toHaveLength(0);
  });

  it('enforces the per-statement mixed-logic operation bound without counting assignment as arithmetic', () => {
    expect(checked('PS -10mm +0sec,R[1]=(' + Array(21).fill('R[2]').join('+') + ')')).toEqual([]);
    expect(result('PS -10mm +0sec,R[1]=(' + Array(22).fill('R[2]').join('+') + ')').diagnostics.some(item => item.message.includes('20'))).toBe(true);
  });
  it('distinguishes the usual ten Softfloat schedules from an absolute grammar limit', () => {
    for (const statement of ['SOFTFLOAT[11]', 'L PR[1] 100mm/sec FINE SOFTFLOAT[11]']) {
      expect(checked(statement)).toEqual([]);
      expect(result(statement).diagnostics.some(item => item.code === 'fanuc.profile-range')).toBe(true);
    }
  });

  it('preserves unknown optional syntax and identifies setup-only chapters instead of fabricating opcodes', () => {
    for (const instruction of ['SOFTLIMIT 10', 'AVOID ON', 'LOCAL_STOP ON', 'GROUP_DO[1]=ON']) {
      const document = result(instruction);
      expect(document.source).toBe(program(instruction));
      expect(document.diagnostics.some(item => item.code?.startsWith('fanuc.unsupported'))).toBe(true);
    }
    expect(FANUC_OPTIONAL_AUDIT.map(item => item.section)).toEqual(['5', '13.1', '15.1.2', '17', '40.2']);
    expect(result('RESUME_PATH_DSBL').diagnostics.some(item => item.code === 'fanuc.manual-spelling-difference')).toBe(true);
  });

  it('bounds recursive indirect references and documents newly checked parameters and precise sections', () => {
    const deep = 'R['.repeat(70) + '1' + ']'.repeat(70);
    expect(result('COL GUARD ADJUST ' + deep).diagnostics.some(item => item.message.includes('64'))).toBe(true);
    for (const name of ['SOFTFLOAT', 'CTV', 'COL GUARD ADJUST', 'RESUME_PROG', 'MAINT_PROG', 'AP_LD', 'RT_LD', 'PSPD', 'PS']) {
      const entry = fanucLanguage.entries.find(item => item.name.toUpperCase() === name.toUpperCase())!;
      expect(entry.category).toMatch(/Optional \/09 §/);
      expect(entry.documentationUrl).toContain('9e9998c2744b94794414a372063410fa20507fc6');
      expect(entry.requiresOption).toBeTruthy();
      expect(entry.parameters?.length).toBeGreaterThan(0);
    }
  });
});

describe('FANUC Fiber Laser Interface /09 named-argument productions', () => {
  it.each([
    'L PR[1] 200mm/sec FINE PRE_LS',
    'L PR[1] 200mm/sec FINE PRE_LS[CH=2]',
    'L PR[1] 200mm/sec FINE PRE_LS[PO=1]',
    'L PR[1] 200mm/sec FINE LS[S=1000,L=100,P=1000,Q=100,V=200]',
    'L PR[1] 200mm/sec FINE LS[S=1000,L=100,P=1000,Q=100,V=200] SLOPEUP[1]',
    'L PR[1] 200mm/sec FINE LE[AT=9999,ST=0,SOF=1] SLOPEDOWN[10]',
    'L PR[1] 200mm/sec FINE LP[S=1000,L=100,P=5,Q=100,I=10,J=5,K=500,H=100,R=3000]',
    'POWER_CONTROL[0]', 'POWER_CONTROL[10]', 'WIRE_CONTROL[Ty=2,Sp=32767]', 'MINUTE_OUTPUT_RQ',
    'CALL LS_PO[S=4000,L=0,P=5,Q=100,SPN=55,TA=0]',
    'CALL LS_PO(S=4000,L=0,P=5,Q=100,SPN=55,TA=0)',
    'L PR[1] 200mm/sec FINE PR_CALL LS_PO[S=4000,L=0,P=5,Q=100,SPN=65535,TA=-99999]',
    'L PR[1] 200mm/sec FINE PR_CALL LS_TA[S=5000,L=0,P=5,Q=100,V=200,TA=99999]',
    'L PR[1] 200mm/sec FINE PR_CALL LS_TA(S=5000,L=0,P=5,Q=100,V=200,TA=0)',
    'L PR[1] 200mm/sec FINE PR_CALL LS_PO(S=4000,L=0,P=5,Q=100,SPN=55,TA=0)',
  ])('parses reviewed named fields and actual standalone/motion/call form: %s', statement => {
    expect(checked(statement)).toEqual([]);
    expect(result(statement).diagnostics.some(item => item.code === 'fanuc.option')).toBe(true);
  });

  it.each([
    'L PR[1] 200mm/sec FINE PRE_LS[PO=2]',
    'L PR[1] 200mm/sec FINE PRE_LS[CH=0]',
    'L PR[1] 200mm/sec FINE LS[S=1000,L=100,P=5,Q=100]',
    'L PR[1] 200mm/sec FINE LS[S=1000,L=100,P=5,Q=100,V=200,S=5]',
    'L PR[1] 200mm/sec FINE LS[S=1000,L=100,P=5,Q=100,V=200,OTHER=1]',
    'L PR[1] 200mm/sec FINE LS[S=1000,L=100,P=5,Q=100,V=R[1]]',
    'L PR[1] 200mm/sec FINE LE[AT=10000,ST=0,SOF=0]',
    'L PR[1] 200mm/sec FINE LE[AT=0.5,ST=0,SOF=0]',
    'L PR[1] 200mm/sec FINE LE[AT=0,ST=0,SOF=2]',
    'L PR[1] 200mm/sec FINE SLOPEUP[1]',
    'J PR[1] 50% FINE LS[S=1000,L=100,P=5,Q=100,V=200] SLOPEUP[1]',
    'L PR[1] 200mm/sec FINE LE[AT=0,ST=0,SOF=0] SLOPEDOWN[0]',
    'POWER_CONTROL[11]', 'WIRE_CONTROL[Ty=3,Sp=20]', 'WIRE_CONTROL[Ty=1,Sp=0]',
    'MINUTE_OUTPUT_RQ 1', 'SLOPEUP[1]',
    'LS[S=1000,L=100,P=5,Q=100,V=200]',
    'L PR[1] 200mm/sec FINE WIRE_CONTROL[Ty=1,Sp=20]',
    'CALL LS_TA[S=5000,L=0,P=5,Q=100,V=200,TA=0]',
    'L PR[1] 200mm/sec FINE LS_PO[S=4000,L=0,P=5,Q=100,SPN=55,TA=0]',
    'L PR[1] 200mm/sec FINE PR_CALL LS_PO[S=4000,L=0,P=5,Q=100,SPN=65536,TA=0]',
    'L PR[1] 200mm/sec FINE PR_CALL LS_TA[S=5000,L=0,P=5,Q=100,V=200,TA=-100000]',
    'L PR[1] 200mm/sec FINE PR_CALL LS_TA[S=5000,L=0,P=5,Q=100,V=200,TA=0] SLOPEUP[1]',
    'S PR[1] 200mm/sec FINE PRE_LS',
    'S PR[1] 200mm/sec FINE LS[S=1000,L=100,P=5,Q=100,V=200]',
    'S PR[1] 200mm/sec FINE LE[AT=0,ST=0,SOF=0]',
    'S PR[1] 200mm/sec FINE LP[S=1000,L=100,P=5,Q=100,I=10,J=5,K=500,H=100,R=3000]',
    'S PR[1] 200mm/sec FINE PR_CALL LS_PO(S=4000,L=0,P=5,Q=100,SPN=55,TA=0)',
    'S PR[1] 200mm/sec FINE PR_CALL LS_TA(S=5000,L=0,P=5,Q=100,V=200,TA=0)',
  ])('rejects missing/duplicate/ranged arguments and the wrong instruction context: %s', statement => {
    expect(result(statement).diagnostics.filter(item => item.severity === 'error')).not.toHaveLength(0);
  });

  it('reports the manual LP I/V and LE SOF discrepancies without rewriting the example text', () => {
    for (const statement of [
      'L PR[1] 200mm/sec FINE LP[S=3000,L=300,P=0,Q=0,V=10,J=5,K=500,H=100,R=3000]',
      'L PR[1] 200mm/sec FINE LE[AT=0,ST=0]',
    ]) {
      const document = result(statement);
      expect(document.source).toBe(program(statement));
      expect(document.diagnostics.filter(item => item.severity === 'error')).toEqual([]);
      expect(document.diagnostics.some(item => item.code?.startsWith('fanuc.manual-'))).toBe(true);
    }
  });
});
