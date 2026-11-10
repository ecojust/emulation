import { describe, expect, it } from 'vitest';
import { parseFanucLs } from '../src/programming/languages/fanuc-parser';
import { informLanguage } from '../src/programming/languages/inform';

// Independently authored programs, checked against the cited production diagrams.
// These are syntax fixtures, not controller-executable or deployment examples.
const ls = (body: string, application = '') => `/PROG FINAL_REVIEW\n/ATTR\nOWNER=MNEDITOR;\n${application ? `/APPL\n${application}\n` : ''}/MN\n${body.split('\n').map((line, index) => `${index + 1}: ${line};`).join('\n')}\n/POS\n/END`;
const jbi = (body: string) => `/JOB\n//NAME FINAL_REVIEW\n//POS\n///NPOS 1,0,0,0,0,0\n///POSTYPE PULSE\n///PULSE\nC00000=0,0,0,0,0,0\n//INST\nNOP\n${body}\nEND\n`;
const fanucErrors = (body: string) => parseFanucLs(ls(body)).diagnostics.filter(item => item.severity === 'error');
const informErrors = (body: string) => informLanguage.analyze(jbi(body)).diagnostics.filter(item => item.severity === 'error');

describe('independent final FANUC / INFORM manual review', () => {
  // B-83284EN/09 §4.19, printed p270: PULSE and mixed assignments in
  // conditional actions require a mixed (parenthesized) condition.
  it.each([
    ['IF (DI[3]),DO[4]=PULSE', 'IF DI[3]=ON,DO[4]=PULSE'],
    ['IF (DI[3]),DO[4]=(DI[5] AND DI[6])', 'IF DI[3]=ON,DO[4]=(DI[5] AND DI[6])'],
  ])('distinguishes FANUC mixed condition action: %s', (legal, illegal) => {
    expect(fanucErrors(legal)).toEqual([]);
    expect(fanucErrors(illegal).length).toBeGreaterThan(0);
  });

  // B-83284EN/09 §4.5.6 pp207–208: AR arguments are valid string sources
  // and numeric substring operands; PR is not a string source.
  it.each([
    ['R[7]=STRLEN AR[2]', 'R[7]=STRLEN PR[2]'],
    ['R[7]=FINDSTR AR[2],SR[3]', 'R[7]=FINDSTR AR[2],PR[3]'],
    ['SR[7]=SUBSTR AR[2],AR[3],AR[4]', 'SR[7]=SUBSTR AR[2],SR[3],AR[4]'],
  ])('checks FANUC documented string argument types: %s', (legal, illegal) => {
    expect(fanucErrors(legal)).toEqual([]);
    expect(fanucErrors(illegal).length).toBeGreaterThan(0);
  });

  // 178649-1CD rev10, physical pp180,191–198,218: keep the P/PX
  // alternatives and LP/LPX array forms distinct, without controller setup.
  it.each([
    ['CNVRT LPX[I003] PX004 UF#(I000) TL#(2)', 'CNVRT LP[I003] PX004 UF#(I000) TL#(2)'],
    ['MFRAME LP003 LP004 LBP004 UUF#(2)', 'MFRAME LP003 LPX004 LBP004 UUF#(2)'],
    ['MFRAME UF#(2) LPX[B003] PX004 PX005', 'MFRAME UF#(2) LP[B003] PX004 PX005'],
    ['SETE LP003 TL#(2)', 'SETE LBP003 TL#(2)'],
    ['GETE LD003 LEX004 (2)', 'GETE LR003 LEX004 (2)'],
    ['GETPOS LPX[I003] STEP#(I004) READCTP LD005', 'GETPOS LPX[I003] STEP#(I004) READCTP LR005'],
  ])('checks INFORM documented position branches: %s', (legal, illegal) => {
    expect(informErrors(legal)).toEqual([]);
    expect(informErrors(illegal).length).toBeGreaterThan(0);
  });

  // 178649-1CD rev10 physical pp318/320 and372/374: only P can be
  // followed by optional BP and then EX; the initial BP/EX branches end.
  it.each([
    ['SFTON P003 BP003 EX003 RF', 'SFTON P003 EX003 BP003 RF'],
    ['SFTON BP003', 'SFTON BP003 P003'],
    ['SFTON EX003', 'SFTON EX003 BF'],
    ['MOVL P003 V=100 +SFT P004 BP004 EX004 RF', 'MOVL P003 V=100 +SFT BP004 P004 RF'],
  ])('follows INFORM shift branch and operand order: %s', (legal, illegal) => {
    expect(informErrors(legal)).toEqual([]);
    expect(informErrors(illegal).length).toBeGreaterThan(0);
  });

  // 178649-1CD rev10 physical pp247/249: optional tags occupy fixed
  // positions, rather than being an unordered collection of parameters.
  it.each([
    ['SETREG MREG#(3) HIGH D003 REGNUM=2', 'SETREG HIGH MREG#(3) D003 REGNUM=2'],
    ['SETREG MREG#(3) HIGH D003 REGNUM=2', 'SETREG MREG#(3) D003 HIGH REGNUM=2'],
    ['SETREG MREG#(3) HIGH D003 REGNUM=2', 'SETREG MREG#(3) REGNUM=2 HIGH D003'],
    ['GETREG D003 MREG#(3) LOW REGNUM=2', 'GETREG LOW D003 MREG#(3) REGNUM=2'],
    ['GETREG D003 MREG#(3) LOW REGNUM=2', 'GETREG D003 MREG#(3) REGNUM=2 LOW'],
  ])('follows INFORM register parameter ordering: %s', (legal, illegal) => {
    expect(informErrors(legal)).toEqual([]);
    expect(informErrors(illegal).length).toBeGreaterThan(0);
  });

  // 178649-1CD rev10 physical p256: robot/base/station order is fixed;
  // EX as the initial target uses the separate station-only production.
  it.each([
    ['MOVJ P003 BP003 EX003 VJ=20', 'MOVJ P003 EX003 BP003 VJ=20'],
    ['MOVJ EX003 VJ=20', 'MOVJ EX003 BP003 VJ=20'],
  ])('follows INFORM motion group ordering: %s', (legal, illegal) => {
    expect(informErrors(legal)).toEqual([]);
    expect(informErrors(illegal).length).toBeGreaterThan(0);
  });

  // 178649-1CD rev10 §1.7.4, printed pp1-33–34: structured external
  // EXP forms serialize logical operators as ANDEXP/OREXP. This restriction
  // is deliberately not imposed on non-structured LOGICEXP/IFEXPRESS forms.
  it.each([
    ['IFTHENEXP IN#(1)=ON ANDEXP B000=0\nENDIF', 'IFTHENEXP IN#(1)=ON AND B000=0\nENDIF'],
    ['WHILEEXP IN#(1)=ON OREXP B000=0\nENDWHILE', 'WHILEEXP IN#(1)=ON OR B000=0\nENDWHILE'],
    ['IFTHENEXP IN#(1)=ON\nELSEIFEXP B000=1 ANDEXP I000=2\nENDIF', 'IFTHENEXP IN#(1)=ON\nELSEIFEXP B000=1 AND I000=2\nENDIF'],
  ])('distinguishes INFORM structured external serialization: %s', (legal, illegal) => {
    expect(informErrors(legal)).toEqual([]);
    expect(informErrors(illegal).length).toBeGreaterThan(0);
  });
});

// B-83284EN/09 §8.10.5 pp519–521: the multiple-pattern CONFIG field
// and PATTERN block belong only to BX/EX. All coordinates below are original.
const pallet = (mode: 'B' | 'BX', extra = '') => `PALLETIZING-[2];
CONFIG:
PALLETIZING-${mode}_2 "Review";
TYPE=DEPALLET;
INCR=1;
PAL REG=2;
ORDER=LCR;
ROWS=[1];
COLUMNS=[1];
LAYERS=[1];
AUXILIARY POS=[NO];
APPR=0;
RTRT=0;
${extra}
BOTTOM:
P[1,1,1]{GP1:UF:0,UT:1,CONFIG:'N U T,0,0,0',X=450mm,Y=10mm,Z=300mm,W=0deg,P=90deg,R=0deg};`;

describe('independent FANUC application-mode review', () => {
  it('accepts both single and multiple path modes with their documented fields', () => {
    for (const app of [pallet('B'), pallet('BX', 'PATTERN=1;') + '\nPATTERN:\nPTN[1]=[*,*,*];']) {
      expect(parseFanucLs(ls('END', app)).diagnostics).toEqual([]);
    }
  });

  it.each([
    pallet('B', 'PATTERN=1;'),
    pallet('B') + '\nPATTERN:\nPTN[1]=[*,*,*];',
  ])('rejects a BX/EX-only field in a B pallet', app => {
    expect(parseFanucLs(ls('END', app)).diagnostics.some(item => item.severity === 'error')).toBe(true);
  });
});
