import { describe, expect, it } from 'vitest';
import { parseFanucLs } from '../src/programming/languages/fanuc-parser';

const groups = `GP1:
 UF:0, UT:2, CONFIG:'N U T, 0, 0, 0',
 X=150mm,Y=-30mm,Z=400mm,W=180deg,P=0deg,R=90deg
 GP2:
 UF:0,UT:0,J1=20mm`;
const point = (key: string) => `P[${key}]{${groups}};`;
const source = (application: string, main = 'END') => `/PROG APP_TEST\n/ATTR\nCOMMENT="App test";\n/APPL\n${application}\n/MN\n : ${main};\n/POS\n/END`;
const pallet = (mode: 'B' | 'BX' | 'E' | 'EX') => `Palletizing[3];
CONFIG:
 PALLETIZING-${mode}_3 "Fixture";
 TYPE=PALLET;
 INCR=1;
 PAL REG=2;
 ORDER=RCL;
 ROWS=[3${mode.startsWith('E') ? ' LINE FIX' : ''}];
 COLUMNS=[2${mode.startsWith('E') ? ' 100 INTER' : ''}];
 LAYERS=[2${mode.startsWith('E') ? ' LINE FIX 1' : ''}];
 AUXILIARY POS=[NO];
 APPR=1;
 RTRT=1;
 ${mode.endsWith('X') ? 'PATTERN=2;' : ''}
BOTTOM:
 ${point('1,1,1')}
 ${point('3,1,1')}
 ${mode.endsWith('X') ? 'PATTERN:\nPTN[1]=[1,*,*];\nPTN[2]=[2,*,*];' : ''}
ROUTE:
 ${mode.endsWith('X') ? 'ROUTE[1]{' : ''}
 ${point('A_1')}
 ${point('BTM')}
 ${point('R_1')}
 ${mode.endsWith('X') ? `}; ROUTE[2]{${point('BTM')}};` : ''}`;
const issues = (input: string) => parseFanucLs(input).diagnostics.filter(item => item.severity === 'error' || item.code?.startsWith('fanuc.unsupported'));

describe('FANUC Basic /09 application payload grammar', () => {
  it.each(['B', 'BX', 'E', 'EX'] as const)('parses pallet %s CONFIG/BOTTOM/PATTERN/ROUTE and preserves multi-group points', mode => {
    const text = source(pallet(mode));
    const doc = parseFanucLs(text);
    expect(issues(text)).toEqual([]);
    expect(doc.source).toBe(text);
    const app = doc.applications[0]; expect(app.kind).toBe('pallet');
    if (app.kind !== 'pallet') throw new Error('Expected pallet');
    expect(app.number).toBe(3); expect(app.pattern).toBe(mode);
    expect(app.bottom.map(p => p.key)).toEqual(['1,1,1', '3,1,1']);
    expect(app.bottom[0].groups.map(group => group.number)).toEqual([1, 2]);
    expect(app.routes[0].points.map(p => p.key)).toEqual(['A_1', 'BTM', 'R_1']);
    expect(app.routes).toHaveLength(mode.endsWith('X') ? 2 : 1);
    expect(app.patterns).toHaveLength(mode.endsWith('X') ? 2 : 0);
  });

  it.each([
    ['TYPE=PALLET', 'TYPE=OTHER'], ['ORDER=RCL', 'ORDER=RRR'],
    ['PAL REG=2', 'PAL REG=33'], ['ROWS=[3]', 'ROWS=[128]'],
    ['APPR=1', 'APPR=9'], ['AUXILIARY POS=[NO]', 'AUXILIARY POS=[MAYBE]'],
    ['PALLETIZING-B_3', 'PALLETIZING-B_4'], ['P[1,1,1]', 'P[1,1]'],
    ['P[A_1]', 'P[A_9]'], ['X=150mm,Y', 'X=150mm Y'],
  ])('rejects malformed application production %s → %s', (from, to) => {
    expect(parseFanucLs(source(pallet('B').replace(from, to))).diagnostics.some(item => item.severity === 'error')).toBe(true);
  });

  it('checks nested route boundaries, duplicate positions and required config fields', () => {
    for (const text of [
      pallet('BX').replace('ROUTE[2]{', 'ROUTE[1]{'),
      pallet('B').replace(point('3,1,1'), point('1,1,1')),
      pallet('B').replace('TYPE=PALLET;', ''),
      pallet('BX').replace('PATTERN=2;', ''),
      pallet('BX').replace('PTN[2]', 'PTN[1]'),
    ]) expect(parseFanucLs(source(text)).diagnostics.some(item => item.severity === 'error')).toBe(true);
  });

  it('parses Basic /09 Handling/Spot/Arc/Dispense/cycle application attributes', () => {
    expect(issues(source(`HANDLING:TRUE;
SPOT:FALSE;
SPOT Welding Equipment Number:2;
ARC:TRUE;
ARC Welding Equipment:1,*,*,*,*,*,*,*;
DISPENSE:TRUE;
DISPENSE_PROCESS;
 CYCLE_TIME:0;
 LAST_CYCLE_TIME:0.2;
 GUN_ON_TIME:0;
 LAST_GUN_ON_TIME:0;
 DEFAULT_USER_FRAME:1;
 DEFAULT_TOOL_FRAME:2;
 EQUIPMENT_NUMBER:1;
CYCLE_REFERENCE=0,0,0,0,0,0,0,0,0,0,0,0;
CYCLE_TARGET=3.2;`))).toEqual([]);
    for (const app of ['HANDLING=TRUE;', 'ARC:3;', 'ARC Welding Equipment:1,,*;', 'CYCLE_REFERENCE=0,0;', 'SPOT Welding Equipment Number:missing;']) expect(parseFanucLs(source(app)).diagnostics.some(item => item.severity === 'error')).toBe(true);
  });

  it('keeps actual unknown application extensions visible without generic warnings on ordinary data', () => {
    const text = source('HANDLING:TRUE;\nVENDOR_FUTURE:some,data;');
    const result = parseFanucLs(text);
    expect(result.diagnostics.filter(item => item.code === 'fanuc.unsupported-application-data')).toHaveLength(1);
    expect(result.source).toBe(text);
  });
});
