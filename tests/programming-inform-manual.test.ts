import { describe, expect, it } from 'vitest';
import coverage from '../vendor/robot-languages/inform/grammar-coverage.json';
import index from '../vendor/robot-languages/inform/instruction-index.json';
import { informLanguage } from '../src/programming/languages/inform';

function job(body: string): string {
  return `/JOB\n//NAME COVERAGE\n//POS\n///NPOS 1,0,0,0,0,0\n///TOOL 0\n///POSTYPE BASE\n///RECTAN\n///RCONF ${Array(24).fill(0).join(',')}\nC00000=410,30,520,180,0,0\n//INST\n///DATE 2026/11/09 12:30\n///ATTR SC,RW,RJ\n///FRAME BASE\n///GROUP1 RB1\nNOP\n${body}\nEND\n`;
}

describe('INFORM main-manual production matrix', () => {
  it('accounts for each of the 117 instruction and attached grammar entries', () => {
    expect(coverage.entries.map(entry => entry.name)).toEqual(index.entries.map(entry => entry.name));
    expect(new Set(coverage.entries.map(entry => entry.name)).size).toBe(117);
    for (const entry of coverage.entries) {
      expect(entry.production.length).toBeGreaterThan(0);
      expect(entry.sourceUrl).toMatch(/178649-1CD#page=\d+$/);
    }
  });
  it.each(coverage.entries)('$name: legal production and minimal malformed neighbor', entry => {
    expect(informLanguage.analyze(job(entry.legal)).diagnostics).toEqual([]);
    expect(informLanguage.analyze(job(entry.illegal)).diagnostics.some(item => item.severity === 'error')).toBe(true);
  });
  it.each([
    ['///DATE 2026/11/09 12:30', '///DATE 2026/13/09 12:30'],
    ['///FRAME BASE', '///FRAME USER64'],
    ['///ATTR SC,RW,RJ', '///ATTR SC,,RW'],
    ['///GROUP1 RB1', '///GROUP1 RB1,RB1'],
    ['//POS', '//POS EXTRA'],
  ])('rejects malformed header %s → %s', (before, after) => {
    expect(informLanguage.analyze(job('').replace(before, after)).diagnostics.some(item => item.severity === 'error')).toBe(true);
  });
  it('keeps unmodeled option grammar visibly unverified', () => {
    for (const body of ['MOVL C00000 V=100 HPSRCH', 'MOVL C00000 V=100 +MOVJ C00000 VJ=10']) {
      expect(informLanguage.analyze(job(body)).diagnostics.some(item => item.code === 'inform-option')).toBe(true);
    }
  });
  it.each([
    'MOVL C00000 V=100 ENWAIT T="slow"',
    'MOVL C00000 V=100 VR=20',
    'MOVL C00000 V=100 PL=1 CR=5',
    'MOVL C00000 V=100 NWAIT ENWAIT T=1',
    'MOVL C00000 ACC=50 V=100',
    'ARATION AO#(3) BV=(S003)',
    'WAIT IN#("bad")=ON',
    'CALL B',
    'SET S003 I003',
  ])('rejects mutually exclusive, out-of-order or ill-typed forms: %s', body => {
    expect(informLanguage.analyze(job(body)).diagnostics.some(item => item.severity === 'error')).toBe(true);
  });
});
