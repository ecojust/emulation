import { describe, expect, it } from 'vitest';
import { informLanguage } from '../src/programming/languages/inform';

function job(body: string) {
  return `/JOB\n//NAME SEARCH\n//POS\n///NPOS 1,0,0,0,0,0\n///TOOL 0\n///POSTYPE PULSE\n///PULSE\nC00000=0,0,0,0,0,0\n//INST\n///ATTR SC,RW\n///GROUP1 RB1\nNOP\n${body}\nEND\n`;
}

describe('INFORM independent search option productions', () => {
  // Search Function 178657-1CD rev1, physical pp15–16 (printed 3-1–3-2).
  // These are original source fixtures, not manual example transcripts.
  it.each([
    'MOVL C00000 V=25 SRCH RIN#(2)=ON DIS=12.5',
    'MOVL C00000 V=25 SRCH RIN#(4)=OFF T=0.5 DIS=0.1 ACC=40 DEC=30',
    'IMOV P003 V=10 BF SRCH RIN#(LI003)=LB004 T=LI005 DIS=LD006',
    'MOVL C00000 V=25 SRCH RIN#(B[I003])=LB[I004] T=I[5] DIS=D[6]',
    'MOVL C00000 V=25 SRCH RIN#(1)=ON T=655.35 DIS=6553.5',
    // Sensor Function 179329-1CD rev3, physical pp41–43 (printed 6-1–6-3).
    'MOVL C00000 V=25 ASRCH SCH#(0) TV=-10 FALL DIS=0',
    'MOVL C00000 V=25 ASRCH SCH#(6) TV=10 RISE T=99.99 DIS=999.99 ACC=40',
    'MOVL C00000 V=25 ASRCH SCH#(2) TV=0.25 RISE T=0 DIS=4',
  ])('accepts the verified production: %s', body => {
    expect(informLanguage.analyze(job(body)).diagnostics).toEqual([]);
  });

  it.each([
    'MOVL C00000 V=25 SRCH',
    'MOVL C00000 V=25 SRCH RIN#(1)=ON',
    'MOVL C00000 V=25 SRCH RIN#(0)=ON DIS=1',
    'MOVL C00000 V=25 SRCH RIN#(5)=ON DIS=1',
    'MOVL C00000 V=25 SRCH RIN#(1.5)=ON DIS=1',
    'MOVL C00000 V=25 SRCH RIN#(R003)=ON DIS=1',
    'MOVL C00000 V=25 SRCH RIN#(1) ON DIS=1',
    'MOVL C00000 V=25 SRCH RIN#(1)=I003 DIS=1',
    'MOVL C00000 V=25 SRCH RIN#(1)=ON T=B003 DIS=1',
    'MOVL C00000 V=25 SRCH RIN#(1)=ON T=0 DIS=1',
    'MOVL C00000 V=25 SRCH RIN#(1)=ON DIS=0',
    'MOVL C00000 V=25 SRCH RIN#(1)=ON DIS=6553.6',
    'MOVL C00000 V=25 SRCH RIN#(1)=ON DIS=1 T=0.5',
    'MOVL C00000 V=25 SRCH RIN#(1)=ON DIS=1+',
    'MOVL C00000 V=25 SRCH RIN#(1)=ON DIS=1 25',
    'MOVL C00000 V=25 UNTIL IN#(1)=ON SRCH RIN#(1)=ON DIS=1',
    'MOVJ C00000 VJ=10 SRCH RIN#(1)=ON DIS=1',
    'MOVC C00000 V=25 SRCH RIN#(1)=ON DIS=1',
    'SRCH RIN#(1)=ON DIS=1',
    'MOVL C00000 V=25 ASRCH',
    'MOVL C00000 V=25 ASRCH SCH#(7) TV=0 RISE DIS=1',
    'MOVL C00000 V=25 ASRCH SCH#(1) TV=10.01 RISE DIS=1',
    'MOVL C00000 V=25 ASRCH SCH#(1) TV=-10.01 RISE DIS=1',
    'MOVL C00000 V=25 ASRCH SCH#(1) TV=0 DIS=1',
    'MOVL C00000 V=25 ASRCH SCH#(1) TV=0 RISE FALL DIS=1',
    'MOVL C00000 V=25 ASRCH SCH#(1) TV=0 RISE T=100 DIS=1',
    'MOVL C00000 V=25 ASRCH SCH#(1) TV=0 RISE DIS=-1',
    'MOVL C00000 V=25 ASRCH SCH#(1) TV=0 RISE DIS=1000',
    'MOVL C00000 V=25 ASRCH SCH#(1) TV=0 RISE DIS=1 T=1',
    'MOVL C00000 V=25 ASRCH SCH#(1) TV="low" RISE DIS=1',
    'MOVL C00000 V=25 ASRCH SCH#(1) TV=0 RISE DIS=1 SRCH RIN#(1)=ON DIS=1',
    'IMOV P003 V=25 ASRCH SCH#(1) TV=0 RISE DIS=1',
    'ASRCH SCH#(1) TV=0 RISE DIS=1',
    'MOVJ C00000 VJ=10 HPSRCH',
    'IMOV P003 V=25 HPSRCH',
    'MOVC C00000 V=25 NSRCH',
    'MOVS C00000 V=25 NSRCH',
  ])('rejects missing, ill-typed, unavailable or out-of-order operands: %s', body => {
    expect(informLanguage.analyze(job(body)).diagnostics.some(d => d.severity === 'error')).toBe(true);
  });

  it('does not claim undocumented ASRCH variable types were verified', () => {
    const diagnostics = informLanguage.analyze(job('MOVL C00000 V=25 ASRCH SCH#(1) TV=R003 RISE DIS=I003')).diagnostics;
    expect(diagnostics).toHaveLength(1);
    expect(diagnostics[0]).toMatchObject({ severity: 'warning', code: 'inform-option' });
  });
  it('keeps parameters after verified options under normal motion validation', () => {
    expect(informLanguage.analyze(job('MOVL C00000 V=25 SRCH RIN#(1)=ON DIS=1 ACC=40 +SFT P003')).diagnostics).toEqual([]);
    expect(informLanguage.analyze(job('MOVL C00000 V=25 ASRCH SCH#(1) TV=0 RISE DIS=1 ACC="slow"')).diagnostics.some(d => d.severity === 'error')).toBe(true);
  });
  it('offers sourced completion and original parameter descriptions for verified options', () => {
    for (const name of ['SRCH', 'ASRCH']) {
      const entry = informLanguage.entries.find(item => item.name === name)!;
      expect(entry.documentationUrl).toMatch(/#page=(15|43)$/);
      expect(entry.parameters?.some(item => item.name === 'DIS')).toBe(true);
    }
  });

  // The heading of §3.4 explicitly covers the SACOR variants with the same diagrams.
  // GETSFT's diagram has separate bypasses for the (TCH#, TV) pair and P destination.
  it.each([
    ['ACORON TF', 'ACORON'],
    ['ACORON PSF SCF#(16) TCH#(3) TV=-10', 'ACORON PSF SCF#(17)'],
    ['ACORON OPF TCH#(0) TV=0', 'ACORON OPF TCH#(0)'],
    ['SACORON TF SCF#(1)', 'SACORON TF TV=0'],
    ['ACOROF', 'ACOROF TF'],
    ['SACOROF', 'SACOROF TCH#(1)'],
    ['ACORCH SCF#(2)', 'ACORCH SCF#(2) TCH#(1) TV=0'],
    ['ACORCH TCH#(1) TV=0.5', 'ACORCH TCH#(1)'],
    ['SACORCH SCF#(1)', 'SACORCH'],
    ['SACORCH TCH#(3) TV=10', 'SACORCH TCH#(4) TV=0'],
    ['AOVRON TCH#(6) TV=10', 'AOVRON TCH#(0) TV=0'],
    ['AOVRON TCH#(1) TV=-10', 'AOVRON TCH#(1) TV=10.1'],
    ['AOVROF', 'AOVROF TCH#(1)'],
    ['GETSFT', 'GETSFT TV=0'],
    ['GETSFT P003', 'GETSFT PX003'],
    ['GETSFT LP[I003]', 'GETSFT I003'],
    ['GETSFT TCH#(0) TV=0', 'GETSFT TCH#(0)'],
    ['GETSFT TCH#(3) TV=0.5 P003', 'GETSFT P003 TCH#(3) TV=0.5'],
    ['SCALIB', 'SCALIB TV=0'],
    ['SCALIB TCH#(0)', 'SCALIB TCH#(-1)'],
    ['SCALIB TCH#(6)', 'SCALIB TCH#(7)'],
  ])('checks sensor production %s and its malformed neighbor', (legal, illegal) => {
    expect(informLanguage.analyze(job(legal)).diagnostics).toEqual([]);
    expect(informLanguage.analyze(job(illegal)).diagnostics.some(d => d.severity === 'error')).toBe(true);
    expect(informLanguage.entries.find(entry => entry.name === legal.split(' ')[0])?.documentationUrl).toMatch(/179329-1CD#page=\d+$/);
  });
  it('keeps undocumented sensor scalar variable types visibly unverified', () => {
    expect(informLanguage.analyze(job('ACORON TF SCF#(I003) TCH#(1) TV=R004')).diagnostics).toEqual([
      expect.objectContaining({ severity: 'warning', code: 'inform-option' }),
    ]);
  });
});
