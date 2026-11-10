import { describe, expect, it } from 'vitest';
import catalog from '../src/programming/languages/rapid-catalog.json';
import { createRapidLanguage, rapidLanguage } from '../src/programming/languages/rapid';

describe('versioned RAPID reference catalog', () => {
  it.each([['6.16', 691, 367, 223, 101], ['7.21', 665, 344, 227, 94]] as const)('matches the pinned %s official chapter set and category counts', (version, total, instructions, functions, types) => {
    const profile = catalog.profiles[version];
    expect(profile.entries).toHaveLength(total);
    expect(new Set(profile.entries.map(e => e.name.toLowerCase())).size).toBe(total);
    expect(profile.entries.filter(e => e.kind === 'instruction')).toHaveLength(instructions);
    expect(profile.entries.filter(e => e.kind === 'function')).toHaveLength(functions);
    expect(profile.entries.filter(e => e.kind === 'datatype')).toHaveLength(types);
    expect(profile.sha256).toMatch(/^[0-9a-f]{64}$/);
    for (const e of profile.entries) {
      expect(e.section).toMatch(/^[123]\.\d+$/);
      if (e.kind !== 'datatype') expect(e.signature.length).toBeGreaterThan(0);
      expect(new Set(e.parameters.map(p => p.name)).size).toBe(e.parameters.length);
      for (const p of e.parameters) expect(e.signature).toContain(p.name);
    }
  });

  it('includes latest APIs that the original third-party 705-name database lacked', () => {
    const keys = rapidLanguage.entries.map(e => e.name);
    for (const name of ['JSONFromVal', 'JSONToVal', 'MoveAbsL', 'MatrixMult', 'MatrixInverse', 'FitSphere', 'CamWaitLoadJob', 'PathLengthGet']) expect(keys).toContain(name);
    expect(rapidLanguage.entries.find(e => e.name === 'MoveL')!.signature).toContain('\\KeepStartPath');
    expect(createRapidLanguage('6.16').entries.find(e => e.name === 'MoveL')!.signature).not.toContain('\\KeepStartPath');
    expect(createRapidLanguage('6.16').entries.map(e => e.name)).not.toContain('JSONFromVal');
  });

  it('retains required, optional, alternatives and types in call order', () => {
    const move = rapidLanguage.entries.find(e => e.name === 'MoveL')!;
    expect(move.parameters?.filter(p => !p.optional).map(p => [p.name, p.type])).toEqual([
      ['ToPoint', 'robtarget'], ['Speed', 'speeddata'], ['Zone', 'zonedata'], ['Tool', 'tooldata'],
    ]);
    expect(move.parameters?.find(p => p.name === '\\Conc')?.description).toContain('开关');
    expect(move.parameters?.find(p => p.name === '\\T')?.description).toContain('替代');
    expect(move.description).toContain('直线');
    expect(move.insertText).toBe('MoveL ${ToPoint}, ${Speed}, ${Zone}, ${Tool};');
    expect(rapidLanguage.entries.find(e => e.name === 'Offs')?.insertText).toBe('Offs(${Point}, ${XOffset}, ${YOffset}, ${ZOffset})');
    expect(move.documentationUrl).toContain('3399634bb95042bb8fe7632cbb8f2255');
  });

  it('corrects Unicode punctuation, multiline return types and the legacy no-Syntax entry', () => {
    for (const profile of Object.values(catalog.profiles)) {
      expect(profile.entries.find(e => e.name === 'PoseInv')).toMatchObject({ signature: 'PoseInv(Pose)', returns: 'pose' });
      expect(profile.entries.find(e => e.name === 'GetSignalOrigin')).toMatchObject({ returns: 'signalorigin' });
      expect(profile.entries.find(e => e.name === 'StrFormat')).toMatchObject({ returns: 'string' });
      expect(profile.entries.find(e => e.name === 'UIAlphaEntry')).toMatchObject({ returns: 'string' });
    }
    expect(catalog.profiles['6.16'].entries.find(e => e.name === 'SCWrite')?.signature).toBe('SCWrite [\\ToNode], Variable;');
  });

  it('provides editor descriptions, typed parameter help, snippets and explicit scope limits', () => {
    for (const entry of rapidLanguage.entries) {
      expect(entry.description.length).toBeGreaterThan(4);
      expect(entry.signature.length).toBeGreaterThan(0);
      for (const p of entry.parameters ?? []) expect(p.description.length).toBeGreaterThan(3);
    }
    expect(rapidLanguage.entries.filter(e => e.kind === 'snippet')).toHaveLength(9);
    expect(rapidLanguage.entries.find(e => e.kind === 'snippet' && e.name === 'module')?.insertText).toContain('${ModuleName}');
    expect(rapidLanguage.limitations.join(' ')).toContain('跨模块类型推导');
    expect(rapidLanguage.limitations.join(' ')).toContain('EGM');
    expect(rapidLanguage.sources.find(s => s.license)?.license).toBe('Apache-2.0');
  });
});
