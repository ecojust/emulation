import { describe, expect, it } from 'vitest';
import { createRapidLanguage, rapidLanguage } from '../src/programming/languages/rapid';

const modern = rapidLanguage;
const legacy = createRapidLanguage('6.16');
const utf8 = (source: string) => new TextDecoder('utf-8', { fatal: true, ignoreBOM: true }).decode(new TextEncoder().encode(source));
const program = 'MODULE Review\r\nPROC main()\r\n! 中文说明\r\nTPWrite "🙂运行";\r\nENDPROC\r\nENDMODULE\r\n';
const system = 'MODULE Review(SYSMODULE)\r\nPERS string status:="准备就绪";\r\nENDMODULE\r\n';

describe('RAPID file format and encoding diagnostics', () => {
  it('declares modern UTF-8 module formats separately from the RW6 profile', () => {
    expect(modern.extensions).toEqual(['modx', 'sysx', 'mod', 'sys']);
    expect(legacy.extensions).toEqual(['mod', 'sys']);
  });

  it.each([['Review.MODX', program], ['Review.sysx', system]])('reads plain UTF-8 source in %s', (name, text) => {
    const source = utf8(text);
    const result = modern.analyze(source, name);
    expect(result.diagnostics).toEqual([]);
    expect(result.symbols.map(symbol => source.slice(symbol.from, symbol.to))).toContain('Review');
    expect(source).toBe(text);
    expect(source).toContain('\r\n');
  });

  it.each(['Review.modx', 'Review.SYSX'])('reports a BOM in %s without shifting source locations', name => {
    const source = utf8('\ufeff' + program);
    const result = modern.analyze(source, name);
    expect(result.diagnostics).toEqual([expect.objectContaining({ from: 0, to: 1, severity: 'warning', code: 'rapid.utf8-bom' })]);
    const module = result.symbols.find(symbol => symbol.name === 'Review')!;
    expect(module.from).toBe(source.indexOf('Review'));
    expect(source.slice(module.from, module.to)).toBe('Review');
    expect(source.charCodeAt(0)).toBe(0xfeff);
  });

  it.each(['Review.modx', 'Review.SYSX'])('reports the incompatible RW6 filename format %s', name => {
    expect(legacy.analyze(program, name).diagnostics).toEqual([expect.objectContaining({ severity: 'warning', code: 'rapid.file-version' })]);
    const empty = legacy.analyze('', name).diagnostics[0];
    expect(empty).toMatchObject({ from: 0, to: 0, code: 'rapid.file-version' });
  });

  it.each(['Review.mod', 'Review.sys'])('does not warn for an ASCII legacy module %s', name => {
    const source = 'MODULE Review\nPROC main()\nTPWrite "Ready";\nENDPROC\nENDMODULE';
    expect(modern.analyze(source, name).diagnostics).toEqual([]);
  });

  it.each(['é', '🙂'])('locates UTF-8 legacy encoding mismatch for %s using UTF-16 offsets', character => {
    const source = `MODULE Review\n! ${character} annotation\nENDMODULE`;
    const offset = source.indexOf(character);
    expect(modern.analyze(source, 'Review.MOD').diagnostics).toEqual([
      expect.objectContaining({ from: offset, to: offset + character.length, severity: 'warning', code: 'rapid.legacy-encoding' }),
    ]);
  });

  it('keeps grammar errors visible alongside the format warning', () => {
    const diagnostics = modern.analyze('\ufeffMODULE Review\nPROC main()\nvalue:=;\nENDPROC\nENDMODULE', 'Review.modx').diagnostics;
    expect(diagnostics.some(diagnostic => diagnostic.severity === 'error')).toBe(true);
    expect(diagnostics.some(diagnostic => diagnostic.code === 'rapid.utf8-bom')).toBe(true);
  });
});
