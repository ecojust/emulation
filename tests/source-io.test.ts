import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
const platform = vi.hoisted(() => ({ nativeDesktop: true, invoke: vi.fn() }));
vi.mock('../src/robot/client', () => ({ get nativeDesktop() { return platform.nativeDesktop; } }));
vi.mock('@tauri-apps/api/core', () => ({ invoke: platform.invoke }));
import { decodeSourceFile, openSourceFiles, saveSourceFile, validateSourceFile } from '../src/programming/source-io';
import { validateSourceFileName } from '../src/programming/documents';

beforeEach(() => { platform.nativeDesktop = true; platform.invoke.mockReset(); });
afterEach(() => { vi.restoreAllMocks(); vi.unstubAllGlobals(); vi.useRealTimers(); });

describe('offline source boundaries', () => {
  it('accepts plain UTF-8 modx/sysx and preserves an imported BOM rather than silently converting it', () => {
    for (const name of ['module.modx', 'System.SYSX', 'legacy.mod', 'legacy.sys']) {
      for (const prefix of ['', '\ufeff']) {
        const text = prefix + 'MODULE Test\r\n! 中文\r\nENDMODULE\r\n';
        const bytes = new TextEncoder().encode(text);
        const file = decodeSourceFile('rapid', name, bytes.buffer);
        expect(file).toEqual({ name, text });
        expect(new TextEncoder().encode(file.text)).toEqual(bytes);
      }
    }
    for (const language of ['fanuc-ls', 'inform', 'krl'] as const) {
      expect(() => validateSourceFile(language, { name: 'module.modx', text: 'MODULE Test\nENDMODULE' })).toThrow();
    }
  });
  it('preserves UTF-8 BOM, mixed CRLF and unicode without rewriting unknown syntax', () => {
    const text = '\ufeffMODULE Test\r\n! 中文😀\nVendorNew \\Option;\r\nENDMODULE';
    const file = decodeSourceFile('rapid', '源程序.MOD', new TextEncoder().encode(text).buffer);
    expect(file.text).toBe(text);
    expect([...new TextEncoder().encode(file.text)]).toEqual([...new TextEncoder().encode(text)]);
  });
  it('rejects undecodable and binary text without replacement characters', () => {
    expect(() => decodeSourceFile('rapid', 'x.mod', new Uint8Array([0xff, 0xfe, 0x20, 0]).buffer)).toThrow(/UTF-8/);
    expect(() => decodeSourceFile('rapid', 'x.mod', new Uint8Array([65, 0, 66]).buffer)).toThrow(/UTF-16/);
    expect(() => validateSourceFile('rapid', { name: 'x.mod', text: '\ud800' })).toThrow(/Unicode/);
  });
  it('uses UTF-8 byte limits and rejects filename paths or mismatched languages', () => {
    for (const name of ['../x.mod', 'dir\\x.mod', 'CON.mod', 'x.ls', 'x.mod ']) expect(() => validateSourceFile('rapid', { name, text: 'valid' })).toThrow();
    expect(() => validateSourceFile('rapid', { name: 'x.mod', text: '中'.repeat(350_000) })).toThrow(/MiB/);
    expect(() => validateSourceFile('krl', { name: 'CELL.DAT', text: 'DEFDAT CELL\nENDDAT' })).not.toThrow();
  });
  it('matches draft filename rules at Unicode whitespace, portable name lengths, and Windows device names', () => {
    const valid = ['a'.repeat(124) + '.mod', '中'.repeat(83) + '.mod', '\u0085allowed.mod', 'COM10.mod'];
    const invalid = ['a'.repeat(125) + '.mod', '中'.repeat(84) + '.mod', '\ufeffhidden.mod', '\u00a0space.mod', 'COM¹.mod', 'lpt².sys', '.mod', 'missing-extension'];
    for (const name of valid) {
      expect(() => validateSourceFileName(name, 'rapid')).not.toThrow();
      expect(() => validateSourceFile('rapid', { name, text: '' })).not.toThrow();
    }
    for (const name of invalid) {
      expect(() => validateSourceFileName(name, 'rapid')).toThrow();
      expect(() => validateSourceFile('rapid', { name, text: '' })).toThrow();
    }
  });
});

describe('native source file delivery', () => {
  it('treats both native dialog cancellations as cancellation without inventing a saved file', async () => {
    platform.invoke.mockResolvedValue(null);
    await expect(openSourceFiles('rapid')).resolves.toBeNull();
    await expect(saveSourceFile('rapid', { name: 'main.mod', text: 'unknown' })).resolves.toBeNull();
    expect(platform.invoke.mock.calls.map(call => call[0])).toEqual(['open_robot_sources', 'save_robot_source']);
  });
  it('passes exact source bytes as UTF-8 text and uses the filename actually confirmed in Save As', async () => {
    const text = '\ufeffMODULE Main\r\nVendorUnknown;\r\nENDMODULE\r\n';
    platform.invoke.mockResolvedValue({ path: '/chosen/module.sys', fileName: 'module.sys' });
    await expect(saveSourceFile('rapid', { name: 'main.mod', text })).resolves.toEqual({ name: 'module.sys', path: '/chosen/module.sys', outcome: 'saved' });
    expect(platform.invoke).toHaveBeenCalledWith('save_robot_source', { request: { languageId: 'rapid', fileName: 'main.mod', text } });
    platform.invoke.mockResolvedValue([{ name: 'module.sys', text }]);
    await expect(openSourceFiles('rapid')).resolves.toEqual([{ name: 'module.sys', text }]);
  });
  it('does not fall back to browser download after a native write failure', async () => {
    platform.invoke.mockRejectedValue(new Error('destination is locked'));
    await expect(saveSourceFile('rapid', { name: 'main.mod', text: 'valid' })).rejects.toThrow('destination is locked');
    expect(platform.invoke).toHaveBeenCalledTimes(1);
  });
  it('rejects malformed native delivery responses and invalid requests before any write', async () => {
    await expect(saveSourceFile('rapid', { name: 'COM¹.mod', text: 'valid' })).rejects.toThrow();
    expect(platform.invoke).not.toHaveBeenCalled();
    for (const response of [undefined, {}, { path: '', fileName: 'main.mod' }, { path: '/chosen/main', fileName: 'main' }]) {
      platform.invoke.mockResolvedValue(response);
      await expect(saveSourceFile('rapid', { name: 'main.mod', text: 'valid' })).rejects.toThrow();
    }
  });
  it('applies file-count, byte-total, encoding and language limits to native import results', async () => {
    for (const response of [
      Array.from({ length: 33 }, (_, index) => ({ name: `p${index}.mod`, text: '' })),
      Array.from({ length: 5 }, (_, index) => ({ name: `p${index}.mod`, text: 'x'.repeat(1024 * 1024) })),
      [{ name: 'main.mod', text: 'bad\0text' }], [{ name: 'main.ls', text: 'foreign' }], undefined,
    ]) {
      platform.invoke.mockResolvedValue(response);
      await expect(openSourceFiles('rapid')).rejects.toThrow();
    }
  });
});

describe('browser reference source IO', () => {
  it('distinguishes a download request from a confirmed native save and retains exact bytes', async () => {
    vi.useFakeTimers(); platform.nativeDesktop = false;
    const link = { href: '', download: '', click: vi.fn(), remove: vi.fn() };
    vi.stubGlobal('document', { createElement: () => link, body: { append: vi.fn() } });
    vi.stubGlobal('window', { setTimeout });
    let blob: Blob | undefined;
    vi.spyOn(URL, 'createObjectURL').mockImplementation(value => { blob = value as Blob; return 'blob:source'; });
    const revoke = vi.spyOn(URL, 'revokeObjectURL').mockImplementation(() => {});
    const text = '\ufeffMODULE Test\r\nunknown;\r\n';
    await expect(saveSourceFile('rapid', { name: 'main.mod', text })).resolves.toEqual({ name: 'main.mod', outcome: 'download-requested' });
    expect(new Uint8Array(await blob!.arrayBuffer())).toEqual(new TextEncoder().encode(text));
    expect(link.download).toBe('main.mod'); expect(link.click).toHaveBeenCalledOnce();
    expect(platform.invoke).not.toHaveBeenCalled(); expect(revoke).not.toHaveBeenCalled();
    await vi.advanceTimersByTimeAsync(60_000); expect(revoke).toHaveBeenCalledWith('blob:source');
  });
  it('resolves file-picker cancellation and removes its temporary input', async () => {
    platform.nativeDesktop = false;
    const input = Object.assign(new EventTarget(), { files: [], click: vi.fn(), remove: vi.fn() });
    vi.stubGlobal('document', { createElement: () => input, body: { append: vi.fn() } });
    vi.stubGlobal('window', Object.assign(new EventTarget(), { setTimeout }));
    const opened = openSourceFiles('rapid');
    input.dispatchEvent(new Event('cancel'));
    await expect(opened).resolves.toBeNull(); expect(input.remove).toHaveBeenCalledOnce();
    expect(platform.invoke).not.toHaveBeenCalled();
  });
});
