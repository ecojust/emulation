import { invoke } from '@tauri-apps/api/core';
import { nativeDesktop } from '../robot/client';
import type { RobotLanguageId } from './types';
import { encodeSourceFile, PROGRAM_EXTENSIONS, validateSourceFileName } from './documents';

export interface ImportedSource { name: string; text: string }
export interface SourceSaveResult { name: string; outcome: 'saved' | 'download-requested'; path?: string }
export const SOURCE_EXTENSIONS = PROGRAM_EXTENSIONS;
const MAX_FILE = 1024 * 1024, MAX_TOTAL = 4 * MAX_FILE, MAX_FILES = 32;
function validatedSourceBytes(language: RobotLanguageId, file: ImportedSource): number {
  if (!file || typeof file.name !== 'string' || typeof file.text !== 'string') throw new Error('源文件数据无效。');
  validateSourceFileName(file.name, language);
  if (file.text.length > MAX_FILE) throw new Error('单个源文件不能超过 1 MiB。');
  if (file.text.includes('\0')) throw new Error('源文件含二进制或 UTF-16 数据；请先转换为 UTF-8。');
  return encodeSourceFile(file.text).byteLength;
}
export function validateSourceFile(language: RobotLanguageId, file: ImportedSource): void { validatedSourceBytes(language, file); }
export function decodeSourceFile(language: RobotLanguageId, name: string, bytes: ArrayBuffer): ImportedSource {
  if (bytes.byteLength > MAX_FILE) throw new Error('单个源文件不能超过 1 MiB。');
  let text: string;
  try { text = new TextDecoder('utf-8', { fatal: true, ignoreBOM: true }).decode(bytes); }
  catch { throw new Error('源文件不是 UTF-8；请先转换编码，避免内容被错误替换。'); }
  const file = { name, text }; validateSourceFile(language, file); return file;
}
function checkFiles(language: RobotLanguageId, value: unknown): ImportedSource[] {
  if (!Array.isArray(value) || value.length > MAX_FILES) throw new Error('一次最多导入 32 个源文件。');
  let total = 0;
  for (const file of value) total += validatedSourceBytes(language, file);
  if (total > MAX_TOTAL) throw new Error('导入的源文件内容不能超过 4 MiB。');
  return value;
}
export async function openSourceFiles(language: RobotLanguageId): Promise<ImportedSource[] | null> {
  if (nativeDesktop) {
    const result = await invoke<ImportedSource[] | null>('open_robot_sources', { languageId: language });
    return result === null ? null : checkFiles(language, result);
  }
  return new Promise((resolve, reject) => {
    const input = document.createElement('input'); input.type = 'file'; input.multiple = true;
    input.accept = SOURCE_EXTENSIONS[language].map(extension => '.' + extension).join(','); input.hidden = true;
    let finished = false, reading = false, timer = 0;
    const done = (files: ImportedSource[] | null, error?: unknown) => {
      if (finished) return; finished = true; clearTimeout(timer); window.removeEventListener('focus', focused); input.remove();
      if (error) reject(error); else resolve(files);
    };
    const focused = () => { timer = window.setTimeout(() => { if (!reading && !input.files?.length) done(null); }, 300); };
    input.addEventListener('cancel', () => done(null), { once: true });
    input.addEventListener('change', () => {
      reading = true; clearTimeout(timer);
      const files = [...(input.files ?? [])];
      if (!files.length) { done(null); return; }
      if (files.length > MAX_FILES || files.some(file => file.size > MAX_FILE) || files.reduce((total, file) => total + file.size, 0) > MAX_TOTAL) {
        done(null, new Error('源文件超出导入限制：32 个文件、单文件 1 MiB、总共 4 MiB。')); return;
      }
      void Promise.all(files.map(async file => decodeSourceFile(language, file.name, await file.arrayBuffer())))
        .then(files => checkFiles(language, files)).then(files => done(files)).catch(error => done(null, error));
    }, { once: true });
    window.addEventListener('focus', focused); document.body.append(input); input.click();
  });
}
export async function saveSourceFile(language: RobotLanguageId, file: ImportedSource): Promise<SourceSaveResult | null> {
  validateSourceFile(language, file);
  if (nativeDesktop) {
    const result = await invoke<{ fileName: string; path: string } | null>('save_robot_source', { request: { languageId: language, fileName: file.name, text: file.text } });
    if (result === null) return null;
    if (!result || typeof result !== 'object' || typeof result.path !== 'string' || !result.path || typeof result.fileName !== 'string') throw new Error('源文件保存未返回有效结果。');
    validateSourceFile(language, { name: result.fileName, text: file.text });
    return { name: result.fileName, path: result.path, outcome: 'saved' };
  }
  const link = document.createElement('a'); const url = URL.createObjectURL(new Blob([file.text], { type: 'text/plain;charset=utf-8' }));
  link.href = url; link.download = file.name; document.body.append(link); link.click(); link.remove();
  // Keep the URL alive long enough for the browser to acquire the actual bytes.
  window.setTimeout(() => URL.revokeObjectURL(url), 60_000);
  return { name: file.name, outcome: 'download-requested' };
}
