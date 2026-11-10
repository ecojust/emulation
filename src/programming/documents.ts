import { getModelProgrammingProfile, getProgramDialect, type ProgramDialectId } from './profiles';
import type { ProgramSourceFile, RobotLanguageId } from './types';

export const SOURCE_PROJECT_SCHEMA_VERSION = 1;
export const SOURCE_LIMITS = Object.freeze({ maximumFiles: 32, fileBytes: 1024 * 1024, projectBytes: 4 * 1024 * 1024, archiveBytes: 20 * 1024 * 1024, nameLength: 128, nameBytes: 255 });
export interface SourceFileInput { name: string; text: string }
export interface SourceDocument extends ProgramSourceFile {
  readonly revision: number;
  /** Difference from the most recently imported or successfully exported source file. */
  readonly dirty: boolean;
  readonly savedSource: Readonly<SourceFileInput> | null;
}
export interface SourceProject {
  readonly schemaVersion: typeof SOURCE_PROJECT_SCHEMA_VERSION;
  readonly modelId: string;
  readonly languageId: RobotLanguageId;
  readonly dialectId: ProgramDialectId;
  /** Every persisted draft mutation, including tab selection and source-file save metadata. */
  readonly revision: number;
  /** Changes only when source, file structure, or dialect changes. Bind analysis/run results to this. */
  readonly sourceRevision: number;
  readonly activeFileId: string | null;
  readonly files: readonly SourceDocument[];
}
const encoder = new TextEncoder();
const byteCounts = new WeakMap<SourceDocument, { current: number; saved: number }>();
const canonicalName = (name: string) => name.normalize('NFC').toLowerCase();
const object = (value: unknown): value is Record<string, unknown> => typeof value === 'object' && value !== null && !Array.isArray(value);
function checkedRevision(value: unknown): number {
  if (typeof value !== 'number' || !Number.isSafeInteger(value) || value < 0) throw new Error('源码工程版本编号无效。');
  return value;
}
function textBytes(text: unknown): number {
  if (typeof text !== 'string' || text.length > SOURCE_LIMITS.fileBytes) throw new Error('单个源文件不能超过 1 MiB。');
  for (let i = 0; i < text.length; i++) {
    const code = text.charCodeAt(i);
    if (code >= 0xd800 && code <= 0xdbff) {
      const next = text.charCodeAt(++i);
      if (!(next >= 0xdc00 && next <= 0xdfff)) throw new Error('源文件包含无效 Unicode 字符，不能无损保存为 UTF-8。');
    } else if (code >= 0xdc00 && code <= 0xdfff) throw new Error('源文件包含无效 Unicode 字符，不能无损保存为 UTF-8。');
  }
  const bytes = encoder.encode(text).byteLength;
  if (bytes > SOURCE_LIMITS.fileBytes) throw new Error('单个源文件不能超过 1 MiB。');
  return bytes;
}
export function validateSourceFileName(name: unknown, languageId: RobotLanguageId): asserts name is string {
  if (typeof name !== 'string' || !name || name.length > SOURCE_LIMITS.nameLength || name.trim() !== name
    || /[\x00-\x1f\x7f/\\:*?"<>|]/.test(name) || /[. ]$/.test(name)
    || /^(?:CON|PRN|AUX|NUL|COM[1-9¹²³]|LPT[1-9¹²³])(?:\.|$)/i.test(name)) throw new Error('文件名必须是有效的本地文件名，不能包含路径或系统保留名称。');
  if (textBytes(name) > SOURCE_LIMITS.nameBytes) throw new Error('文件名的 UTF-8 编码不能超过 255 字节。');
  const dot = name.lastIndexOf('.');
  const extensions = PROGRAM_EXTENSIONS[languageId];
  if (dot < 1 || !extensions.includes(name.slice(dot + 1).toLowerCase())) throw new Error(`当前语言只接受 ${extensions.map(extension => '.' + extension).join(' / ')} 文件。`);
}
export const PROGRAM_EXTENSIONS: Readonly<Record<RobotLanguageId, readonly string[]>> = Object.freeze({ rapid: Object.freeze(['modx', 'sysx', 'mod', 'sys']), 'fanuc-ls': Object.freeze(['ls']), inform: Object.freeze(['jbi']), krl: Object.freeze(['src', 'dat']) });
function document(id: string, input: SourceFileInput, revision: number, savedSource: SourceFileInput | null): SourceDocument {
  if (!id || id.length > 128) throw new Error('源文件标识无效。');
  return Object.freeze({ id, name: input.name, text: input.text, revision,
    savedSource: savedSource ? Object.freeze({ name: savedSource.name, text: savedSource.text }) : null,
    dirty: !savedSource || savedSource.name !== input.name || savedSource.text !== input.text });
}
function validateFiles(files: readonly SourceDocument[], languageId: RobotLanguageId): void {
  if (files.length > SOURCE_LIMITS.maximumFiles) throw new Error('一个源码工程最多包含 32 个文件。');
  const ids = new Set<string>(), names = new Set<string>();
  let size = 0;
  for (const file of files) {
    validateSourceFileName(file.name, languageId);
    if (ids.has(file.id) || names.has(canonicalName(file.name))) throw new Error('工程包含重复文件名或文件标识。');
    ids.add(file.id); names.add(canonicalName(file.name));
    if (file.savedSource) validateSourceFileName(file.savedSource.name, languageId);
    let bytes = byteCounts.get(file);
    if (!bytes) {
      const current = textBytes(file.text);
      bytes = { current, saved: file.savedSource ? file.savedSource.text === file.text ? current : textBytes(file.savedSource.text) : 0 };
      if (Object.isFrozen(file)) byteCounts.set(file, bytes);
    }
    size += bytes.current;
  }
  // Each baseline remains bounded by fileBytes and maximumFiles. Old larger disk
  // versions must not prevent marking a genuinely successful export as clean.
  if (size > SOURCE_LIMITS.projectBytes) throw new Error('工程正文总计不能超过 4 MiB。');
}
function changed(project: SourceProject, patch: Partial<SourceProject>, sourceChanged = true): SourceProject {
  const next = { ...project, ...patch, revision: checkedRevision(project.revision + 1), sourceRevision: checkedRevision(project.sourceRevision + Number(sourceChanged)) };
  validateFiles(next.files, next.languageId);
  return Object.freeze(next);
}
export function createSourceProject(modelId: string): SourceProject {
  const profile = getModelProgrammingProfile(modelId);
  return Object.freeze({ schemaVersion: SOURCE_PROJECT_SCHEMA_VERSION, modelId, languageId: profile.languageId, dialectId: profile.defaultDialectId,
    revision: 0, sourceRevision: 0, activeFileId: null, files: Object.freeze([]) });
}
export function nextSourceFileName(project: SourceProject): string {
  const extension = getProgramDialect(project.dialectId).extensions[0], names = new Set(project.files.map(file => canonicalName(file.name)));
  for (let index = 1; index <= SOURCE_LIMITS.maximumFiles + 1; index++) {
    const candidate = `${index === 1 ? 'main' : 'main_' + index}.${extension}`;
    if (!names.has(candidate)) return candidate;
  }
  throw new Error('不能生成可用的文件名。');
}
export function addSourceDocument(project: SourceProject, name = nextSourceFileName(project), text = '', id: string = crypto.randomUUID()): SourceProject {
  const file = document(id, { name, text }, 0, null);
  return changed(project, { files: Object.freeze([...project.files, file]), activeFileId: id });
}
export function importSourceDocuments(project: SourceProject, files: readonly SourceFileInput[], idFactory: () => string = () => crypto.randomUUID()): SourceProject {
  if (!files.length) return project;
  const incoming = files.map(input => document(idFactory(), input, 0, input));
  return changed(project, { files: Object.freeze([...project.files, ...incoming]), activeFileId: incoming[0].id });
}
function existing(project: SourceProject, id: string): SourceDocument {
  const file = project.files.find(file => file.id === id);
  if (!file) throw new Error('源文件已不存在，请重新选择。');
  return file;
}
export function updateSourceText(project: SourceProject, id: string, text: string): SourceProject {
  const file = existing(project, id);
  if (file.text === text) return project;
  const next = document(id, { name: file.name, text }, checkedRevision(file.revision + 1), file.savedSource);
  return changed(project, { files: Object.freeze(project.files.map(item => item.id === id ? next : item)) });
}
export function renameSourceDocument(project: SourceProject, id: string, name: string): SourceProject {
  const file = existing(project, id);
  if (file.name === name) return project;
  const next = document(id, { name, text: file.text }, checkedRevision(file.revision + 1), file.savedSource);
  return changed(project, { files: Object.freeze(project.files.map(item => item.id === id ? next : item)) });
}
export function removeSourceDocument(project: SourceProject, id: string): SourceProject {
  existing(project, id);
  const files = Object.freeze(project.files.filter(file => file.id !== id));
  return changed(project, { files, activeFileId: project.activeFileId === id ? files[0]?.id ?? null : project.activeFileId });
}
export function selectSourceDocument(project: SourceProject, id: string): SourceProject {
  existing(project, id);
  return project.activeFileId === id ? project : changed(project, { activeFileId: id }, false);
}
export function setSourceDialect(project: SourceProject, id: string): SourceProject {
  const profile = getModelProgrammingProfile(project.modelId), dialect = getProgramDialect(id);
  if (!profile.dialectIds.includes(dialect.id)) throw new Error('此语言版本不属于当前机器人品牌。');
  return project.dialectId === id ? project : changed(project, { dialectId: dialect.id });
}
/** Pass the exact snapshot written to disk; a late save never marks newer edits clean. */
export function markSourceFileSaved(project: SourceProject, id: string, exported: SourceFileInput): SourceProject {
  const file = existing(project, id);
  if (file.savedSource?.name === exported.name && file.savedSource.text === exported.text) return project;
  const next = document(id, file, file.revision, exported);
  return changed(project, { files: Object.freeze(project.files.map(item => item.id === id ? next : item)) }, false);
}
/** Validate storage/import structure only. Unknown robot syntax remains untouched. */
export function validateSourceProject(value: unknown, expectedModelId: string): SourceProject {
  if (!object(value) || value.schemaVersion !== SOURCE_PROJECT_SCHEMA_VERSION) throw new Error('源码工程格式版本不支持；原草稿未被覆盖。');
  const profile = getModelProgrammingProfile(expectedModelId);
  if (value.modelId !== expectedModelId) throw new Error('源码工程属于其他机器人型号，请切换到对应型号。');
  if (value.languageId !== profile.languageId || typeof value.dialectId !== 'string' || !profile.dialectIds.includes(value.dialectId as ProgramDialectId)) throw new Error('工程语言或版本与机器人品牌不匹配。');
  if (!Array.isArray(value.files) || value.files.length > SOURCE_LIMITS.maximumFiles) throw new Error('源码工程文件列表无效或超过 32 个文件。');
  const revision = checkedRevision(value.revision), sourceRevision = checkedRevision(value.sourceRevision);
  if (sourceRevision > revision) throw new Error('源码工程版本编号无效。');
  const files = value.files.map(file => {
    if (!object(file) || typeof file.id !== 'string' || typeof file.name !== 'string' || typeof file.text !== 'string') throw new Error('源码工程包含无效文件。');
    const saved = file.savedSource;
    if (saved !== null && (!object(saved) || typeof saved.name !== 'string' || typeof saved.text !== 'string')) throw new Error('源文件保存基线无效。');
    return document(file.id, { name: file.name, text: file.text }, checkedRevision(file.revision), saved as SourceFileInput | null);
  });
  validateFiles(files, profile.languageId);
  if (value.activeFileId !== null && (typeof value.activeFileId !== 'string' || !files.some(file => file.id === value.activeFileId))) throw new Error('工程当前文件标识无效。');
  if (files.length && value.activeFileId === null) throw new Error('工程缺少当前文件标识。');
  return Object.freeze({ schemaVersion: SOURCE_PROJECT_SCHEMA_VERSION, modelId: expectedModelId, languageId: profile.languageId,
    dialectId: value.dialectId as ProgramDialectId, revision, sourceRevision, activeFileId: value.activeFileId as string | null, files: Object.freeze(files) });
}
export function serializeSourceProject(project: SourceProject): string {
  const text = JSON.stringify(validateSourceProject(project, project.modelId), null, 2);
  if (encoder.encode(text).byteLength > SOURCE_LIMITS.archiveBytes) throw new Error('工程归档超过 20 MiB，不能导出。');
  return text;
}
export function parseSourceProject(text: string, expectedModelId: string): SourceProject {
  if (typeof text !== 'string' || text.length > SOURCE_LIMITS.archiveBytes || encoder.encode(text).byteLength > SOURCE_LIMITS.archiveBytes) throw new Error('工程归档不能超过 20 MiB。');
  let value: unknown;
  try { value = JSON.parse(text.startsWith('\ufeff') ? text.slice(1) : text); } catch { throw new Error('源码工程不是有效的 JSON 文件；原草稿未被覆盖。'); }
  return validateSourceProject(value, expectedModelId);
}
export function decodeSourceFile(bytes: Uint8Array): string {
  if (bytes.byteLength > SOURCE_LIMITS.fileBytes) throw new Error('单个源文件不能超过 1 MiB。');
  try { return new TextDecoder('utf-8', { fatal: true, ignoreBOM: true }).decode(bytes); }
  catch { throw new Error('源文件不是有效 UTF-8，请明确转换编码后再导入。'); }
}
export function encodeSourceFile(text: string): Uint8Array {
  textBytes(text);
  return encoder.encode(text);
}
