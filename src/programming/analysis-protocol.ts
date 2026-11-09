import { getProgramDialect, type ProgramDialectId } from './profiles';
import type { LanguageAnalysis, ProgramSourceFile } from './types';
export interface AnalysisContext { modelId: string; sourceRevision: number; dialectId: ProgramDialectId }
export interface SourceAnalysisRequest extends AnalysisContext { files: readonly ProgramSourceFile[] }
export interface ProjectAnalysis { byFile: Record<string, LanguageAnalysis> }
export interface AnalysisMessage extends SourceAnalysisRequest { requestId: string }
/** The request ID and complete source context must both match before a result is usable. */
export interface AnalysisReply extends AnalysisContext { requestId: string; result?: ProjectAnalysis; error?: string }

const object = (value: unknown): value is Record<string, unknown> => typeof value === 'object' && value !== null && !Array.isArray(value);
const utf8 = new TextEncoder();
/** Snapshot before posting: caller edits must not change which ranges a response validates against. */
export function snapshotAnalysisRequest(value: unknown): SourceAnalysisRequest {
  if (!object(value) || typeof value.modelId !== 'string' || !value.modelId || value.modelId.length > 128
    || !Number.isSafeInteger(value.sourceRevision) || (value.sourceRevision as number) < 0
    || typeof value.dialectId !== 'string' || !Array.isArray(value.files) || value.files.length > 32) throw new Error('语法分析请求无效。');
  const dialectId = getProgramDialect(value.dialectId).id;
  const ids = new Set<string>();
  let bytes = 0;
  const files = value.files.map((file: unknown) => {
    if (!object(file) || typeof file.id !== 'string' || !file.id || file.id.length > 128 || ids.has(file.id)
      || typeof file.name !== 'string' || !file.name || file.name.length > 255
      || typeof file.text !== 'string' || file.text.length > 1024 * 1024) throw new Error('语法分析文件无效。');
    ids.add(file.id);
    const size = utf8.encode(file.text).length; bytes += size;
    if (size > 1024 * 1024 || bytes > 4 * 1024 * 1024) throw new Error('源文件超出分析大小限制。');
    return Object.freeze({ id: file.id, name: file.name, text: file.text });
  });
  return Object.freeze({ modelId: value.modelId, sourceRevision: value.sourceRevision as number, dialectId, files: Object.freeze(files) });
}
