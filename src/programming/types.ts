/** Source offsets are UTF-16 offsets, matching CodeMirror and JavaScript strings. */
export type RobotLanguageId = 'rapid' | 'fanuc-ls' | 'inform' | 'krl';
export type SymbolKind = 'instruction' | 'function' | 'type' | 'keyword' | 'variable' | 'routine' | 'constant' | 'snippet';
export interface LanguageParameter {
  name: string;
  type?: string;
  description: string;
  optional?: boolean;
}
export interface LanguageEntry {
  name: string;
  kind: SymbolKind;
  signature: string;
  /** Original concise explanation; do not copy manuals wholesale. */
  description: string;
  /** CodeMirror snippet syntax: ${name} tab stops, ${} final cursor. */
  insertText?: string;
  parameters?: readonly LanguageParameter[];
  category?: string;
  documentationUrl?: string;
  availableSince?: string;
  requiresOption?: string;
}
export interface SourceDiagnostic {
  from: number;
  to: number;
  severity: 'error' | 'warning' | 'info';
  message: string;
  code?: string;
}
export interface SourceSymbol {
  name: string;
  kind: SymbolKind;
  from: number;
  to: number;
  signature?: string;
  description?: string;
}
export interface SourceFold { from: number; to: number }
export interface LanguageAnalysis {
  diagnostics: SourceDiagnostic[];
  symbols: SourceSymbol[];
  folds: SourceFold[];
}
export interface LanguageSource {
  title: string;
  url: string;
  revision?: string;
  license?: string;
}
export interface LexicalProfile {
  lineComment: string;
  blockComment?: readonly [string, string];
  /** LS ! comments end at a semicolon; other profiles use a newline. */
  commentTerminator?: string;
  keywords: readonly string[];
  types: readonly string[];
  stringEscape: 'backslash' | 'double-quote' | 'none';
  caseInsensitive: boolean;
}
export interface RobotLanguageDefinition {
  id: RobotLanguageId;
  brand: string;
  name: string;
  dialect: string;
  extensions: readonly string[];
  lexical: LexicalProfile;
  entries: readonly LanguageEntry[];
  sources: readonly LanguageSource[];
  /** Exact implemented grammar coverage and remaining controller-specific limits. */
  coverage: readonly string[];
  limitations: readonly string[];
  analyze(source: string, fileName?: string): LanguageAnalysis;
}
export interface ProgramSourceFile { id: string; name: string; text: string }
export interface FileDiagnostic extends SourceDiagnostic { fileId: string }
