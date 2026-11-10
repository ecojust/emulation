import { useEffect, useLayoutEffect, useRef, useState, type ReactNode } from 'react';
import { Compartment, EditorSelection, EditorState, StateEffect, Transaction } from '@codemirror/state';
import { EditorView, crosshairCursor, drawSelection, dropCursor, highlightActiveLine, highlightActiveLineGutter, highlightSpecialChars, hoverTooltip, keymap, lineNumbers, rectangularSelection } from '@codemirror/view';
import { HighlightStyle, StreamLanguage, bracketMatching, foldGutter, foldKeymap, foldService, indentOnInput, indentUnit, syntaxHighlighting, syntaxTree, type StreamParser } from '@codemirror/language';
import { autocompletion, closeBrackets, closeBracketsKeymap, completionKeymap, snippet, startCompletion, type Completion, type CompletionContext } from '@codemirror/autocomplete';
import { defaultKeymap, history, historyKeymap, indentWithTab, isolateHistory, redo, undo } from '@codemirror/commands';
import { highlightSelectionMatches, openSearchPanel, search, searchKeymap } from '@codemirror/search';
import { lintGutter, lintKeymap, nextDiagnostic, setDiagnostics } from '@codemirror/lint';
import { tags } from '@lezer/highlight';
import type { LanguageAnalysis, LanguageEntry, ProgramSourceFile, RobotLanguageDefinition } from './types';
import { copyRapid, nativeDesktop } from '../robot/client';
import './editor.css';

export interface SourceInsertion {
  changes: { from: number; to: number; insert: string }[];
  cursor?: number;
  message?: string;
}
export interface RobotCodeEditorProps {
  modelKey: string;
  language: RobotLanguageDefinition;
  files: readonly (ProgramSourceFile & { dirty?: boolean })[];
  activeFileId: string | null;
  onActiveFileChange: (id: string) => void;
  onTextChange: (id: string, text: string) => boolean | void;
  analysisByFile?: Readonly<Record<string, LanguageAnalysis | undefined>>;
  pendingFileIds?: readonly string[];
  analysisError?: string;
  onRetryAnalysis?: () => void;
  readOnly?: boolean;
  active?: boolean;
  toolbar?: ReactNode;
  status?: string;
  dirty?: boolean;
  onNewFile?: (name: string) => void;
  onRenameFile?: (id: string, name: string) => void;
  onRemoveFile?: (id: string) => void;
  onImportFiles?: () => void;
  onSave?: () => void;
  onExport?: () => void;
  captureDisabledReason?: string | null;
  onInsertCurrent?: (input: { fileId: string; source: string; from: number; to: number }) => SourceInsertion | Promise<SourceInsertion>;
}

interface SourceMap { text: string; toRaw: number[]; toView: number[]; separator: string }
/** CM uses LF internally. Source changes retain untouched original line endings and BOM. */
function sourceMap(source: string): SourceMap {
  const characters: string[] = [], toRaw = [0], toView = [0];
  let position = 0;
  for (let index = 0; index < source.length;) {
    const start = index;
    if (source[index] === '\r') { index += source[index + 1] === '\n' ? 2 : 1; characters.push('\n'); }
    else { characters.push(source[index]); index++; }
    position++;
    for (let raw = start + 1; raw <= index; raw++) toView[raw] = position;
    toRaw[position] = index;
  }
  return { text: characters.join(''), toRaw, toView, separator: /\r\n|\r|\n/.exec(source)?.[0] ?? '\n' };
}
const foldedAnalysisChanged = StateEffect.define<null>();
const wordPattern = /[$#]?[A-Za-z_\u0080-\uffff][\w$#\u0080-\uffff]*/g;
const normalizeName = (language: RobotLanguageDefinition, name: string) => {
  const normalized = name.replace(/[\t ]+/g, ' ');
  return language.lexical.caseInsensitive ? normalized.toUpperCase() : normalized;
};
interface CatalogMatcher {
  entries: Map<string, LanguageEntry>;
  names: RegExp;
  composite: RegExp | null;
  phrases: string[];
  maxPhraseLength: number;
}
const catalogMatchers = new WeakMap<RobotLanguageDefinition, CatalogMatcher>();
function catalogMatcher(language: RobotLanguageDefinition): CatalogMatcher {
  const cached = catalogMatchers.get(language); if (cached) return cached;
  const entries = new Map(language.entries.map(entry => [normalizeName(language, entry.name), entry]));
  const names = [...entries.keys()].sort((first, second) => second.length - first.length);
  const pattern = (name: string) => {
    const escaped = name.split(' ').map(part => part.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')).join('[\\t ]+');
    // PALLETIZING-B_1 uses a catalogued instruction followed by its numeric argument.
    const continuation = language.id === 'fanuc-ls' && name.startsWith('PALLETIZING-') ? '[A-Za-z0-9$#\\u0080-\\uffff-]'
      : language.id === 'fanuc-ls' ? '[\\w$#\\u0080-\\uffff-]' : '[\\w$#\\u0080-\\uffff]';
    return escaped + '(?!' + continuation + ')';
  };
  const flags = language.lexical.caseInsensitive ? 'i' : '';
  const composites = names.filter(name => /[ +\-]/.test(name));
  const phrases = names.filter(name => name.includes(' '));
  const matcher = { entries, names: new RegExp(names.map(pattern).join('|') || '(?!)', 'g' + flags),
    composite: composites.length ? new RegExp('^(?:' + composites.map(pattern).join('|') + ')', flags) : null,
    phrases, maxPhraseLength: Math.max(0, ...phrases.map(name => name.length)) };
  catalogMatchers.set(language, matcher); return matcher;
}
function catalogMatches(text: string, language: RobotLanguageDefinition) {
  const matcher = catalogMatcher(language);
  return [...text.matchAll(matcher.names)].filter(match => !match.index || !/[\w$#\u0080-\uffff]/.test(text[match.index - 1]));
}
function phrasePrefix(state: EditorState, position: number, language: RobotLanguageDefinition): number | undefined {
  const matcher = catalogMatcher(language); if (!matcher.phrases.length) return undefined;
  const line = state.doc.lineAt(position), prefix = line.text.slice(0, position - line.from);
  for (let start = Math.max(0, prefix.length - matcher.maxPhraseLength * 4); start < prefix.length; start++) {
    if (!/[A-Za-z_$#\u0080-\uffff]/.test(prefix[start]) || start && /[\w$#\u0080-\uffff]/.test(prefix[start - 1])) continue;
    const candidate = normalizeName(language, prefix.slice(start));
    if (!candidate.includes(' ') || !matcher.phrases.some(name => name.startsWith(candidate))) continue;
    if (!/comment|string/i.test(syntaxTree(state).resolveInner(line.from + start, 1).name)) return line.from + start;
  }
  return undefined;
}
function lexer(language: RobotLanguageDefinition): StreamParser<{ block: boolean; comment: boolean }> {
  const profile = language.lexical;
  const keywords = new Set(profile.keywords.map(name => normalizeName(language, name)));
  const types = new Set(profile.types.map(name => normalizeName(language, name)));
  const matcher = catalogMatcher(language), entries = matcher.entries;
  return {
    startState: () => ({ block: false, comment: false }),
    token(stream, state) {
      if (state.block && profile.blockComment) {
        if (stream.skipTo(profile.blockComment[1])) { stream.match(profile.blockComment[1]); state.block = false; } else stream.skipToEnd();
        return 'comment';
      }
      if (state.comment && profile.commentTerminator) {
        if (stream.skipTo(profile.commentTerminator)) { stream.match(profile.commentTerminator); state.comment = false; } else stream.skipToEnd();
        return 'comment';
      }
      if (stream.eatSpace()) return null;
      if (profile.blockComment && stream.match(profile.blockComment[0])) { state.block = true; return 'comment'; }
      const commentPosition = language.id !== 'fanuc-ls' || /^\s*(?:\d*\s*:\s*)?$/.test(stream.string.slice(0, stream.pos));
      if (commentPosition && (profile.lineComment && stream.match(profile.lineComment) || language.id === 'fanuc-ls' && stream.match('--'))) {
        if (profile.commentTerminator) {
          if (stream.skipTo(profile.commentTerminator)) stream.match(profile.commentTerminator);
          else { stream.skipToEnd(); state.comment = true; }
        } else stream.skipToEnd();
        return 'comment';
      }
      if (stream.peek() === '"' || language.id === 'fanuc-ls' && stream.peek() === "'") {
        const quote = stream.peek();
        stream.next();
        while (!stream.eol()) {
          const character = stream.next();
          if (character === '\\' && profile.stringEscape === 'backslash') { stream.next(); continue; }
          if (character === quote) {
            if (profile.stringEscape === 'double-quote' && stream.peek() === quote) { stream.next(); continue; }
            break;
          }
        }
        return 'string';
      }
      if (stream.match(/\/{1,3}[A-Za-z][\w]*/)) return 'meta';
      if (stream.match(/(?:\d+(?:\.\d*)?|\.\d+)(?:[Ee][+-]?\d+)?/)) return 'number';
      if (stream.match(/\\[$#]?[A-Za-z_][\w]*/)) return 'attributeName';
      const composite = matcher.composite && stream.match(matcher.composite);
      if (Array.isArray(composite)) {
        const entry = entries.get(normalizeName(language, composite[0]));
        return entry?.kind === 'type' ? 'typeName' : entry?.kind === 'keyword' ? 'keyword' : 'builtin';
      }
      const identifier = stream.match(/[$#]?[A-Za-z_\u0080-\uffff][\w$#\u0080-\uffff]*/);
      if (Array.isArray(identifier)) {
        const name = normalizeName(language, identifier[0]);
        if (keywords.has(name)) return 'keyword';
        if (types.has(name)) return 'typeName';
        const entry = entries.get(name);
        return entry?.kind === 'type' ? 'typeName' : entry?.kind === 'keyword' ? 'keyword' : entry ? 'builtin' : 'variableName';
      }
      if (stream.match(/[()[\]{},;:.]/)) return 'punctuation';
      if (stream.match(/[-+*/=<>!&|^%]+/)) return 'operator';
      stream.next(); return null;
    },
    languageData: { commentTokens: { line: profile.lineComment }, closeBrackets: { brackets: ['(', '[', '{', '"'] } },
  };
}
const highlighting = HighlightStyle.define([
  { tag: tags.keyword, color: '#8254a0' }, { tag: tags.typeName, color: '#2c877c' },
  { tag: tags.standard(tags.variableName), color: '#2a6eac' }, { tag: tags.variableName, color: '#465b76' },
  { tag: tags.string, color: '#9b6c35' }, { tag: tags.number, color: '#b4654d' },
  { tag: tags.comment, color: '#8b9aab', fontStyle: 'italic' }, { tag: tags.meta, color: '#6481a7' },
  { tag: tags.operator, color: '#6880a0' }, { tag: tags.punctuation, color: '#8a98a8' },
  { tag: tags.attributeName, color: '#6c8e5c' },
]);
function entryDescription(entry: LanguageEntry): HTMLElement {
  const container = document.createElement('div'); container.className = 'robot-code-tooltip';
  const title = document.createElement('strong'); title.textContent = entry.name; container.append(title);
  const signature = document.createElement('pre'); signature.textContent = entry.signature; container.append(signature);
  const description = document.createElement('p'); description.textContent = entry.description; container.append(description);
  if (entry.parameters?.length) {
    const list = document.createElement('dl');
    for (const parameter of entry.parameters) {
      const name = document.createElement('dt'); name.textContent = parameter.name + (parameter.type ? ': ' + parameter.type : '') + (parameter.optional ? '（可选）' : '');
      const detail = document.createElement('dd'); detail.textContent = parameter.description; list.append(name, detail);
    }
    container.append(list);
  }
  if (entry.requiresOption) { const note = document.createElement('p'); note.textContent = '控制器选项：' + entry.requiresOption; container.append(note); }
  if (entry.availableSince) { const note = document.createElement('p'); note.textContent = '版本：' + entry.availableSince; container.append(note); }
  return container;
}
function wordAt(view: EditorView, position: number, language: RobotLanguageDefinition): { word: string; from: number; to: number } | null {
  const line = view.state.doc.lineAt(position);
  for (const match of catalogMatches(line.text, language)) {
    const from = line.from + match.index!, to = from + match[0].length;
    if (from <= position && position <= to) return { word: match[0], from, to };
  }
  for (const match of line.text.matchAll(wordPattern)) {
    const from = line.from + match.index!, to = from + match[0].length;
    if (from <= position && position <= to) return { word: match[0], from, to };
  }
  return null;
}
function instructionBefore(state: EditorState, position: number, language: RobotLanguageDefinition): LanguageEntry | undefined {
  const start = Math.max(0, position - 4096), prefix = state.sliceDoc(start, position);
  const statementStart = prefix.lastIndexOf(';') + 1, statement = prefix.slice(statementStart);
  const matcher = catalogMatcher(language), words = catalogMatches(statement, language);
  for (let index = words.length - 1; index >= 0; index--) {
    const match = words[index], entry = matcher.entries.get(normalizeName(language, match[0]));
    if (entry && (entry.kind === 'instruction' || entry.kind === 'function') && !/comment|string/i.test(syntaxTree(state).resolveInner(start + statementStart + match.index!, 1).name)) return entry;
  }
  return undefined;
}
const clamp = (position: number, length: number) => Math.max(0, Math.min(length, Number.isFinite(position) ? position : 0));

export default function RobotCodeEditor(props: RobotCodeEditorProps) {
  const latest = useRef(props); latest.current = props;
  const host = useRef<HTMLDivElement>(null), editor = useRef<EditorView | null>(null);
  const states = useRef(new Map<string, EditorState>());
  const rawSources = useRef(new Map<string, string>());
  const activeId = useRef<string | null>(null), map = useRef<SourceMap>(sourceMap(''));
  const languageCompartment = useRef(new Compartment()), readOnlyCompartment = useRef(new Compartment());
  const externalChange = useRef(false), insertedRaw = useRef<string | null>(null);
  const sourceVersion = useRef(0);
  const replacementSnapshots = useRef<{ fileId: string; normalized: string; raw: string }[]>([]);
  const [cursor, setCursor] = useState({ line: 1, column: 1 });
  const [panel, setPanel] = useState<'diagnostics' | 'reference'>('diagnostics');
  const [collapsed, setCollapsed] = useState(false), [query, setQuery] = useState('');
  const [reference, setReference] = useState<string | null>(null);
  const [notice, setNotice] = useState(''), [inserting, setInserting] = useState(false);
  const [dialog, setDialog] = useState<{ kind: 'new' | 'rename' | 'delete'; name: string; fileId?: string; source?: string } | null>(null);
  const [menu, setMenu] = useState<{ x: number; y: number } | null>(null);
  const activeFile = props.files.find(file => file.id === props.activeFileId) ?? null;
  const pending = Boolean(activeFile && props.pendingFileIds?.includes(activeFile.id));
  const diagnostics = props.files.flatMap(file => (props.pendingFileIds?.includes(file.id) ? [] : props.analysisByFile?.[file.id]?.diagnostics ?? []).map(item => ({ ...item, file })));
  const entries = props.language.entries.filter(entry => !query.trim() || [entry.name, entry.signature, entry.description, entry.category].join(' ').toLowerCase().includes(query.trim().toLowerCase()));
  const referenceEntry = entries.find(entry => entry.name === reference) ?? entries[0];
  const readOnly = Boolean(props.readOnly || !activeFile);
  function rememberReplacement(fileId: string, raw: string) {
    const records = replacementSnapshots.current;
    if (records.some(record => record.fileId === fileId && record.raw === raw)) return;
    records.push({ fileId, normalized: sourceMap(raw).text, raw });
    let bytes = records.reduce((sum, record) => sum + 2 * (record.normalized.length + record.raw.length), 0);
    while (records.length > 32 || bytes > 16 * 1024 * 1024) { const first = records.shift()!; bytes -= 2 * (first.normalized.length + first.raw.length); }
  }

  function completions(context: CompletionContext) {
    if (/comment|string/i.test(syntaxTree(context.state).resolveInner(context.pos, -1).name)) return null;
    const current = latest.current;
    const prefix = current.language.id === 'inform' ? /\+?[$#]?[A-Za-z_\u0080-\uffff][\w$#\u0080-\uffff]*/
      : current.language.id === 'fanuc-ls' ? /[$#]?[A-Za-z_\u0080-\uffff][\w$#\u0080-\uffff-]*/ : /[$#]?[A-Za-z_\u0080-\uffff][\w$#\u0080-\uffff]*/;
    const matched = context.matchBefore(prefix);
    const parameterPrefix = context.matchBefore(/\\[A-Za-z_]*/);
    const phraseFrom = parameterPrefix ? undefined : phrasePrefix(context.state, context.pos, current.language);
    if (!matched && phraseFrom === undefined && !parameterPrefix && !context.explicit) return null;
    const source = rawSources.current.get(activeId.current ?? '') ?? '';
    const currentMap = sourceMap(source), rawPosition = currentMap.toRaw[context.pos] ?? source.length;
    const symbols = current.pendingFileIds?.includes(activeId.current ?? '') ? [] : current.analysisByFile?.[activeId.current ?? '']?.symbols ?? [];
    const options: Completion[] = (parameterPrefix ? [] : current.language.entries).map(entry => ({ label: entry.name, type: entry.kind === 'instruction' || entry.kind === 'routine' ? 'function' : entry.kind === 'type' ? 'class' : entry.kind,
      detail: entry.signature, info: () => entryDescription(entry), apply: entry.insertText ? snippet(entry.insertText) : entry.name }));
    const instruction = instructionBefore(context.state, parameterPrefix?.from ?? phraseFrom ?? matched?.from ?? context.pos, current.language);
    for (const parameter of instruction?.parameters ?? []) if (parameter.name.startsWith('\\')) options.push({ label: parameter.name, type: 'property', detail: parameter.type ?? '可选参数', info: parameter.description,
      apply: parameter.type?.toLowerCase() === 'switch' ? parameter.name : snippet(parameter.name + ':=${value}'), boost: 2 });
    const known = new Set(options.map(option => normalizeName(current.language, option.label)));
    for (const symbol of symbols) if (!parameterPrefix && !known.has(normalizeName(current.language, symbol.name)) && symbol.from <= rawPosition) {
      options.push({ label: symbol.name, type: symbol.kind === 'routine' ? 'function' : symbol.kind === 'type' ? 'class' : symbol.kind,
        detail: symbol.signature ?? '当前文件', info: symbol.description, boost: 1 });
      known.add(normalizeName(current.language, symbol.name));
    }
    const validFor = current.language.id === 'inform' ? /^[+\\$#]?[\w\u0080-\uffff]*$/ : current.language.id === 'fanuc-ls' ? /^[\\$#]?[\w\u0080-\uffff\t -]*$/ : /^[\\$#]?[\w\u0080-\uffff]*$/;
    return { from: parameterPrefix?.from ?? phraseFrom ?? matched?.from ?? context.pos, options, validFor };
  }
  function extensions(language: RobotLanguageDefinition) {
    return [StreamLanguage.define(lexer(language)), syntaxHighlighting(highlighting),
      EditorView.contentAttributes.of({ 'aria-label': language.name + ' 代码编辑器', 'aria-multiline': 'true', spellcheck: 'false' }),
      autocompletion({ override: [completions], maxRenderedOptions: 80 }),
      hoverTooltip((view, position, side) => {
        if (/comment|string/i.test(syntaxTree(view.state).resolveInner(position, side).name)) return null;
        const word = wordAt(view, position, latest.current.language);
        if (!word || position === word.from && side < 0 || position === word.to && side > 0) return null;
        const current = latest.current;
        const entry = current.language.entries.find(item => normalizeName(current.language, item.name) === normalizeName(current.language, word.word));
        const symbol = current.pendingFileIds?.includes(activeId.current ?? '') ? undefined : current.analysisByFile?.[activeId.current ?? '']?.symbols.find(item => normalizeName(current.language, item.name) === normalizeName(current.language, word.word));
        const instruction = word.from > 0 && view.state.sliceDoc(word.from - 1, word.from) === '\\' ? instructionBefore(view.state, word.from - 1, current.language) : undefined;
        const parameter = instruction?.parameters?.find(item => normalizeName(current.language, item.name.replace(/^\\/, '')) === normalizeName(current.language, word.word));
        if (!entry && !symbol && !parameter) return null;
        const help: LanguageEntry = parameter ? { name: instruction!.name + ' · ' + parameter.name, kind: 'variable', signature: parameter.name + (parameter.type ? ' : ' + parameter.type : ''), description: parameter.description, parameters: [parameter] } : entry ?? { name: symbol!.name, kind: symbol!.kind, signature: symbol!.signature ?? symbol!.name, description: symbol!.description ?? '当前文件中声明的符号。' };
        return { pos: word.from, end: word.to, above: true, create: () => ({ dom: entryDescription(help) }) };
      }),
    ];
  }
  function makeState(text: string) {
    return EditorState.create({ doc: sourceMap(text).text, extensions: [
      lineNumbers(), highlightActiveLineGutter(), highlightSpecialChars(), history(), foldGutter({ foldingChanged: update => update.transactions.some(transaction => transaction.effects.some(effect => effect.is(foldedAnalysisChanged))) }),
      drawSelection(), dropCursor(), indentOnInput(), bracketMatching(), closeBrackets(), rectangularSelection(), crosshairCursor(), highlightActiveLine(), highlightSelectionMatches(),
      search({ top: true }), lintGutter(), indentUnit.of('  '),
      keymap.of([...closeBracketsKeymap, ...completionKeymap, ...searchKeymap, ...historyKeymap, ...foldKeymap, ...lintKeymap, ...defaultKeymap, indentWithTab]),
      foldService.of((_state, from, to) => {
        if (latest.current.pendingFileIds?.includes(activeId.current ?? '')) return null;
        const analysis = latest.current.analysisByFile?.[activeId.current ?? ''];
        const candidate = analysis?.folds.find(fold => { const start = map.current.toView[clamp(fold.from, map.current.toView.length - 1)]; return start >= from && start <= to; });
        if (!candidate) return null;
        const start = map.current.toView[clamp(candidate.from, map.current.toView.length - 1)], end = map.current.toView[clamp(candidate.to, map.current.toView.length - 1)];
        return end > start ? { from: start, to: end } : null;
      }),
      languageCompartment.current.of(extensions(latest.current.language)),
      readOnlyCompartment.current.of([EditorState.readOnly.of(Boolean(latest.current.readOnly)), EditorView.editable.of(!latest.current.readOnly)]),
      EditorView.updateListener.of(update => {
        const id = activeId.current;
        let acceptedState = update.state;
        if (update.docChanged && id && !externalChange.current) {
          setNotice('');
          const original = rawSources.current.get(id) ?? '', previous = map.current;
          let text = '', previousRaw = 0;
          update.changes.iterChanges((from, to, _nextFrom, _nextTo, inserted) => {
            text += original.slice(previousRaw, previous.toRaw[from]) + inserted.toString().replace(/\n/g, previous.separator);
            previousRaw = previous.toRaw[to];
          });
          text += original.slice(previousRaw);
          if (insertedRaw.current !== null) { text = insertedRaw.current; insertedRaw.current = null; }
          if (update.transactions.some(transaction => transaction.isUserEvent('undo') || transaction.isUserEvent('redo'))) {
            const normalized = update.state.doc.toString();
            const originalReplacement = [...replacementSnapshots.current].reverse().find(record => record.fileId === id && record.normalized === normalized);
            if (originalReplacement) text = originalReplacement.raw;
          }
          rawSources.current.set(id, text); map.current = sourceMap(text); sourceVersion.current++;
          let accepted = true, rejection = '当前修改未被保存，请检查文件大小或输入字符。';
          try { accepted = latest.current.onTextChange(id, text) !== false; }
          catch (error) { accepted = false; rejection = error instanceof Error ? error.message : rejection; }
          if (!accepted) {
            acceptedState = update.startState;
            rawSources.current.set(id, original); map.current = previous;
            const rejectedVersion = sourceVersion.current;
            queueMicrotask(() => {
              if (editor.current !== update.view || activeId.current !== id || sourceVersion.current !== rejectedVersion) return;
              externalChange.current = true; update.view.setState(update.startState); externalChange.current = false;
              const position = update.startState.selection.main.head, line = update.startState.doc.lineAt(position);
              setCursor({ line: line.number, column: position - line.from + 1 }); setNotice(rejection);
            });
          }
        }
        if (id) states.current.set(id, acceptedState);
        if (update.docChanged || update.selectionSet) {
          const position = update.state.selection.main.head, line = update.state.doc.lineAt(position);
          setCursor({ line: line.number, column: position - line.from + 1 });
        }
      }),
    ] });
  }

  useLayoutEffect(() => {
    if (!host.current) return;
    const view = new EditorView({ state: makeState(''), parent: host.current }); editor.current = view;
    return () => { sourceVersion.current++; editor.current = null; view.destroy(); states.current.clear(); rawSources.current.clear(); replacementSnapshots.current = []; };
  }, []);
  useLayoutEffect(() => {
    const view = editor.current; if (!view) return;
    const file = props.files.find(item => item.id === props.activeFileId);
    for (const id of states.current.keys()) if (!props.files.some(item => item.id === id)) { states.current.delete(id); rawSources.current.delete(id); }
    const switched = activeId.current !== (file?.id ?? null);
    if (switched) {
      if (activeId.current && props.files.some(item => item.id === activeId.current)) states.current.set(activeId.current, view.state);
      activeId.current = file?.id ?? null; sourceVersion.current++;
      const text = file?.text ?? ''; rawSources.current.set(file?.id ?? '', text); map.current = sourceMap(text);
      externalChange.current = true;
      view.setState(file && states.current.get(file.id) || makeState(text));
      externalChange.current = false; setNotice(''); setMenu(null);
      view.dispatch({ effects: [languageCompartment.current.reconfigure(extensions(props.language)), readOnlyCompartment.current.reconfigure([EditorState.readOnly.of(readOnly), EditorView.editable.of(!readOnly)])] });
    }
    const text = file?.text ?? '';
    if (switched || rawSources.current.get(file?.id ?? '') !== text) {
      const mapped = sourceMap(text);
      rawSources.current.set(file?.id ?? '', text); map.current = mapped; sourceVersion.current++;
      if (view.state.doc.toString() !== mapped.text) {
        externalChange.current = true;
        view.dispatch({ changes: { from: 0, to: view.state.doc.length, insert: mapped.text }, selection: EditorSelection.cursor(Math.min(view.state.selection.main.head, mapped.text.length)), annotations: [Transaction.addToHistory.of(false), isolateHistory.of('full')] });
        externalChange.current = false;
      }
    }
    const position = view.state.selection.main.head, line = view.state.doc.lineAt(position); setCursor({ line: line.number, column: position - line.from + 1 });
  }, [props.activeFileId, props.files]);
  useEffect(() => { sourceVersion.current++; editor.current?.dispatch({ effects: languageCompartment.current.reconfigure(extensions(props.language)) }); }, [props.language]);
  useEffect(() => { editor.current?.dispatch({ effects: readOnlyCompartment.current.reconfigure([EditorState.readOnly.of(readOnly), EditorView.editable.of(!readOnly)]) }); }, [readOnly]);
  useEffect(() => {
    const view = editor.current; if (!view) return;
    const values = pending ? [] : props.analysisByFile?.[props.activeFileId ?? '']?.diagnostics ?? [];
    view.dispatch(setDiagnostics(view.state, values.map(item => { const from = map.current.toView[clamp(item.from, map.current.toView.length - 1)], to = map.current.toView[clamp(item.to, map.current.toView.length - 1)]; return { from, to: Math.max(from, to), severity: item.severity, message: item.message }; })));
    view.dispatch({ effects: foldedAnalysisChanged.of(null) });
  }, [props.activeFileId, props.analysisByFile, pending, props.files]);
  useEffect(() => { if (props.active) editor.current?.requestMeasure(); else setMenu(null); }, [props.active]);
  useEffect(() => {
    if (!menu) return;
    const close = () => setMenu(null), key = (event: KeyboardEvent) => { if (event.key === 'Escape') close(); };
    window.addEventListener('pointerdown', close); window.addEventListener('keydown', key); window.addEventListener('resize', close);
    return () => { window.removeEventListener('pointerdown', close); window.removeEventListener('keydown', key); window.removeEventListener('resize', close); };
  }, [menu]);

  function locate(file: ProgramSourceFile, from: number, to: number) {
    props.onActiveFileChange(file.id);
    requestAnimationFrame(() => {
      const view = editor.current; if (!view || activeId.current !== file.id) return;
      const mapped = map.current, start = mapped.toView[clamp(from, file.text.length)], end = mapped.toView[clamp(to, file.text.length)];
      view.dispatch({ selection: EditorSelection.range(start, Math.max(start, end)), effects: EditorView.scrollIntoView(start, { y: 'center' }) }); view.focus();
    });
  }
  function insertReference(entry: LanguageEntry) {
    const view = editor.current; if (!view || readOnly) return;
    const range = view.state.selection.main;
    snippet(entry.insertText ?? entry.name)(view, { label: entry.name }, range.from, range.to); view.focus();
  }
  async function insertCurrent() {
    const view = editor.current, fileId = activeId.current;
    if (!view || !fileId || !props.onInsertCurrent || readOnly || inserting || props.captureDisabledReason) return;
    const source = rawSources.current.get(fileId) ?? '', selection = view.state.selection.main, originalMap = map.current;
    const version = sourceVersion.current; setInserting(true); setMenu(null);
    try {
      const result = await props.onInsertCurrent({ fileId, source, from: originalMap.toRaw[selection.from], to: originalMap.toRaw[selection.to] });
      if (editor.current !== view || activeId.current !== fileId || sourceVersion.current !== version || rawSources.current.get(fileId) !== source || latest.current.readOnly || latest.current.captureDisabledReason) { setNotice('当前文件或机器人状态已改变，请重新插入当前姿态。'); return; }
      const changes = [...result.changes].sort((a, b) => a.from - b.from || a.to - b.to);
      let previous = 0, next = '';
      for (const change of changes) {
        if (!Number.isInteger(change.from) || !Number.isInteger(change.to) || change.from < previous || change.to < change.from || change.to > source.length || typeof change.insert !== 'string') throw new Error('姿态插入返回了无效的源码范围');
        next += source.slice(previous, change.from) + change.insert; previous = change.to;
      }
      next += source.slice(previous); const mapped = sourceMap(next);
      rememberReplacement(fileId, source); rememberReplacement(fileId, next);
      insertedRaw.current = next;
      view.dispatch({ changes: changes.map(change => ({ from: originalMap.toView[change.from], to: originalMap.toView[change.to], insert: sourceMap(change.insert).text })),
        ...(result.cursor === undefined ? {} : { selection: EditorSelection.cursor(mapped.toView[clamp(result.cursor, next.length)]) }), annotations: isolateHistory.of('full'), userEvent: 'input.pose' });
      insertedRaw.current = null; setNotice(result.message || '已在源码中插入当前姿态。'); view.focus();
    } catch (error) { insertedRaw.current = null; setNotice(error instanceof Error ? error.message : '姿态插入失败'); }
    finally { setInserting(false); }
  }
  function confirmDialog() {
    if (!dialog || props.readOnly) return;
    if (dialog.kind !== 'new' && (activeFile?.id !== dialog.fileId || activeFile?.text !== dialog.source)) { setNotice('文件已改变，请重新确认文件操作。'); setDialog(null); return; }
    const name = dialog.name.trim();
    if ((dialog.kind === 'new' || dialog.kind === 'rename') && !name) return;
    if (dialog.kind === 'new') props.onNewFile?.(name);
    else if (dialog.kind === 'rename' && activeFile) props.onRenameFile?.(activeFile.id, name);
    else if (dialog.kind === 'delete' && activeFile) props.onRemoveFile?.(activeFile.id);
    setDialog(null);
  }
  const command = (run: (view: EditorView) => unknown) => { const view = editor.current; if (view) { run(view); view.focus(); } };
  async function copyDocumentation(url: string) {
    try { await copyRapid(url); setNotice('官方链接已复制。'); }
    catch { setNotice('复制链接失败，可手动复制：' + url); }
  }
  return <section className="robot-code-workspace" aria-label="机器人源码工作区" data-language-id={props.language.id} data-language-dialect={props.language.dialect}>
    <div className="robot-code-filebar"><div className="robot-code-current-file" aria-label="当前模块" data-source-file-id={activeFile?.id} data-source-dirty={Boolean(activeFile?.dirty)} title={activeFile?.name}>
      <span>{activeFile?.name ?? '未选择模块'}</span>{activeFile?.dirty && <i aria-label="已修改">●</i>}
    </div></div>
    <div className="robot-code-tools">
      {props.onNewFile && <button type="button" disabled={props.readOnly} onClick={() => setDialog({ kind: 'new', name: 'Program' + (props.language.extensions[0]?.startsWith('.') ? '' : '.') + (props.language.extensions[0] ?? 'txt') })}>新建文件</button>}
      {props.onImportFiles && <button type="button" disabled={props.readOnly} onClick={props.onImportFiles}>导入源码</button>}
      {props.onSave && <button type="button" disabled={props.readOnly} onClick={props.onSave}>保存草稿</button>}
      {props.onExport && <button type="button" disabled={readOnly} onClick={props.onExport}>导出源码</button>}
      {props.onRenameFile && <button type="button" disabled={readOnly} onClick={() => setDialog({ kind: 'rename', name: activeFile?.name ?? '', fileId: activeFile?.id, source: activeFile?.text })}>重命名</button>}
      {props.onRemoveFile && <button type="button" disabled={readOnly} onClick={() => setDialog({ kind: 'delete', name: activeFile?.name ?? '', fileId: activeFile?.id, source: activeFile?.text })}>删除文件</button>}
      {props.onInsertCurrent && <button type="button" disabled={readOnly || inserting || Boolean(props.captureDisabledReason)} title={props.captureDisabledReason ?? ''} onClick={() => void insertCurrent()}>插入当前姿态</button>}
      <button type="button" disabled={!activeFile} onClick={() => command(openSearchPanel)}>查找</button>
      <button type="button" disabled={readOnly} onClick={() => command(undo)}>撤销</button>
      <button type="button" disabled={readOnly} onClick={() => command(redo)}>重做</button>
      <button type="button" disabled={readOnly} onClick={() => command(startCompletion)}>补全</button>
      {props.toolbar}<small>{props.language.dialect}</small>
    </div>
    <div className="robot-code-editor-area">
      <div className="robot-code-surface" ref={host} onContextMenu={event => { if (!activeFile || !props.onInsertCurrent) return; event.preventDefault(); setMenu({ x: Math.max(8, Math.min(event.clientX, window.innerWidth - 210)), y: Math.max(8, Math.min(event.clientY, window.innerHeight - 55)) }); }}/>
      {!activeFile && <div className="robot-code-no-file"><strong>{props.files.length ? '从左侧选择一个模块' : '还没有模块文件'}</strong><p>在场景中将 Path 转换成程序，或使用上方按钮新建、导入模块。</p></div>}
    </div>
    <section className={'robot-code-bottom' + (collapsed ? ' is-collapsed' : '')} aria-label="源码辅助面板">
      <div className="robot-code-panel-tabs" role="tablist" aria-label="源码辅助视图"><button type="button" role="tab" aria-selected={panel === 'diagnostics'} onClick={() => { setPanel('diagnostics'); setCollapsed(false); }}>问题 {diagnostics.length}</button><button type="button" role="tab" aria-selected={panel === 'reference'} onClick={() => { setPanel('reference'); setCollapsed(false); }}>指令参考</button><button className="robot-code-panel-collapse" type="button" aria-label={collapsed ? '展开源码辅助面板' : '折叠源码辅助面板'} onClick={() => setCollapsed(value => !value)}>{collapsed ? '展开' : '收起'}</button></div>
      {!collapsed && <div className="robot-code-panel-body">{panel === 'diagnostics' ? <>
        <ul className="robot-code-diagnostics">{diagnostics.map((item, index) => { const prefix = item.file.text.slice(0, item.from), line = prefix.split(/\r\n|\r|\n/).length, column = prefix.length - Math.max(prefix.lastIndexOf('\n'), prefix.lastIndexOf('\r')); return <li key={item.file.id + ':' + index}><button type="button" onClick={() => locate(item.file, item.from, item.to)}><span className={'diagnostic-severity severity-' + item.severity}>{item.severity === 'error' ? '错误' : item.severity === 'warning' ? '警告' : '信息'}</span><span className="diagnostic-location">{item.file.name}:{line}:{column}</span><span>{item.message}</span></button></li>; })}</ul>
        {!diagnostics.length && <p className="robot-code-empty" role={props.analysisError ? 'alert' : undefined}>{props.analysisError ? '源码检查失败：' + props.analysisError : pending ? '正在检查源码…' : '当前检查未报告问题。控制器选项和现场配置仍需在对应控制器确认。'}</p>}
        {props.analysisError && props.onRetryAnalysis && <button type="button" className="text-button" onClick={props.onRetryAnalysis}>重试源码检查</button>}
        <button type="button" className="text-button" disabled={!diagnostics.length} onClick={() => command(nextDiagnostic)}>定位下一个问题</button>
      </> : <>
        <details className="robot-code-language-notes"><summary>{props.language.dialect} 语言说明</summary><ul>{props.language.coverage.map((item, index) => <li key={'coverage-' + index}>{item}</li>)}{props.language.limitations.map((item, index) => <li key={'limit-' + index}>{item}</li>)}</ul>{props.language.sources.map((source, index) => <span className="robot-code-source-link" key={index}>{nativeDesktop ? <><span>{source.title}{source.revision ? ' · ' + source.revision : ''}</span><button type="button" title={source.url} onClick={() => void copyDocumentation(source.url)}>复制官方链接</button></> : <a href={source.url} target="_blank" rel="noreferrer">{source.title}{source.revision ? ' · ' + source.revision : ''}</a>}</span>)}</details>
        <input className="robot-code-reference-search" aria-label="查找指令参考" placeholder="查找指令、参数或说明" value={query} onChange={event => setQuery(event.target.value)}/>
        <div className="robot-code-reference-layout"><ul className="robot-code-reference-list" role="listbox" aria-label="指令参考列表">{entries.map(entry => <li key={entry.name}><button type="button" role="option" aria-selected={referenceEntry?.name === entry.name} onClick={() => setReference(entry.name)}>{entry.name}</button></li>)}</ul>
          {referenceEntry ? <article className="robot-code-reference-detail"><h3>{referenceEntry.name}</h3><pre>{referenceEntry.signature}</pre><p>{referenceEntry.description}</p>{referenceEntry.parameters?.length ? <dl>{referenceEntry.parameters.map((parameter, index) => <div key={index}><dt>{parameter.name}{parameter.type ? ' : ' + parameter.type : ''}{parameter.optional ? '（可选）' : ''}</dt><dd>{parameter.description}</dd></div>)}</dl> : null}{referenceEntry.requiresOption && <p>控制器选项：{referenceEntry.requiresOption}</p>}<div className="robot-code-reference-actions"><button type="button" disabled={readOnly} onClick={() => insertReference(referenceEntry)}>插入指令</button>{referenceEntry.documentationUrl && (nativeDesktop ? <button type="button" title={referenceEntry.documentationUrl} onClick={() => void copyDocumentation(referenceEntry.documentationUrl!)}>复制官方链接</button> : <a href={referenceEntry.documentationUrl} target="_blank" rel="noreferrer">官方说明</a>)}</div></article> : <p className="robot-code-empty">没有匹配的指令。</p>}
        </div>
      </>}</div>}
    </section>
    <footer className="robot-code-statusbar"><span role="status" aria-label="源码状态">{notice && notice + ' · '}{props.status || (pending ? '正在检查源码…' : props.dirty ? '源码已修改' : '草稿就绪')}</span><code>{props.language.name} · {cursor.line}:{cursor.column}</code></footer>
    {dialog && <div className="robot-code-dialog-backdrop"><form className="robot-code-dialog" role="dialog" aria-modal="true" aria-label={dialog.kind === 'new' ? '新建源码文件' : dialog.kind === 'rename' ? '重命名源码文件' : '删除源码文件'} onKeyDown={event => { if (event.key === 'Escape') { event.preventDefault(); setDialog(null); } if (event.key === 'Tab') { const elements = [...event.currentTarget.querySelectorAll<HTMLElement>('input:not(:disabled),button:not(:disabled)')]; const first = elements[0], last = elements.at(-1); if (event.shiftKey && document.activeElement === first) { event.preventDefault(); last?.focus(); } else if (!event.shiftKey && document.activeElement === last) { event.preventDefault(); first?.focus(); } } }} onSubmit={event => { event.preventDefault(); confirmDialog(); }}><h3>{dialog.kind === 'new' ? '新建源码文件' : dialog.kind === 'rename' ? '重命名源码文件' : '删除源码文件'}</h3>{dialog.kind === 'new' || dialog.kind === 'rename' ? <label>文件名称<input autoFocus aria-label="源码文件名称" value={dialog.name} onChange={event => setDialog({ ...dialog, name: event.target.value })}/></label> : <p>将删除草稿文件：<strong>{dialog.name}</strong></p>}<div><button autoFocus={dialog.kind === 'delete'} type="button" onClick={() => setDialog(null)}>取消</button><button type="submit" disabled={Boolean(props.readOnly || (dialog.kind === 'new' || dialog.kind === 'rename') && !dialog.name.trim())}>{dialog.kind === 'delete' ? '删除' : '确定'}</button></div></form></div>}
    {menu && <div className="robot-code-context-menu" role="menu" aria-label="源码右键菜单" style={{ left: menu.x, top: menu.y }} onPointerDown={event => event.stopPropagation()}><button autoFocus type="button" role="menuitem" disabled={readOnly || inserting || Boolean(props.captureDisabledReason)} onClick={() => void insertCurrent()}>插入当前姿态</button></div>}
  </section>;
}
