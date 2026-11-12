import type { LanguageAnalysis, SourceDiagnostic } from '../types';
import { templateParameterReporter } from '../template-parameter-diagnostics';
import { parseFanucApplications, type FanucApplication } from './fanuc-application';
import { parseFanucOptionalStatement, parseFanucOptionalMotionOption, parseFanucOptionalMotionSpeed } from './fanuc-optional';
import { validateFanucAttribute } from './fanuc-attributes';

/** Independent, recovering LS structural parser. Source text is never rewritten or discarded. */
export interface FanucToken { text: string; upper: string; from: number; to: number; line: number; kind: 'word' | 'number' | 'string' | 'punctuation' | 'section' | 'comment' }
export interface FanucStatement { from: number; to: number; kind: string; tokens: readonly FanucToken[] }
export interface FanucGroup { number: number; from: number; to: number; representation: 'cartesian' | 'joint' | 'unknown'; axes: Readonly<Record<string, { value: number | null; unit: string }>>; unresolvedFrames?: readonly ('UF' | 'UT')[] }
export interface FanucPosition { index: number; from: number; to: number; groups: FanucGroup[] }
export interface FanucDocument extends LanguageAnalysis { source: string; statements: FanucStatement[]; positions: FanucPosition[]; applications: FanucApplication[] }
export interface FanucParseOptions { /** Recognize only generated /POS UF/UT placeholders, never controller-ready values. */ templateParameters?: boolean }
const MAX_SOURCE_BYTES = 1_048_576, MAX_TOKENS = 100_000, MAX_DIAGNOSTICS = 500;
const REGISTERS = new Set(['R', 'PR', 'SR', 'AR', 'P', 'LBL', 'DI', 'DO', 'RI', 'RO', 'GI', 'GO', 'AI', 'AO', 'UI', 'UO', 'SI', 'SO', 'WI', 'WO', 'WSI', 'WSO', 'F', 'M', 'TIMER', 'TIMER_OVERFLOW', 'TCP_SPD', 'RSR', 'PL', 'JOINT_MAX_SPEED', 'UFRAME', 'UTOOL', 'PAYLOAD', 'UALM']);
const FUNCTIONS = new Set(['SIN', 'COS', 'TAN', 'ASIN', 'ACOS', 'ATAN', 'ATAN2', 'SQRT', 'ABS', 'TRUNC', 'ROUND', 'LN', 'EXP']);
const VALUES = new Set(['ON', 'OFF', 'TRUE', 'FALSE', 'LPOS', 'JPOS', 'START', 'STOP', 'RESET', 'PULSE', 'UFRAME_NUM', 'UTOOL_NUM', 'OVERRIDE', 'ERROR_PROG', 'RESUME_PROG', 'MAINT_PROG', 'ERR_PROG', 'ERR_NUM', 'LINEAR_MAX_SPEED', 'ENABLE', 'DISABLE']);
const validProgramName = (name: string) => /^[A-Za-z_][A-Za-z0-9_]{0,35}$/.test(name) && !/^(CON|PRN|AUX|NUL|COM[1-9]|LPT[1-9])$/i.test(name);
const SECTIONS = new Set(['/PROG', '/ATTR', '/APPL', '/MN', '/POS', '/END']);
const BINARY: Record<string, number> = { OR: 1, AND: 3, '=': 4, '<>': 4, '<': 5, '>': 5, '<=': 5, '>=': 5, '+': 6, '-': 6, '*': 7, '/': 7, DIV: 7, MOD: 7 };
export class SyntaxFault extends Error { constructor(public readonly token: FanucToken | undefined, message: string) { super(message); } }

function lex(source: string, diagnostic: (token: FanucToken | undefined, message: string, code?: string, severity?: SourceDiagnostic['severity']) => void): FanucToken[] {
  const tokens: FanucToken[] = [];
  let at = 0, line = 1, lineStart = true, squareDepth = 0;
  const push = (from: number, kind: FanucToken['kind'], tokenLine = line) => { const text = source.slice(from, at); tokens.push({ text, upper: text.toUpperCase(), from, to: at, line: tokenLine, kind }); };
  while (at < source.length && tokens.length < MAX_TOKENS) {
    const c = source[at];
    if (/\s/.test(c)) { if (c === '\n') { line++; lineStart = true; } at++; continue; }
    const from = at, tokenLine = line;
    if (lineStart && c === '/' && /^\/[A-Za-z]+\b/.test(source.slice(at, at + 30))) {
      at += /^\/[A-Za-z]+/.exec(source.slice(at, at + 30))![0].length; push(from, 'section');
    } else if (squareDepth === 0 && (lineStart || [':', ';'].includes(tokens.at(-1)?.upper ?? '')) && (c === '!' || source.startsWith('//', at) || source.startsWith('--', at))) {
      while (at < source.length && source[at] !== ';') { if (source[at] === '\n') line++; at++; } push(from, 'comment', tokenLine);
    } else if (c === '"' || c === "'") {
      const quote = c; at++; let closed = false;
      while (at < source.length) {
        if (source[at] === '\n') line++;
        if (source[at] === quote) { at++; closed = true; break; }
        at++;
      }
      push(from, 'string', tokenLine);
      if (!closed) diagnostic(tokens.at(-1), '字符串缺少结束引号', 'fanuc.unclosed-string');
    } else if (/[A-Za-z_$]/.test(c)) {
      at++; while (at < source.length && /[A-Za-z0-9_$]/.test(source[at])) at++; push(from, 'word');
    } else if (/\d/.test(c) || c === '.' && /\d/.test(source[at + 1] ?? '')) {
      at += /^(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?/.exec(source.slice(at))![0].length; push(from, 'number');
    } else {
      at += ['<=', '>=', '<>', '!='].includes(source.slice(at, at + 2)) ? 2 : 1; push(from, 'punctuation');
      if (c === '[') squareDepth++;
      else if (c === ']') squareDepth = Math.max(0, squareDepth - 1);
    }
    lineStart = false;
  }
  if (at < source.length) diagnostic(tokens.at(-1), '文件 token 数量超过 100000，后续结构尚未检查', 'fanuc.limit');
  return tokens;
}

export class Cursor {
  index = 0;
  constructor(readonly tokens: readonly FanucToken[]) {}
  get token() { return this.tokens[this.index]; }
  get done() { return this.index >= this.tokens.length; }
  is(text: string) { return this.token?.upper === text; }
  take() { const value = this.token; if (!value) throw new SyntaxFault(this.tokens.at(-1), '指令参数不完整'); this.index++; return value; }
  eat(text: string) { if (this.is(text)) { this.index++; return true; } return false; }
  expect(text: string) { if (!this.eat(text)) throw new SyntaxFault(this.token ?? this.tokens.at(-1), `此处需要 ${text}`); }
  end() { if (!this.done) throw new SyntaxFault(this.token, '指令包含多余或无法识别的参数'); }
}

interface Reference { name: string; index: number | null; component: boolean; from: number; to: number }
function systemReference(cursor: Cursor, depth = 0): void {
  if (depth > 64) throw new SyntaxFault(cursor.token, '系统变量引用嵌套超过 64 层');
  const first = cursor.take();
  if (!first.text.startsWith('$')) throw new SyntaxFault(first, '系统变量名称需要 $ 前缀');
  while (cursor.is('.') || cursor.is('[')) {
    if (cursor.eat('.')) { const field = cursor.take(); if (field.kind !== 'word') throw new SyntaxFault(field, '系统变量字段名称无效'); }
    else {
      cursor.expect('['); expression(cursor, 0, depth + 1);
      let dimensions = 1;
      while (cursor.eat(',')) { if (++dimensions > 8) throw new SyntaxFault(cursor.token, '系统变量索引超过 8 维限制'); expression(cursor, 0, depth + 1); }
      cursor.expect(']');
    }
  }
}

type FanucValueKind = 'number' | 'boolean' | 'string' | 'position' | 'pallet' | 'unknown';
interface ExpressionInfo { kind: FanucValueKind; mixed: boolean; edge: boolean; errorNumber: boolean }
const valueInfo = (kind: FanucValueKind, mixed = false, edge = false, errorNumber = false): ExpressionInfo => ({ kind, mixed, edge, errorNumber });
const referenceKind = (ref: Reference): FanucValueKind => {
  if (['AR'].includes(ref.name)) return 'unknown';
  if (['P', 'UFRAME', 'UTOOL'].includes(ref.name) || ref.name === 'PR' && !ref.component) return 'position';
  if (ref.name === 'PL') return 'pallet';
  if (ref.name === 'SR') return 'string';
  if (['DI', 'DO', 'RI', 'RO', 'UI', 'UO', 'SI', 'SO', 'WI', 'WO', 'WSI', 'WSO', 'F', 'M', 'TIMER_OVERFLOW'].includes(ref.name)) return 'boolean';
  return 'number';
};
const expectKind = (info: ExpressionInfo, allowed: readonly FanucValueKind[], token: FanucToken | undefined, message: string) => {
  if (info.kind !== 'unknown' && !allowed.includes(info.kind)) throw new SyntaxFault(token, message);
};
function expression(cursor: Cursor, min = 0, depth = 0, mixed = false, palletCondition = false): ExpressionInfo {
  if (depth > 64) throw new SyntaxFault(cursor.token, '表达式嵌套超过 64 层');
  const token = cursor.take(); let result: ExpressionInfo;
  if (['+', '-', '!'].includes(token.upper)) {
    result = expression(cursor, 8, depth + 1, mixed, palletCondition);
    expectKind(result, token.upper === '!' ? ['boolean'] : mixed ? ['number'] : ['number', 'boolean', 'string'], token, '一元操作符与操作数类型不匹配');
    result = { ...result, kind: token.upper === '!' ? 'boolean' : 'number' };
  } else if (token.upper === '(') {
    result = expression(cursor, 0, depth + 1, true, palletCondition); cursor.expect(')');
    expectKind(result, ['number', 'boolean'], token, '混合逻辑表达式不能使用完整位置、码垛或字符串数据');
    if (result.edge || result.errorNumber) throw new SyntaxFault(token, 'ON+/OFF-/ERR_NUM 只能用于普通条件，不能放入混合逻辑');
    result = { ...result, mixed: true };
  } else if (token.kind === 'number') {
    if (!Number.isFinite(Number(token.text))) throw new SyntaxFault(token, '数值必须有限'); result = valueInfo('number', mixed);
  } else if (token.kind === 'string') result = valueInfo('string', mixed);
  else if (token.upper === '[') {
    for (let item = 0; item < 3; item++) {
      if (!cursor.eat('*')) {
        if (cursor.is('R')) reference(cursor, depth + 1);
        else {
          if (cursor.is('AR')) throw new SyntaxFault(cursor.token, '码垛分量间接形式需要 R[]');
          scalar(cursor, depth + 1, true, 1, 127);
          if (cursor.eat('-')) {
            if (!palletCondition || mixed || REGISTERS.has(cursor.token?.upper ?? '')) throw new SyntaxFault(cursor.token, 'a-b 余数模式只用于普通码垛比较，需要固定整数');
            scalar(cursor, depth + 1, true, 0, 127);
          }
        }
      }
      if (item < 2) cursor.expect(',');
    }
    cursor.expect(']'); result = valueInfo('pallet', mixed);
  } else if (token.kind === 'word') {
    if (REGISTERS.has(token.upper) || FUNCTIONS.has(token.upper)) { cursor.index--; result = valueInfo(referenceKind(reference(cursor, depth + 1)), mixed); }
    else if (token.text.startsWith('$')) { cursor.index--; systemReference(cursor, depth + 1); result = valueInfo('unknown', mixed); }
    else if (['ON', 'OFF'].includes(token.upper)) {
      const edge = token.upper === 'ON' && cursor.eat('+') || token.upper === 'OFF' && cursor.eat('-'); result = valueInfo('boolean', mixed, edge);
    } else if (['LPOS', 'JPOS', 'UFRAME_NUM', 'UTOOL_NUM', 'OVERRIDE', 'LINEAR_MAX_SPEED', 'ERR_NUM'].includes(token.upper)) {
      result = valueInfo(['LPOS', 'JPOS'].includes(token.upper) ? 'position' : 'number', mixed, false, token.upper === 'ERR_NUM');
      if (cursor.eat('[')) { if (!/^GP[1-8]$/.test(cursor.take().upper)) throw new SyntaxFault(cursor.token, '组选择器需要 GP1 至 GP8'); cursor.expect(']'); }
    } else throw new SyntaxFault(token, `未识别的 TP 表达式操作数 ${token.text}`);
  } else throw new SyntaxFault(token, '此处需要数值、寄存器或表达式');
  if (mixed && (['position', 'pallet', 'string'].includes(result.kind) || result.edge || result.errorNumber)) throw new SyntaxFault(token, '混合逻辑不接受此数据种类；位置／码垛／字符串和边沿条件需普通指令');
  while (!cursor.done && BINARY[cursor.token.upper] !== undefined && BINARY[cursor.token.upper] >= min) {
    const operator = cursor.take(), rhs = expression(cursor, BINARY[operator.upper] + 1, depth + 1, mixed, palletCondition);
    const combined = { mixed: result.mixed || rhs.mixed, edge: result.edge || rhs.edge, errorNumber: result.errorNumber || rhs.errorNumber };
    if (['AND', 'OR', 'XOR'].includes(operator.upper)) {
      expectKind(result, ['boolean'], operator, '逻辑操作符左侧必须为布尔数据'); expectKind(rhs, ['boolean'], operator, '逻辑操作符右侧必须为布尔数据');
      if (mixed && (combined.edge || combined.errorNumber)) throw new SyntaxFault(operator, '边沿或 ERR_NUM 条件不能参与混合逻辑运算');
      result = { ...combined, kind: 'boolean' };
    } else if (['=', '<>', '<', '>', '<=', '>='].includes(operator.upper)) {
      if (result.kind === 'position' || rhs.kind === 'position') throw new SyntaxFault(operator, '完整位置不能作为 TP 比较操作数');
      if (result.kind === 'pallet' || rhs.kind === 'pallet') {
        if (!['=', '<>'].includes(operator.upper) || result.kind !== 'pallet' || rhs.kind !== 'pallet') throw new SyntaxFault(operator, '码垛数据只支持同类型 = 或 <> 比较');
      } else if (mixed && !['=', '<>'].includes(operator.upper)) {
        expectKind(result, ['number'], operator, '混合逻辑大小比较需要数值'); expectKind(rhs, ['number'], operator, '混合逻辑大小比较需要数值');
      }
      result = { ...combined, kind: 'boolean' };
    } else if (['position', 'pallet'].includes(result.kind) || ['position', 'pallet'].includes(rhs.kind)) {
      if (!['+', '-'].includes(operator.upper) || result.kind !== rhs.kind) throw new SyntaxFault(operator, '位置／码垛算术只接受同类数据的加减');
      result = { ...combined, kind: result.kind };
    } else {
      if (mixed) { expectKind(result, ['number'], operator, '混合逻辑算术需要数值'); expectKind(rhs, ['number'], operator, '混合逻辑算术需要数值'); }
      // Ordinary string concatenation/conversion depends on the left operand (§4.5.6).
      if (result.kind === 'string' && operator.upper !== '+') throw new SyntaxFault(operator, '字符串运算只支持 + 连接');
      result = { ...combined, kind: result.kind === 'string' ? 'string' : 'number' };
    }
  }
  return result;
}

function conditionExpression(cursor: Cursor): ExpressionInfo {
  const begin = cursor.index, result = expression(cursor, 0, 0, false, true);
  if (!result.mixed) {
    const operators: FanucToken[] = []; let brackets = 0;
    for (const token of cursor.tokens.slice(begin, cursor.index)) {
      if (token.upper === '[') brackets++;
      else if (token.upper === ']') brackets--;
      else if (!brackets && ['AND', 'OR', 'XOR'].includes(token.upper)) operators.push(token);
    }
    if (operators.length > 4) throw new SyntaxFault(operators[4], '普通 IF/WAIT/SKIP 最多组合五个条件；更复杂条件需混合逻辑括号');
    if (operators.some(token => token.upper === 'XOR') || new Set(operators.map(token => token.upper)).size > 1) throw new SyntaxFault(operators[0], '普通条件不能混用 AND 和 OR；请使用带括号的混合逻辑');
  }
  return result;
}

function reference(cursor: Cursor, depth = 0): Reference {
  if (depth > 64) throw new SyntaxFault(cursor.token, '引用嵌套超过 64 层');
  const first = cursor.take();
  if (!REGISTERS.has(first.upper) && !FUNCTIONS.has(first.upper)) throw new SyntaxFault(first, '此处需要寄存器、I/O 或位置引用');
  cursor.expect('[');
  if (/^GP\d+$/.test(cursor.token?.upper ?? '') && cursor.tokens[cursor.index + 1]?.upper === ':') {
    const group = cursor.take(); cursor.take();
    if (first.upper !== 'PR' || !/^GP[1-8]$/.test(group.upper)) throw new SyntaxFault(group, '只有 PR 支持 GP1 至 GP8 组限定引用');
  }
  const indexToken = cursor.token; let hasComponent = false;
  // TP indirection permits nested register references, not arbitrary arithmetic indices.
  if (FUNCTIONS.has(first.upper)) {
    // Optional Function /09 §31.2: constants and expressions are not function arguments.
    const argument = () => {
      if (!['R', 'AR'].includes(cursor.token?.upper ?? '')) throw new SyntaxFault(cursor.token, 'J593 数学函数参数只能是 R[] 或 AR[]，不能直接使用常量或表达式');
      reference(cursor, depth + 1);
    };
    argument();
    if (first.upper === 'ATAN2') { cursor.expect(','); argument(); }
  } else {
    const component = () => {
      if (cursor.token?.kind === 'number') { const number = cursor.take(); if (!Number.isSafeInteger(Number(number.text)) || Number(number.text) < 0) throw new SyntaxFault(number, '引用索引必须是非负整数'); }
      else if (['R', 'AR'].includes(cursor.token?.upper ?? '')) reference(cursor, depth + 1);
      else throw new SyntaxFault(cursor.token, '索引需要整数或 R[] / AR[] 间接引用');
    };
    component();
    if (indexToken?.kind === 'number') {
      const index = Number(indexToken.text);
      if (index === 0 && !['UFRAME', 'UTOOL'].includes(first.upper)) throw new SyntaxFault(indexToken, `${first.upper} 索引从 1 开始`);
      if (first.upper === 'PL' && index > 32 || first.upper === 'LBL' && index > 32766) throw new SyntaxFault(indexToken, `${first.upper} 索引超出 /09 的 ${first.upper === 'PL' ? '1 至 32' : '1 至 32766'} 范围`);
    }
    if (cursor.eat(',')) {
      hasComponent = true;
      if (first.upper !== 'PR') throw new SyntaxFault(cursor.token, `${first.upper} 不接受位置分量索引`);
      const coordinate = cursor.token; component();
      if (coordinate?.kind === 'number' && (Number(coordinate.text) < 1 || Number(coordinate.text) > 9)) throw new SyntaxFault(coordinate, 'PR 分量需要 1 至 9，或间接索引');
      if (cursor.is(',')) throw new SyntaxFault(cursor.token, 'PR 引用最多包含寄存器索引和一个位置分量');
    }
  }
  if (cursor.eat(':')) while (!cursor.done && !cursor.is(']')) cursor.take();
  const end = cursor.token; cursor.expect(']');
  return { name: first.upper, component: hasComponent, index: indexToken?.kind === 'number' ? Number(indexToken.text) : null, from: first.from, to: end!.to };
}

/** Scalar operand family used by instructions whose grammar excludes mixed expressions. */
function scalar(cursor: Cursor, depth = 0, integer = false, min = -Infinity, max = Infinity): void {
  if (['R', 'AR'].includes(cursor.token?.upper ?? '')) { reference(cursor, depth + 1); return; }
  const sign = cursor.eat('-') ? -1 : (cursor.eat('+'), 1), token = cursor.take();
  const number = sign * Number(token.text);
  if (token.kind !== 'number' || !Number.isFinite(number) || integer && !Number.isInteger(number) || number < min || number > max) throw new SyntaxFault(token, '此处需要范围内的数值常量或 R[] / AR[]');
}

function splitStatements(tokens: readonly FanucToken[], diagnostic: (token: FanucToken | undefined, message: string, code?: string, severity?: SourceDiagnostic['severity']) => void): FanucToken[][] {
  const output: FanucToken[][] = []; let current: FanucToken[] = [], previousLine = -1;
  for (let index = 0; index < tokens.length; index++) {
    const token = tokens[index];
    if (token.line !== previousLine) {
      previousLine = token.line;
      if (token.kind === 'number' && tokens[index + 1]?.upper === ':') { index++; continue; }
      if (token.upper === ':') continue;
      if (current.length === 0 && token.kind !== 'comment') diagnostic(token, 'LS /MN 指令行应以可选行号和冒号开始', 'fanuc.line-prefix', 'warning');
    }
    if (token.upper === ';') { output.push(current); current = []; } else current.push(token);
  }
  if (current.length) { diagnostic(current.at(-1), '指令缺少结束分号', 'fanuc.missing-semicolon'); output.push(current); }
  return output;
}

/** Parse all recognized syntax, recover per statement, and preserve unknown instructions. */
export function parseFanucLs(source: string, options: FanucParseOptions = {}): FanucDocument {
  const document: FanucDocument = { source, diagnostics: [], symbols: [], folds: [], statements: [], positions: [], applications: [] };
  const reportTemplateParameter = templateParameterReporter(document.diagnostics);
  let diagnosticCount = 0, lastNormalIndex = -1;
  const diagnostic = (token: FanucToken | undefined, message: string, code = 'fanuc.syntax', severity: SourceDiagnostic['severity'] = 'error') => {
    const from = Math.min(source.length, token?.from ?? 0), to = Math.min(source.length, Math.max(from, token?.to ?? from));
    if (code === 'template-parameter') { reportTemplateParameter({ from, to, severity, message, code }); return; }
    if (diagnosticCount >= MAX_DIAGNOSTICS) {
      document.diagnostics[lastNormalIndex] = { from: 0, to: 0, severity: 'error', code: 'fanuc.limit', message: '诊断达到 500 条上限，后续问题未全部列出' };
      return;
    }
    diagnosticCount++;
    lastNormalIndex = document.diagnostics.length;
    document.diagnostics.push({ from, to, severity, message, code });
  };
  if (source.length > MAX_SOURCE_BYTES || new TextEncoder().encode(source).byteLength > MAX_SOURCE_BYTES) { diagnostic(undefined, 'LS 文件超过 1 MiB，未执行完整检查', 'fanuc.limit'); return document; }
  const tokens = lex(source, diagnostic);
  const sections: { name: string; token: FanucToken; tokens: FanucToken[] }[] = [];
  for (const token of tokens) {
    if (token.kind === 'section') {
      sections.push({ name: token.upper, token, tokens: [] });
      if (!SECTIONS.has(token.upper)) diagnostic(token, `未知 LS 区段 ${token.text} 已保留，未验证其语法`, 'fanuc.unsupported-section', 'warning');
    } else if (sections.length) sections.at(-1)!.tokens.push(token);
    else diagnostic(token, 'LS 程序应从 /PROG 区段开始', 'fanuc.missing-program');
  }
  if (!sections.length) { if (source.trim()) diagnostic(tokens[0], '未找到 /PROG 程序区段', 'fanuc.missing-program'); return document; }
  const seenSections = new Set<string>();
  for (const section of sections) {
    if (seenSections.has(section.name) && section.name !== '/APPL') diagnostic(section.token, `重复的 ${section.name} 区段`, 'fanuc.duplicate-section');
    seenSections.add(section.name);
    const order = ['/PROG', '/ATTR', '/APPL', '/MN', '/POS', '/END'];
    const previous = sections[sections.indexOf(section) - 1];
    if (previous && order.indexOf(section.name) >= 0 && order.indexOf(previous.name) > order.indexOf(section.name)) diagnostic(section.token, 'LS 区段顺序无效', 'fanuc.section-order');
    if (section.tokens.length) document.folds.push({ from: section.token.to, to: section.tokens.at(-1)!.to });
  }
  for (const section of sections.filter(section => section.name === '/END')) if (section.tokens.length) diagnostic(section.tokens[0], '/END 后不能附带程序内容', 'fanuc.after-end');
  if (sections[0].name !== '/PROG') diagnostic(sections[0].token, '第一个区段必须是 /PROG', 'fanuc.missing-program');
  if (!seenSections.has('/ATTR')) diagnostic(sections[0].token, '缺少 /ATTR 程序属性区段', 'fanuc.missing-attributes');
  if (!seenSections.has('/MN')) diagnostic(sections[0].token, '缺少 /MN 指令区段', 'fanuc.missing-main');
  if (!seenSections.has('/END')) diagnostic(tokens.at(-1), '建议用 /END 结束完整 LS 文件；当前文件尾未显式结束', 'fanuc.missing-end', 'warning');
  const header = sections.find(section => section.name === '/PROG');
  if (header) {
    const name = header.tokens[0];
    if (!name || !validProgramName(name.text)) diagnostic(name ?? header.token, '程序名需 1 至 36 个字母、数字或下划线，不能数字开头或使用 CON/PRN/AUX/NUL/COMn/LPTn 保留名', 'fanuc.program-name');
    else document.symbols.push({ name: name.text, kind: 'routine', from: name.from, to: name.to, signature: '/PROG ' + name.text });
    if (header.tokens.length > 2) diagnostic(header.tokens[2], '/PROG 包含多余字段', 'fanuc.program-header');
    if (header.tokens[1] && !['MACRO', 'COND', 'COLLECTION', 'JOB', 'PROCESS'].includes(header.tokens[1].upper)) diagnostic(header.tokens[1], '未知程序子类型，需核对控制器版本', 'fanuc.unsupported-subtype', 'warning');
  }
  for (const section of sections.filter(section => section.name === '/APPL')) document.applications.push(...parseFanucApplications(section.tokens, { diagnostic, folds: document.folds, symbols: document.symbols }));
  for (const section of sections.filter(section => section.name === '/ATTR')) {
    let current: FanucToken[] = [];
    const check = () => {
      if (!current.length) return;
      if (section.name === '/ATTR') validateFanucAttribute(current, diagnostic);
      if (current[0].upper === 'LOCAL_REGISTERS') {
        const counts = current.filter(token => token.kind === 'number').map(token => Number(token.text));
        if (counts.length !== 3 || counts.some((value, index) => !Number.isInteger(value) || value < 0 || value > [100, 30, 20][index])) diagnostic(current[0], 'LOCAL_REGISTERS 需要数值/位置/字符串三个数量（上限 100/30/20）', 'fanuc.local-registers');
        diagnostic(current[0], '局部寄存器需要 FANUC software v9.40 或更新版本', 'fanuc.version', 'info');
      }
      current = [];
    };
    for (const token of section.tokens) { if (token.upper === ';') check(); else current.push(token); }
    if (current.length) { diagnostic(current.at(-1), '程序属性缺少结束分号', 'fanuc.missing-semicolon'); check(); }
  }
  const labels = new Map<number, FanucToken>(), labelUses: Reference[] = [], positionUses: Reference[] = [];
  const blocks: { kind: 'IF' | 'FOR'; token: FanucToken; alternate: boolean }[] = [];
  let selectionOpen = false;
  const useReference = (cursor: Cursor, expected?: string[]) => {
    const result = reference(cursor);
    if (expected?.includes('PR') && result.name === 'PR' && result.component) throw new SyntaxFault(cursor.tokens[cursor.index - 1], '此处需要完整 PR 位置，不能使用 PR 分量');
    if (expected && !expected.includes(result.name)) throw new SyntaxFault(cursor.tokens[cursor.index - 1], `此处需要 ${expected.join(' 或 ')} 引用`);
    if (result.name === 'P') positionUses.push(result);
    if (result.name === 'LBL') labelUses.push(result);
    return result;
  };
  const callArguments = (cursor: Cursor, concurrent = false) => {
    if (cursor.eat('(')) {
      if (concurrent) throw new SyntaxFault(cursor.tokens[cursor.index - 1], '此调用形式不接受参数列表，带参数调用只能用于允许的 CALL 形式');
      let count = 0;
      const argument = () => {
        if (++count > 30) throw new SyntaxFault(cursor.token, 'CALL 最多 30 个参数（Basic /09 §4.7.6）');
        if (cursor.token?.kind === 'string' && cursor.token.text.startsWith('"') && cursor.tokens[cursor.index + 1]?.upper === '=') {
          const description = cursor.take(); cursor.expect('=');
          diagnostic(description, '带双引号参数说明的 ASCII 形式需要 7DC3（V8.30）或更新版本（Basic /09 §4.7.7.5）', 'fanuc.version', 'info');
        }
        if (cursor.token?.kind === 'string') { const literal = cursor.take(); if ([...literal.text.slice(1, -1)].length > 34) throw new SyntaxFault(literal, 'CALL 字符串参数最多 34 个字符'); }
        else if (['R', 'AR', 'SR'].includes(cursor.token?.upper ?? '')) reference(cursor);
        else if (cursor.eat('(')) { scalar(cursor); cursor.expect(')'); }
        else scalar(cursor);
      };
      if (!cursor.eat(')')) { argument(); while (cursor.eat(',')) argument(); cursor.expect(')'); }
    }
  };
  const call = (cursor: Cursor, concurrent = false) => {
    if (cursor.is('SR')) useReference(cursor, ['SR']);
    else { const name = cursor.take(); if (!validProgramName(name.text)) throw new SyntaxFault(name, '调用需要有效程序名（1 至 36 字符）或 SR[]'); }
    callArguments(cursor, concurrent);
  };
  const action = (cursor: Cursor, allowAssignment = true) => {
    if (cursor.eat('JMP')) useReference(cursor, ['LBL']);
    else if (cursor.eat('CALL')) call(cursor);
    else if (allowAssignment) assignment(cursor);
    else throw new SyntaxFault(cursor.token, '此分支只接受 JMP LBL[] 或 CALL；IF 中的赋值／PULSE 需要混合逻辑条件');
  };
  const selectionValue = (cursor: Cursor) => {
    if (['R', 'AR', 'SR'].includes(cursor.token?.upper ?? '')) reference(cursor);
    else { if (REGISTERS.has(cursor.token?.upper ?? '')) throw new SyntaxFault(cursor.token, 'SELECT 比较值需要常量或 R[] / AR[] / SR[]（附录 A.3.7）'); scalar(cursor); }
  };
  const monitorCondition = (cursor: Cursor) => {
    let join: string | undefined, count = 0;
    const numeric = ['R', 'AI', 'AO', 'GI', 'GO'];
    const boolean = ['DO', 'DI', 'RO', 'RI', 'SO', 'SI', 'UO', 'UI'];
    do {
      if (++count > 5) throw new SyntaxFault(cursor.token, 'WHEN 最多组合五个条件');
      const left = cursor.token;
      if (left?.text.startsWith('$')) systemReference(cursor);
      else if (left?.upper === 'ERR_NUM') cursor.take();
      else useReference(cursor, [...numeric, ...boolean]);
      const operator = cursor.take();
      if (!(left?.upper === 'ERR_NUM' ? ['='] : boolean.includes(left?.upper ?? '') ? ['=', '<>'] : ['=', '<>', '<', '>', '<=', '>=']).includes(operator.upper)) throw new SyntaxFault(operator, 'WHEN 比较操作符与操作数不匹配');
      if (boolean.includes(left?.upper ?? '')) {
        if (cursor.eat('ON')) cursor.eat('+');
        else if (cursor.eat('OFF')) cursor.eat('-');
        else useReference(cursor, [...boolean, 'R']);
      } else if (left?.upper === 'ERR_NUM') { if (REGISTERS.has(cursor.token?.upper ?? '')) throw new SyntaxFault(cursor.token, 'WHEN ERR_NUM 需要固定报警编号'); scalar(cursor, 0, true, 0); }
      else if (cursor.is('R')) reference(cursor);
      else { if (REGISTERS.has(cursor.token?.upper ?? '')) throw new SyntaxFault(cursor.token, 'WHEN 数值比较右侧需要常量或 R[]'); scalar(cursor); }
      if (!cursor.is('AND') && !cursor.is('OR')) break;
      const next = cursor.take(); if (join && join !== next.upper) throw new SyntaxFault(next, 'WHEN 不能混合 AND 和 OR'); join = next.upper;
    } while (!cursor.done);
  };
  const assignment = (cursor: Cursor) => {
    const first = cursor.token; let target: Reference | undefined;
    if (first && REGISTERS.has(first.upper)) {
      const ref = reference(cursor); target = ref;
      if (['DI', 'RI', 'AI', 'GI', 'UI', 'SI', 'WI', 'WSI', 'P', 'LBL', 'AR', 'TIMER_OVERFLOW', 'TCP_SPD', 'PAYLOAD', 'UALM'].includes(ref.name)) throw new SyntaxFault(first, `${ref.name} 不是此赋值形式的可写目标`);
    } else if (first?.text.startsWith('$')) systemReference(cursor);
    else if (first && ['UFRAME_NUM', 'UTOOL_NUM', 'OVERRIDE', 'LINEAR_MAX_SPEED'].includes(first.upper)) {
      cursor.take();
      if (cursor.eat('[')) { const group = cursor.take(); if (!/^GP[1-8]$/.test(group.upper) || !['UFRAME_NUM', 'UTOOL_NUM'].includes(first.upper)) throw new SyntaxFault(group, '仅坐标系选择器接受 GP1 至 GP8'); cursor.expect(']'); }
    }
    else throw new SyntaxFault(first, '赋值左侧需要可写寄存器、输出或系统变量');
    cursor.expect('=');
    if (target?.name === 'M') {
      if (!cursor.is('(')) throw new SyntaxFault(cursor.token, 'M[] 标记定义需要括号内的混合逻辑，清除使用 ()（Basic /09 §9.12.4）');
      if (cursor.tokens[cursor.index + 1]?.upper === ')') { cursor.take(); cursor.take(); return; }
    }
    if (first && ['UFRAME', 'UTOOL'].includes(first.upper)) { useReference(cursor, ['PR']); return; }
    if (first && ['UFRAME_NUM', 'UTOOL_NUM', 'LINEAR_MAX_SPEED', 'JOINT_MAX_SPEED', 'OVERRIDE'].includes(first.upper)) {
      scalar(cursor, 0, false, 0); if (first.upper === 'OVERRIDE') cursor.eat('%'); return;
    }
    if (['STRLEN', 'FINDSTR', 'SUBSTR'].includes(cursor.token?.upper ?? '')) {
      const operation = cursor.take();
      if (target?.name !== (operation.upper === 'SUBSTR' ? 'SR' : 'R')) throw new SyntaxFault(first, 'STRLEN/FINDSTR 写入 R；SUBSTR 写入 SR');
      useReference(cursor, ['SR', 'AR']);
      if (operation.upper === 'FINDSTR') { cursor.expect(','); useReference(cursor, ['SR', 'AR']); }
      if (operation.upper === 'SUBSTR') { cursor.expect(','); scalar(cursor, 0, true, 1); cursor.expect(','); scalar(cursor, 0, true, 0); }
      return;
    }
    if (first?.upper === 'RSR') {
      const state = cursor.take();
      if (!['ENABLE', 'DISABLE'].includes(state.upper)) throw new SyntaxFault(state, 'RSR 需要 ENABLE 或 DISABLE');
      return;
    }
    if (first?.upper === 'TIMER' && ['START', 'STOP', 'RESET'].includes(cursor.token?.upper ?? '')) { cursor.take(); return; }
    if (cursor.eat('PULSE')) {
      if (!first || !['DO', 'RO'].includes(first.upper)) throw new SyntaxFault(first, 'PULSE 只能写入 DO[] 或 RO[]');
      if (cursor.eat(',')) { scalar(cursor, 0, false, 0); if (cursor.eat('(')) { cursor.expect('SEC'); cursor.expect(')'); } else cursor.expect('SEC'); }
      return;
    }
    const rightBegin = cursor.index, right = expression(cursor), kind = target ? referenceKind(target) : first?.text.startsWith('$') ? 'unknown' : 'number';
    if (!right.mixed) {
      const operations: FanucToken[] = []; let brackets = 0, previous: FanucToken | undefined;
      for (const token of cursor.tokens.slice(rightBegin, cursor.index)) {
        if (token.upper === '[') brackets++;
        else if (token.upper === ']') brackets--;
        else if (!brackets && ['+', '-', '*', '/', 'MOD', 'DIV'].includes(token.upper) && previous && !Object.hasOwn(BINARY, previous.upper)) operations.push(token);
        previous = token;
      }
      if (operations.length > 5) throw new SyntaxFault(operations[5], '普通寄存器运算最多五个运算符；更复杂公式需要混合逻辑括号');
      if (operations.some(token => ['+', '-'].includes(token.upper)) && operations.some(token => ['*', '/', 'MOD', 'DIV'].includes(token.upper))) throw new SyntaxFault(operations[0], '普通寄存器运算不能混合加减与乘除；请使用混合逻辑括号');
    }
    if (kind === 'position') expectKind(right, ['position'], first, '完整 PR/坐标系赋值需要位置数据');
    else if (kind === 'pallet') expectKind(right, ['pallet'], first, 'PL 赋值需要 PL 或三元素码垛向量');
    else if (kind === 'string') expectKind(right, ['number', 'string'], first, 'SR 赋值需要字符串或数值');
    else if (kind !== 'unknown') expectKind(right, target?.name === 'R' || target?.name === 'PR' && target.component ? ['number', 'boolean', 'string'] : ['number', 'boolean'], first, '标量赋值不能使用完整位置、码垛或字符串数据');
    if (cursor.eat('%') && first?.upper !== 'OVERRIDE') throw new SyntaxFault(first, '赋值中的百分号仅用于 OVERRIDE');

  };
  const motionOptions = (cursor: Cursor, type: string, passingPoint = false) => {
    let outputCount = 0, hasWristJoint = false;
    let minimalRotation: FanucToken | undefined;
    let count = 0;
    while (!cursor.done && !/^GP[1-8]$/.test(cursor.token.upper) && !(passingPoint && ['P', 'PR'].includes(cursor.token.upper))) {
      if (passingPoint && ++count > 1) throw new SyntaxFault(cursor.token, 'C 经过点最多一个附加运动选项（Basic /09 §4.3.5）');
      const next = cursor.token.upper;
      if (['A', 'S'].includes(type) && ['INC', 'SKIP', 'SKIPJUMP', 'SEARCH', 'SPOT', 'BREAK', 'EV', 'IND', 'DO', 'RO', 'GO', 'AO'].includes(next)) throw new SyntaxFault(cursor.token, `${type} 不支持此附加选项（Basic /09 §9.19.14；Optional /09 §65.9）`);
      if (type === 'A' && ['SS', 'SE'].includes(next) || type === 'S' && (next === 'WEAVE' || /^CTV(?:R|\d)/.test(next))) throw new SyntaxFault(cursor.token, `${type} 不支持此附加选项`);
      if (parseFanucOptionalMotionOption(cursor, type, diagnostic)) continue;
      const option = cursor.take();
      if (option.upper === 'WJNT') { if (type === 'J') throw new SyntaxFault(option, 'Wjnt 是笛卡尔运动附加选项，不能用于 J'); hasWristJoint = true; continue; }
      if (['INC', 'PTH', 'BREAK', 'COORD', 'RTCP'].includes(option.upper)) continue;
      if (['DO', 'RO', 'GO', 'AO'].includes(option.upper)) {
        if (++outputCount > 5) throw new SyntaxFault(option, '每个运动最多 5 个直接输出选项（Basic /09 §9.8）');
        cursor.index--; assignment(cursor); continue;
      }
      if (option.upper === 'MROT') {
        minimalRotation = option;
        diagnostic(option, 'MROT 需要 HandlingTool MROT 选件（Optional /09 §41）', 'fanuc.option', 'info'); continue;
      }
      if (option.upper === 'VIA') {
        if (!['J', 'L'].includes(type)) throw new SyntaxFault(option, 'VIA 仅接受 J/L');
        cursor.expect('('); const mode = cursor.take();
        if (!['DEFAULT', 'FINE', 'CNT'].includes(mode.upper)) throw new SyntaxFault(mode, 'VIA 需要 DEFAULT、FINE 或 CNT');
        if (mode.upper !== 'DEFAULT') { cursor.expect(','); cursor.expect('Z'); cursor.expect('='); scalar(cursor); }
        if (mode.upper === 'CNT') {
          if (type !== 'L') throw new SyntaxFault(mode, 'VIA 的 RT_LD/AP_LD 仅用于 L');
          cursor.expect(','); cursor.expect('RT_LD'); cursor.expect('='); scalar(cursor, 0, false, 0);
          cursor.expect(','); cursor.expect('AP_LD'); cursor.expect('='); scalar(cursor, 0, false, 0);
        }
        cursor.expect(')'); diagnostic(option, 'VIA 仅适用 SCARA、7DF1/20 及更新版本（Optional /09 §63）', 'fanuc.option', 'info'); continue;
      }
      if (/^ACC\d+$/.test(option.upper)) {
        const amount = Number(option.upper.slice(3));
        if (amount > 500) throw new SyntaxFault(option, 'ACC 超出 /09 附录 A.3.2 的 0 至 500 范围');
        if (amount > 100) diagnostic(option, '/09 正文 §4.3.5 与附录 A.3.2 对 ACC 范围描述不同；超过 100 需核对控制器', 'fanuc.manual-range-difference', 'warning');
        continue;
      }
      if (['ACC', 'AP_LD', 'RT_LD'].includes(option.upper)) { scalar(cursor, 0, false, 0, option.upper === 'ACC' ? 500 : Infinity); continue; }
      if (option.upper === 'EV' || option.upper === 'IND') {
        if (option.upper === 'IND') { cursor.expect('.'); cursor.expect('EV'); }
        scalar(cursor, 0, false, 1, 100); cursor.expect('%'); continue;
      }
      if (option.upper === 'ORNT_BASE') {
        if (!['C', 'A'].includes(type)) throw new SyntaxFault(option, 'ORNT_BASE 只用于 C/A');
        if (cursor.eat('UF') || cursor.eat('LDR')) { cursor.expect('['); scalar(cursor, 0, true, 0); cursor.expect(','); const axis = cursor.take(); if (!['X', 'Y', 'Z'].includes(axis.upper)) throw new SyntaxFault(axis, 'ORNT_BASE 方向需要 X、Y 或 Z'); cursor.expect(']'); }
        continue;
      }
      if (['OFFSET', 'TOOL_OFFSET'].includes(option.upper)) { if (cursor.eat(',')) useReference(cursor, ['PR']); continue; }
      if (['SKIP', 'SKIPJUMP'].includes(option.upper)) { if (option.upper === 'SKIPJUMP') diagnostic(option, 'SkipJump 不在选定 Basic/Optional /09 生产式中，保留并需核对控制器版本', 'fanuc.unsupported-motion-option', 'warning'); cursor.expect(','); useReference(cursor, ['LBL']); if (cursor.eat(',')) { useReference(cursor, ['PR']); cursor.expect('='); const pose = cursor.take(); if (!['LPOS', 'JPOS'].includes(pose.upper)) throw new SyntaxFault(pose, '高速 Skip 记录位置需要 LPOS 或 JPOS'); } continue; }
      if (['TB', 'TA', 'DB'].includes(option.upper)) {
        scalar(cursor, 0, false, 0, option.upper === 'DB' ? 999.9 : option.upper === 'TB' ? 30 : .5); cursor.expect(option.upper === 'DB' ? 'MM' : 'SEC'); cursor.eat(',');
        if (cursor.eat('CALL')) call(cursor);
        else if (cursor.eat('POINT_LOGIC')) diagnostic(option, 'Point Logic 的嵌入子程序数据尚未解析，不能视为完整语法通过', 'fanuc.unsupported-point-logic', 'warning');
        else if (['DO', 'RO', 'GO', 'AO'].includes(cursor.token?.upper ?? '')) assignment(cursor);
        else throw new SyntaxFault(cursor.token, 'TB/TA/DB 需要 CALL、DO/RO/GO/AO 输出或 POINT_LOGIC');
        continue;
      }
      diagnostic(option, `运动选项 ${option.text} 的参数尚未验证；保留整条原始指令`, 'fanuc.unsupported-motion-option', 'warning'); cursor.index = cursor.tokens.length;
    }
    if (minimalRotation && type !== 'J' && !hasWristJoint) throw new SyntaxFault(minimalRotation, 'MROT 用于 J 或带 Wjnt 的笛卡尔运动');
  };
  const motion = (cursor: Cursor, type: string) => {
    const first = cursor.tokens[0];
    if (/^PAL_\d+$/.test(cursor.token?.upper ?? '')) {
      const pallet = cursor.take();
      if (!['J', 'L'].includes(type) || Number(pallet.upper.slice(4)) < 1 || Number(pallet.upper.slice(4)) > 16) throw new SyntaxFault(pallet, '码垛点仅用于 J/L，编号为 1 至 16');
      cursor.expect('['); const point = cursor.take();
      if (!/^(BTM|[AR]_[1-8])$/.test(point.upper)) throw new SyntaxFault(point, '码垛点需要 BTM、A_1..A_8 或 R_1..R_8');
      cursor.expect(']'); diagnostic(pallet, '码垛运动需要 J500 选件', 'fanuc.option', 'info');
    } else useReference(cursor, ['P', 'PR']);
    if (type === 'C') { motionOptions(cursor, type, true); useReference(cursor, ['P', 'PR']); }
    const speed = cursor.token, optionalSpeed = parseFanucOptionalMotionSpeed(cursor, type, diagnostic);
    if (optionalSpeed) { /* complete option-specific speed production */ }
    else if (cursor.is('R')) reference(cursor);
    else if (cursor.is('AR')) throw new SyntaxFault(speed, 'AR[] 不能直接用于间接运动速度；请先复制到 R[]');
    else if (cursor.eat('WELD_SPEED')) diagnostic(speed, 'WELD_SPEED 需要 ArcTool 焊接速度配置', 'fanuc.option', 'info');
    else { const number = cursor.take(); if (number.kind !== 'number' || !Number.isFinite(Number(number.text)) || Number(number.text) <= 0) throw new SyntaxFault(number, '运动速度必须为正数或 R[]'); }
    if (optionalSpeed) { /* no explicit unit follows max_speed */ }
    else if (cursor.eat('%')) { if (type !== 'J') throw new SyntaxFault(speed, '百分比速度仅用于 J；L/C/A/S 需要路径单位或时间'); if (speed?.kind === 'number' && Number(speed.text) > 100) throw new SyntaxFault(speed, '百分比速度不能大于 100'); }
    else if (['MM', 'CM', 'INCH', 'DEG'].includes(cursor.token?.upper ?? '')) {
      const distance = cursor.take(); cursor.expect('/'); const unit = cursor.take();
      if (type === 'J' || !['MM/SEC', 'CM/MIN', 'INCH/MIN', 'DEG/SEC'].includes(`${distance.upper}/${unit.upper}`)) throw new SyntaxFault(distance, '路径速度仅支持 mm/sec、cm/min、inch/min、deg/sec，不能用于 J');
    }
    else if (!cursor.eat('SEC') && !cursor.eat('MSEC') && speed?.upper !== 'WELD_SPEED') throw new SyntaxFault(cursor.token, '运动速度缺少单位；R[] 间接速度仍需 % 或路径／时间单位');
    if (!cursor.eat('FINE')) {
      const term = cursor.take(), match = /^(CNT|CR)(\d+(?:\.\d+)?)$/.exec(term.upper);
      const family = match?.[1] ?? term.upper;
      if (!['CNT', 'CR'].includes(family)) throw new SyntaxFault(term, '终止类型需要 FINE、CNT 或选件 CR');
      if (family === 'CR') {
        if (type === 'J' || type === 'S') throw new SyntaxFault(term, 'CR 不能用于 J/S 运动（Optional /09 §14.2/§65.9）');
        diagnostic(term, 'CR 距离终止需要 Advanced Constant Path 选件', 'fanuc.option', 'info');
      }
      if (match) {
        let numericText = match[2];
        if (cursor.token?.kind === 'number' && cursor.token.text.startsWith('.') && cursor.token.from === term.to) numericText += cursor.take().text;
        const value = Number(numericText);
        if (!Number.isFinite(value) || value < 0 || family === 'CNT' && (!Number.isInteger(value) || value > 100) || family === 'CR' && value > 1000) throw new SyntaxFault(term, 'CNT 必须为 0 至 100 整数；CR 距离必须为 0 至 1000 mm');
      } else if (cursor.is('R')) reference(cursor);
      else if (cursor.is('AR')) throw new SyntaxFault(cursor.token, '运动终止参数间接引用需要 R[]');
      else scalar(cursor, 0, family === 'CNT', 0, family === 'CNT' ? 100 : 1000);
    }
    motionOptions(cursor, type);
    if (type === 'S') diagnostic(first, `${type} 运动需要对应控制器选件；此处仅检查源结构`, 'fanuc.option', 'info');
  };
  const main = sections.find(section => section.name === '/MN');
  if (main) for (const row of splitStatements(main.tokens, diagnostic)) {
    if (!row.length) continue;
    const first = row[0];
    const statement: FanucStatement = { from: first.from, to: row.at(-1)!.to, kind: first.upper, tokens: row }; document.statements.push(statement);
    if (row.every(token => token.kind === 'comment')) { statement.kind = 'comment'; continue; }
    const cursor = new Cursor(row);
    try {
      if (header?.tokens[1]?.upper === 'COND' && first.upper !== 'WHEN') throw new SyntaxFault(first, 'Cond 子类型只接受 WHEN 条件监视指令');
      if (parseFanucOptionalStatement(cursor, diagnostic, {
        assignment,
        condition: (conditionCursor: Cursor) => { const token = conditionCursor.token; expectKind(expression(conditionCursor), ['boolean'], token, '条件需要布尔表达式'); },
      })) { selectionOpen = false; cursor.end(); continue; }
      const keyword = cursor.take().upper;
      const isSelectCase = keyword === '=' || keyword === 'ELSE' && cursor.is(',');
      if (keyword !== 'SELECT' && !isSelectCase) selectionOpen = false;
      if (['J', 'L', 'C', 'A', 'S'].includes(keyword)) motion(cursor, keyword);
      else if (keyword === 'INDEPENDENT' || keyword === 'SIMULTANEOUS') {
        cursor.expect('GP'); const groups = new Set<string>();
        while (!cursor.done) {
          const group = cursor.take();
          if (!/^GP[1-8]$/.test(group.upper) || groups.has(group.upper)) throw new SyntaxFault(group, '运动组需要不重复的 GP1 至 GP8');
          groups.add(group.upper); const move = cursor.take();
          if (!['J', 'L'].includes(move.upper)) throw new SyntaxFault(move, '独立／同步组仅接受 J/L（Basic /09 §4.17）');
          motion(cursor, move.upper);
        }
        if (groups.size < 2) throw new SyntaxFault(first, '独立／同步组需要至少两个运动组');
        diagnostic(first, '分组运动需要多任务选件；已检查各组语法', 'fanuc.option', 'info');
      }
      else if (keyword === 'PALLETIZING') {
        cursor.expect('-'); const pattern = cursor.take();
        if (!/^(B|BX|E|EX|END)_(?:[1-9]|1[0-6])$/.test(pattern.upper)) throw new SyntaxFault(pattern, '码垛指令需要 B/BX/E/EX/END 和 1 至 16 编号');
        diagnostic(first, '码垛需要 J500 选件', 'fanuc.option', 'info');
      }
      else if (keyword === 'TOOL') {
        const operation = cursor.take();
        if (!['ATTACH', 'DETACH'].includes(operation.upper)) throw new SyntaxFault(operation, '伺服工具指令需要 ATTACH 或 DETACH');
        cursor.expect('['); scalar(cursor, 0, true, 1, 4); cursor.expect(']');
        diagnostic(first, 'TOOL ATTACH/DETACH 需要 Servo Tool Change 选件（Optional /09 §32.6）', 'fanuc.option', 'info');
      }
      else if (keyword === 'DIAG_REC' || keyword === 'DIAG_REC_SEC') {
        const open = cursor.take();
        if (!['[', '('].includes(open.upper)) throw new SyntaxFault(open, '诊断指令需要括号参数列表');
        scalar(cursor, 0, true, 1, 10); cursor.expect(','); scalar(cursor, 0, keyword === 'DIAG_REC', 0);
        if (keyword === 'DIAG_REC') { cursor.expect(','); scalar(cursor, 0, true, 1); }
        cursor.expect(open.upper === '[' ? ']' : ')');
      }
      else if (keyword === 'CALL' || keyword === 'RUN') call(cursor, keyword === 'RUN');
      else if (keyword === 'JMP') useReference(cursor, ['LBL']);
      else if (keyword === 'LBL') {
        cursor.index = 0; const ref = reference(cursor);
        if (ref.index === null) throw new SyntaxFault(first, '标签定义必须使用固定整数');
        if (labels.has(ref.index)) diagnostic(first, `标签 LBL[${ref.index}] 重复定义`, 'fanuc.duplicate-label');
        else { labels.set(ref.index, first); document.symbols.push({ name: `LBL[${ref.index}]`, kind: 'constant', from: first.from, to: ref.to }); }
      } else if (keyword === 'IF') {
        const condition = conditionExpression(cursor); expectKind(condition, ['boolean'], first, 'IF 条件需要布尔表达式');
        if (cursor.eat('THEN')) { if (!condition.mixed) throw new SyntaxFault(first, 'IF_THEN 需要括号内的混合逻辑条件'); blocks.push({ kind: 'IF', token: first, alternate: false }); }
        else { cursor.expect(','); action(cursor, condition.mixed); }
      } else if (keyword === 'FOR') {
        useReference(cursor, ['R']); cursor.expect('='); scalar(cursor);
        if (!cursor.eat('TO') && !cursor.eat('DOWNTO')) throw new SyntaxFault(cursor.token, 'FOR 需要 TO 或 DOWNTO');
        scalar(cursor); blocks.push({ kind: 'FOR', token: first, alternate: false });
      } else if (keyword === 'ENDIF' || keyword === 'ENDFOR') {
        const expected = keyword === 'ENDIF' ? 'IF' : 'FOR';
        if (blocks.at(-1)?.kind !== expected) throw new SyntaxFault(first, `${keyword} 没有匹配的 ${expected}`);
        const block = blocks.pop()!; document.folds.push({ from: block.token.to, to: first.from });
      } else if (keyword === 'ELSE' && !cursor.is(',')) {
        const block = blocks.at(-1); if (!block || block.kind !== 'IF' || block.alternate) throw new SyntaxFault(first, 'ELSE 缺少匹配 IF，或重复出现'); block.alternate = true;
      } else if (keyword === 'SELECT') {
        useReference(cursor, ['R']); cursor.expect('='); selectionValue(cursor); cursor.expect(','); action(cursor, false); selectionOpen = true;
        while (cursor.is('=') || cursor.is('ELSE')) {
          if (cursor.eat('=')) selectionValue(cursor); else { cursor.expect('ELSE'); selectionOpen = false; }
          cursor.expect(','); action(cursor, false);
          if (!selectionOpen) break;
        }
      } else if (isSelectCase) {
        if (!selectionOpen) throw new SyntaxFault(first, 'SELECT 分支缺少前面的 SELECT');
        if (keyword === '=') selectionValue(cursor); else selectionOpen = false;
        cursor.expect(','); action(cursor, false);
      } else if (keyword === 'WAIT') {
        const condition = conditionExpression(cursor); if (condition.mixed) expectKind(condition, ['boolean'], first, '混合逻辑 WAIT 条件需要布尔表达式');
        if (cursor.eat('(')) { cursor.expect('SEC'); cursor.expect(')'); } else cursor.eat('SEC');
        if (cursor.eat(',')) { cursor.expect('TIMEOUT'); useReference(cursor, ['LBL']); }
        else if (cursor.eat('TIMEOUT')) { cursor.expect(','); useReference(cursor, ['LBL']); }
      } else if (keyword === 'SKIP') { cursor.expect('CONDITION'); const condition = conditionExpression(cursor); expectKind(condition, ['boolean'], first, 'SKIP CONDITION 需要比较或布尔条件'); }
      else if (keyword === 'OFFSET' || keyword === 'TOOL_OFFSET') { cursor.expect('CONDITION'); useReference(cursor, ['PR']); if (cursor.eat(',')) useReference(cursor, [keyword === 'OFFSET' ? 'UFRAME' : 'UTOOL']); }
      else if (keyword === 'MONITOR') { cursor.eat('END'); if (cursor.is('SR')) throw new SyntaxFault(cursor.token, 'MONITOR 需要固定 Cond 程序名'); call(cursor, true); }
      else if (keyword === 'TC_ONLINE') {
        if (!cursor.eat('ENABLE') && !cursor.eat('DISABLE')) {
          if (!cursor.is('(')) throw new SyntaxFault(cursor.token, 'TC_ONLINE 需要括号内的混合逻辑，或 ENABLE/DISABLE');
          expectKind(expression(cursor), ['boolean'], first, 'TC_ONLINE 表达式需要布尔结果');
        }
        diagnostic(first, 'TC_ONLINE 定义需 $MIX_LOGIC.$USE_TCOL；ENABLE/DISABLE 还受 USE_TCOLSIM 配置约束（Basic /09 §9.12.4）', 'fanuc.option', 'info');
      }
      else if (keyword === 'WHEN') {
        if (header?.tokens[1]?.upper !== 'COND') throw new SyntaxFault(first, 'WHEN 只能用于 Cond 子类型程序');
        monitorCondition(cursor); cursor.expect(','); cursor.expect('CALL'); if (cursor.is('SR')) throw new SyntaxFault(cursor.token, 'WHEN 的动作需要固定程序名'); call(cursor, true);
      }
      else if (keyword === 'COL') { cursor.expect('DETECT'); const state = cursor.take(); if (!['ON', 'OFF'].includes(state.upper)) throw new SyntaxFault(state, 'COL DETECT 需要 ON 或 OFF'); }
      else if (keyword === 'LOCK' || keyword === 'UNLOCK') cursor.expect('PREG');
      else if (keyword === 'MESSAGE') { cursor.expect('['); while (!cursor.done && !cursor.is(']')) cursor.take(); cursor.expect(']'); }
      else if (keyword === 'PAYLOAD') {
        cursor.expect('[');
        if (/^GP[1-8]$/.test(cursor.token?.upper ?? '')) {
          const mask = new Set<number>(); mask.add(Number(cursor.take().upper.slice(2)));
          while (cursor.eat(',')) { const group = cursor.take(), number = Number(group.text); if (group.kind !== 'number' || !Number.isInteger(number) || number < 1 || number > 8 || mask.has(number)) throw new SyntaxFault(group, 'PAYLOAD 运动组掩码需要不重复的 1 至 8'); mask.add(number); }
          cursor.expect(':');
        }
        if (cursor.is('R')) reference(cursor); else if (cursor.is('AR')) throw new SyntaxFault(cursor.token, 'PAYLOAD 间接计划编号需要 R[]'); else scalar(cursor, 0, true, 1, 10);
        if (cursor.eat(':')) while (!cursor.done && !cursor.is(']')) cursor.take(); cursor.expect(']');
      }
      else if (keyword === 'UALM') { cursor.index = 0; reference(cursor); }
      else if (['ERROR_PROG', 'ERR_PROG', 'RESUME_PROG', 'MAINT_PROG'].includes(keyword)) { cursor.expect('='); call(cursor, true); }
      else if (['END', 'PAUSE', 'ABORT', 'CLEAR_RESUME_PROG', 'RETURN_PATH_DSBL'].includes(keyword)) { /* no operands */ }
      else if (REGISTERS.has(keyword) || VALUES.has(keyword) || first.text.startsWith('$')) { cursor.index = 0; assignment(cursor); }
      else {
        statement.kind = 'unsupported';
        const opening = row.findIndex(token => token.upper === '('), nameEnd = opening < 0 ? row.length : opening;
        const name = source.slice(first.from, row[nameEnd - 1]?.to ?? first.to).trim();
        const reservedApplication = ['ARC', 'SPOT', 'WEAVE', 'VISION', 'TRACK', 'REC', 'SS', 'SE'].includes(first.upper);
        if (!reservedApplication && /^[A-Za-z_][A-Za-z0-9_ .-]*$/.test(name)) {
          if (name.length > 36) throw new SyntaxFault(first, '宏名称最多 36 字符（Basic /09 §9.1.1）');
          cursor.index = nameEnd; callArguments(cursor);
          diagnostic(first, `宏 ${name} 的参数形式已检查，但缺少控制器宏别名／程序映射，名称尚未核定；源码已保留`, 'fanuc.unsupported-macro', 'warning');
        } else {
          diagnostic(first, `未完整验证 ${first.text} 指令及参数；源码已保留，不能据此确认可编译或可仿真`, 'fanuc.unsupported-instruction', 'warning'); cursor.index = row.length;
        }
      }
      if (blocks.length > 10) throw new SyntaxFault(first, 'IF/FOR 总嵌套层数不能超过 10（Basic /09 §4.7.5）');
      cursor.end();
    } catch (error) { if (error instanceof SyntaxFault) diagnostic(error.token, error.message); else throw error; }
  }
  for (const block of blocks) diagnostic(block.token, `${block.kind} 缺少 ${block.kind === 'IF' ? 'ENDIF' : 'ENDFOR'}`, 'fanuc.unclosed-block');
  for (const ref of labelUses) {
    if (ref.index !== null && !labels.has(ref.index)) diagnostic({ ...tokens[0], ...ref, text: '', upper: '' }, `未找到 LBL[${ref.index}] 定义`, 'fanuc.missing-label');
  }
  const positions = sections.find(section => section.name === '/POS');
  if (positions) parsePositions(positions.tokens, document, diagnostic, options);
  const positionIds = new Set(document.positions.map(position => position.index));
  for (const ref of positionUses) if (ref.index !== null && !positionIds.has(ref.index)) diagnostic({ ...tokens[0], ...ref, text: '', upper: '' }, `未定义 P[${ref.index}]；位置可能尚未示教`, 'fanuc.missing-position', 'warning');
  return document;
}

/** Match a complete literal without joining whitespace-separated or partly edited tokens. */
function takeFrameTemplateParameter(cursor: Cursor, frame: 'UF' | 'UT'): FanucToken | undefined {
  const expected = ['{', '{', frame, '}', '}'];
  const tokens = cursor.tokens.slice(cursor.index, cursor.index + expected.length);
  if (tokens.length !== expected.length || tokens.some((token, index) => token.text !== expected[index] || index > 0 && tokens[index - 1].to !== token.from)) return undefined;
  cursor.index += tokens.length;
  const text = `{{${frame}}}`;
  return { ...tokens[0], to: tokens.at(-1)!.to, text, upper: text };
}

/** Shared ordinary/pallet position payload; template options are passed only by /POS parsing. */
export function parseFanucPositionGroups(cursor: Cursor, diagnostic: (token: FanucToken | undefined, message: string, code?: string, severity?: SourceDiagnostic['severity']) => void, options: FanucParseOptions = {}): FanucGroup[] {
  const groups: FanucGroup[] = [], tokens = cursor.tokens;
      while (!cursor.done && !cursor.is('}')) {
        const groupToken = cursor.take(), match = /^GP(\d+)$/.exec(groupToken.upper);
        if (!match || Number(match[1]) < 1 || Number(match[1]) > 8) throw new SyntaxFault(groupToken, '位置需要 GP1 至 GP8 运动组');
        cursor.expect(':');
        if (groups.some(group => group.number === Number(match[1]))) diagnostic(groupToken, '位置重复定义同一运动组', 'fanuc.duplicate-group');
        const axes: Record<string, { value: number | null; unit: string }> = {};
        let config = false; const fields = new Set<string>(), unresolvedFrames: ('UF' | 'UT')[] = [];
        while (!cursor.done && !cursor.is('}') && !/^GP\d+$/.test(cursor.token.upper)) {
          const field = cursor.take();
          if (fields.has(field.upper)) diagnostic(field, `重复位置字段 ${field.text}`, 'fanuc.duplicate-coordinate'); fields.add(field.upper);
          if (['UF', 'UT', 'CONFIG'].includes(field.upper)) {
            cursor.expect(':');
            const frame = field.upper === 'UF' || field.upper === 'UT' ? field.upper : undefined;
            const templateParameter = options.templateParameters === true && frame ? takeFrameTemplateParameter(cursor, frame) : undefined;
            if (templateParameter && frame) {
              unresolvedFrames.push(frame);
              diagnostic(templateParameter, `${frame} 模板参数尚未配置，需替换为目标控制器的实际坐标系编号，不能直接用于控制器`, 'template-parameter', 'warning');
            } else {
              const value = cursor.take();
              if (field.upper === 'CONFIG') {
                config = true;
                if (value.kind === 'string' && /^["'][NF]\s+[LR]\s+[UD]\s+[TB]\s*,\s*[+-]?\d+\s*,\s*[+-]?\d+\s*,\s*[+-]?\d+["']$/i.test(value.text)) diagnostic(value, '额外左右构型位的 CONFIG ASCII 序列化尚无已核定样例；原文保留，不能从抽象构型图推断控制器格式', 'fanuc.unverified-configuration', 'warning');
                else if (value.kind !== 'string' || !/^["'][NF]\s+[UD]\s+[TB]\s*,\s*[+-]?\d+\s*,\s*[+-]?\d+\s*,\s*[+-]?\d+["']$/i.test(value.text)) diagnostic(value, 'CONFIG 需要 N/F、U/D、T/B 和三个整数转数', 'fanuc.configuration');
              } else if (value.upper !== 'F' && (value.kind !== 'number' || !Number.isSafeInteger(Number(value.text)) || Number(value.text) < 0)) diagnostic(value, 'UF/UT 需要非负整数或浮动标记 F', 'fanuc.frame-index');
            }
          } else if (/^(?:[XYZWPR]|J[1-9]|E[1-9])$/.test(field.upper)) {
            cursor.expect('='); const sign = cursor.eat('-') ? -1 : (cursor.eat('+'), 1);
            let value: number | null;
            if (cursor.eat('*')) { while (cursor.eat('*')) { /* uninitialized value */ } value = null; diagnostic(field, '未初始化的位置分量不能用于运动', 'fanuc.uninitialized-position', 'warning'); }
            else { const number = cursor.take(); value = sign * Number(number.text); if (number.kind !== 'number' || !Number.isFinite(value)) throw new SyntaxFault(number, '位置分量需要有限数值'); }
            const unit = cursor.take(); if (!['MM', 'DEG'].includes(unit.upper)) throw new SyntaxFault(unit, '位置单位必须是 mm 或 deg');
            if (/^[XYZ]$/.test(field.upper) && unit.upper !== 'MM' || /^[WPR]$/.test(field.upper) && unit.upper !== 'DEG') diagnostic(unit, '笛卡尔位置或姿态单位不匹配', 'fanuc.position-unit');
            axes[field.upper] = { value, unit: unit.upper.toLowerCase() };
          } else throw new SyntaxFault(field, `尚未支持的位置字段 ${field.text}`);
          if (!cursor.is('}') && !/^GP\d+$/.test(cursor.token?.upper ?? '')) cursor.expect(',');
        }
        if (!fields.has('UF') || !fields.has('UT')) diagnostic(groupToken, '位置运动组必须指定 UF 和 UT', 'fanuc.position-frames');
        const cartesian = ['X', 'Y', 'Z', 'W', 'P', 'R'].filter(name => Object.hasOwn(axes, name));
        const joint = Object.keys(axes).filter(name => /^J\d+$/.test(name));
        if (cartesian.length && (cartesian.length !== 6 || joint.length || !config)) diagnostic(groupToken, '笛卡尔组必须包含 X/Y/Z/W/P/R 和 CONFIG，且不能混入 J 轴', 'fanuc.position-components');
        if (!cartesian.length && !joint.length) diagnostic(groupToken, '运动组没有有效笛卡尔或关节坐标', 'fanuc.position-components');
        if (joint.length && config) diagnostic(groupToken, '关节形式位置不使用笛卡尔 CONFIG 字段', 'fanuc.position-components');
        groups.push({ number: Number(match[1]), from: groupToken.from, to: tokens[cursor.index - 1]?.to ?? groupToken.to, representation: cartesian.length ? 'cartesian' : joint.length ? 'joint' : 'unknown', axes, ...(unresolvedFrames.length ? { unresolvedFrames } : {}) });
      }
  return groups;
}

function parsePositions(tokens: readonly FanucToken[], document: FanucDocument, diagnostic: (token: FanucToken | undefined, message: string, code?: string, severity?: SourceDiagnostic['severity']) => void, options: FanucParseOptions) {
  const cursor = new Cursor(tokens), ids = new Set<number>();
  while (!cursor.done) {
    const startIndex = cursor.index, first = cursor.token;
    try {
      const ref = reference(cursor); if (ref.name !== 'P' || ref.index === null) throw new SyntaxFault(first, '/POS 中需要固定 P[index] 定义');
      if (ids.has(ref.index)) diagnostic(first, `P[${ref.index}] 重复定义`, 'fanuc.duplicate-position'); ids.add(ref.index);
      cursor.expect('{');
      const position: FanucPosition = { index: ref.index, from: ref.from, to: ref.to, groups: [] };
      position.groups = parseFanucPositionGroups(cursor, diagnostic, options);
      if (!position.groups.length) throw new SyntaxFault(cursor.token, 'P 位置缺少运动组');
      const close = cursor.token; cursor.expect('}'); cursor.expect(';'); position.to = close!.to; document.positions.push(position);
      document.symbols.push({ name: `P[${position.index}]`, kind: 'variable', from: position.from, to: position.to, description: `${position.groups.length} 个运动组` });
      document.folds.push({ from: first.to, to: close!.from });
    } catch (error) {
      if (!(error instanceof SyntaxFault)) throw error;
      diagnostic(error.token, error.message);
      while (!cursor.done && !cursor.eat(';')) cursor.take();
    }
    if (cursor.index <= startIndex) cursor.index++;
  }
}
