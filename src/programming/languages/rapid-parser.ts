import type { LanguageAnalysis, SourceDiagnostic, SourceSymbol } from '../types';

export type RapidVersion = '6.16' | '7.21';
export const rapidReservedWords = ('ALIAS AND BACKWARD BREAK CASE CONNECT CONST CONTINUE DEFAULT DIV DO ELSE ELSEIF ENDFOR ENDFUNC ENDIF ENDMODULE ENDPROC ENDRECORD ENDTEST ENDTRAP ENDWHILE ERROR EXIT FALSE FOR FROM FUNC GOTO IF INOUT LOCAL MOD MODULE NOSTEPIN NOT NOVIEW OR PERS PROC RAISE READONLY RECORD RETRY RETURN STEP SYSMODULE TEST THEN TO TRAP TRUE TRYNEXT UNDO VAR VIEWONLY WHILE WITH XOR').split(' ');
const reserved = new Set(rapidReservedWords);
const placeholders = new Set(['TDN', 'DDN', 'RDN', 'PAR', 'ALT', 'DIM', 'SMT', 'VAR', 'EIT', 'CSE', 'EXP', 'ARG', 'ID']);
const letter = /[A-Za-zÀ-ÖØ-öø-ÿ]/;
const ident = /[A-Za-zÀ-ÖØ-öø-ÿ0-9_]/;
export interface RapidToken { text: string; upper: string; from: number; to: number; kind: 'word' | 'number' | 'string' | 'symbol' | 'placeholder' | 'eof' }
const MAX_SOURCE = 1_000_000;
const MAX_TOKENS = 150_000;

/** Offsets deliberately count UTF-16 code units, as CodeMirror does. */
export function tokenizeRapid(source: string): { tokens: RapidToken[]; diagnostics: SourceDiagnostic[] } {
  const tokens: RapidToken[] = [];
  const diagnostics: SourceDiagnostic[] = [];
  let at = 0;
  const error = (from: number, to: number, message: string) => { if (diagnostics.length < 200) diagnostics.push({ from, to, severity: 'error', code: 'rapid.lexical', message }); };
  const emit = (from: number, kind: RapidToken['kind']) => tokens.push({ text: source.slice(from, at), upper: source.slice(from, at).toUpperCase(), from, to: at, kind });
  while (at < source.length && tokens.length < MAX_TOKENS) {
    const from = at;
    const c = source[at];
    if (/\s/.test(c)) { at++; continue; }
    if (c === '!') { while (at < source.length && !/[\r\n]/.test(source[at])) at++; continue; }
    if (c === '"') {
      at++;
      let closed = false;
      while (at < source.length && !/[\r\n]/.test(source[at])) {
        if (source[at] === '"') {
          if (source[at + 1] === '"') { at += 2; continue; }
          at++; closed = true; break;
        }
        if (source[at] === '\\') {
          if (source[at + 1] === '\\') at += 2;
          else if (/^[0-9a-f]{2}$/i.test(source.slice(at + 1, at + 3))) at += 3;
          else { error(at, Math.min(at + 2, source.length), 'RAPID 字符串转义应为两个十六进制数字，或双反斜杠。'); at++; }
        } else at++;
      }
      if (!closed) error(from, at, '字符串缺少结束双引号，不能跨行。');
      emit(from, 'string'); continue;
    }
    if (letter.test(c)) {
      at++;
      while (at < source.length && ident.test(source[at])) at++;
      if (at - from > 32) error(from, at, 'RAPID 标识符最多 32 个字符。');
      emit(from, 'word'); continue;
    }
    if (/[0-9]/.test(c) || c === '.' && /[0-9]/.test(source[at + 1] ?? '')) {
      const rest = source.slice(at);
      const diagnosticCount = diagnostics.length;
      const radix = /^0([xobd])/i.exec(rest);
      if (radix) {
        const digit = { x: /[0-9a-f]/i, o: /[0-7]/, b: /[01]/, d: /[0-9]/ }[radix[1].toLowerCase()]!;
        at += 2; const start = at;
        while (at < source.length && digit.test(source[at])) at++;
        if (at === start) error(from, at, '进制前缀后缺少有效数字。');
        if (radix[1].toLowerCase() === 'd' && /[eE]/.test(source[at] ?? '')) {
          const exp = /^[eE][+-]?\d+/.exec(source.slice(at));
          if (exp) at += exp[0].length;
        }
      } else {
        const match = /^(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?/.exec(rest)!;
        at += match[0].length;
      }
      if (at < source.length && ident.test(source[at])) { while (at < source.length && ident.test(source[at])) at++; error(from, at, '数字包含无效位或指数。'); }
      const literal = source.slice(from, at);
      if (diagnostics.length === diagnosticCount && !Number.isFinite(Number(/^0d/i.test(literal) ? literal.slice(2) : literal))) {
        error(from, at, '数值字面量超出 IEEE 754 有限数值范围。');
      }
      emit(from, 'number'); continue;
    }
    if (c === '<') {
      const hole = /^<([A-Z]+)>/i.exec(source.slice(at));
      if (hole && placeholders.has(hole[1].toUpperCase())) { at += hole[0].length; emit(from, 'placeholder'); continue; }
    }
    const pair = source.slice(at, at + 2);
    if ([':=', '<>', '>=', '<='].includes(pair)) at += 2;
    else if ('{}()[],.=<>+-*/:;\\?%|'.includes(c)) at++;
    else { at++; error(from, at, `无法识别 RAPID 字符 ${JSON.stringify(c)}。`); }
    emit(from, 'symbol');
  }
  if (at < source.length) error(at, source.length, '文件超出 150000 个语法标记的分析上限。');
  tokens.push({ text: '', upper: '', from: source.length, to: source.length, kind: 'eof' });
  return { tokens, diagnostics };
}

class ParseError extends Error {}
const dataWords = new Set(['VAR', 'PERS', 'CONST', 'TASK']);
const handlerWords = ['BACKWARD', 'ERROR', 'UNDO'];
const blockEnds = new Set(['ENDMODULE', 'ENDPROC', 'ENDFUNC', 'ENDTRAP', 'ENDRECORD', 'ENDIF', 'ENDFOR', 'ENDWHILE', 'ENDTEST', 'ELSE', 'ELSEIF', 'CASE', 'DEFAULT', ...handlerWords]);

/** Recursive-descent syntax validator for the referenced RAPID kernel, not a controller compiler. */
class RapidParser {
  private at = 0;
  private depth = 0;
  private last: RapidToken;
  readonly analysis: LanguageAnalysis;
  constructor(private readonly source: string, private readonly tokens: RapidToken[], diagnostics: SourceDiagnostic[], private readonly version: RapidVersion) {
    this.last = tokens[0]; this.analysis = { diagnostics, symbols: [], folds: [] };
  }
  private get token(): RapidToken { return this.tokens[this.at]; }
  private get word(): string { return this.token.upper; }
  private is(value: string): boolean { return this.word === value; }
  private take(): RapidToken { const token = this.token; if (token.kind !== 'eof') this.at++; this.last = token; return token; }
  private eat(value: string): boolean { if (!this.is(value)) return false; this.take(); return true; }
  private fail(message: string, token = this.token): never {
    if (this.analysis.diagnostics.length < 200) this.analysis.diagnostics.push({ from: token.from, to: token.to, severity: 'error', code: 'rapid.syntax', message });
    throw new ParseError(message);
  }
  private expect(value: string): RapidToken { if (!this.is(value)) this.fail(`此处需要 ${value}，实际为 ${this.token.text || '文件结尾'}。`); return this.take(); }
  private identifier(): RapidToken {
    if (this.is('<ID>')) return this.hole('<ID>');
    const isReserved = reserved.has(this.word) && !(this.version === '6.16' && ['BREAK', 'CONTINUE'].includes(this.word));
    if (this.token.kind !== 'word' || isReserved) this.fail('此处需要非保留字的 RAPID 标识符。');
    return this.take();
  }
  private hole(value: string): RapidToken {
    const token = this.expect(value);
    if (this.analysis.diagnostics.length < 200) this.analysis.diagnostics.push({ from: token.from, to: token.to, severity: 'warning', code: 'rapid.placeholder', message: '编辑占位符尚未填写，不能据此确认程序可执行。' });
    return token;
  }
  private symbol(token: RapidToken, kind: SourceSymbol['kind'], signature?: string): void {
    this.analysis.symbols.push({ name: token.text, kind, from: token.from, to: token.to, signature });
  }
  private fold(start: RapidToken, end: RapidToken): void { if (this.source.slice(start.to, end.from).includes('\n')) this.analysis.folds.push({ from: start.to, to: end.from }); }
  private nested<T>(run: () => T): T {
    if (this.depth >= 128) this.fail('语法嵌套超过 128 层。');
    this.depth++;
    try { return run(); } finally { this.depth--; }
  }
  private recover(before: number): void {
    if (this.at === before && this.token.kind !== 'eof') this.take();
    while (this.token.kind !== 'eof' && !blockEnds.has(this.word) && !this.is(';') && !this.is('MODULE')) this.take();
    this.eat(';');
  }
  private attempt(run: () => void): void { const before = this.at; try { run(); } catch (error) { if (!(error instanceof ParseError)) throw error; this.recover(before); } }
  parse(): LanguageAnalysis {
    while (this.token.kind !== 'eof') this.attempt(() => this.module());
    return this.analysis;
  }
  private module(): void {
    const start = this.expect('MODULE'); const name = this.identifier(); this.symbol(name, 'routine', `MODULE ${name.text}`);
    if (this.eat('(')) {
      do { if (!['SYSMODULE', 'NOVIEW', 'NOSTEPIN', 'VIEWONLY', 'READONLY'].includes(this.word)) this.fail('无效模块属性。'); this.take(); } while (this.eat(','));
      this.expect(')');
    }
    let phase = 0;
    while (!this.is('ENDMODULE') && this.token.kind !== 'eof') this.attempt(() => {
      const local = this.eat('LOCAL');
      if (this.is('RECORD') || this.is('ALIAS') || this.is('<TDN>')) {
        if (phase > 0) this.fail('类型定义应位于模块的数据与例程声明之前。');
        this.typeDefinition();
      } else if (dataWords.has(this.word) || this.is('<DDN>')) {
        if (phase > 1) this.fail('模块数据声明应位于例程声明之前。');
        phase = 1; this.dataDeclaration(local);
      } else { phase = 2; this.routine(); }
    });
    const end = this.expect('ENDMODULE'); this.fold(start, end);
  }
  private typeDefinition(): void {
    if (this.is('<TDN>')) { this.hole('<TDN>'); return; }
    if (this.eat('ALIAS')) { const type = this.identifier(); const name = this.identifier(); this.symbol(name, 'type', `ALIAS ${type.text} ${name.text}`); this.expect(';'); return; }
    const start = this.expect('RECORD'); const name = this.identifier(); this.symbol(name, 'type', `RECORD ${name.text}`);
    let count = 0;
    while (!this.is('ENDRECORD') && this.token.kind !== 'eof') { this.identifier(); this.identifier(); this.expect(';'); count++; }
    if (!count) this.fail('RECORD 至少需要一个成员。');
    const end = this.expect('ENDRECORD'); this.fold(start, end);
  }
  private dataDeclaration(local = false): void {
    if (this.is('<DDN>')) { this.hole('<DDN>'); return; }
    const task = this.eat('TASK');
    if (local && task) this.fail('LOCAL 与 TASK 不能同时修饰数据声明。');
    if (!['VAR', 'PERS', 'CONST'].includes(this.word) || task && this.is('CONST')) this.fail('此处需要 VAR、PERS，或非 TASK 的 CONST 声明。');
    const kind = this.take().upper; const type = this.identifier(); const name = this.identifier();
    this.symbol(name, kind === 'CONST' ? 'constant' : 'variable', `${kind} ${type.text} ${name.text}`);
    if (this.eat('{')) { this.expressions('}'); }
    if (this.eat(':=')) this.expression();
    else if (kind === 'CONST') this.fail('CONST 声明必须有初始值。');
    this.expect(';');
  }
  private routine(): void {
    if (this.is('<RDN>')) { this.hole('<RDN>'); return; }
    if (!['PROC', 'FUNC', 'TRAP'].includes(this.word)) this.fail('模块中应声明 PROC、FUNC 或 TRAP 例程。');
    const start = this.take(); const kind = start.upper;
    const result = kind === 'FUNC' ? this.identifier().text : '';
    const name = this.identifier(); this.symbol(name, kind === 'FUNC' ? 'function' : 'routine', `${kind} ${result ? result + ' ' : ''}${name.text}`);
    if (kind !== 'TRAP') { this.expect('('); this.parameters(); this.expect(')'); }
    while (dataWords.has(this.word) || this.is('LOCAL') || this.is('<DDN>')) { const local = this.eat('LOCAL'); this.dataDeclaration(local); }
    this.statements(new Set([...handlerWords, `END${kind}`]));
    if (this.eat('BACKWARD')) { if (kind !== 'PROC') this.fail('BACKWARD 只适用于 PROC。', this.last); this.statements(new Set(['ERROR', 'UNDO', `END${kind}`])); }
    if (this.eat('ERROR')) {
      if (this.eat('(')) {
        do { if (this.token.kind === 'number') this.take(); else this.identifier(); } while (this.eat(','));
        this.expect(')');
      }
      this.statements(new Set(['UNDO', `END${kind}`]));
    }
    if (this.eat('UNDO')) this.statements(new Set([`END${kind}`]));
    const end = this.expect(`END${kind}`); this.fold(start, end);
  }
  private parameters(): void {
    if (this.is(')')) return;
    this.parameterGroup();
    while (!this.is(')') && this.token.kind !== 'eof') {
      if (this.eat(',')) this.parameterGroup();
      else if (this.is('\\')) this.parameterGroup();
      else this.fail('形参之间需要逗号，或以反斜杠开始可选形参。');
    }
  }
  private parameterGroup(): void {
    if (this.is('<PAR>')) { this.hole('<PAR>'); return; }
    const optional = this.eat('\\'); this.parameter(optional);
    if (optional) while (this.eat('|')) this.parameter(true);
  }
  private parameter(optional: boolean): void {
    if (optional && this.is('<ALT>')) { this.hole('<ALT>'); return; }
    if (['VAR', 'PERS', 'INOUT'].includes(this.word)) this.take();
    const type = this.identifier(); const name = this.identifier(); this.symbol(name, 'variable', `${type.text} ${name.text}`);
    if (type.upper === 'SWITCH' && !optional) this.fail('switch 形参需要置于反斜杠可选参数组。', type);
    if (this.eat('{')) {
      if (this.is('<DIM>')) this.hole('<DIM>');
      else { this.expect('*'); while (this.eat(',')) this.expect('*'); }
      this.expect('}');
    }
  }
  private statements(stops: Set<string>): void {
    while (this.token.kind !== 'eof' && !stops.has(this.word)) this.attempt(() => this.nested(() => this.statement()));
  }
  private statement(simpleOnly = false): void {
    if (this.is('<SMT>')) { this.hole('<SMT>'); this.eat(';'); return; }
    const start = this.token;
    if (['IF', 'FOR', 'WHILE', 'TEST'].includes(this.word)) {
      if (simpleOnly) this.fail('紧凑 IF 后只能使用简单语句。');
      const block = this.take().upper;
      if (block === 'IF') {
        this.expression();
        if (!this.eat('THEN')) { this.statement(true); return; }
        this.statements(new Set(['ELSEIF', 'ELSE', 'ENDIF', '<EIT>']));
        while (this.is('ELSEIF') || this.is('<EIT>')) {
          if (this.is('<EIT>')) this.hole('<EIT>');
          else { this.take(); this.expression(); this.expect('THEN'); this.statements(new Set(['ELSEIF', 'ELSE', 'ENDIF', '<EIT>'])); }
        }
        if (this.eat('ELSE')) this.statements(new Set(['ENDIF']));
      } else if (block === 'FOR') {
        const variable = this.identifier(); this.symbol(variable, 'variable', 'FOR 循环变量');
        this.expect('FROM'); this.expression(); this.expect('TO'); this.expression(); if (this.eat('STEP')) this.expression();
        this.expect('DO'); this.statements(new Set(['ENDFOR']));
      } else if (block === 'WHILE') { this.expression(); this.expect('DO'); this.statements(new Set(['ENDWHILE'])); }
      else {
        this.expression();
        while (this.is('CASE') || this.is('<CSE>')) {
          if (this.is('<CSE>')) this.hole('<CSE>');
          else { this.take(); this.expression(); while (this.eat(',')) this.expression(); this.expect(':'); this.statements(new Set(['CASE', 'DEFAULT', 'ENDTEST', '<CSE>'])); }
        }
        if (this.eat('DEFAULT')) { this.expect(':'); this.statements(new Set(['ENDTEST'])); }
      }
      const end = this.expect(`END${block}`); this.fold(start, end); return;
    }
    if (this.eat('GOTO')) { this.identifier(); this.expect(';'); return; }
    if (this.eat('CONNECT')) { this.variable(); this.expect('WITH'); this.identifier(); this.expect(';'); return; }
    if (this.is('RETURN') || this.is('RAISE')) { this.take(); if (!this.is(';')) this.expression(); this.expect(';'); return; }
    if (['EXIT', 'RETRY', 'TRYNEXT', ...(this.version === '7.21' ? ['BREAK', 'CONTINUE'] : [])].includes(this.word)) {
      this.take(); this.expect(';'); return;
    }
    if (this.eat('%')) { this.expression(); this.expect('%'); this.arguments(';'); this.expect(';'); return; }
    if (this.is('<VAR>')) { this.variable(); this.expect(':='); this.expression(); this.expect(';'); return; }
    this.identifier();
    if (this.eat(':')) { if (simpleOnly) this.fail('紧凑 IF 后不允许标签。', start); return; }
    const accessed = this.accessors();
    if (this.eat(':=')) { this.expression(); this.expect(';'); return; }
    if (accessed) this.fail('数组或记录成员之后需要赋值运算符 :=。');
    this.arguments(';'); this.expect(';');
  }
  private variable(): void { if (this.is('<VAR>')) { this.hole('<VAR>'); return; } this.identifier(); this.accessors(); }
  private accessors(): boolean {
    let found = false;
    if (this.eat('{')) { found = true; this.expressions('}'); }
    while (this.eat('.')) { found = true; this.identifier(); }
    return found;
  }
  private arguments(end: string): void {
    if (this.is(end)) return;
    this.argument();
    while (!this.is(end) && this.token.kind !== 'eof') {
      if (this.eat(',')) this.argument();
      else if (this.is('\\')) this.argument();
      else this.fail('参数之间需要逗号；命名可选参数以反斜杠开头。');
    }
  }
  private argument(): void {
    if (this.is('<ARG>')) { this.hole('<ARG>'); return; }
    if (this.eat('\\')) {
      this.identifier();
      if (this.eat(':=')) this.expression();
      else if (this.eat('?')) this.variable();
    } else {
      if (this.token.kind === 'word' && this.tokens[this.at + 1]?.upper === ':=') { this.identifier(); this.take(); }
      this.expression();
    }
  }
  private expressions(end: string): void { this.expression(); while (this.eat(',')) this.expression(); this.expect(end); }
  private expression(): void { this.nested(() => { if (this.is('<EXP>')) { this.hole('<EXP>'); return; } this.eat('NOT'); this.logicalTerm(); while (this.is('OR') || this.is('XOR')) { this.take(); this.logicalTerm(); } }); }
  private logicalTerm(): void { this.relation(); while (this.eat('AND')) this.relation(); }
  private relation(): void { this.simple(); if (['<', '<=', '=', '>', '>=', '<>'].includes(this.word)) { this.take(); this.simple(); } }
  private simple(): void { if (this.is('+') || this.is('-')) this.take(); this.term(); while (this.is('+') || this.is('-')) { this.take(); this.term(); } }
  private term(): void { this.primary(); while (['*', '/', 'DIV', 'MOD'].includes(this.word)) { this.take(); this.primary(); } }
  private primary(): void {
    if (['string', 'number'].includes(this.token.kind) || this.is('TRUE') || this.is('FALSE')) { this.take(); return; }
    if (this.eat('(')) { this.expression(); this.expect(')'); return; }
    if (this.eat('[')) { this.expressions(']'); return; }
    if (this.is('<EXP>')) { this.hole('<EXP>'); return; }
    this.identifier();
    if (this.eat('(')) { this.arguments(')'); this.expect(')'); }
    else this.accessors();
  }
}

export function analyzeRapid(source: string, version: RapidVersion = '7.21'): LanguageAnalysis {
  if (source.length > MAX_SOURCE) return { symbols: [], folds: [], diagnostics: [{ from: 0, to: source.length, severity: 'error', code: 'rapid.size', message: 'RAPID 文件超出 1000000 字符的离线分析上限。' }] };
  const lexed = tokenizeRapid(source);
  return new RapidParser(source, lexed.tokens, lexed.diagnostics, version).parse();
}
