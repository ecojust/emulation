import type { LanguageAnalysis, SourceDiagnostic } from '../types';
import { informInstructionEntries } from './inform-catalog';
import { informSensorCommands, parseInformSearchOption, parseInformSensorCommand } from './inform-options';

interface Token { text: string; from: number; to: number; kind: 'word' | 'number' | 'string' | 'punct' }
interface Line { text: string; from: number; to: number }
const variable = /^(?:\$?(?:LBP|LEX|LPX|LP|LB|LI|LD|LR|LS|BP|EX|PX|B|I|D|R|S|P|FL|TF|TM|LG|MG|NG|G))(?:\d+)?$/i;
const position = /^(?:C|BC|EC)\d{5}$/i;
const known = new Set([...informInstructionEntries.map(entry => entry.name), ...informSensorCommands]);
const ioWords = new Set(['SIN#', 'SOUT#', 'IN#', 'OT#', 'IG#', 'OG#', 'RIN#', 'BAG#', 'OGH#', 'IGH#', 'IGU#', 'OGU#', 'MREG#', 'AG#', 'AO#', 'AI#', 'REG#']);
const binary = new Map([['OREXP', 1], ['OR', 1], ['ANDEXP', 2], ['AND', 2], ['XOR', 2], ['=', 3], ['<>', 3], ['<', 3], ['>', 3], ['<=', 3], ['>=', 3], ['+', 4], ['-', 4], ['*', 5], ['/', 5], ['MOD', 5]]);
const fullOperandCommands = new Set(['SET']);
const noOperands = new Set(['NOP', 'END', 'PAUSE', 'CWAIT', 'ADVINIT', 'ADVSTOP', 'CLS', 'ABORT', 'LATESTJOB', 'TOOLON', 'TOOLOF']);
const motion = new Set(['MOVJ', 'MOVL', 'MOVC', 'MOVS', 'IMOV']);
const motionTags: Record<string, readonly string[]> = {
  MOVJ: ['VJ', 'PL', 'SPDL', 'MT', 'MTE', 'ACC', 'DEC'],
  MOVL: ['V', 'VR', 'VE', 'VMAX', 'PL', 'CR', 'SPDL', 'PD', 'MTE', 'ACC', 'DEC'],
  MOVC: ['V', 'VR', 'VE', 'PL', 'SPDL', 'MTE', 'ACC', 'DEC', 'COORD'],
  MOVS: ['V', 'VR', 'VE', 'PL', 'SPDL', 'ACC', 'DEC'],
  IMOV: ['VJ', 'V', 'VR', 'VE', 'PL', 'ACC', 'DEC'],
};
const attachmentNames = new Set(['IF', 'UNTIL', 'ENWAIT', 'DIALSB', 'IFEXPRESS', 'FOREXPRESS', 'LOGICEXP', 'RELEXPR', 'TRAP', '+SFTON3D', '+SFT', 'CAPTURE', 'SRCH', 'ASRCH']);

/** Scanner retains UTF-16 offsets, never evaluates source or calls controller APIs. */
function tokenize(text: string, offset: number, report: (from: number, to: number, message: string) => void): Token[] {
  const tokens: Token[] = [];
  let i = 0;
  while (i < text.length && tokens.length < 20_000) {
    const start = i, char = text[i];
    if (/\s/.test(char)) { i++; continue; }
    if (char === "'") break;
    if (char === '"') {
      i++;
      while (i < text.length && text[i] !== '"') i++;
      if (i === text.length) report(offset + start, offset + i, '字符串缺少结束双引号。'); else i++;
      tokens.push({ text: text.slice(start, i), from: offset + start, to: offset + i, kind: 'string' }); continue;
    }
    const number = /^(?:\d+(?:\.\d*)?|\.\d+)(?:[Ee][+-]?\d+)?/.exec(text.slice(i));
    if (number) { if (!Number.isFinite(Number(number[0]))) report(offset + start, offset + start + number[0].length, '数值超出有限范围。'); i += number[0].length; tokens.push({ text: number[0], from: offset + start, to: offset + i, kind: 'number' }); continue; }
    const word = /^[\p{L}_$][\p{L}\p{N}_$]*(?:#)?/u.exec(text.slice(i));
    if (word) { i += word[0].length; tokens.push({ text: word[0].toUpperCase(), from: offset + start, to: offset + i, kind: 'word' }); continue; }
    const op = /^(?:<=|>=|<>|[+\-*/=<>(),\[\]:])/.exec(text.slice(i));
    if (op) { i += op[0].length; tokens.push({ text: op[0], from: offset + start, to: offset + i, kind: 'punct' }); continue; }
    i++;
    report(offset + start, offset + i, `无法识别的字符“${char}”。`);
  }
  if (i < text.length && tokens.length >= 20_000) report(offset + i, offset + text.length, '单行词法项过多，停止分析该行。');
  return tokens;
}

function expression(tokens: Token[], start = 0): number {
  let cursor = start, depth = 0;
  const primary = (): void => {
    if (++depth > 128) throw new Error('表达式嵌套超过 128 层分析上限。');
    try {
    const token = tokens[cursor++];
    if (!token) throw new Error('表达式缺少操作数。');
    if (['+', '-', 'NOT', 'NOTEXP'].includes(token.text)) { primary(); return; }
    if (token.text === '(') { parse(0); if (tokens[cursor++]?.text !== ')') throw new Error('表达式缺少右圆括号。'); return; }
    if (!['word', 'number', 'string'].includes(token.kind)) throw new Error('表达式操作数无效。');
    if (token.kind === 'word' && !token.text.endsWith('#') && !['ON', 'OFF', 'INVERT', 'SIN', 'COS', 'SQRT', 'ATAN'].includes(token.text) && !(variable.test(token.text) && (/\d$/.test(token.text) || tokens[cursor]?.text === '['))) throw new Error(`表达式中的 ${token.text} 不是有效变量或常量。`);
    if (token.kind === 'word' && tokens[cursor]?.text === '[') {
      cursor++; parse(0); if (tokens[cursor++]?.text !== ']') throw new Error('数组下标缺少右方括号。');
    } else if (token.kind === 'word' && (token.text.endsWith('#') || ['SIN', 'COS', 'SQRT', 'ATAN'].includes(token.text)) && tokens[cursor]?.text === '(') {
      cursor++; parse(0);
      while (tokens[cursor]?.text === ',') { cursor++; parse(0); }
      if (tokens[cursor++]?.text !== ')') throw new Error('索引或函数参数缺少右圆括号。');
    }
    } finally { depth--; }
  };
  const parse = (minimum: number): void => {
    primary();
    while (cursor < tokens.length) {
      const precedence = binary.get(tokens[cursor].text);
      if (precedence === undefined || precedence < minimum) break;
      cursor++; parse(precedence + 1);
    }
  };
  parse(0); return cursor;
}
function firstOperand(tokens: Token[]): number {
  if (!tokens.length) throw new Error('缺少变量或目标操作数。');
  if (tokens[0].text.endsWith('#') && tokens[1]?.text !== '(') throw new Error('带 # 的编号引用需要括号。');
  if (variable.test(tokens[0].text) && !/\d$/.test(tokens[0].text) && tokens[1]?.text !== '[') throw new Error('变量缺少编号或数组下标。');
  let end = 1;
  if (tokens[end]?.text === '[' || (tokens[0].text.endsWith('#') && tokens[end]?.text === '(')) {
    const close = tokens[end].text === '[' ? ']' : ')';
    end++;
    const after = expression(tokens, end);
    if (tokens[after]?.text !== close) throw new Error('变量下标或 I/O 编号未闭合。');
    numericExpression(tokens.slice(end, after));
    end = after + 1;
  }
  return end;
}
function motionExpression(tokens: Token[], start: number): number {
  const extension = tokens.findIndex((part, i) => i >= start && part.text === '+' && ['MOVJ', 'MOVL', 'MOVC', 'MOVS', 'IMOV', 'SFT', 'SFTON3D'].includes(tokens[i + 1]?.text));
  return expression(extension < 0 ? tokens : tokens.slice(0, extension), start);
}
function typedValue(tokens: Token[], families: readonly string[], constants = true): boolean {
  if (!tokens.length) return false;
  if (constants && ((tokens.length === 1 && tokens[0].kind === 'number') || (tokens.length === 2 && ['+', '-'].includes(tokens[0].text) && tokens[1].kind === 'number'))) return true;
  if (tokens[0].kind !== 'word' || !variable.test(tokens[0].text) || tokens[0].text.startsWith('$') || (!/\d$/.test(tokens[0].text) && tokens[1]?.text !== '[')) return false;
  return families.includes(tokens[0].text.replace(/^L/, '').replace(/\d+$/, '')) && firstOperand(tokens) === tokens.length;
}
function stringValue(tokens: Token[]): boolean {
  return (tokens.length === 1 && tokens[0].kind === 'string') || typedValue(tokens, ['S'], false);
}
function numericExpression(tokens: Token[]): void {
  if (expression(tokens) !== tokens.length) throw new Error('数值表达式含多余参数。');
  if (tokens.some(token => token.kind === 'string' || (token.kind === 'word' && !/^(?:B|LB|I|LI|D|LD|R|LR)(?:\d+)?$/.test(token.text)) || (token.kind === 'punct' && !['+', '-', '*', '/', '(', ')', '[', ']'].includes(token.text)))) throw new Error('数值表达式仅支持数值常量、B/I/D/R 变量和四则运算。');
}

function condition(tokens: Token[], depth = 0): void {
  if (depth > 128) throw new Error('条件嵌套超过 128 层分析上限。');
  let nesting = 0, split = -1, comparison = -1, comparisons = 0, outerClose = -1;
  for (let i = 0; i < tokens.length; i++) {
    const text = tokens[i].text;
    if (text === '(' || text === '[') nesting++;
    else if (text === ')' || text === ']') { nesting--; if (nesting === 0 && outerClose < 0) outerClose = i; }
    else if (nesting === 0 && ['ANDEXP', 'OREXP', 'AND', 'OR'].includes(text)) split = i;
    else if (nesting === 0 && ['=', '<>', '<', '>', '<=', '>='].includes(text)) { comparisons++; comparison = i; }
  }
  if (split >= 0) { condition(tokens.slice(0, split), depth + 1); condition(tokens.slice(split + 1), depth + 1); return; }
  if (comparisons > 1) throw new Error('一个比较条件只能包含一个关系运算符。');
  if (tokens[0]?.text === '(' && outerClose === tokens.length - 1) { condition(tokens.slice(1, -1), depth + 1); return; }
  if (comparison <= 0 || comparison === tokens.length - 1) throw new Error('条件需要完整的 左值 比较运算符 右值。');
  if (expression(tokens.slice(0, comparison)) !== comparison || expression(tokens.slice(comparison + 1)) !== tokens.length - comparison - 1) throw new Error('条件比较两侧含多余参数。');
  const left = tokens.slice(0, comparison), right = tokens.slice(comparison + 1), head = left[0].text, op = tokens[comparison].text;
  if (firstOperand(left) !== left.length) throw new Error('关系条件左侧需要单个变量或 I/O 引用。');
  const boolean = ['IN#', 'OT#', 'SIN#', 'SOUT#'].includes(head) || /^(?:FL|TF)\d+$/.test(head);
  const group = ['IG#', 'OG#', 'IGH#', 'OGH#'].includes(head);
  const userGroup = ['IGU#', 'OGU#'].includes(head);
  if (boolean) {
    if (op !== '=' || (!(right.length === 1 && ['ON', 'OFF'].includes(right[0].text)) && !typedValue(right, ['B'], false))) throw new Error('位条件需要 = ON/OFF 或 B/LB 变量。');
  } else if (group) {
    if (!['=', '<>'].includes(op) || !typedValue(right, ['B'])) throw new Error('组信号条件需要 =/<> 和字节值。');
  } else if (userGroup) {
    if (!typedValue(right, ['D'])) throw new Error('用户组条件需要 D/LD 变量或整数常量。');
  } else if (typedValue(left, ['S'], false)) {
    if (!stringValue(right)) throw new Error('字符串条件需要字符串常量或 S/LS 变量。');
  } else if (typedValue(left, ['B', 'I', 'D', 'R', 'TM'], false)) {
    if (!typedValue(right, ['B', 'I', 'D', 'R', 'TM'])) throw new Error('数值条件需要数值常量或变量。');
  } else throw new Error('此变量或引用不能作为关系条件左值。');
}

interface Atom { head: string; tokens: Token[]; tag?: string }
function atoms(tokens: Token[]): Atom[] {
  const output: Atom[] = []; let cursor = 0;
  const one = (): Atom => {
    const start = cursor, token = tokens[cursor++];
    if (!token) throw new Error('缺少操作数。');
    if (token.text === '+' || token.text === '-') { if (tokens[cursor++]?.kind !== 'number') throw new Error('正负号后需要数值。'); }
    else if (token.text === '(') {
      const stop = expression(tokens, cursor);
      if (tokens[stop]?.text !== ')') throw new Error('元素编号未闭合。');
      numericExpression(tokens.slice(cursor, stop)); cursor = stop + 1;
    } else if (token.kind === 'word') {
      cursor = start + firstOperand(tokens.slice(start));
    } else if (!['number', 'string'].includes(token.kind)) throw new Error('无效操作数。');
    return { head: token.text, tokens: tokens.slice(start, cursor) };
  };
  while (cursor < tokens.length) {
    if (tokens[cursor].kind === 'word' && tokens[cursor + 1]?.text === '=') {
      const tag = tokens[cursor].text; cursor += 2;
      output.push({ ...one(), tag });
    } else output.push(one());
  }
  return output;
}
/** Parameter shapes with independently written rules; controller types/options remain separate. */
function additionalSchema(opcode: string, tokens: Token[]): boolean {
  const supported = new Set(['ADD', 'SUB', 'MUL', 'DIV', 'AND', 'OR', 'XOR', 'SQRT', 'SIN', 'COS', 'ASC', 'CHR$', 'LEN', 'NOT', 'ATAN', 'MSG', 'INPUT', 'PRINT', 'CLEAR', 'SETTM', 'VAL', 'VAL2STR', 'MID$', 'CAT$', 'STRSTR', 'MULMAT', 'INVMAT', 'SETE', 'GETE', 'GETS', 'GETARG', 'GETNAME', 'GETPOS', 'SETFILE', 'GETFILE', 'SETREG', 'GETREG', 'GETPRM', 'SETPRM', 'PULSE', 'AOUT', 'ARATION', 'ARATIOF', 'ANTOUT', 'HAND', 'HSEN', 'SPEED', 'SFTON', 'SFTOF', 'SFTON3D', 'SFTOF3D', 'MSHIFT', 'CNVRT', 'MFRAME', 'REFP', 'SETUALM', 'DIALOG', 'ARCON', 'ARCOF', 'VWELD', 'AWELD', 'ARCSET', 'WVON', 'WVOF', 'ARCCTS', 'ARCCTE', 'GUNCL', 'SPOT', 'STROKE', 'STRWAIT']);
  if (!supported.has(opcode)) return false;
  const values = atoms(tokens);
  const fail = () => { throw new Error(`${opcode} 的操作数数量、类型或标签形状无效；请查看指令签名。`); };
  const count = (min: number, max = min) => { if (values.length < min || values.length > max) fail(); };
  const v = (atom: Atom | undefined) => Boolean(atom && !atom.tag && !atom.head.startsWith('$') && variable.test(atom.head) && (/\d$/.test(atom.head) || atom.tokens[1]?.text === '['));
  const family = (atom: Atom) => atom.head.replace(/^L/, '').replace(/\d+$/, '');
  const type = (atom: Atom | undefined, families: string[]) => Boolean(atom && v(atom) && families.includes(family(atom)));
  const literal = (atom: Atom | undefined) => Boolean(atom && !atom.tag && (atom.tokens[0].kind === 'number' || ['+', '-'].includes(atom.head)));
  const numeric = (atom: Atom | undefined) => type(atom, ['B', 'I', 'D', 'R']) || literal(atom);
  const p = (atom: Atom | undefined) => v(atom) && /^(?:P|LP|PX|LPX|BP|LBP|EX|LEX)/.test(atom!.head);
  const str = (atom: Atom | undefined) => Boolean(atom && !atom.tag && (atom.tokens[0].kind === 'string' || (v(atom) && /^(?:S|LS)/.test(atom.head))));
  const scalar = (atom: Atom | undefined) => Boolean(atom && !atom.tag && ((v(atom) && /^(?:LB|LI|LD|LR|B|I|D|R)/.test(atom.head)) || atom.tokens[0].kind === 'number' || ['+', '-', '('].includes(atom.head)));
  const ref = (atom: Atom | undefined, heads: string[]) => Boolean(atom && !atom.tag && heads.includes(atom.head) && atom.tokens[1]?.text === '(' && atom.tokens.at(-1)?.text === ')');
  const tags = (allowed: string[], required: string[] = []) => {
    const found = values.filter(value => value.tag).map(value => value.tag!);
    if (new Set(found).size !== found.length || found.some(tag => !allowed.includes(tag)) || required.some(tag => !found.includes(tag))) fail();
    if (values.some(value => value.tag && !scalar({ ...value, tag: undefined }))) fail();
  };
  const coordinate = (atom: Atom | undefined) => Boolean(atom && !atom.tag && (['BF', 'RF', 'TF'].includes(atom.head) || ref(atom, ['UF#'])));
  if (['ADD', 'SUB', 'MUL', 'DIV'].includes(opcode)) {
    count(2, 3); const positions = ['P', 'BP', 'EX'];
    if (type(values[0], positions)) {
      if (opcode === 'ADD' || opcode === 'SUB') { if (values.length !== 2 || !type(values[1], [family(values[0])])) fail(); }
      else if ((values.length === 3 && values[1].head !== '(') || (!type(values.at(-1), ['D']) && !literal(values.at(-1)))) fail();
    } else if (values.length !== 2 || !type(values[0], ['B', 'I', 'D', 'R']) || !numeric(values[1])) fail();
  } else if (['AND', 'OR', 'XOR', 'NOT'].includes(opcode)) {
    count(2); if (!type(values[0], ['B']) || (!type(values[1], ['B']) && !literal(values[1]))) fail();
  } else if (['SQRT', 'SIN', 'COS', 'ATAN'].includes(opcode)) {
    count(2); if (!type(values[0], ['R']) || (!type(values[1], ['R']) && !literal(values[1]))) fail();
  } else if (opcode === 'ASC' || opcode === 'LEN') {
    count(2); if (!type(values[0], opcode === 'ASC' ? ['B'] : ['B', 'I', 'D']) || !str(values[1])) fail();
  } else if (opcode === 'CHR$') {
    count(2); if (!type(values[0], ['S']) || (!type(values[1], ['B']) && !literal(values[1]))) fail();
  } else if (opcode === 'MSG') { count(1); if (!str(values[0])) fail(); }
  else if (opcode === 'INPUT') { count(1); if (!v(values[0]) || !/^(?:B|I|D|R)(?:\d|$)/.test(values[0].head)) fail(); }
  else if (opcode === 'PRINT') {
    count(0, 2); let cursor = values[0]?.tokens[0].kind === 'string' ? 1 : 0;
    if (values[cursor] && type(values[cursor], ['B', 'I', 'D', 'R', 'S'])) cursor++;
    if (cursor !== values.length) fail();
  } else if (opcode === 'CLEAR') {
    count(1, 2);
    if (type(values[0], ['B', 'I', 'D', 'R'])) { if (values.length !== 2 || (!literal(values[1]) && values[1].head !== 'ALL')) fail(); }
    else if (values.length !== 1 || (!['STACK', 'QUE', 'MPF', '$ERRNO'].includes(values[0].head) && !/^\$(?:B|I|D|R)(?:\d+)?$/.test(values[0].head))) fail();
  }
  else if (opcode === 'SETTM') { count(2); if (!/^TM\d+$/.test(values[0].head) || (!['TSTART', 'TSTOP', 'TRESET'].includes(values[1].head) && !/^TM\d+$/.test(values[1].head) && !scalar(values[1]))) fail(); }
  else if (opcode === 'VAL') { count(2, 3); if (!type(values[0], ['B', 'I', 'D', 'R']) || !str(values[1]) || (values[2] && values[2].tag !== 'RADIX')) fail(); tags(['RADIX']); }
  else if (opcode === 'VAL2STR') { count(2); if (!type(values[0], ['S']) || !type(values[1], ['B', 'I', 'D', 'R'])) fail(); }
  else if (opcode === 'MID$') { count(4); if (!str(values[0]) || !v(values[0]) || !str(values[1]) || !scalar(values[2]) || !scalar(values[3])) fail(); }
  else if (opcode === 'CAT$') { count(3); if (!type(values[0], ['S']) || !str(values[1]) || !str(values[2])) fail(); }
  else if (opcode === 'STRSTR') { count(3, 4); if (!type(values[0], ['B']) || !type(values[1], ['S']) || !str(values[2]) || (values[3] && !type(values[3], ['B']))) fail(); }
  else if (opcode === 'MULMAT' || opcode === 'INVMAT') { count(opcode === 'MULMAT' ? 3 : 2); if (!values.every(value => type(value, ['P']))) fail(); }
  else if (opcode === 'SETE') { count(2, 3); if (!type(values[0], ['P', 'BP', 'EX']) || (values.length === 2 ? !type(values[0], ['P']) || !ref(values[1], ['TL#']) : values[1].head !== '(' || (!type(values[2], ['D']) && !literal(values[2])))) fail(); }
  else if (opcode === 'GETE') { count(3); if (!type(values[0], ['D']) || !type(values[1], ['P', 'BP', 'EX']) || values[2].head !== '(') fail(); }
  else if (opcode === 'GETS') {
    count(2); const dest = family(values[0]), src = values[1].head;
    if (!v(values[0]) || (!(['B', 'I', 'D', 'R', 'S'].includes(dest) && src === '$RV') && !(dest === 'D' && src === '$EERNO') && !(src.startsWith('$') && src.slice(1).replace(/\d+$/, '') === dest && ['B', 'I', 'D', 'R', 'PX'].includes(dest)))) fail();
  } else if (opcode === 'GETARG') { count(2); if (!v(values[0]) || !values[0].head.startsWith('L') || !ref(values[1], ['IARG#'])) fail(); }
  else if (opcode === 'GETNAME') { count(2); if (!type(values[0], ['S']) || (!(v(values[1]) && !values[1].head.startsWith('L')) && !ref(values[1], ['IN#', 'OT#']))) fail(); }
  else if (opcode === 'GETPOS') { count(2, 4); if (!type(values[0], ['PX']) || !ref(values[1], ['STEP#']) || (values.length > 2 && (values.length !== 4 || values[2].head !== 'READCTP' || !type(values[3], ['D'])))) fail(); }
  else if (opcode === 'SETFILE' || opcode === 'GETFILE') {
    count(3); const file = opcode === 'SETFILE' ? values[0] : values[1], element = opcode === 'SETFILE' ? values[1] : values[2], value = opcode === 'SETFILE' ? values[2] : values[0];
    if (!ref(file, ['SPR#', 'UDC#', 'WEV#', 'PDF#', ...(opcode === 'GETFILE' ? ['AMF#'] : [])]) || element.head !== '(' || !(opcode === 'SETFILE' ? type(value, ['D']) || literal(value) : type(value, ['D']))) fail();
  } else if (opcode === 'SETREG' || opcode === 'GETREG') {
    count(2, 4); tags(['REGNUM']);
    let cursor = 0;
    if (opcode === 'GETREG' && !type(values[cursor++], ['I', 'D'])) fail();
    if (!ref(values[cursor++], ['MREG#'])) fail();
    if (['LOW', 'HIGH'].includes(values[cursor]?.head)) cursor++;
    if (opcode === 'SETREG') { if (!type(values[cursor], ['I', 'D']) && !literal(values[cursor])) fail(); cursor++; }
    if (values[cursor]?.tag === 'REGNUM') cursor++;
    if (cursor !== values.length) fail();
  } else if (opcode === 'GETPRM' || opcode === 'SETPRM') {
    count(3, 4); tags(['KIND', 'PRMNO', 'GRPNO'], ['KIND', 'PRMNO']);
    if (opcode === 'GETPRM' ? !type(values[0], ['D']) || values[1].tag !== 'KIND' || values[2].tag !== 'PRMNO' : values[0].tag !== 'KIND' || values[1].tag !== 'PRMNO' || !type(values.at(-1), ['D'])) fail();
    if (opcode === 'SETPRM' && values.length === 4 && values[2].tag !== 'GRPNO') fail();
    if (opcode === 'GETPRM' && values[3] && !['MIN', 'MAX'].includes(values[3].head) && values[3].tag !== 'GRPNO') fail();
  } else if (opcode === 'PULSE') {
    count(1, 3); tags(['T']); if (!ref(values[0], ['OT#', 'OG#', 'OGH#', 'OGU#'])) fail();
    const data = values.slice(1).filter(value => !value.tag);
    if (values[0].head === 'OT#' ? data.length !== 0 : data.length !== 1 || (!type(data[0], values[0].head === 'OGU#' ? ['D'] : ['B']) && !literal(data[0]))) fail();
    if (values.some((value, index) => value.tag && index !== values.length - 1)) fail();
  } else if (opcode === 'AOUT') {
    count(2); if (!ref(values[0], ['AO#']) || (!type(values[1], ['I']) && !literal(values[1]))) fail();
  } else if (opcode === 'ANTOUT') {
    count(2, 3); tags(['ANT', 'ANTDIS']);
    if (!ref(values[0], ['AT#', 'AG#']) || (values[0].head === 'AT#' ? !['ON', 'OFF'].includes(values[1].head) || Boolean(values[1].tag) : !type(values[1], ['B']) && !literal(values[1]))) fail();
    if (values[2] && !values[2].tag) fail();
  } else if (opcode === 'ARATIOF') { count(1); if (!ref(values[0], ['AO#'])) fail(); }
  else if (opcode === 'ARATION') {
    count(1, 6); if (!ref(values[0], ['AO#'])) fail(); tags(['BV', 'V', 'OFV', 'DELAY']);
    let cursor = 1; for (const tag of ['BV', 'V', 'OFV']) if (values[cursor]?.tag === tag) cursor++;
    if (values[cursor]?.head === 'FINE') { cursor++; if (values[cursor++]?.tag !== 'DELAY') fail(); }
    if (cursor !== values.length) fail();
  } else if (opcode === 'HAND' || opcode === 'HSEN') {
    count(2, 3); if (!literal(values[0]) || (!['ON', 'OFF'].includes(values[1].head) && !type(values[1], ['B']))) fail();
    if (values[2] && (opcode === 'HAND' ? values[2].head !== 'ALL' : values[2].head !== 'FOREVER' && !(values[2].tag === 'T' && (literal({ ...values[2], tag: undefined }) || type({ ...values[2], tag: undefined }, ['I']))))) fail();
  }
  else if (opcode === 'SPEED') {
    count(1, 4); const order = ['VJ', 'V', 'VR', 'VE']; tags(order);
    if (values.some((value, index) => !value.tag || (index > 0 && order.indexOf(value.tag) <= order.indexOf(values[index - 1].tag!)))) fail();
  }
  else if (opcode === 'SFTON' || opcode === 'SFTON3D') {
    count(1, opcode === 'SFTON3D' ? 2 : 5); let cursor = /^RB[1-8]$/.test(values[0].head) ? 1 : 0;
    const families: string[] = [];
    while (p(values[cursor])) {
      const family = values[cursor].head.replace(/^L/, '').replace(/\d+$/, '');
      const order = ['P', 'BP', 'EX'];
      if (!order.includes(family) || (families.length && (families[0] !== 'P' || order.indexOf(family) <= order.indexOf(families.at(-1)!))) || (opcode === 'SFTON3D' && family !== 'P')) fail();
      families.push(family); cursor++;
    }
    if (!families.length) fail();
    if (opcode === 'SFTON' && coordinate(values[cursor])) { if (families[0] !== 'P') fail(); cursor++; }
    if (cursor !== values.length) fail();
  } else if (opcode === 'SFTOF' || opcode === 'SFTOF3D') {
    count(0, 1); if (values[0] && !/^RB[1-8]$/.test(values[0].head) && !(opcode === 'SFTOF' && values[0].head === 'ST')) fail();
  } else if (opcode === 'MSHIFT') { count(4); if (![values[0], values[2], values[3]].every(value => p(value) && /^(?:PX|LPX)/.test(value.head)) || (!coordinate(values[1]) && values[1].head !== 'MTF')) fail(); }
  else if (opcode === 'CNVRT') { count(3, 4); if (!type(values[0], ['PX']) || !type(values[1], ['PX']) || (!coordinate(values[2]) && values[2].head !== 'MTF') || (values[3] && !ref(values[3], ['TL#']))) fail(); }
  else if (opcode === 'MFRAME') {
    count(3, 4); if (!ref(values[0], ['UF#']) && !type(values[0], ['P'])) fail();
    if (type(values[1], ['PX'])) { if (values.length !== 4 || !values.slice(1).every(value => type(value, ['PX']))) fail(); }
    else if (!type(values[1], ['P']) || (values.length === 4 && !type(values[2], ['BP'])) || !(values.at(-1)!.head === 'BF' || ref(values.at(-1), ['UUF#']))) fail();
  } else if (opcode === 'REFP') {
    count(2, 5); let cursor = /^(?:RB[1-8]|ST(?:[1-9]|1\d|2[0-4]))$/.test(values[1].head) ? 2 : 1;
    if (!literal(values[0]) || cursor === values.length || !values.slice(cursor).every(value => position.test(value.head) || type(value, ['P', 'BP', 'EX']))) fail();
  }
  else if (opcode === 'SETUALM') {
    count(2, 4); tags(['SMODE']);
    const data = values.filter(value => !value.tag);
    if (!scalar(data[0]) || !scalar(data.at(-1)) || ![2, 3].includes(data.length) || (data.length === 3 && !str(data[1]))) fail();
    if (values.some((value, index) => value.tag && index !== values.length - 1)) fail();
  } else if (opcode === 'DIALOG') {
    const byte = (atom: Atom | undefined, local = true) => v(atom) && (local ? /^(?:B|LB)(?:\d|$)/ : /^B(?:\d|$)/).test(atom!.head);
    if (!byte(values[0], false)) fail();
    let cursor = 1, buttons = 0;
    while (cursor < values.length) {
      if (values[cursor++].head !== 'DIALSB' || ++buttons > 5) fail();
      if (!byte(values[cursor++]) || !byte(values[cursor++]) || !str(values[cursor]) || !v(values[cursor++])) fail();
      if (values[cursor]?.head !== 'DIALSB' && cursor < values.length) {
        if (!byte(values[cursor++]) || !byte(values[cursor++]) || !str(values[cursor]) || !v(values[cursor++])) fail();
      } else if (buttons === 1) fail();
    }
    if (!buttons) fail();
  } else if (['VWELD', 'AWELD'].includes(opcode)) {
    count(1, 2); if (values.length === 2 && !/^WELD[1-8]$/.test(values[0].head)) fail();
    if (!scalar(values.at(-1))) fail();
  } else if (opcode === 'WVOF') {
    count(0, 1); if (values[0] && !/^RB[1-8]$/.test(values[0].head)) fail();
  } else if (opcode === 'WVON') {
    count(1, 6); tags(['AMP', 'FREQ', 'ANGL', 'DIR']);
    let cursor = /^RB[1-8]$/.test(values[0]?.head) ? 1 : 0;
    if (ref(values[cursor], ['WEV#'])) {
      cursor++; if (values[cursor]?.tag === 'DIR') cursor++;
    } else {
      if (values[cursor++]?.tag !== 'AMP' || values[cursor++]?.tag !== 'FREQ') fail();
      if (values[cursor]?.tag === 'ANGL') cursor++;
      if (values[cursor]?.tag === 'DIR') cursor++;
    }
    if (cursor !== values.length) fail();
  } else if (['ARCON', 'ARCOF', 'ARCCTS', 'ARCCTE', 'ARCSET'].includes(opcode)) {
    let cursor = /^WELD[1-8]$/.test(values[0]?.head) ? 1 : 0;
    const isStart = opcode === 'ARCON', isEnd = opcode === 'ARCOF', ramp = opcode === 'ARCCTS' || opcode === 'ARCCTE';
    const allowed = isStart ? ['AC', 'AV', 'AVP', 'T', 'V'] : isEnd ? ['AC', 'AV', 'AVP', 'T'] : ramp ? ['AC', 'AV', 'AVP', 'AN3', 'AN4', 'DIS'] : ['AC', 'AV', 'AVP', 'V', 'AN3', 'AN4', 'ACOND'];
    tags(allowed);
    const file = ref(values[cursor], [isEnd ? 'AEF#' : 'ASF#']);
    if (file) {
      cursor++;
      if (opcode === 'ARCSET' && values[cursor]?.tag === 'ACOND') cursor++;
    } else {
      const current = values[cursor]?.tag === 'AC'; if (current) cursor++;
      const voltage = ['AV', 'AVP'].includes(values[cursor]?.tag ?? ''); if (voltage) cursor++;
      if ((isStart || isEnd) && current && !voltage) fail();
      if ((isStart || isEnd) && voltage && !current) fail();
      const following = isStart || isEnd ? ['T', ...(isStart ? ['V'] : [])] : ramp ? ['AN3', 'AN4', 'DIS'] : ['V', 'AN3', 'AN4'];
      for (const tag of following) if (values[cursor]?.tag === tag) cursor++;
    }
    if (isStart && !file && values[cursor]?.head === 'RETRY') { cursor++; if (values[cursor]?.head === 'REPLAY') cursor++; }
    if (isEnd && !file && values[cursor]?.head === 'ANTSTK') cursor++;
    if (cursor !== values.length) fail();
  } else if (opcode === 'GUNCL') {
    count(2, 3); tags(['T', 'ATT'], ['T']);
    if (!ref(values[0], ['GUN#']) || values[1].tag !== 'T' || (values[2] && values[2].tag !== 'ATT')) fail();
  } else if (opcode === 'SPOT') {
    count(3, 7); let cursor = 0;
    for (let gun = 0; gun < 2; gun++) {
      if (!ref(values[cursor++], ['GUN#']) || values[cursor]?.tag !== 'MODE' || !scalar({ ...values[cursor++], tag: undefined }) || values[cursor]?.tag !== 'WTM' || !scalar({ ...values[cursor++], tag: undefined })) fail();
      if (!ref(values[cursor], ['GUN#'])) break;
    }
    if (values[cursor]?.tag === 'ATT' && scalar({ ...values[cursor], tag: undefined })) cursor++;
    if (cursor !== values.length) fail();
  } else if (opcode === 'STROKE' || opcode === 'STRWAIT') {
    count(2); if (!ref(values[0], ['GUN#']) || !['LONG', 'SHORT'].includes(values[1].head) || values[1].tag) fail();
  }
  return true;
}

export function analyzeInform(source: string, _fileName?: string): LanguageAnalysis {
  const result: LanguageAnalysis = { diagnostics: [], symbols: [], folds: [] };
  const add = (from: number, to: number, message: string, severity: SourceDiagnostic['severity'] = 'error', code = 'inform-syntax') => {
    if (result.diagnostics.length >= 300) return;
    if (result.diagnostics.length === 299) { result.diagnostics.push({ from: 0, to: Math.min(1, source.length), severity: 'error', message: '诊断已达到 300 项上限；后续内容未完整分析。', code: 'analysis-limit' }); return; }
    result.diagnostics.push({ from: Math.max(0, Math.min(source.length, from)), to: Math.max(0, Math.min(source.length, Math.max(from, to))), severity, message, code });
  };
  if (source.length > 1_048_576) { add(0, 1, '源码超过 1 MB 分析上限。'); return result; }
  const lines: Line[] = [];
  let offset = 0;
  for (const raw of source.split('\n')) { const text = raw.endsWith('\r') ? raw.slice(0, -1) : raw; lines.push({ text, from: offset, to: offset + text.length }); offset += raw.length + 1; }
  let section: 'none' | 'job' | 'position' | 'instruction' = 'none';
  let sectionStart = 0, name = '', ended = false, instructions = 0, cartesianRecords = false;
  const sections = new Set<string>(), definitions = new Map<string, number>(), labels = new Map<string, number>();
  const references: { name: string; token: Token }[] = [], jumps: { name: string; token: Token }[] = [];
  let counts: number[] | null = null;
  const actualCounts = [0, 0, 0, 0, 0, 0];
  const blocks: { kind: string; from: number; body: number; variable?: string; finalBranch: boolean }[] = [];
  const validateExpression = (tokens: Token[], line: Line) => {
    try { if (expression(tokens) !== tokens.length) throw new Error('表达式含多余操作数或缺少运算符。'); }
    catch (error) { add(tokens[0]?.from ?? line.from, tokens.at(-1)?.to ?? line.to, (error as Error).message); }
  };
  const validateCondition = (tokens: Token[], line: Line) => {
    try { condition(tokens); } catch (error) { add(tokens[0]?.from ?? line.from, tokens.at(-1)?.to ?? line.to, (error as Error).message); }
  };
  const close = (expected: string, line: Line, variableName?: string) => {
    const top = blocks.at(-1);
    if (!top || top.kind !== expected) { add(line.from, line.to, `结束标记没有匹配的 ${expected} 块。`); return; }
    if (expected === 'FOR' && variableName && top.variable !== variableName) add(line.from, line.to, 'NEXT 的循环变量与 FOR 不一致。');
    blocks.pop(); if (line.from > top.body) result.folds.push({ from: top.body, to: line.from });
  };
  for (const line of lines) {
    const text = line.text.trim();
    if (!text) continue;
    if (text.startsWith("'")) {
      if ([...text.slice(1)].length > 32) add(line.from, line.to, 'INFORM 注释最多 32 个字符；控制器编码字节数仍需核对。');
      continue;
    }
    if (text.startsWith('/')) {
      if (section === 'instruction' && instructions && !/^\/\/\//.test(text)) add(line.from, line.to, '指令区之后不能重新开始文件分区。');
      const header = /^(\/{1,3})([A-Z][A-Z0-9]*)(?:\s+(.*))?$/i.exec(text);
      if (!header) { add(line.from, line.to, '无效的 JBI 文件头格式。'); continue; }
      const [, slashes, keyRaw, rest = ''] = header, key = keyRaw.toUpperCase();
      if (slashes === '/' && key === 'JOB') {
        if (section !== 'none' || sections.has('JOB')) add(line.from, line.to, '/JOB 必须且只能出现在文件开头。');
        if (rest) add(line.from, line.to, '/JOB 不接受参数。');
        section = 'job'; sections.add('JOB'); continue;
      }
      if (slashes === '//' && key === 'NAME') {
        if (section !== 'job' || sections.has('NAME')) add(line.from, line.to, '//NAME 应在 /JOB 后出现一次。');
        name = rest.trim(); if (!name || name.length > 32) add(line.from, line.to, '作业名称须为 1 至 32 个字符；实际文件编码长度仍需控制器复核。');
        sections.add('NAME'); result.symbols.push({ name, kind: 'routine', from: line.from, to: line.to, signature: `JOB:${name}` }); continue;
      }
      if (slashes === '//' && (key === 'POS' || key === 'INST')) {
        if (!sections.has('NAME') || sections.has(key) || (key === 'POS' && section === 'instruction')) add(line.from, line.to, `//${key} 分区顺序或重复声明无效。`);
        if (rest) add(line.from, line.to, `//${key} 不接受参数。`);
        if (line.from > sectionStart && section !== 'none') result.folds.push({ from: sectionStart, to: line.from });
        section = key === 'POS' ? 'position' : 'instruction'; sectionStart = line.to; sections.add(key); continue;
      }
      if (slashes !== '///' || section === 'none' || section === 'job') { add(line.from, line.to, '此头部字段不在有效分区中。'); continue; }
      if (section === 'instruction' && instructions) add(line.from, line.to, '指令头字段必须位于第一条可执行指令之前。');
      const positionFields = new Set(['NPOS', 'USER', 'TOOL', 'POSTYPE', 'PULSE', 'RECTAN', 'RCONF']);
      if (positionFields.has(key) && section !== 'position') add(line.from, line.to, `${key} 必须位于 //POS 位置区。`);
      if ((['DATE', 'ATTR', 'COMM', 'FRAME', 'LVARS'].includes(key) || /^GROUP[1-4]$/.test(key)) && section !== 'instruction') add(line.from, line.to, `${key} 必须位于 //INST 头部。`);
      const numericList = (length: number) => {
        const values = rest.split(',').map(value => value.trim());
        if (values.length !== length || values.some(value => !/^\d+$/.test(value) || !Number.isSafeInteger(Number(value)))) { add(line.from, line.to, `${key} 需要 ${length} 个非负整数。`); return null; }
        return values.map(Number);
      };
      if (key === 'NPOS') { if (section !== 'position' || counts) add(line.from, line.to, 'NPOS 必须在位置区出现一次。'); counts = numericList(6); }
      else if (key === 'LVARS') numericList(8);
      else if (key === 'RCONF') {
        const values = rest.split(',');
        if (![6, 24].includes(values.length) || values.some(value => !/^[+-]?\d+$/.test(value.trim()))) add(line.from, line.to, 'RCONF 需要整数配置列表（YRC1000 文档为 24 项）。');
        else if (values.length === 6) add(line.from, line.to, '六项 RCONF 为旧格式；当前 YRC1000 配置兼容性尚未核验。', 'warning', 'inform-dialect');
      } else if (key === 'TOOL' || key === 'USER') { if (!/^\d+$/.test(rest) || Number(rest) > 63 || (key === 'USER' && Number(rest) < 1)) add(line.from, line.to, `${key} 编号不在 ${key === 'USER' ? '1' : '0'}–63 范围内。`); }
      else if (key === 'POSTYPE') { if (!/^(PULSE|BASE|ROBOT|TOOL|USER|MTOOL)$/i.test(rest)) add(line.from, line.to, 'POSTYPE 需要有效的位置类型。'); }
      else if (key === 'PULSE' || key === 'RECTAN') { cartesianRecords = key === 'RECTAN'; if (rest) add(line.from, line.to, `${key} 字段不接受额外参数。`); }
      else if (/^GROUP[1-4]$/.test(key)) {
        const groups = rest.replace(/\s/g, '').split(',');
        if (groups.length > 3 || new Set(groups).size !== groups.length || groups.some(group => !/^(?:RB[1-8]|BS[1-8]|ST(?:[1-9]|1\d|2[0-4]))$/i.test(group))) add(line.from, line.to, 'GROUP 需要最多三个不重复的 RB/BS/ST 控制组。');
      } else if (key === 'DATE') {
        const date = /^(\d{4})\/(\d{1,2})\/(\d{1,2})\s+(\d{1,2}):(\d{2})$/.exec(rest);
        if (!date || +date[2] < 1 || +date[2] > 12 || +date[3] < 1 || +date[3] > 31 || +date[4] > 23 || +date[5] > 59) add(line.from, line.to, 'DATE 需要有效的 YYYY/MM/DD HH:mm。');
      } else if (key === 'COMM') {
        if ([...rest].length > 32) add(line.from, line.to, 'COMM 最多 32 个字符；编码字节数仍需控制器核对。');
      } else if (key === 'ATTR') {
        const attrs = rest.split(',').map(value => value.trim().toUpperCase());
        if (attrs.length > 16 || attrs.some(value => !/^[A-Z]+$/.test(value)) || new Set(attrs).size !== attrs.length) add(line.from, line.to, 'ATTR 需要最多 16 个不重复的属性名称。');
        else if (attrs.some(value => !['JD', 'DD', 'SC', 'RO', 'WO', 'RW', 'RJ'].includes(value))) add(line.from, line.to, 'ATTR 含官方公开头部列表以外的属性；需核对控制器版本。', 'warning', 'inform-dialect');
      } else if (key === 'FRAME') {
        if (!/^(?:BASE|ROBOT|USER(?:[1-9]|[1-5]\d|6[0-3]))$/i.test(rest)) add(line.from, line.to, 'FRAME 需要 BASE、ROBOT 或 USER1–USER63。');
      } else add(line.from, line.to, `头部字段 ${key} 尚未建模；请按控制器版本核对。`, 'warning', 'inform-unchecked');
      continue;
    }
    if (section === 'position') {
      const record = /^((C|BC|EC)\d{5}|(P|BP|EX)\d{3,4})\s*=\s*(.+)$/i.exec(text);
      if (!record) { add(line.from, line.to, '位置区需要 C/BC/EC/P/BP/EX 的数值记录。'); continue; }
      const identifier = record[1].toUpperCase(), values = record[4].split(',');
      if (values.some(value => !/^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[Ee][+-]?\d+)?$/.test(value.trim()) || !Number.isFinite(Number(value)))) add(line.from, line.to, '位置分量必须为有限数值。');
      if (!cartesianRecords && values.some(value => !Number.isInteger(Number(value)))) add(line.from, line.to, 'PULSE 位置分量必须为整数。');
      if (cartesianRecords && /^(?:C|P)\d/.test(identifier) && ![6, 7].includes(values.length)) add(line.from, line.to, '笛卡尔机器人位置需要 6 或 7 个分量。');
      const maximum = /^(?:BC|BP)/.test(identifier) ? 3 : /^(?:EC|EX)/.test(identifier) ? 6 : 7;
      if (values.length > maximum) add(line.from, line.to, `此位置记录最多 ${maximum} 个轴分量。`);
      if (/^(?:C|BC|EC)/.test(identifier) && Number(identifier.match(/\d+$/)![0]) > 9997) add(line.from, line.to, '示教位置记录编号范围为 00000–09997。');
      if (definitions.has(identifier)) add(line.from, line.to, `位置 ${identifier} 重复定义。`);
      definitions.set(identifier, line.from);
      const family = identifier.match(/^[A-Z]+/)![0], index = ['C', 'BC', 'EC', 'P', 'BP', 'EX'].indexOf(family); actualCounts[index]++;
      result.symbols.push({ name: identifier, kind: 'variable', from: line.from, to: line.to, description: '本作业位置记录' }); continue;
    }
    if (section !== 'instruction') { add(line.from, line.to, '指令需要放在 //INST 分区中。'); continue; }
    const tokens = tokenize(line.text, line.from, (from, to, message) => add(from, to, message));
    if (!tokens.length) continue;
    if (ended) { add(line.from, line.to, 'END 后不能再添加可执行指令。'); continue; }
    if (tokens[0].text === '*') {
      if (tokens.length !== 2 || !['word', 'number'].includes(tokens[1].kind) || [...tokens[1].text].length > 8) add(line.from, line.to, '标签格式应为 *名称，名称最多八个字符。');
      else { const label = tokens[1]; if (labels.has(label.text)) add(label.from, label.to, '标签重复定义。'); labels.set(label.text, label.from); result.symbols.push({ name: `*${label.text}`, kind: 'constant', from: label.from, to: label.to }); }
      continue;
    }
    const opcode = tokens[0].text, rest = tokens.slice(1); instructions++;
    if (blocks.length > 128) { add(line.from, line.to, '结构控制嵌套超过 128 层分析上限。', 'error', 'analysis-limit'); return result; }
    const conditionAt = rest.findIndex((token, index) => ['IF', 'IFEXPRESS', 'UNTIL'].includes(token.text) && rest[index - 1]?.text !== ':');
    const operands = conditionAt < 0 || motion.has(opcode) ? rest : rest.slice(0, conditionAt);
    if (conditionAt >= 0 && opcode !== 'CALL' && !motion.has(opcode)) {
      if (!['JUMP', 'RET', 'PAUSE', 'ABORT', 'PULSE'].includes(opcode) || (opcode === 'PULSE' && rest[conditionAt].text !== 'IFEXPRESS')) add(line.from, line.to, `${opcode} 不支持此条件附加项。`);
      const condition = rest.slice(conditionAt + 1), capture = condition.findIndex(token => token.text === 'CAPTURE');
      validateCondition(capture < 0 ? condition : condition.slice(0, capture), line);
      if (rest[conditionAt].text === 'UNTIL' && !motion.has(opcode)) add(line.from, line.to, 'UNTIL 只能附加在运动指令上。');
      if (capture >= 0) {
        const target = condition.slice(capture + 1);
        try {
          if (rest[conditionAt].text !== 'UNTIL' || !/^(?:PX|LPX)(?:\d+)?$/.test(target[0]?.text ?? '') || firstOperand(target) !== target.length) throw new Error('CAPTURE 只能在 UNTIL 条件后指定一个 PX/LPX 扩展位置变量。');
        } catch (error) { add(line.from, line.to, (error as Error).message); }
      }
    }
    // Delimiter integrity is checked even for technology-specific instructions.
    const delimiters: Token[] = [];
    for (const token of rest) {
      if (token.text === '[' || token.text === '(') delimiters.push(token);
      else if (token.text === ']' || token.text === ')') {
        const open = delimiters.pop();
        if (!open || (open.text === '[' ? ']' : ')') !== token.text) add(token.from, token.to, '括号不匹配。');
      }
    }
    for (const token of delimiters) add(token.from, token.to, '括号没有闭合。');
    if (opcode === 'IFTHENEXP' || opcode === 'WHILEEXP') {
      if (rest.some(token => ['AND', 'OR'].includes(token.text))) add(line.from, line.to, '外部结构条件使用 ANDEXP/OREXP；AND/OR 属于示教器显示形式。', 'error', 'inform-external-format');
      validateCondition(rest, line); blocks.push({ kind: opcode, from: line.from, body: line.to, finalBranch: false }); continue;
    }
    if (opcode === 'SWITCH') {
      try {
        const next = firstOperand(rest);
        if (!/^(?:I|LI)(?:\d+)?$/.test(rest[0]?.text ?? '') || rest[next]?.text !== 'CASE') throw new Error('SWITCH 首行需要 SWITCH I/LI变量 CASE 分支值。');
        validateExpression(rest.slice(next + 1), line);
      } catch (error) { add(line.from, line.to, (error as Error).message); }
      blocks.push({ kind: opcode, from: line.from, body: line.to, finalBranch: false }); continue;
    }
    if (opcode === 'FOR') {
      const match = /^FOR\s+((?:I|LI)\d+|(?:I|LI)\[[^\]]+\])\s*(?:START\s*)?=\s*(.+?)\s+TO\s+(.+?)(?:\s+STEP\s*=?\s+(.+))?$/i.exec(text);
      if (!match) add(line.from, line.to, 'FOR 格式应为 FOR I/LI变量=初值 TO 终值 [STEP 步长]（外部文件与示教器一致）。');
      else {
        try {
          const counter = tokenize(match[1], line.from + line.text.indexOf(match[1]), (from, to, message) => add(from, to, message));
          if (!typedValue(counter, ['I'], false)) throw new Error('FOR 需要有效的 I/LI 计数变量。');
          for (const part of match.slice(2).filter(Boolean)) numericExpression(tokenize(part, line.from + line.text.indexOf(part), (from, to, message) => add(from, to, message)));
        } catch (error) { add(line.from, line.to, (error as Error).message); }
      }
      blocks.push({ kind: 'FOR', variable: match?.[1].replace(/\s/g, '').toUpperCase(), from: line.from, body: line.to, finalBranch: false }); continue;
    }
    if (['ENDIF', 'ENDWHILE', 'ENDSWITCH', 'NEXT'].includes(opcode)) {
      if (opcode !== 'NEXT' && rest.length) add(line.from, line.to, `${opcode} 不接受操作数。`);
      if (opcode === 'NEXT' && !rest.length) add(line.from, line.to, 'NEXT 缺少循环变量。');
      close(({ ENDIF: 'IFTHENEXP', ENDWHILE: 'WHILEEXP', ENDSWITCH: 'SWITCH', NEXT: 'FOR' } as Record<string, string>)[opcode], line, rest.map(token => token.text).join('')); continue;
    }
    if (['ELSEIFEXP', 'ELSE', 'CASE', 'DEFAULT'].includes(opcode)) {
      const top = blocks.at(-1), expected = ['CASE', 'DEFAULT'].includes(opcode) ? 'SWITCH' : 'IFTHENEXP';
      if (!top || top.kind !== expected || top.finalBranch) add(line.from, line.to, `${opcode} 的所在块或分支顺序无效。`);
      else if (opcode === 'ELSE' || opcode === 'DEFAULT') top.finalBranch = true;
      if (opcode === 'ELSEIFEXP') {
        if (rest.some(token => ['AND', 'OR'].includes(token.text))) add(line.from, line.to, '外部结构条件使用 ANDEXP/OREXP；AND/OR 属于示教器显示形式。', 'error', 'inform-external-format');
        validateCondition(rest, line);
      }
      else if (opcode === 'CASE') validateExpression(rest, line);
      else if (rest.length) add(line.from, line.to, `${opcode} 不接受操作数。`);
      continue;
    }
    if (['IFTHEN', 'ELSEIF', 'WHILE'].includes(opcode)) { add(line.from, line.to, '当前分析外部 .JBI 格式：请使用 IFTHENEXP/ELSEIFEXP/WHILEEXP；示教器显示格式不同。', 'error', 'inform-external-format'); continue; }
    if (opcode === 'END') ended = true;
    if (noOperands.has(opcode)) { if (operands.length) add(operands[0].from, line.to, `${opcode} 不接受这些操作数。`); continue; }
    if (attachmentNames.has(opcode)) { add(line.from, line.to, `${opcode} 是附加语法项，不能作为此处的独立指令。`); continue; }
    if (!known.has(opcode)) { add(tokens[0].from, tokens[0].to, `未识别指令 ${opcode}；可能是技术选件或宏，当前未验证。`, 'warning', 'inform-unknown-instruction'); continue; }
    try {
      const sensor = parseInformSensorCommand(opcode, operands, { operand: firstOperand, expression: motionExpression, typed: typedValue });
      if (sensor) {
        if (sensor.uncheckedVariables) add(line.from, line.to, 'Sensor 公开 rev3 语法图没有给出此编号/阈值的变量参数类型；该变量形式尚未验证。', 'warning', 'inform-option');
        continue;
      }
      if (motion.has(opcode)) {
        const variableTarget = /^(?:P|LP)(?:\d+)?$/.test(operands[0]?.text ?? '') || (['MOVJ', 'IMOV'].includes(opcode) && /^(?:EX|LEX)(?:\d+)?$/.test(operands[0]?.text ?? ''));
        const recordTarget = /^C\d{5}$/.test(operands[0]?.text ?? '') || (opcode === 'MOVJ' && /^EC\d{5}$/.test(operands[0]?.text ?? ''));
        if (!variableTarget && !(opcode !== 'IMOV' && recordTarget)) throw new Error(`${opcode} 缺少有效位置目标。`);
        let cursor = position.test(operands[0].text) ? 1 : firstOperand(operands);
        if (position.test(operands[0].text)) references.push({ name: operands[0].text, token: operands[0] });
        const tags = new Set<string>();
        const tagNames = new Set(Object.values(motionTags).flat());
        const extensionFlags = new Set(['HPSRCH', 'NSRCH']);
        const attached = new Set<string>();
        let lastRank = 0;
        const alternatives = new Set<string>();
        const ordered = (rank: number, group?: string) => {
          if (rank < lastRank) throw new Error('运动附加项顺序不符合本指令语法图。');
          if (group && alternatives.has(group)) throw new Error(`运动的 ${group} 参数只能选择一项。`);
          if (group) alternatives.add(group); lastRank = rank;
        };
        while (cursor < operands.length) {
          const token = operands[cursor];
          if (/^(?:BC|EC)\d{5}$/.test(token.text)) { ordered(0, token.text.slice(0, 2)); if (!/^C\d/.test(operands[0].text) || (token.text.startsWith('BC') && alternatives.has('EC'))) throw new Error('示教目标需要按 C→BC→EC 顺序，独立 EC 后不能追加目标。'); references.push({ name: token.text, token }); cursor++; continue; }
          if (/^(?:BP|LBP|EX|LEX)(?:\d+)?$/.test(token.text)) { const family = token.text.replace(/^L/, '').replace(/\d+$/, ''); ordered(0, family); if (!/^(?:P|LP)(?:\d+)?$/.test(operands[0].text) || (family === 'BP' && alternatives.has('EX'))) throw new Error('变量目标需要按 P→BP→EX 顺序，独立 EX 后不能追加目标。'); cursor += firstOperand(operands.slice(cursor)); continue; }
          if (token.text === 'UNTIL') {
            if (!['MOVJ', 'MOVL', 'IMOV'].includes(opcode)) throw new Error(`${opcode} 不支持 UNTIL。`);
            ordered(7, '检测');
            if (attached.has('UNTIL')) throw new Error('UNTIL 附加项重复。'); attached.add('UNTIL');
            const start = ++cursor; cursor = motionExpression(operands, cursor); validateCondition(operands.slice(start, cursor), line);
            if (operands[cursor]?.text === 'CAPTURE') {
              cursor++;
              if (!/^(?:PX|LPX)(?:\d+)?$/.test(operands[cursor]?.text ?? '')) throw new Error('CAPTURE 需要 PX/LPX 扩展位置变量。');
              cursor += firstOperand(operands.slice(cursor));
            }
            continue;
          }
          if (token.text === 'NWAIT' || (token.text === 'CHK' && opcode === 'MOVJ') || (token.text === 'FPT' && opcode === 'MOVC')) {
            if (token.text === 'CHK' && !variableTarget) throw new Error('CHK 仅可检查位置变量目标。');
            ordered(token.text === 'CHK' ? 1 : token.text === 'FPT' ? 11 : 5, token.text === 'NWAIT' ? '等待模式' : undefined);
            if (attached.has(token.text)) throw new Error(`${token.text} 标记重复。`); attached.add(token.text); cursor++; continue;
          }
          if (/^T\d+$/.test(token.text) && opcode !== 'IMOV') { if (!recordTarget) throw new Error('T 点变量只能附加于示教位置记录。'); ordered(1, '点变量'); cursor++; continue; }
          if (opcode === 'IMOV' && ['BF', 'RF', 'TF', 'MTF', 'UF#'].includes(token.text)) {
            ordered(6);
            if (attached.has('coordinates')) throw new Error('IMOV 坐标系重复。'); attached.add('coordinates');
            cursor += token.text === 'UF#' ? firstOperand(operands.slice(cursor)) : 1; continue;
          }
          if (['ENWAIT', 'TRAP'].includes(token.text) || (token.text === '+' && ['SFT', 'SFTON3D'].includes(operands[cursor + 1]?.text))) {
            const kind = token.text === '+' ? operands[++cursor].text : token.text;
            if (attached.has(kind)) throw new Error(`${kind} 附加项重复。`); attached.add(kind); cursor++;
            if (kind === 'ENWAIT') {
              ordered(5, '等待模式');
              if (!['T', 'DIS'].includes(operands[cursor]?.text) || operands[cursor + 1]?.text !== '=') throw new Error('ENWAIT 需要 T=时间 或 DIS=距离。');
              if (opcode === 'IMOV') throw new Error('IMOV 不支持 ENWAIT。');
              const tag = operands[cursor].text, start = cursor + 2;
              cursor = motionExpression(operands, start);
              if (!typedValue(operands.slice(start, cursor), tag === 'T' ? ['I'] : ['B', 'I', 'D'])) throw new Error('ENWAIT 的时间/距离参数类型无效。');
            } else if (kind === 'TRAP') {
              ordered(12);
              if (operands[cursor]?.text !== '*' || !['word', 'number'].includes(operands[cursor + 1]?.kind)) throw new Error('TRAP 需要 *标签。');
              jumps.push({ name: operands[cursor + 1].text, token: operands[cursor + 1] }); cursor += 2;
            } else {
              if (kind === 'SFTON3D' && !['MOVJ', 'MOVL', 'MOVC'].includes(opcode)) throw new Error(`${opcode} 不支持 +SFTON3D。`);
              ordered(14, '附加偏移');
              if (/^RB[1-8]$/.test(operands[cursor]?.text)) cursor++;
              const families: string[] = [];
              while (/^(?:P|LP|BP|LBP|EX|LEX)(?:\d+)?$/.test(operands[cursor]?.text ?? '')) {
                const family = operands[cursor].text.replace(/^L/, '').replace(/\d+$/, '');
                const order = ['P', 'BP', 'EX'];
                if ((families.length && (families[0] !== 'P' || order.indexOf(family) <= order.indexOf(families.at(-1)!))) || (kind === 'SFTON3D' && family !== 'P')) throw new Error(`+${kind} 位置变量类型、数量或顺序无效。`);
                families.push(family); cursor += firstOperand(operands.slice(cursor));
              }
              if (!families.length) throw new Error(`+${kind} 缺少位置变量。`);
              if (kind === 'SFT' && ['BF', 'RF', 'TF', 'UF#'].includes(operands[cursor]?.text)) {
                if (families[0] !== 'P') throw new Error('+SFT 单独 BP/EX 分支不接受坐标系。');
                cursor += operands[cursor].text === 'UF#' ? firstOperand(operands.slice(cursor)) : 1;
              }
            }
            continue;
          }
          if (token.text === 'SRCH' || token.text === 'ASRCH') {
            ordered(7, '检测');
            const parsed = parseInformSearchOption(opcode, operands, cursor, { operand: firstOperand, expression: motionExpression, typed: typedValue });
            cursor = parsed.cursor;
            if (parsed.uncheckedVariables) add(token.from, operands[cursor - 1].to, 'ASRCH 公开 rev3 语法图只明确常量范围，未给变量参数类型；此变量形式尚未验证。', 'warning', 'inform-option');
            continue;
          }
          if (extensionFlags.has(token.text) || (token.text === '+' && ['MOVJ', 'MOVL', 'MOVC', 'MOVS', 'IMOV'].includes(operands[cursor + 1]?.text))) {
            if ((token.text === 'HPSRCH' && opcode !== 'MOVL') || (token.text === 'NSRCH' && !['MOVJ', 'MOVL', 'IMOV'].includes(opcode))) throw new Error(`${opcode} 不支持 ${token.text} 附加项。`);
            add(token.from, line.to, '此运动含技术选件或附加控制，完整参数组合尚未核验。', 'warning', 'inform-option'); break;
          }
          if (token.kind !== 'word' || operands[cursor + 1]?.text !== '=') throw new Error('运动目标后需要 标签=值 或已支持的附加标记。');
          const tag = token.text;
          if (tags.has(tag)) add(token.from, token.to, `运动标签 ${tag} 重复。`); tags.add(tag);
          const start = cursor + 2;
          // A coordinated +MOV*/+SFT form is not arithmetic in a preceding tag.
          const values = operands;
          cursor = motionExpression(values, start);
          if (!tagNames.has(tag)) add(token.from, token.to, `运动项 ${tag} 尚未建模。`, 'warning', 'inform-unchecked');
          else if (!motionTags[opcode].includes(tag)) throw new Error(`${opcode} 不支持 ${tag} 标签。`);
          else {
            const speed = ['VJ', 'V', 'VR', 'VE', 'VMAX'].includes(tag), positioning = ['PL', 'CR', 'SPDL', 'PD'].includes(tag);
            const rank = speed ? 2 : positioning ? 3 : ['MT', 'MTE'].includes(tag) ? 4 : tag === 'ACC' ? 8 : tag === 'DEC' ? 9 : 10;
            ordered(rank, speed ? '速度' : positioning ? '定位方式' : undefined);
            if (!typedValue(values.slice(start, cursor), ['B', 'I', 'D'])) throw new Error('运动标签值需要数值常量或 B/I/D 变量。');
          }
          const numeric = cursor === start + 1 && values[start].kind === 'number' ? Number(values[start].text)
            : cursor === start + 2 && ['+', '-'].includes(values[start].text) && values[start + 1].kind === 'number' ? Number(values[start + 1].text) * (values[start].text === '-' ? -1 : 1) : null;
          if (numeric !== null && ['VJ', 'V', 'VR', 'VE'].includes(tag) && (numeric <= 0 || (tag === 'VJ' && numeric > 100))) add(token.from, values[cursor - 1].to, `${tag} 速度超出有效范围。`);
        }
        const speedTag = opcode === 'MOVJ' || (opcode === 'IMOV' && /^(?:EX|LEX)/.test(operands[0].text)) ? 'VJ' : 'V';
        if (![speedTag, 'VR', 'VE', 'VMAX'].some(tag => tags.has(tag))) add(line.from, line.to, `${opcode} 未显式设置速度，请核对 SPEED 及控制器默认速度。`, 'warning', 'inform-speed');
      } else if (fullOperandCommands.has(opcode)) {
        if ((!variable.test(operands[0]?.text ?? '') && !ioWords.has(operands[0]?.text ?? '')) || operands[0].text.startsWith('$')) throw new Error(`${opcode} 需要可写目标变量。`);
        const next = firstOperand(operands);
        const dest = operands.slice(0, next), source = operands.slice(next), family = operands[0].text.replace(/^L/, '').replace(/\d+$/, '');
        if (family === 'FL' || operands[0].text === 'OT#') {
          if (source[0]?.text !== 'LOGICEXP') throw new Error('SET FL/OT# 需要 LOGICEXP 条件。'); validateCondition(source.slice(1), line);
        } else if (family === 'S') {
          if (!stringValue(source)) throw new Error('SET S/LS 需要字符串常量或字符串变量。');
        } else if (['P', 'BP', 'EX'].includes(family)) {
          if (!typedValue(source, [family], false)) throw new Error('位置赋值需要相同类型的位置变量。');
        } else if (typedValue(dest, ['B', 'I', 'D', 'R'], false) || ['OG#', 'OGU#'].includes(operands[0].text)) numericExpression(source);
        else throw new Error('SET 不支持此目标变量类型。');
      } else if (['INC', 'DEC'].includes(opcode)) {
        if (!typedValue(operands, ['B', 'I', 'D'], false)) throw new Error(`${opcode} 需要且仅需要一个 B/I/D 整数变量。`);
      } else if (opcode === 'TIMER') {
        if (operands[0]?.text !== 'T' || operands[1]?.text !== '=') throw new Error('TIMER 需要 T=时间。');
        if (!typedValue(operands.slice(2), ['I'])) throw new Error('TIMER 时间需要常量或 I/LI 变量。');
        if (operands.length === 4 && operands[2].text === '-' && operands[3].kind === 'number' && Number(operands[3].text) > 0) throw new Error('等待时间不能为负数。');
      } else if (opcode === 'DOUT' || opcode === 'DIN') {
        const next = firstOperand(operands);
        if (opcode === 'DOUT' && !['OT#', 'OG#', 'OGH#', 'OGU#'].includes(operands[0]?.text)) throw new Error('DOUT 需要数字输出目标 OT#/OG#/OGH#/OGU#。');
        if (opcode === 'DIN') {
          const userGroup = ['IGU#', 'OGU#'].includes(operands[next]?.text), targetPattern = userGroup ? /^(?:D|LD)(?:\d+)?$/ : /^(?:B|LB)(?:\d+)?$/;
          if (!targetPattern.test(operands[0]?.text) || !['IN#', 'OT#', 'IG#', 'OG#', 'SIN#', 'SOUT#', 'IGH#', 'OGH#', 'IGU#', 'OGU#'].includes(operands[next]?.text) || firstOperand(operands.slice(next)) + next !== operands.length) throw new Error('DIN 需要 B/LB 变量和数字输入/输出源；用户组 IGU#/OGU# 需要 D/LD 变量。');
        } else {
          const data = operands.slice(next), invert = data.length === 1 && data[0].text === 'INVERT', bit = operands[0].text === 'OT#', user = operands[0].text === 'OGU#';
          if (bit ? !((data.length === 1 && ['ON', 'OFF', 'INVERT'].includes(data[0].text)) || typedValue(data, ['B'], false)) : !(typedValue(data, user ? ['D'] : ['B']) || (!user && invert))) throw new Error('DOUT 的数据类型与输出种类不符。');
        }
      } else if (opcode === 'WAIT') {
        const values = operands[0]?.text === 'FOREXPRESS' ? operands.slice(1) : operands;
        const timeout = values.findIndex(token => token.text === 'T');
        validateCondition(timeout < 0 ? values : values.slice(0, timeout), line);
        if (timeout >= 0) { if (values[timeout + 1]?.text !== '=' || !typedValue(values.slice(timeout + 2), ['I'])) throw new Error('等待超时需使用 T=常量或 I/LI 变量。'); }
      } else if (opcode === 'RET') {
        if (operands.length && !typedValue(operands, ['B', 'I', 'D', 'R']) && !stringValue(operands)) throw new Error('RET 返回值需要数值/字符串常量或变量。');
      } else if (opcode === 'JUMP') {
        let cursor = 0, local = false;
        if (operands[0]?.text === '*' && ['word', 'number'].includes(operands[1]?.kind)) {
          jumps.push({ name: operands[1].text, token: operands[1] }); cursor = 2; local = true;
        } else if (operands[0]?.text === 'LABEL' && operands[1]?.text === ':') {
          if (!/^(?:B|LB|S|LS)(?:\d+)?$/.test(operands[2]?.text ?? '') && operands[2]?.text !== 'IG#') throw new Error('LABEL: 需要 B/LB/S/LS 变量或 IG# 引用。');
          cursor = 2 + firstOperand(operands.slice(2)); local = true;
        } else if (operands[0]?.text === 'JOB' && operands[1]?.text === ':' && ['word', 'number', 'string'].includes(operands[2]?.kind)) cursor = 3;
        else if (/^(?:B|LB|I|LI|D|LD|S|LS)(?:\d+)?$/.test(operands[0]?.text ?? '') || ['IG#', 'JET#'].includes(operands[0]?.text)) cursor = firstOperand(operands);
        else if (operands[0]?.text === 'QUE') cursor = 1;
        else throw new Error('JUMP 需要本地标签、LABEL:、JOB: 或有效间接作业目标。');
        if (local) {
          for (const tag of ['LBLST', 'LBLDF']) if (operands[cursor]?.text === tag) {
            if (operands[cursor + 1]?.text !== '=') throw new Error(`${tag} 需要等号和值。`);
            cursor = expression(operands, cursor + 2);
          }
        } else {
          if (operands[0]?.text === 'JET#') {
            if (operands[cursor]?.text !== 'ENTRY' || operands[cursor + 1]?.text !== '=') throw new Error('JET# 跳转必须指定 ENTRY=登记号。');
            cursor = expression(operands, cursor + 2);
          }
          if (operands[cursor]?.text === 'UF#') cursor += firstOperand(operands.slice(cursor));
        }
        if (cursor !== operands.length) throw new Error('JUMP 目标或附加标签后含多余参数。');
      } else if (opcode === 'CALL') {
        let cursor = 0;
        if (rest[0]?.text === 'JOB' && rest[1]?.text === ':' && ['word', 'number', 'string'].includes(rest[2]?.kind)) cursor = 3;
        else if (/^(?:B|LB|I|LI|D|LD|S|LS)(?:\d+)?$/.test(rest[0]?.text ?? '') || ['IG#', 'JET#'].includes(rest[0]?.text)) cursor = firstOperand(rest);
        else if (rest[0]?.text === 'QUE') { cursor = 1; add(line.from, line.to, 'QUE 调用需要队列技术选件。', 'warning', 'inform-option'); }
        else throw new Error('CALL 需要 JOB:名称、间接变量、IG#/JET# 或 QUE 目标。');
        if (rest[0]?.text === 'JET#') { if (rest[cursor]?.text !== 'ENTRY' || rest[cursor + 1]?.text !== '=') throw new Error('JET# 调用必须指定 ENTRY=登记号。'); cursor = expression(rest, cursor + 2); }
        if (rest[cursor]?.text === 'UF#') cursor += firstOperand(rest.slice(cursor));
        if (['IF', 'IFEXPRESS'].includes(rest[cursor]?.text)) { const start = cursor + 1; cursor = expression(rest, start); validateCondition(rest.slice(start, cursor), line); }
        if (rest[cursor]?.text === '(') {
          const begin = ++cursor; let count = 0;
          if (rest[cursor]?.text !== ')') for (;;) {
            cursor = expression(rest, cursor); count++;
            if (count > 8) throw new Error('CALL 最多接受八个实参。');
            if (rest[cursor]?.text !== ',') break; cursor++;
          }
          if (rest[cursor++]?.text !== ')') throw new Error('CALL 实参列表未闭合。');
          if (cursor === begin) throw new Error('CALL 参数列表无效。');
        }
        if (cursor !== rest.length) throw new Error('CALL 目标、条件或实参之后含多余参数。');
      } else if (additionalSchema(opcode, operands)) {
        if (opcode === 'REFP') for (const token of operands) if (position.test(token.text)) references.push({ name: token.text, token });
      }
      else {
        const entry = informInstructionEntries.find(item => item.name === opcode)!;
        if (!operands.length && !['ARCON', 'ARCOF', 'TOOLON', 'TOOLOF', 'WVON', 'ARCSET'].includes(opcode)) throw new Error(`${opcode} 缺少操作数。`);
        add(line.from, line.to, `${opcode} 已识别；${entry.requiresOption ? `需要${entry.requiresOption}，` : ''}当前仅核对词法/括号，完整参数与类型规则尚未覆盖。`, 'warning', 'inform-unchecked');
      }
    } catch (error) { add(line.from, line.to, (error as Error).message); }
  }
  for (const required of ['JOB', 'NAME', 'POS', 'INST']) if (!sections.has(required)) add(0, Math.min(source.length, 1), `缺少 ${required === 'JOB' ? '/' : '//'}${required} 分区。`);
  if (sections.has('POS') && !counts) add(0, Math.min(source.length, 1), '位置区缺少 NPOS 计数。');
  if (!ended) add(source.length, source.length, '作业缺少 END。');
  for (const block of blocks) add(block.from, block.body, `${block.kind} 块没有结束标记。`);
  if (counts) counts.forEach((count, i) => { if (count !== actualCounts[i]) add(0, Math.min(1, source.length), `NPOS 的 ${['C', 'BC', 'EC', 'P', 'BP', 'EX'][i]} 数量 ${count} 与实际 ${actualCounts[i]} 不符。`); });
  for (const ref of references) if (!definitions.has(ref.name)) add(ref.token.from, ref.token.to, `未定义的位置 ${ref.name}。`, 'error', 'inform-reference');
  for (const jump of jumps) if (!labels.has(jump.name)) add(jump.token.from, jump.token.to, `未定义的本地标签 *${jump.name}。`, 'error', 'inform-reference');
  if (section === 'instruction' && source.length > sectionStart) result.folds.push({ from: sectionStart, to: source.length });
  return result;
}
