import type { LanguageAnalysis, SourceDiagnostic } from '../types';

interface Token { text: string; from: number; to: number; kind: 'name' | 'number' | 'literal' | 'punct' }
const builtInTypes = new Set(['BOOL', 'INT', 'REAL', 'CHAR', 'FRAME', 'POS', 'E6POS', 'AXIS', 'E6AXIS']);
const operators = new Map([['OR', 1], ['B_OR', 1], ['EXOR', 2], ['B_EXOR', 2], ['AND', 3], ['B_AND', 3], ['==', 4], ['<>', 4], ['<', 4], ['>', 4], ['<=', 4], ['>=', 4], ['+', 5], ['-', 5], ['*', 6], ['/', 6], [':', 7]]);
const movements = new Set(['PTP', 'PTP_REL', 'LIN', 'LIN_REL', 'CIRC', 'CIRC_REL', 'SPL', 'SPL_REL', 'SLIN', 'SLIN_REL', 'SCIRC', 'SCIRC_REL', 'SPTP', 'SPTP_REL']);
const approximations = new Set(['C_PTP', 'C_DIS', 'C_ORI', 'C_VEL', 'C_SPL', '#BASE', '#TOOL']);

function scan(text: string, offset: number, report: (from: number, to: number, message: string) => void): Token[] {
  const tokens: Token[] = []; let i = 0;
  while (i < text.length && tokens.length < 20_000) {
    const start = i;
    if (/\s/.test(text[i])) { i++; continue; }
    if (text[i] === ';') break;
    if (text[i] === '"' || text[i] === "'") {
      const quote = text[i++];
      while (i < text.length && text[i] !== quote) { if (text[i] === '\\' && i + 1 < text.length) i++; i++; }
      if (i === text.length) report(offset + start, offset + i, '字符串或字符常量没有闭合。'); else i++;
      const literal = text.slice(start, i);
      if (quote === "'" && literal.endsWith("'") && !/^'(?:H[0-9A-F]+|B[01]+|[^'\\]|\\.)'$/i.test(literal)) report(offset + start, offset + i, '字符或二/十六进制常量格式无效。');
      tokens.push({ text: literal, from: offset + start, to: offset + i, kind: 'literal' }); continue;
    }
    const number = /^(?:\d+(?:\.\d*)?|\.\d+)(?:[Ee][+-]?\d+)?/.exec(text.slice(i));
    if (number) {
      i += number[0].length; if (!Number.isFinite(Number(number[0]))) report(offset + start, offset + i, '数值超出有限范围。');
      tokens.push({ text: number[0], from: offset + start, to: offset + i, kind: 'number' }); continue;
    }
    const name = /^[#$]?[A-Za-z_][A-Za-z0-9_$]*/.exec(text.slice(i));
    if (name) { i += name[0].length; tokens.push({ text: name[0].toUpperCase(), from: offset + start, to: offset + i, kind: 'name' }); continue; }
    const punct = /^(?:==|<>|<=|>=|[+\-*/=:<>()[\]{},.])/.exec(text.slice(i));
    if (punct) { i += punct[0].length; tokens.push({ text: punct[0], from: offset + start, to: offset + i, kind: 'punct' }); continue; }
    report(offset + i, offset + i + 1, `无法识别的字符“${text[i]}”。`); i++;
  }
  if (tokens.length >= 20_000 && i < text.length) report(offset + i, offset + text.length, '单行词法项超过 20000 项上限。');
  return tokens;
}
function splitTop(tokens: Token[], delimiter: string): Token[][] {
  const groups: Token[][] = [[]]; let depth = 0;
  for (const token of tokens) {
    if (['(', '[', '{'].includes(token.text)) depth++;
    if ([')', ']', '}'].includes(token.text)) depth--;
    if (!depth && token.text === delimiter) groups.push([]); else groups.at(-1)!.push(token);
  }
  return groups;
}
function expressionEnd(tokens: Token[], start = 0): number {
  let cursor = start, depth = 0;
  const parse = (minimum: number): void => {
    if (++depth > 128) throw new Error('表达式嵌套超过 128 层分析上限。');
    try {
      const token = tokens[cursor++];
      if (!token) throw new Error('表达式缺少操作数。');
      if (['+', '-', 'NOT', 'B_NOT'].includes(token.text)) parse(8);
      else if (token.text === '(') { parse(0); if (tokens[cursor++]?.text !== ')') throw new Error('表达式缺少右圆括号。'); }
      else if (token.text === '{') {
        if (tokens[cursor]?.kind === 'name' && tokens[cursor + 1]?.text === ':') cursor += 2;
        if (tokens[cursor]?.text === '}') throw new Error('结构常量至少需要一个分量。');
        for (;;) {
          if (tokens[cursor++]?.kind !== 'name') throw new Error('结构常量需要分量名称。');
          parse(0);
          if (tokens[cursor]?.text !== ',') break; cursor++;
        }
        if (tokens[cursor++]?.text !== '}') throw new Error('结构常量缺少逗号或右花括号。');
      } else if (!['name', 'number', 'literal'].includes(token.kind)) throw new Error('表达式操作数无效。');
      if (token.kind === 'name') for (;;) {
        if (tokens[cursor]?.text === '.') {
          cursor++; if (tokens[cursor++]?.kind !== 'name') throw new Error('成员访问缺少字段名称。');
        } else if (tokens[cursor]?.text === '(' || tokens[cursor]?.text === '[') {
          const close = tokens[cursor++].text === '(' ? ')' : ']';
          if (tokens[cursor]?.text !== close) {
            parse(0); while (tokens[cursor]?.text === ',') { cursor++; parse(0); }
          }
          if (tokens[cursor++]?.text !== close) throw new Error('参数列表或数组下标没有闭合。');
        } else break;
      }
      while (cursor < tokens.length) {
        const precedence = operators.get(tokens[cursor].text);
        if (precedence === undefined || precedence < minimum) break;
        cursor++; parse(precedence + 1);
      }
    } finally { depth--; }
  };
  parse(0); return cursor;
}


/** A writable reference is a name with optional fields/subscripts, never a call or arithmetic. */
function reference(tokens: Token[], fields = true): void {
  if (tokens[0]?.kind !== 'name' || tokens[0].text.startsWith('#')) throw new Error('目标需要变量名称。');
  let cursor = 1;
  while (cursor < tokens.length) {
    if (fields && tokens[cursor].text === '.') {
      if (tokens[++cursor]?.kind !== 'name') throw new Error('成员访问缺少字段名称。'); cursor++;
    } else if (tokens[cursor].text === '[') {
      cursor++;
      let dimensions = 0;
      if (tokens[cursor]?.text !== ']') for (;;) {
        cursor = expressionEnd(tokens, cursor); dimensions++;
        if (dimensions > 3) throw new Error('数组下标超过三维。');
        if (tokens[cursor]?.text !== ',') break; cursor++;
      }
      if (tokens[cursor++]?.text !== ']') throw new Error('数组下标没有闭合。');
    } else throw new Error('赋值/声明目标不能包含调用或运算符。');
  }
}
function validateParameter(tokens: Token[], typed: boolean): void {
  if (!tokens.length) throw new Error('参数不能为空。');
  const parts = splitTop(tokens, ':');
  if (parts.length > 2 || (parts.length === 2 && (parts[1].length !== 1 || !['IN', 'OUT'].includes(parts[1][0].text)))) throw new Error('参数模式需要 :IN 或 :OUT。');
  reference(parts[0], false);
  if (typed && parts[0][0].text.startsWith('$')) throw new Error('外部参数需要类型名称。');
}

/** Structural KSS 8.x editor analysis, independent of any controller/compiler runtime. */
export function analyzeKrl(source: string, fileName = ''): LanguageAnalysis {
  const result: LanguageAnalysis = { diagnostics: [], symbols: [], folds: [] };
  const add = (from: number, to: number, message: string, severity: SourceDiagnostic['severity'] = 'error', code = 'krl-syntax') => {
    if (result.diagnostics.length >= 300) return;
    if (result.diagnostics.length === 299) { result.diagnostics.push({ from: 0, to: Math.min(1, source.length), message: '诊断达到 300 项上限；后续内容未完整分析。', severity: 'error', code: 'analysis-limit' }); return; }
    result.diagnostics.push({ from: Math.min(source.length, Math.max(0, from)), to: Math.min(source.length, Math.max(from, to)), message, severity, code });
  };
  if (source.length > 1_048_576) { add(0, 1, '源码超过 1 MiB 分析上限。'); return result; }
  const blocks: { kind: string; from: number; body: number; finalBranch: boolean }[] = [];
  const types = new Set(builtInTypes), folds: number[] = [], labels = new Map<string, Set<string>>(), references: { scope: string; name: string; token: Token }[] = [];
  let routine = '', offset = 0, moduleCount = 0;
  const expression = (tokens: Token[]) => { if (expressionEnd(tokens) !== tokens.length) throw new Error('表达式含多余操作数或缺少运算符。'); };
  const assignments = (tokens: Token[]) => {
    for (const group of splitTop(tokens, ',')) {
      const pairs = splitTop(group, '=');
      if (pairs.length !== 2 || pairs[0][0]?.kind !== 'name') throw new Error('赋值需要可写目标 = 表达式。');
      reference(pairs[0]); expression(pairs[1]);
    }
  };
  for (const raw of source.split('\n')) {
    const line = raw.replace(/\r$/, ''), from = offset, to = from + line.length; offset += raw.length + 1;
    if (/^\s*;\s*FOLD\b/i.test(line)) folds.push(to);
    if (/^\s*;\s*ENDFOLD\b/i.test(line)) { const start = folds.pop(); if (start !== undefined && start < from) result.folds.push({ from: start, to: from }); }
    if (/^\s*&/.test(line)) {
      if (!/^\s*&(ACCESS|REL|PARAM|COMMENT)\b/i.test(line)) add(from, to, '此 KRL 文件指令尚未核验。', 'warning', 'krl-unchecked');
      continue;
    }
    const tokens = scan(line, from, (start, end, message) => add(start, end, message)); if (!tokens.length) continue;
    if (blocks.length >= 128) { add(from, to, '结构嵌套超过 128 层分析上限。', 'error', 'analysis-limit'); return result; }
    let items = tokens;
    if (items[0].text === 'GLOBAL') items = items.slice(1);
    const command = items[0]?.text ?? '', args = items.slice(1);
    const opening = (kind: string) => blocks.push({ kind, from, body: to, finalBranch: false });
    const closing = (kinds: string[]) => {
      const top = blocks.at(-1);
      if (!top || !kinds.includes(top.kind)) throw new Error(`${command} 没有匹配的 ${kinds.join('/')}。`);
      blocks.pop(); if (from > top.body) result.folds.push({ from: top.body, to: from });
      if (['DEF', 'DEFFCT', 'DEFDAT'].includes(top.kind)) routine = '';
    };
    try {
      if (command === 'DEF' || command === 'DEFFCT' || command === 'DEFDAT') {
        if (blocks.length) throw new Error('例程或数据模块不能嵌套在尚未结束的块中。');
        const nameIndex = command === 'DEFFCT' ? 1 : 0, name = args[nameIndex];
        if (name?.kind !== 'name') throw new Error(`${command} 缺少有效名称。`);
        if (command !== 'DEFDAT') {
          const params = args.slice(nameIndex + 1);
          if (params[0]?.text !== '(' || params.at(-1)?.text !== ')') throw new Error('例程声明需要圆括号参数列表。');
          for (const parameter of splitTop(params.slice(1, -1), ',')) {
            if (!parameter.length && params.length === 2) continue;
            validateParameter(parameter, false);
          }
        } else if (args.slice(1).some(token => token.text !== 'PUBLIC')) throw new Error('DEFDAT 名称后仅接受 PUBLIC。');
        routine = name.text; moduleCount++; labels.set(routine, new Set());
        result.symbols.push({ name: name.text, kind: command === 'DEFDAT' ? 'variable' : command === 'DEFFCT' ? 'function' : 'routine', from: name.from, to: name.to, signature: line.trim() }); opening(command);
        if (/\.dat$/i.test(fileName) && command !== 'DEFDAT') add(from, to, '.dat 文件应使用 DEFDAT 数据模块。');
        if (/\.src$/i.test(fileName) && command === 'DEFDAT') add(from, to, 'DEFDAT 应保存在对应 .dat 文件。');
        continue;
      }
      if (['END', 'ENDFCT', 'ENDDAT', 'ENDIF', 'ENDFOR', 'ENDWHILE', 'ENDLOOP', 'ENDSWITCH', 'ENDSPLINE'].includes(command)) {
        const expected: Record<string, string[]> = { END: ['DEF'], ENDFCT: ['DEFFCT'], ENDDAT: ['DEFDAT'], ENDIF: ['IF'], ENDFOR: ['FOR'], ENDWHILE: ['WHILE'], ENDLOOP: ['LOOP'], ENDSWITCH: ['SWITCH'], ENDSPLINE: ['SPLINE', 'PTP_SPLINE'] };
        if (args.length && !(command === 'ENDSPLINE' && args.every(token => approximations.has(token.text)))) throw new Error(`${command} 不接受这些参数。`);
        closing(expected[command]); continue;
      }
      if (!routine) throw new Error('语句必须位于 DEF/DEFFCT/DEFDAT 模块内。');
      if (command === 'STRUC' || command === 'ENUM') {
        if (args[0]?.kind !== 'name' || args.length < 2) throw new Error(`${command} 缺少类型名称或定义。`);
        types.add(args[0].text); result.symbols.push({ name: args[0].text, kind: 'type', from: args[0].from, to: args[0].to, signature: line.trim() });
        if (command === 'ENUM') { for (const group of splitTop(args.slice(1), ',')) if (group.length !== 1 || group[0].kind !== 'name') throw new Error('ENUM 成员需为逗号分隔的名称。'); }
        else {
          let hasType = false;
          for (const group of splitTop(args.slice(1), ',')) {
            let member = group;
            if (group[0]?.kind === 'name' && group[1]?.kind === 'name') { hasType = true; member = group.slice(1); }
            if (!hasType) throw new Error('STRUC 首个字段需要类型名称。');
            reference(member, false);
          }
        }
        continue;
      }
      if (command === 'DECL' || command === 'CONST' || types.has(command)) {
        let declaration = command === 'DECL' ? args : items;
        if (declaration[0]?.text === 'GLOBAL') declaration = declaration.slice(1);
        if (declaration[0]?.text === 'CONST') declaration = declaration.slice(1);
        const type = declaration[0]; if (type?.kind !== 'name') throw new Error('变量声明缺少类型。');
        for (const group of splitTop(declaration.slice(1), ',')) {
          const pairs = splitTop(group, '='), id = pairs[0]?.[0];
          if (!id || id.kind !== 'name' || pairs.length > 2) throw new Error('变量声明缺少名称或初始化格式错误。');
          reference(pairs[0], false); if (pairs.length === 2) expression(pairs[1]);
          result.symbols.push({ name: id.text, kind: command === 'CONST' ? 'constant' : 'variable', from: id.from, to: id.to, signature: `${type.text} ${id.text}` });
        }
        continue;
      }
      if (command === 'EXT' || command === 'EXTFCT') {
        const id = args[command === 'EXTFCT' ? 1 : 0], params = args.slice(command === 'EXTFCT' ? 2 : 1);
        if (id?.kind !== 'name' || params[0]?.text !== '(' || params.at(-1)?.text !== ')') throw new Error('外部例程声明需要名称与类型参数列表。');
        for (const part of splitTop(params.slice(1, -1), ',')) { if (!part.length && params.length === 2) continue; validateParameter(part, true); }
        result.symbols.push({ name: id.text, kind: command === 'EXTFCT' ? 'function' : 'routine', from: id.from, to: id.to, signature: line.trim() }); continue;
      }
      if (command === 'SIGNAL') {
        if (args[0]?.kind !== 'name') throw new Error('SIGNAL 缺少名称。');
        for (const part of splitTop(args.slice(1), 'TO')) expression(part);
        result.symbols.push({ name: args[0].text, kind: 'variable', from: args[0].from, to: args[0].to, signature: line.trim() }); continue;
      }
      if (blocks[0]?.kind === 'DEFDAT') { assignments(items); continue; }
      if (command === 'IF') { if (args.at(-1)?.text !== 'THEN') throw new Error('IF 条件后需要 THEN。'); expression(args.slice(0, -1)); opening('IF'); continue; }
      if (command === 'WHILE' || command === 'SWITCH') { expression(args); opening(command); continue; }
      if (command === 'LOOP' || command === 'REPEAT') { if (args.length) throw new Error(`${command} 不接受条件。`); opening(command); continue; }
      if (command === 'UNTIL') { expression(args); closing(['REPEAT']); continue; }
      if (command === 'ELSE' || command === 'CASE' || command === 'DEFAULT') {
        const expected = command === 'ELSE' ? 'IF' : 'SWITCH', top = blocks.at(-1);
        if (!top || top.kind !== expected || top.finalBranch) throw new Error(`${command} 所在块或分支顺序无效。`);
        if (command === 'CASE') for (const part of splitTop(args, ',')) expression(part);
        else { if (args.length) throw new Error(`${command} 不接受参数。`); top.finalBranch = true; }
        continue;
      }
      if (command === 'FOR') {
        const toParts = splitTop(args, 'TO'); if (toParts.length !== 2) throw new Error('FOR 需要 TO 终值。'); assignments(toParts[0]);
        const endParts = splitTop(toParts[1], 'STEP'); if (endParts.length > 2) throw new Error('FOR STEP 重复。'); endParts.forEach(expression); opening('FOR'); continue;
      }
      if (command === 'SPLINE' || command === 'PTP_SPLINE') {
        if (args.length) { if (args[0].text !== 'WITH') throw new Error('样条块参数需要 WITH。'); assignments(args.slice(1)); }
        opening(command); continue;
      }
      if (movements.has(command)) {
        let targets = args;
        const withIndex = targets.findIndex(token => token.text === 'WITH');
        if (withIndex >= 0) { assignments(targets.slice(withIndex + 1)); targets = targets.slice(0, withIndex); }
        while (targets.length && approximations.has(targets.at(-1)!.text)) targets = targets.slice(0, -1);
        const caIndex = targets.findIndex(token => token.text === 'CA');
        if (caIndex >= 0) { if (targets[caIndex + 1]?.text !== '=') throw new Error('CA 参数缺少等号。'); expression(targets.slice(caIndex + 2)); targets = targets.slice(0, caIndex); }
        const parts = splitTop(targets, ',');
        if (parts.length !== (command.includes('CIRC') ? 2 : 1)) throw new Error(`${command} 的位置操作数数量无效。`);
        parts.forEach(expression);
        if (command.startsWith('S') && !blocks.some(block => block.kind === 'SPLINE' || block.kind === 'PTP_SPLINE')) add(from, to, '样条段不在样条块内；独立样条形式需按 KSS 版本核验。', 'warning', 'krl-version');
        continue;
      }
      if (command === 'WAIT') { if (!['FOR', 'SEC'].includes(args[0]?.text)) throw new Error('WAIT 需要 FOR 条件或 SEC 时间。'); expression(args.slice(1)); continue; }
      if (['CONTINUE', 'HALT', 'RESUME', 'EXIT'].includes(command)) {
        if (args.length) throw new Error(`${command} 不接受参数。`);
        if (command === 'EXIT' && !blocks.some(block => ['FOR', 'WHILE', 'LOOP', 'REPEAT'].includes(block.kind))) throw new Error('EXIT 必须在循环内。');
        continue;
      }
      if (command === 'RETURN') { if (args.length) expression(args); continue; }
      if (command === 'BRAKE') { if (args.length && !(args.length === 1 && args[0].text === 'F')) throw new Error('BRAKE 仅接受可选 F。'); continue; }
      if (items.length === 2 && items[0].kind === 'name' && items[1].text === ':') {
        const defined = labels.get(routine)!; if (defined.has(command)) throw new Error('标签重复定义。'); defined.add(command);
        result.symbols.push({ name: command, kind: 'constant', from: items[0].from, to: items[0].to }); continue;
      }
      if (command === 'GOTO') { if (args.length !== 1 || args[0].kind !== 'name') throw new Error('GOTO 需要一个标签名称。'); references.push({ scope: routine, name: args[0].text, token: args[0] }); continue; }
      if (command === 'INTERRUPT') {
        if (args[0]?.text === 'DECL') {
          const when = args.findIndex(token => token.text === 'WHEN'), action = args.findIndex(token => token.text === 'DO');
          if (when < 2 || action <= when + 1) throw new Error('中断声明需要 DECL 编号 WHEN 条件 DO 动作。');
          expression(args.slice(1, when)); expression(args.slice(when + 1, action)); expression(args.slice(action + 1));
        } else { if (!['ON', 'OFF', 'ENABLE', 'DISABLE'].includes(args[0]?.text)) throw new Error('中断控制需使用 ON/OFF/ENABLE/DISABLE。'); if (args.length > 1) expression(args.slice(1)); }
        continue;
      }
      if (command === 'TRIGGER' || ['TIME_BLOCK', 'CONST_VEL', 'ANIN', 'ANOUT', 'IMPORT'].includes(command)) {
        if (!args.length) throw new Error(`${command} 缺少参数。`);
        add(from, to, `${command} 已识别；完整参数、提前运行或控制器选件规则尚未覆盖。`, 'warning', 'krl-unchecked'); continue;
      }
      if (splitTop(items, '=').length > 1) { assignments(items); continue; }
      if (items[0]?.kind === 'name' && items[1]?.text === '(') { expression(items); continue; }
      add(from, to, `语句 ${command} 尚未建模；未将其视作语法通过。`, 'warning', 'krl-unchecked');
    } catch (error) { add(from, to, (error as Error).message); }
  }
  for (const block of blocks) add(block.from, block.body, `${block.kind} 缺少结束标记。`);
  for (const reference of references) if (!labels.get(reference.scope)?.has(reference.name)) add(reference.token.from, reference.token.to, `当前例程未定义标签 ${reference.name}。`);
  if (!moduleCount) add(0, Math.min(1, source.length), '缺少 DEF/DEFFCT 或 DEFDAT 模块。');
  return result;
}
