import type { SourceDiagnostic, SourceFold, SourceSymbol } from '../types';
import { Cursor, SyntaxFault, parseFanucPositionGroups, type FanucGroup, type FanucToken } from './fanuc-parser';

export interface FanucApplicationPoint { key: string; from: number; to: number; groups: FanucGroup[] }
export interface FanucPalletApplication {
  kind: 'pallet'; number: number; pattern: 'B' | 'BX' | 'E' | 'EX' | null;
  from: number; to: number; fields: Readonly<Record<string, readonly FanucToken[]>>;
  bottom: FanucApplicationPoint[]; routes: { number: number; points: FanucApplicationPoint[] }[];
  patterns: { number: number; indices: readonly (number | '*')[] }[];
}
export interface FanucSimpleApplication { kind: 'attribute'; name: string; from: number; to: number; values: readonly FanucToken[] }
export type FanucApplication = FanucPalletApplication | FanucSimpleApplication;
interface Context {
  diagnostic(token: FanucToken | undefined, message: string, code?: string, severity?: SourceDiagnostic['severity']): void;
  folds: SourceFold[]; symbols: SourceSymbol[];
}
const SECTIONS = new Set(['CONFIG', 'BOTTOM', 'PATTERN', 'ROUTE']);
const integer = (cursor: Cursor, min = 0, max = Number.MAX_SAFE_INTEGER): number => {
  const token = cursor.take(), value = Number(token.text);
  if (token.kind !== 'number' || !Number.isSafeInteger(value) || value < min || value > max) throw new SyntaxFault(token, `此处需要 ${min} 至 ${max} 的整数`);
  return value;
};
const numeric = (cursor: Cursor): number => {
  const sign = cursor.eat('-') ? -1 : (cursor.eat('+'), 1), token = cursor.take(), value = sign * Number(token.text);
  if (token.kind !== 'number' || !Number.isFinite(value)) throw new SyntaxFault(token, '此处需要有限数值');
  return value;
};
const oneOf = (cursor: Cursor, allowed: readonly string[]) => { const token = cursor.take(); if (!allowed.includes(token.upper)) throw new SyntaxFault(token, `此处需要 ${allowed.join(' / ')}`); return token.upper; };

/** B-83284EN/09 §8.10.5 ASCII application data and §10 pallet data.
 * Does not reinterpret controller setup files as TP instructions. */
export function parseFanucApplications(tokens: readonly FanucToken[], context: Context): FanucApplication[] {
  const cursor = new Cursor(tokens), result: FanucApplication[] = [], seenPallets = new Set<number>();
  let currentPallet: FanucPalletApplication | null = null, block = '', pointKeys = new Set<string>();
  const report = context.diagnostic;
  const sectionAhead = () => SECTIONS.has(cursor.token?.upper ?? '') && cursor.tokens[cursor.index + 1]?.upper === ':';
  const point = (kind: 'bottom' | 'route'): FanucApplicationPoint => {
    const start = cursor.take(); if (start.upper !== 'P') throw new SyntaxFault(start, '码垛位置需要 P[...]');
    cursor.expect('['); let key: string;
    if (kind === 'bottom') {
      const indices = [integer(cursor, 1, 127)]; cursor.expect(','); indices.push(integer(cursor, 1, 127)); cursor.expect(','); indices.push(integer(cursor, 1, 127)); key = indices.join(',');
    } else {
      const slot = cursor.take();
      if (!/^(BTM|[AR]_[1-8])$/.test(slot.upper)) throw new SyntaxFault(slot, '路径位置需要 BTM、A_1..A_8 或 R_1..R_8');
      key = slot.upper;
    }
    cursor.expect(']'); cursor.expect('{'); const open = cursor.tokens[cursor.index - 1];
    const groups = parseFanucPositionGroups(cursor, report);
    if (!groups.length) throw new SyntaxFault(cursor.token, '码垛位置缺少运动组');
    const end = cursor.take(); if (end.upper !== '}') throw new SyntaxFault(end, '码垛位置缺少 }'); cursor.expect(';');
    const scope = kind === 'bottom' ? `pallet:${currentPallet?.number}:bottom:${key}` : `pallet:${currentPallet?.number}:route:${currentPallet?.routes.at(-1)?.number ?? 1}:${key}`;
    if (pointKeys.has(scope)) report(start, `重复码垛位置 ${key}`, 'fanuc.duplicate-pallet-point'); pointKeys.add(scope);
    context.folds.push({ from: open.to, to: end.from });
    return { key, from: start.from, to: end.to, groups };
  };
  const finishPallet = () => {
    if (!currentPallet) return;
    if (!currentPallet.pattern) report(tokens.find(token => token.from === currentPallet!.from), '码垛应用缺少 CONFIG 的类型声明', 'fanuc.pallet-config');
    const required = ['TYPE', 'INCR', 'PAL REG', 'ORDER', 'ROWS', 'COLUMNS', 'LAYERS', 'AUXILIARY POS', 'APPR', 'RTRT'];
    for (const name of required) if (!Object.hasOwn(currentPallet.fields, name)) report(tokens.find(token => token.from === currentPallet!.from), `码垛 CONFIG 缺少 ${name}`, 'fanuc.pallet-config');
    if (['BX', 'EX'].includes(currentPallet.pattern ?? '') && !Object.hasOwn(currentPallet.fields, 'PATTERN')) report(tokens.find(token => token.from === currentPallet!.from), 'BX/EX CONFIG 需要 PATTERN 路径数量', 'fanuc.pallet-config');
    if (['B', 'E'].includes(currentPallet.pattern ?? '') && (Object.hasOwn(currentPallet.fields, 'PATTERN') || currentPallet.patterns.length)) report(tokens.find(token => token.from === currentPallet!.from), 'PATTERN 路径数量与 PTN 分配仅用于 BX/EX，B/E 使用单一路径', 'fanuc.pallet-config');
    currentPallet.to = cursor.tokens[Math.max(0, cursor.index - 1)]?.to ?? currentPallet.from;
    context.folds.push({ from: currentPallet.from, to: currentPallet.to });
    currentPallet = null; block = ''; pointKeys = new Set();
  };
  while (!cursor.done) {
    const before = cursor.index, first = cursor.token;
    try {
      if (cursor.eat(';') || first.kind === 'comment' && !!cursor.take()) continue;
      if (cursor.is('PALLETIZING') && (cursor.tokens[cursor.index + 1]?.upper === '[' || cursor.tokens[cursor.index + 1]?.upper === '-' && cursor.tokens[cursor.index + 2]?.upper === '[')) {
        cursor.take();
        finishPallet(); cursor.eat('-'); cursor.expect('['); const number = integer(cursor, 1, 16); cursor.expect(']'); cursor.expect(';');
        if (seenPallets.has(number)) report(first, `重复 PALLETIZING[${number}]`, 'fanuc.duplicate-pallet'); seenPallets.add(number);
        currentPallet = { kind: 'pallet', number, pattern: null, from: first.from, to: first.to, fields: {}, bottom: [], routes: [], patterns: [] };
        result.push(currentPallet); context.symbols.push({ name: `PALLETIZING[${number}]`, kind: 'variable', from: first.from, to: cursor.tokens[cursor.index - 1].to });
        continue;
      }
      if (currentPallet && sectionAhead()) { block = cursor.take().upper; cursor.expect(':'); if (block === 'ROUTE' && !currentPallet.routes.length) currentPallet.routes.push({ number: 1, points: [] }); continue; }
      if (currentPallet && block === 'CONFIG') {
        if (cursor.eat('PALLETIZING')) {
          cursor.expect('-'); const pattern = cursor.take(), match = /^(B|BX|E|EX)_(\d+)$/.exec(pattern.upper);
          if (!match || Number(match[2]) !== currentPallet.number) throw new SyntaxFault(pattern, '码垛类型编号必须与 PALLETIZING 声明一致');
          if (currentPallet.pattern) throw new SyntaxFault(pattern, '重复码垛类型声明');
          currentPallet.pattern = match[1] as FanucPalletApplication['pattern'];
          if (cursor.token?.kind === 'string') cursor.take(); cursor.expect(';'); continue;
        }
        const nameTokens: FanucToken[] = [];
        while (!cursor.done && !cursor.is('=')) { if (cursor.is(';') || cursor.is(':')) throw new SyntaxFault(cursor.token, 'CONFIG 字段缺少 ='); nameTokens.push(cursor.take()); }
        const name = nameTokens.map(token => token.upper).join(' '); cursor.expect('='); const from = cursor.index;
        if (Object.hasOwn(currentPallet.fields, name)) throw new SyntaxFault(first, `重复码垛字段 ${name}`);
        if (name === 'TYPE') oneOf(cursor, ['PALLET', 'DEPALLET']);
        else if (name === 'INCR') { const amount = numeric(cursor); if (!Number.isSafeInteger(amount)) throw new SyntaxFault(first, 'INCR 需要整数'); }
        else if (name === 'PAL REG') integer(cursor, 1, 32);
        else if (name === 'ORDER') oneOf(cursor, ['RCL', 'RLC', 'CRL', 'CLR', 'LRC', 'LCR']);
        else if (['ROWS', 'COLUMNS', 'LAYERS'].includes(name)) {
          cursor.expect('['); integer(cursor, 1, 127);
          if (!cursor.is(']')) {
            if (cursor.eat('LINE')) { /* two-point layout */ }
            else if (cursor.token?.kind === 'word') { const mode = cursor.take(); report(mode, `排列模式 ${mode.text} 的 ASCII 拼写尚无原厂导出样本；已保留，未认定为有效枚举`, 'fanuc.unverified-pallet-serialization', 'warning'); }
            else numeric(cursor);
            oneOf(cursor, ['FIX', 'INTER']);
            if (name === 'LAYERS') integer(cursor, 1, 16);
            if (['B', 'BX'].includes(currentPallet.pattern ?? '')) throw new SyntaxFault(first, 'B/BX 不接受 E/EX 的排列／姿态附加字段');
          } else if (['E', 'EX'].includes(currentPallet.pattern ?? '')) throw new SyntaxFault(first, 'E/EX 需要排列模式及姿态控制字段');
          cursor.expect(']');
        } else if (name === 'AUXILIARY POS') { cursor.expect('['); const enabled = oneOf(cursor, ['YES', 'NO']); if (enabled === 'YES' && !cursor.is(']')) oneOf(cursor, ['FIX', 'INTER']); cursor.expect(']'); }
        else if (name === 'APPR' || name === 'RTRT') integer(cursor, 0, 8);
        else if (name === 'PATTERN') integer(cursor, 1, 16);
        else { report(first, `未声明的应用扩展字段 ${name}`, 'fanuc.unsupported-application-field', 'warning'); while (!cursor.done && !cursor.is(';')) cursor.take(); }
        (currentPallet.fields as Record<string, readonly FanucToken[]>)[name] = cursor.tokens.slice(from, cursor.index); cursor.expect(';'); continue;
      }
      if (currentPallet && block === 'BOTTOM') { currentPallet.bottom.push(point('bottom')); continue; }
      if (currentPallet && block === 'PATTERN') {
        cursor.expect('PTN'); cursor.expect('['); const number = integer(cursor, 1, 16); cursor.expect(']'); cursor.expect('='); cursor.expect('[');
        const indices: (number | '*')[] = [];
        for (let index = 0; index < 3; index++) { indices.push(cursor.eat('*') ? '*' : integer(cursor, 1, 127)); if (index < 2) cursor.expect(','); }
        cursor.expect(']'); cursor.expect(';');
        if (currentPallet.patterns.some(pattern => pattern.number === number)) throw new SyntaxFault(first, `重复 PTN[${number}]`);
        currentPallet.patterns.push({ number, indices }); continue;
      }
      if (currentPallet && block === 'ROUTE') {
        if (cursor.eat('ROUTE')) {
          cursor.expect('['); const number = integer(cursor, 1, 16); cursor.expect(']'); cursor.expect('{'); const opening = cursor.tokens[cursor.index - 1];
          const existing = currentPallet.routes.find(route => route.number === number);
          if (existing && existing.points.length) throw new SyntaxFault(first, `重复 ROUTE[${number}]`);
          const route = existing ?? { number, points: [] }; if (!existing) currentPallet.routes.push(route);
          // Place this route last so point duplicate keys have the current scope.
          currentPallet.routes = [...currentPallet.routes.filter(item => item !== route), route];
          while (!cursor.done && !cursor.is('}')) route.points.push(point('route'));
          const end = cursor.take(); if (end.upper !== '}') throw new SyntaxFault(end, 'ROUTE 缺少 }'); cursor.expect(';'); context.folds.push({ from: opening.to, to: end.from });
        } else currentPallet.routes.at(-1)!.points.push(point('route'));
        continue;
      }
      // Basic application attributes outside a pallet tree.
      const names: FanucToken[] = [];
      while (!cursor.done && ![':', '=', ';'].includes(cursor.token.upper)) names.push(cursor.take());
      const name = names.map(token => token.upper).join(' '), separator = cursor.take(), begin = cursor.index;
      if (name === 'DISPENSE_PROCESS') { if (separator.upper !== ';') throw new SyntaxFault(separator, 'DISPENSE_PROCESS 后需要分号'); }
      else if (['HANDLING', 'SPOT', 'ARC', 'DISPENSE'].includes(name)) { if (separator.upper !== ':') throw new SyntaxFault(separator, '应用启用字段使用 :'); oneOf(cursor, ['TRUE', 'FALSE']); cursor.expect(';'); }
      else if (['SPOT WELDING EQUIPMENT NUMBER', 'DEFAULT_USER_FRAME', 'DEFAULT_TOOL_FRAME', 'EQUIPMENT_NUMBER'].includes(name)) { if (separator.upper !== ':') throw new SyntaxFault(separator, '应用字段使用 :'); integer(cursor); cursor.expect(';'); }
      else if (name === 'ARC WELDING EQUIPMENT') {
        if (separator.upper !== ':') throw new SyntaxFault(separator, 'ARC WELDING EQUIPMENT 使用 :'); let count = 0;
        do { if (++count > 8) throw new SyntaxFault(cursor.token, '焊接设备列表最多八个运动组'); if (!cursor.eat('*')) integer(cursor, 1); } while (cursor.eat(',')); cursor.expect(';');
      } else if (['CYCLE_TIME', 'LAST_CYCLE_TIME', 'GUN_ON_TIME', 'LAST_GUN_ON_TIME', 'CYCLE_TARGET'].includes(name)) {
        if (![':', '='].includes(separator.upper)) throw new SyntaxFault(separator, '应用数值字段需要 : 或 ='); numeric(cursor); cursor.expect(';');
      } else if (name === 'CYCLE_REFERENCE') {
        if (separator.upper !== '=') throw new SyntaxFault(separator, 'CYCLE_REFERENCE 使用 =');
        for (let index = 0; index < 12; index++) { numeric(cursor); if (index < 11) cursor.expect(','); } cursor.expect(';');
      } else {
        report(first, `应用 ${name || first.text} 需要对应应用手册，当前配置数据未验证`, 'fanuc.unsupported-application-data', 'warning');
        if (separator.upper !== ';') while (!cursor.done && !cursor.eat(';')) cursor.take();
      }
      result.push({ kind: 'attribute', name, from: first.from, to: cursor.tokens[cursor.index - 1]?.to ?? first.to, values: cursor.tokens.slice(begin, Math.max(begin, cursor.index - 1)) });
    } catch (error) {
      if (!(error instanceof SyntaxFault)) throw error;
      report(error.token, error.message, 'fanuc.application-syntax');
      while (!cursor.done && !cursor.eat(';') && !sectionAhead()) cursor.take();
    }
    if (cursor.index <= before) cursor.take();
  }
  finishPallet();
  return result;
}
