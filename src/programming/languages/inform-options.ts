import type { LanguageEntry } from '../types';

/** Original implementation from the two public option manuals, not a pendant macro. */
export interface InformOptionToken { text: string; kind: 'word' | 'number' | 'string' | 'punct' }
interface OptionGrammar<T extends InformOptionToken> {
  operand(tokens: T[]): number;
  expression(tokens: T[], start: number): number;
  typed(tokens: T[], families: readonly string[], constants?: boolean): boolean;
}
export const informOptionSources = [
  { title: 'YRC1000 Search Function', url: 'https://www.motoman.com/getmedia/365C2AD3-133A-4E72-9630-BB1E9F28537D/178657-1CD', revision: '178657-1CD / HW1483392 revision 1', license: '仅参考语法事实；未分发厂商正文或图' },
  { title: 'YRC1000 Sensor Function', url: 'https://www.motoman.com/getmedia/18535E76-FAAA-4921-833C-0C5C4EA7D397/179329-1CD', revision: '179329-1CD / HW1483394 revision 3', license: '仅参考语法事实；未分发厂商正文或图' },
] as const;

const sensorEntries = [
  ['ACORON', 'ACORON TF/PSF/OPF [SCF#(file)] [TCH#(channel) TV=threshold]', '按工具、前进或可选坐标方向启用实时路径修正。', 24],
  ['SACORON', 'SACORON TF/PSF/OPF [SCF#(file)] [TCH#(channel) TV=threshold]', '协调作业的实时路径修正启动；控制组与选件配置仍需控制器核对。', 24],
  ['ACOROF', 'ACOROF', '结束实时路径修正。', 27],
  ['SACOROF', 'SACOROF', '结束协调作业的实时路径修正。', 27],
  ['ACORCH', 'ACORCH SCF#(file) / TCH#(channel) TV=threshold', '更改监视条件文件，或更改通道与阈值；两种参数分支不能混合。', 28],
  ['SACORCH', 'SACORCH SCF#(file) / TCH#(channel) TV=threshold', '更改协调路径修正条件；参数分支与 ACORCH 相同。', 28],
  ['AOVRON', 'AOVRON TCH#(channel) TV=threshold', '启用基于模拟传感输入的实时速度修正。', 33],
  ['AOVROF', 'AOVROF', '结束实时速度修正。', 36],
  ['GETSFT', 'GETSFT [TCH#(channel) TV=threshold] [Pvariable]', '生成基坐标下的偏移量；可选择传感通道、阈值和保存位置变量。', 38],
  ['SCALIB', 'SCALIB [TCH#(channel)]', '校准指定传感通道；0 或省略表示全部通道。编辑器不会执行校准。', 46],
] as const;
export const informSensorCommands = new Set<string>(sensorEntries.map(([name]) => name));
export const informOptionEntries: readonly LanguageEntry[] = [
  { name: 'SRCH', kind: 'keyword', category: '搜索选件附加项', signature: 'SRCH RIN#(input)=ON/OFF/Bvariable [T=delay] DIS=distance',
    description: '在 MOVL/IMOV 的直线运动中搜索直接输入条件，必须指定过行程距离；信号成立后停止并记录检测位置。',
    insertText: 'SRCH RIN#(${1})=${ON} DIS=${10.0}', requiresOption: 'Search Function HW1483392', availableSince: 'YRC1000 / 178657-1CD rev1', documentationUrl: `${informOptionSources[0].url}#page=15`,
    parameters: [{ name: 'input', description: '直接输入 1–4；也可使用 B/I/D 及局部、数组变量。' }, { name: 'T', description: '可选延时，0.01–655.35 秒；常量或 I/LI 变量。', optional: true }, { name: 'DIS', description: '必填过行程距离，0.1–6553.5 mm；常量或 B/I/D 变量。' }] },
  { name: 'ASRCH', kind: 'keyword', category: '传感选件附加项', signature: 'ASRCH SCH#(channel) TV=threshold RISE/FALL [T=delay] DIS=distance',
    description: '在 MOVL 上按模拟输入阈值的上升或下降方向搜索。公开 rev3 明确常量形式；变量参数缺少类型说明时保留未验证警告。',
    insertText: 'ASRCH SCH#(${1}) TV=${0.0} ${RISE} DIS=${0.0}', requiresOption: 'Sensor Function HW1483394', availableSince: 'YRC1000 / 179329-1CD rev3', documentationUrl: `${informOptionSources[1].url}#page=43`,
    parameters: [{ name: 'channel', description: '0 表示多通道，1–6 为指定传感通道。' }, { name: 'TV', description: '阈值 −10 至 10 V。' }, { name: 'RISE/FALL', description: '必选一个阈值跨越方向。' }, { name: 'T', description: '可选延时 0–99.99 秒。', optional: true }, { name: 'DIS', description: '必填过行程距离 0–999.99 mm。' }] },
  ...sensorEntries.map(([name, signature, description, page]) => ({ name, signature, description, kind: 'instruction' as const, category: '传感选件', requiresOption: 'Sensor Function HW1483394', availableSince: 'YRC1000 / 179329-1CD rev3', documentationUrl: `${informOptionSources[1].url}#page=${page}`, insertText: name,
    parameters: signature.split(' ').slice(1).filter(part => part !== '/').map(part => ({ name: part, description: '参数分支、常量范围及可选组合见本指令说明；公开手册未说明的变量参数仍会标记未验证。', optional: part.startsWith('[') })) })),
];

function literal(tokens: InformOptionToken[]): number | null {
  if (tokens.length === 1 && tokens[0].kind === 'number') return Number(tokens[0].text);
  if (tokens.length === 2 && ['+', '-'].includes(tokens[0].text) && tokens[1].kind === 'number') return Number(tokens[1].text) * (tokens[0].text === '-' ? -1 : 1);
  return null;
}

/** Returns the first unconsumed motion token so ACC/DEC/TRAP are still checked. */
export function parseInformSearchOption<T extends InformOptionToken>(
  parent: string, tokens: T[], start: number,
  grammar: OptionGrammar<T>,
): { cursor: number; uncheckedVariables: boolean } {
  const helpers = grammar;
  const opcode = tokens[start]?.text;
  const analog = opcode === 'ASRCH';
  if (!analog && opcode !== 'SRCH') throw new Error('不支持的搜索附加项。');
  if (analog ? parent !== 'MOVL' : !['MOVL', 'IMOV'].includes(parent)) throw new Error(`${parent} 不支持 ${opcode} 附加项。`);
  let cursor = start + 1, uncheckedVariables = false;
  const value = (parts: T[], families: readonly string[], min: number, max: number, name: string, integer = false) => {
    if (!helpers.typed(parts, families)) throw new Error(`${opcode} 的 ${name} 需要数值常量或指定类型变量。`);
    const number = literal(parts);
    if (number !== null && (!Number.isFinite(number) || number < min || number > max || (integer && !Number.isInteger(number)))) throw new Error(`${opcode} 的 ${name} 范围为 ${min}–${max}${integer ? ' 整数' : ''}。`);
    if (analog && number === null) uncheckedVariables = true;
  };
  const reference = analog ? 'SCH#' : 'RIN#';
  if (tokens[cursor]?.text !== reference || tokens[cursor + 1]?.text !== '(') throw new Error(`${opcode} 需要 ${reference}(编号)。`);
  const end = cursor + helpers.operand(tokens.slice(cursor));
  value(tokens.slice(cursor + 2, end - 1), ['B', 'I', 'D'], analog ? 0 : 1, analog ? 6 : 4, reference, true);
  cursor = end;
  const tag = (name: string, families: readonly string[], min: number, max: number) => {
    if (tokens[cursor]?.text !== name || tokens[cursor + 1]?.text !== '=') throw new Error(`${opcode} 缺少 ${name}=值，或参数顺序不正确。`);
    const begin = cursor + 2;
    cursor = helpers.expression(tokens, begin);
    value(tokens.slice(begin, cursor), families, min, max, name);
  };
  if (analog) {
    tag('TV', ['B', 'I', 'D', 'R'], -10, 10);
    if (!['RISE', 'FALL'].includes(tokens[cursor]?.text)) throw new Error('ASRCH 需要 RISE 或 FALL 检测方向。');
    cursor++;
    if (tokens[cursor]?.text === 'T') tag('T', ['B', 'I', 'D', 'R'], 0, 99.99);
    tag('DIS', ['B', 'I', 'D', 'R'], 0, 999.99);
  } else {
    if (tokens[cursor++]?.text !== '=') throw new Error('SRCH 的输入条件需要等号。');
    if (['ON', 'OFF'].includes(tokens[cursor]?.text)) cursor++;
    else {
      const end = cursor + helpers.operand(tokens.slice(cursor));
      if (!helpers.typed(tokens.slice(cursor, end), ['B'], false)) throw new Error('SRCH 输入状态需要 ON/OFF 或 B/LB 变量。');
      cursor = end;
    }
    if (tokens[cursor]?.text === 'T') tag('T', ['I'], 0.01, 655.35);
    tag('DIS', ['B', 'I', 'D'], 0.1, 6553.5);
  }
  if (['T', 'DIS', 'TV', 'RISE', 'FALL', 'SCH#', 'RIN#'].includes(tokens[cursor]?.text)) throw new Error(`${opcode} 的搜索参数重复或顺序不正确。`);
  return { cursor, uncheckedVariables };
}

/** The sensor manual provides constant shapes; undocumented scalar variables stay visible. */
export function parseInformSensorCommand<T extends InformOptionToken>(opcode: string, tokens: T[], grammar: OptionGrammar<T>): { uncheckedVariables: boolean } | null {
  if (!informSensorCommands.has(opcode)) return null;
  let cursor = 0, uncheckedVariables = false;
  const number = (parts: T[], name: string, min: number, max: number, integer: boolean) => {
    if (!grammar.typed(parts, integer ? ['B', 'I', 'D'] : ['B', 'I', 'D', 'R'])) throw new Error(`${opcode} 的 ${name} 需要数值常量或编号变量。`);
    const value = literal(parts);
    if (value === null) uncheckedVariables = true;
    else if (!Number.isFinite(value) || value < min || value > max || (integer && !Number.isInteger(value))) throw new Error(`${opcode} 的 ${name} 范围为 ${min}–${max}${integer ? ' 整数' : ''}。`);
  };
  const ref = (name: string, min: number, max: number) => {
    if (tokens[cursor]?.text !== name || tokens[cursor + 1]?.text !== '(') throw new Error(`${opcode} 需要 ${name}(编号)。`);
    const end = cursor + grammar.operand(tokens.slice(cursor));
    number(tokens.slice(cursor + 2, end - 1), name, min, max, true);
    cursor = end;
  };
  const threshold = () => {
    if (tokens[cursor]?.text !== 'TV' || tokens[cursor + 1]?.text !== '=') throw new Error(`${opcode} 的 TCH# 后需要 TV=阈值。`);
    const start = cursor + 2;
    cursor = grammar.expression(tokens, start);
    number(tokens.slice(start, cursor), 'TV', -10, 10, false);
  };
  if (['ACORON', 'SACORON'].includes(opcode)) {
    if (!['TF', 'PSF', 'OPF'].includes(tokens[cursor]?.text)) throw new Error(`${opcode} 需要 TF/PSF/OPF 坐标方向。`);
    cursor++;
    if (tokens[cursor]?.text === 'SCF#') ref('SCF#', 1, 16);
    if (tokens[cursor]?.text === 'TCH#') { ref('TCH#', 0, 3); threshold(); }
  } else if (['ACORCH', 'SACORCH'].includes(opcode)) {
    if (tokens[cursor]?.text === 'SCF#') ref('SCF#', 1, 16);
    else { ref('TCH#', 0, 3); threshold(); }
  } else if (opcode === 'AOVRON') {
    ref('TCH#', 1, 6); threshold();
  } else if (opcode === 'GETSFT') {
    if (tokens[cursor]?.text === 'TCH#') { ref('TCH#', 0, 3); threshold(); }
    if (cursor < tokens.length) {
      const start = cursor;
      cursor += grammar.operand(tokens.slice(cursor));
      if (!grammar.typed(tokens.slice(start, cursor), ['P'], false)) throw new Error('GETSFT 保存目标需要 P/LP 位置变量。');
    }
  } else if (opcode === 'SCALIB') {
    if (tokens[cursor]?.text === 'TCH#') ref('TCH#', 0, 6);
  }
  if (cursor !== tokens.length) throw new Error(`${opcode} 含多余、重复或顺序错误的参数。`);
  return { uncheckedVariables };
}
