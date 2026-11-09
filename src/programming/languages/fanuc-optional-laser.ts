import type { LanguageEntry } from '../types';
import { Cursor, SyntaxFault, type FanucToken } from './fanuc-parser';
import type { FanucOptionalReport } from './fanuc-optional';

/** B-83284EN-2/09 §62 grammar; none of these instructions execute laser hardware. */
const manual = 'https://github.com/zhaomengkang/FANUC-Robot-Manual/blob/9e9998c2744b94794414a372063410fa20507fc6/Manual/eDoc/R-30iB%20Plus/content/B-83284EN-2_REV_09/book.pdf';
type Range = readonly [min: number, max: number, integer?: boolean];
type Fields = Readonly<Record<string, Range>>;
const nonnegative: Range = [0, Infinity], bit: Range = [0, 1, true];
const process: Fields = { S: nonnegative, L: nonnegative, P: nonnegative, Q: nonnegative };
const schemas: Record<string, Fields> = {
  LS: { ...process, V: nonnegative },
  LE: { AT: [0, 9999, true], ST: [0, 9999, true], SOF: bit },
  LP: { ...process, I: nonnegative, J: nonnegative, K: [0, Infinity, true], H: [0, Infinity, true], R: [0, Infinity, true] },
  LS_PO: { ...process, SPN: [0, 65535, true], TA: [-99999, 99999, true] },
  LS_TA: { ...process, V: nonnegative, TA: [-99999, 99999, true] },
  WIRE_CONTROL: { TY: [0, 2, true], SP: [1, 32767, true] },
};
function number(cursor: Cursor, range: Range): number {
  const sign = cursor.eat('-') ? -1 : (cursor.eat('+'), 1), token = cursor.take(), value = sign * Number(token.text);
  if (token.kind !== 'number' || !Number.isFinite(value) || range[2] && !Number.isSafeInteger(value) || value < range[0] || value > range[1]) throw new SyntaxFault(token, '激光参数需要已列范围内的' + (range[2] ? '整数' : '数值') + '常量');
  return value;
}
function argumentsFor(cursor: Cursor, kind: string, report: FanucOptionalReport, first: FanucToken) {
  const schema = kind === 'PRE_LS' ? { CH: [1, Infinity, true] as Range, PO: bit } : schemas[kind];
  const seen = new Set<string>();
  // §62.12/13 format diagrams use [], while teaching-screen examples use ().
  const closing = ['LS_PO', 'LS_TA'].includes(kind) && cursor.eat('(') ? ')' : (cursor.expect('['), ']');
  do {
    const key = cursor.take();
    // §62.6.1.4 format specifies I, but its worked example prints V instead.
    const alias = kind === 'LP' && key.upper === 'V';
    const name = alias ? 'I' : key.upper, range = schema[name];
    if (!range) throw new SyntaxFault(key, kind + ' 未定义参数 ' + key.text);
    if (seen.has(name)) throw new SyntaxFault(key, kind + ' 参数不能重复：' + name);
    seen.add(name); cursor.expect('='); number(cursor, range);
    if (alias) report(key, 'LP §62.6.1.4 格式表使用 I，示例使用 V；该拼写差异需目标控制器确认', 'fanuc.manual-spelling-difference', 'warning');
  } while (cursor.eat(','));
  cursor.expect(closing);
  if (kind !== 'PRE_LS') for (const name of Object.keys(schema)) if (!seen.has(name)) {
    if (kind === 'LE' && name === 'SOF') report(first, '§62.9.2 界面例省略 SOF，但§62.6.1.3格式表含SOF；需核对该控制器的默认待机行为', 'fanuc.manual-format-difference', 'warning');
    else throw new SyntaxFault(first, kind + ' 缺少参数 ' + name);
  }
}
function notice(report: FanucOptionalReport, token: FanucToken, section: string) {
  report(token, 'Fiber Laser Interface（Optional /09 §' + section + '）需要实际激光器型号、控制器选件和配置；只检查文本，不控制硬件', 'fanuc.option', 'info');
}
function schedule(cursor: Cursor, min: number) {
  cursor.expect('['); number(cursor, [min, 10, true]); cursor.expect(']');
}
function motionWords(cursor: Cursor): Set<string> {
  const words = new Set<string>(); let depth = 0;
  for (const token of cursor.tokens) {
    if (['[', '('].includes(token.upper)) depth++;
    if (depth === 0) words.add(token.upper);
    if ([']', ')'].includes(token.upper)) depth--;
  }
  return words;
}
export function parseFanucLaserStatement(cursor: Cursor, report: FanucOptionalReport): boolean {
  const first = cursor.token;
  if (!first) return false;
  if (first.upper === 'CALL' && ['LS_PO', 'LS_TA'].includes(cursor.tokens[cursor.index + 1]?.upper ?? '') && ['[', '('].includes(cursor.tokens[cursor.index + 2]?.upper ?? '')) {
    cursor.take(); const kind = cursor.take();
    if (kind.upper === 'LS_TA') throw new SyntaxFault(kind, 'LS_TA 只支持运动中的 PR_CALL，不能使用 CALL（§62.13.1）');
    argumentsFor(cursor, 'LS_PO', report, kind); cursor.end(); notice(report, first, '62.12.5'); return true;
  }
  if (first.upper === 'POWER_CONTROL') {
    cursor.take(); schedule(cursor, 0); cursor.end(); notice(report, first, '62.8.3'); return true;
  }
  if (first.upper === 'WIRE_CONTROL') {
    cursor.take(); argumentsFor(cursor, 'WIRE_CONTROL', report, first); cursor.end(); notice(report, first, '62.10.1'); return true;
  }
  if (first.upper === 'MINUTE_OUTPUT_RQ') {
    cursor.take(); cursor.end(); notice(report, first, '62.11.1'); return true;
  }
  if (['LS', 'LE', 'LP', 'PRE_LS', 'SLOPEUP', 'SLOPEDOWN', 'LS_PO', 'LS_TA'].includes(first.upper)
    || first.upper === 'PR_CALL' && ['LS_PO', 'LS_TA'].includes(cursor.tokens[cursor.index + 1]?.upper ?? '')) {
    throw new SyntaxFault(first, '此激光产生式需要附加在运动行；LS_PO 也可使用 CALL（§62）');
  }
  return false;
}
export function parseFanucLaserMotionOption(cursor: Cursor, motionType: string, report: FanucOptionalReport): boolean {
  const first = cursor.token;
  if (!first) return false;
  if (motionType === 'S' && (['PRE_LS', 'LS', 'LE', 'LP', 'SLOPEUP', 'SLOPEDOWN'].includes(first.upper)
    || first.upper === 'PR_CALL' && ['LS_PO', 'LS_TA'].includes(cursor.tokens[cursor.index + 1]?.upper ?? ''))) {
    throw new SyntaxFault(first, '样条运动不支持此激光附加选项（§65.9；§62 未定义 S 扩展）');
  }
  if (first.upper === 'PR_CALL' && ['LS_PO', 'LS_TA'].includes(cursor.tokens[cursor.index + 1]?.upper ?? '')) {
    cursor.take(); const kind = cursor.take(); argumentsFor(cursor, kind.upper, report, kind);
    const words = motionWords(cursor);
    if (kind.upper === 'LS_TA' && (words.has('SLOPEUP') || words.has('SLOPEDOWN'))) throw new SyntaxFault(first, 'PR_CALL LS_TA 不能使用斜坡控制（§62.13.1）');
    notice(report, first, kind.upper === 'LS_TA' ? '62.13.1' : '62.12.5'); return true;
  }
  if (first.upper === 'PRE_LS') {
    cursor.take(); if (cursor.is('[')) argumentsFor(cursor, 'PRE_LS', report, first);
    notice(report, first, '62.6.1.1/62.7.4/62.12.4'); return true;
  }
  if (['LS', 'LE', 'LP'].includes(first.upper)) {
    cursor.take(); argumentsFor(cursor, first.upper, report, first); notice(report, first, '62.6.1'); return true;
  }
  if (first.upper === 'SLOPEUP' || first.upper === 'SLOPEDOWN') {
    cursor.take(); schedule(cursor, 1);
    const words = motionWords(cursor), required = first.upper === 'SLOPEUP' ? 'LS' : 'LE';
    if (!words.has(required)) throw new SyntaxFault(first, first.upper + ' 必须与 ' + required + ' 在同一运动行（§62.9）');
    if (!['L', 'C', 'A'].includes(motionType)) throw new SyntaxFault(first, '斜坡控制只适用于直线或圆周运动（§62.9.3）');
    notice(report, first, first.upper === 'SLOPEUP' ? '62.9.1.3' : '62.9.2.3'); return true;
  }
  if (first.upper === 'LS_PO' || first.upper === 'LS_TA') throw new SyntaxFault(first, first.upper + ' 需要 PR_CALL 前缀');
  if (['POWER_CONTROL', 'WIRE_CONTROL', 'MINUTE_OUTPUT_RQ'].includes(first.upper)) throw new SyntaxFault(first, '此 Sub Laser 产生式需要独立指令行（§62.6.2）');
  return false;
}

const parameterDescriptions: Record<string, string> = {
  S: '峰值功率，W；范围由激光器型号确定。', L: '脉冲底部功率，W。', P: '脉冲频率，Hz。', Q: '脉冲占空比，实际范围由激光器型号确定。',
  V: '处理速度；单位和范围由Laser Initial Setup确定。', AT: '气体后流时间，0至9999毫秒。', ST: '关闭快门延迟，0至9999毫秒。', SOF: '待机关闭标志，0或1。',
  I: '初始频率的步进增量，Hz；手册示例存在I/V拼写差异。', J: '占空比步进增量。', K: '每步时间，毫秒。', H: '步数。', R: '穿孔时间，毫秒。',
  SPN: '扫描程序号，0至65535。', TA: '时间偏移，-99999至99999毫秒。', TY: '送丝方式，0关闭/1恒定/2随速。', SP: '送丝速度，1至32767。',
};
const laserEntry = (name: string, signature: string, section: string, fields: readonly string[] = []): LanguageEntry => ({
  name, kind: 'instruction', signature, description: 'Fiber Laser Interface §' + section + ' 的源码形式；不执行激光输出或I/O，参数范围还依赖实际激光器。',
  category: 'Optional /09 §' + section, documentationUrl: manual, requiresOption: 'Fiber Laser Interface 及实际激光器配置',
  insertText: ({ PRE_LS: 'PRE_LS', POWER_CONTROL: 'POWER_CONTROL[1]', SLOPEUP: 'SLOPEUP[1]', SLOPEDOWN: 'SLOPEDOWN[1]' } as Record<string, string>)[name] ?? signature,
  parameters: name === 'PRE_LS'
    ? [{ name: 'CH', type: 'positive integer', description: '可选的激光请求通道；数量依现场配置。', optional: true }, { name: 'PO', type: '0 | 1', description: '可选直接功率覆盖模式。', optional: true }]
    : ['POWER_CONTROL', 'SLOPEUP', 'SLOPEDOWN'].includes(name)
      ? [{ name: 'schedule', type: name === 'POWER_CONTROL' ? 'integer 0..10' : 'integer 1..10', description: '已配置的计划编号；POWER_CONTROL[0]关闭功率跟随。' }]
      : fields.map(name => ({ name, type: name === 'TA' ? 'signed integer' : 'number', description: parameterDescriptions[name] })),
});
export const FANUC_LASER_ENTRIES: readonly LanguageEntry[] = [
  laserEntry('PRE_LS', 'PRE_LS [CH=channel] | PRE_LS [PO=0|1]', '62.6.1.1/62.7.4/62.12.4'),
  laserEntry('LS', 'LS[S=1000,L=100,P=1000,Q=100,V=1000]', '62.6.1.2', ['S', 'L', 'P', 'Q', 'V']),
  laserEntry('LE', 'LE[AT=0,ST=0,SOF=0]', '62.6.1.3', ['AT', 'ST', 'SOF']),
  laserEntry('LP', 'LP[S=1000,L=100,P=1000,Q=100,I=10,J=5,K=500,H=100,R=3000]', '62.6.1.4', ['S', 'L', 'P', 'Q', 'I', 'J', 'K', 'H', 'R']),
  laserEntry('POWER_CONTROL', 'POWER_CONTROL[0..10]', '62.8.3'),
  laserEntry('SLOPEUP', 'SLOPEUP[1..10]', '62.9.1.3'),
  laserEntry('SLOPEDOWN', 'SLOPEDOWN[1..10]', '62.9.2.3'),
  laserEntry('WIRE_CONTROL', 'WIRE_CONTROL[Ty=1,Sp=200]', '62.10.1', ['TY', 'SP']),
  laserEntry('MINUTE_OUTPUT_RQ', 'MINUTE_OUTPUT_RQ', '62.11.1'),
  laserEntry('LS_PO', 'CALL LS_PO(S=4000,L=0,P=5,Q=100,SPN=55,TA=0)', '62.12.5', ['S', 'L', 'P', 'Q', 'SPN', 'TA']),
  laserEntry('LS_TA', 'PR_CALL LS_TA(S=5000,L=0,P=5,Q=100,V=200,TA=-20)', '62.13.1', ['S', 'L', 'P', 'Q', 'V', 'TA']),
];
