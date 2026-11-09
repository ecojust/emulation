import type { LanguageEntry, SourceDiagnostic } from '../types';
import { Cursor, SyntaxFault, type FanucToken } from './fanuc-parser';
import { FANUC_LASER_ENTRIES, parseFanucLaserMotionOption, parseFanucLaserStatement } from './fanuc-optional-laser';

/** Original grammar implementation from B-83284EN-2/09, fixed public manual mirror. */
const manual = 'https://github.com/zhaomengkang/FANUC-Robot-Manual/blob/9e9998c2744b94794414a372063410fa20507fc6/Manual/eDoc/R-30iB%20Plus/content/B-83284EN-2_REV_09/book.pdf';
export type FanucOptionalReport = (token: FanucToken | undefined, message: string, code?: string, severity?: SourceDiagnostic['severity']) => void;
export interface FanucOptionalDelegates { assignment(cursor: Cursor): void; condition(cursor: Cursor): void }
export const FANUC_OPTIONAL_AUDIT = [
  { section: '5', classification: 'no-new-TP-production', reason: 'Operation group DO uses the Motion DO setup menu and existing DO/RO signals.' },
  { section: '13.1', classification: 'no-new-TP-production', reason: 'Soft limits are set using the Limit Set menu, not a SOFTLIMIT instruction.' },
  { section: '15.1.2', classification: 'no-new-TP-production', reason: 'Singularity avoidance is a per-program DETAIL setting, not an AVOID instruction.' },
  { section: '17', classification: 'no-new-TP-production', reason: 'Local stop uses dedicated hardware, configured I/O and system variables, not a LOCAL_STOP opcode.' },
  { section: '40.2', classification: 'existing-TP-CALL-production', reason: 'Touch Skip calls SETSKCOL(s,g) and CLSKP_Gg through ordinary CALL; no TOUCH_SKIP opcode.' },
] as const;

function literal(cursor: Cursor, integer = false, min = -Infinity, max = Infinity): number {
  const sign = cursor.eat('-') ? -1 : (cursor.eat('+'), 1), token = cursor.take();
  const number = sign * Number(token.text);
  if (token.kind !== 'number' || !Number.isFinite(number) || integer && !Number.isSafeInteger(number) || number < min || number > max) {
    throw new SyntaxFault(token, '需要范围内的' + (integer ? '整数' : '数值') + '常量');
  }
  return number;
}
function register(cursor: Cursor, keywordConsumed = false, depth = 0): void {
  if (depth > 64) throw new SyntaxFault(cursor.token, '寄存器间接索引超过 64 层');
  if (!keywordConsumed) cursor.expect('R');
  cursor.expect('[');
  if (cursor.is('R')) register(cursor, false, depth + 1);
  else literal(cursor, true, 1);
  if (cursor.eat(':')) while (!cursor.done && !cursor.is(']')) cursor.take();
  cursor.expect(']');
}
function numberOrRegister(cursor: Cursor, integer = false, min = -Infinity, max = Infinity): void {
  if (cursor.is('R')) register(cursor);
  else literal(cursor, integer, min, max);
}
function indexedSchedule(cursor: Cursor, maximum: number): number {
  cursor.expect('['); const value = literal(cursor, true, 1, maximum);
  if (cursor.eat(':')) while (!cursor.done && !cursor.is(']')) cursor.take();
  cursor.expect(']');
  return value;
}
function softfloatSchedule(cursor: Cursor, report: FanucOptionalReport, first: FanucToken) {
  const value = indexedSchedule(cursor, Infinity);
  if (value > 10) report(first, '§3.1只说明通常有10个Softfloat计划；此编号超出通常配置，需核对控制器计划数量', 'fanuc.profile-range', 'warning');
}
function indirectIndex(cursor: Cursor, maximum = Infinity): void {
  cursor.expect('['); numberOrRegister(cursor, true, 1, maximum);
  if (cursor.eat(':')) while (!cursor.done && !cursor.is(']')) cursor.take();
  cursor.expect(']');
}
function optionNotice(report: FanucOptionalReport, token: FanucToken, section: string, option: string) {
  report(token, option + ' 依赖控制器选件和现场配置（Optional /09 §' + section + '）；此处仅检查源码语法', 'fanuc.option', 'info');
}
function programName(cursor: Cursor) {
  const name = cursor.take();
  if (name.kind !== 'word' || !/^[A-Za-z_][A-Za-z0-9_]{0,35}$/.test(name.text) || /^(CON|PRN|AUX|NUL|COM[1-9]|LPT[1-9])$/i.test(name.text)) throw new SyntaxFault(name, '恢复程序需要有效的固定程序名称');
}
/** Ignore human-readable reference comments when validating optional operands. */
function semanticTokens(tokens: readonly FanucToken[]): FanucToken[] {
  const output: FanucToken[] = [];
  let depth = 0, commentAt = -1;
  for (let index = 0; index < tokens.length; index++) {
    const token = tokens[index];
    if (token.upper === ']') {
      if (commentAt === depth) commentAt = -1;
      depth--;
    }
    if (commentAt >= 0) continue;
    if (token.upper === ':' && depth > 0 && !/^GP[1-8]$/.test(tokens[index - 1]?.upper ?? '')) { commentAt = depth; continue; }
    output.push(token);
    if (token.upper === '[') depth++;
  }
  return output;
}
function parsePathSwitch(cursor: Cursor, report: FanucOptionalReport, delegates: FanucOptionalDelegates, first: FanucToken) {
  literal(cursor); cursor.expect('MM'); literal(cursor); cursor.expect('SEC'); cursor.expect(',');
  const body = semanticTokens(cursor.tokens.slice(cursor.index));
  const forbidden = new Set(['AR', 'SR', 'PL', 'LPOS', 'JPOS', 'PULSE', 'START', 'STOP', 'RESET', 'SIN', 'COS', 'TAN', 'ASIN', 'ACOS', 'ATAN', 'ATAN2', 'SQRT', 'ABS', 'TRUNC', 'ROUND', 'LN', 'EXP', 'XOR']);
  for (let index = 0; index < body.length; index++) {
    const token = body[index];
    if (token.kind === 'string' || forbidden.has(token.upper) && body[index - 1]?.upper !== '.') throw new SyntaxFault(token, 'PS 只接受 §16 列出的混合逻辑操作数；不接受 AR、字符串、函数或脉冲');
    if (token.upper === 'PR' && body[index - 1]?.upper !== '.') {
      let depth = 0, component = false;
      for (let at = index + 1; at < body.length; at++) {
        if (body[at].upper === '[') depth++;
        if (body[at].upper === ',' && depth === 1) component = true;
        if (body[at].upper === ']' && --depth === 0) break;
      }
      if (!component) throw new SyntaxFault(token, 'PS 只接受 PR[i,j] 分量，不接受完整位置');
    }
  }
  const operations = new Set(['+', '-', '*', '/', 'MOD', 'DIV', 'AND', 'OR', '!', 'NOT', '=', '<', '>', '<=', '>=', '<>']);
  let assignmentCount = 0;
  if (cursor.eat('IF')) {
    if (!cursor.is('(')) throw new SyntaxFault(cursor.token, 'PS 的 IF 条件需要圆括号');
    delegates.condition(cursor); cursor.expect(',');
  }
  if (cursor.eat('TC_ONLINE')) {
    if (!cursor.is('(')) throw new SyntaxFault(cursor.token, 'TC_ONLINE 需要圆括号中的布尔条件');
    delegates.condition(cursor);
  } else {
    assignmentCount = 1;
    const target = cursor.token;
    if (!target || !['DO', 'RO', 'WO', 'AO', 'GO', 'SO', 'UO', 'F', 'R', 'PR', 'M'].includes(target.upper) && !target.text.startsWith('$')) throw new SyntaxFault(target, 'PS 左值必须是 §16 列出的输出、标量寄存器、位置分量或系统变量');
    delegates.assignment(cursor);
  }
  // The assignment "=" is not an expression operation.
  if (body.filter(token => operations.has(token.upper)).length - assignmentCount > 20) throw new SyntaxFault(first, '单个 PS 的条件和右侧公式最多 20 个运算；跨行总量还需控制器验证');
  cursor.end(); optionNotice(report, first, '16', 'Path Switching J693');
}

export function parseFanucOptionalStatement(cursor: Cursor, report: FanucOptionalReport, delegates: FanucOptionalDelegates): boolean {
  if (parseFanucLaserStatement(cursor, report)) return true;
  const first = cursor.token;
  if (!first) return false;
  if (first.upper === 'SOFTFLOAT') {
    cursor.take();
    if (!cursor.eat('END')) softfloatSchedule(cursor, report, first);
    cursor.end(); optionNotice(report, first, '3.1', 'Softfloat'); return true;
  }
  if (first.upper === 'FOLLOW' && cursor.tokens[cursor.index + 1]?.upper === 'UP') {
    cursor.take(); cursor.expect('UP'); cursor.end(); optionNotice(report, first, '3.1', 'Softfloat FOLLOW UP'); return true;
  }
  if (first.upper === 'COL' && cursor.tokens[cursor.index + 1]?.upper === 'GUARD') {
    cursor.take(); cursor.expect('GUARD'); cursor.expect('ADJUST');
    let groupMask = false;
    if (/^GP\d+$/.test(cursor.token?.upper ?? '')) {
      groupMask = true;
      const token = cursor.take(), group = Number(token.upper.slice(2)), seen = new Set<number>([group]);
      if (group < 1 || group > 8) throw new SyntaxFault(token, '运动组范围为 GP1 至 GP8');
      while (cursor.eat(',')) { const next = cursor.token, group = literal(cursor, true, 1, 8); if (seen.has(group)) throw new SyntaxFault(next, '运动组不能重复'); seen.add(group); }
      cursor.expect(':');
    }
    if (!cursor.done || groupMask) numberOrRegister(cursor, true, 1, 200);
    cursor.end(); optionNotice(report, first, '8.4.2', 'High-Sensitivity Collision Detection'); return true;
  }
  if (['RESUME_PROG', 'MAINT_PROG'].includes(first.upper) && cursor.tokens[cursor.index + 1]?.upper === '[') {
    cursor.take(); indexedSchedule(cursor, 5); cursor.expect('='); programName(cursor); cursor.end();
    optionNotice(report, first, first.upper === 'RESUME_PROG' ? '6.4' : '6.6', 'Automatic Error Recovery J924'); return true;
  }
  if (['CLEAR_RESUME_PROG', 'RETURN_PATH_DSBL', 'RESUME_PATH_DSBL'].includes(first.upper)) {
    cursor.take(); cursor.end(); optionNotice(report, first, first.upper === 'CLEAR_RESUME_PROG' ? '6.4' : '6.5', 'Automatic Error Recovery J924');
    if (first.upper === 'RESUME_PATH_DSBL') report(first, '§6.5 正文使用 RETURN_PATH_DSBL，但示例出现 RESUME_PATH_DSBL；需核对目标控制器拼写', 'fanuc.manual-spelling-difference', 'warning');
    return true;
  }
  if (first.upper === 'PS') { cursor.take(); parsePathSwitch(cursor, report, delegates, first); return true; }
  if (first.upper === 'SAMPLE') {
    cursor.take();
    if (cursor.eat('START')) indexedSchedule(cursor, 5); else cursor.expect('END');
    cursor.end(); optionNotice(report, first, '21.3', 'Data Monitor'); return true;
  }
  if (first.upper === 'STITCH') {
    cursor.take();
    if (!cursor.eat('END')) indexedSchedule(cursor, 20);
    cursor.end(); optionNotice(report, first, '50.1.1–50.1.2', 'Stitch'); return true;
  }
  if (first.upper === 'TORQ_LIMIT') {
    cursor.take(); literal(cursor, false, 0, 100); cursor.expect('%'); cursor.end();
    optionNotice(report, first, '27.1', 'Torque Limit'); return true;
  }
  if (first.upper === 'SEARCH' && ['START', 'END'].includes(cursor.tokens[cursor.index + 1]?.upper ?? '')) {
    cursor.take();
    if (cursor.eat('START')) { indirectIndex(cursor, 32); cursor.expect('PR'); indirectIndex(cursor); }
    else cursor.expect('END');
    cursor.end(); optionNotice(report, first, '39.5', 'Touch Sensing'); return true;
  }
  if (first.upper === 'TOUCH' && cursor.tokens[cursor.index + 1]?.upper === 'OFFSET') {
    cursor.take(); cursor.expect('OFFSET');
    if (!cursor.eat('END')) { cursor.expect('PR'); indirectIndex(cursor); }
    cursor.end(); optionNotice(report, first, '39.5', 'Touch Sensing'); return true;
  }
  return false;
}

function topLevelTokens(tokens: readonly FanucToken[]): FanucToken[] {
  const output: FanucToken[] = []; let depth = 0;
  for (const token of tokens) {
    if (['[', '('].includes(token.upper)) depth++;
    if (depth === 0) output.push(token);
    if ([']', ')'].includes(token.upper)) depth--;
  }
  return output;
}
export function parseFanucOptionalMotionOption(cursor: Cursor, motionType: string, report: FanucOptionalReport): boolean {
  if (parseFanucLaserMotionOption(cursor, motionType, report)) return true;
  const first = cursor.token;
  if (!first) return false;
  if (first.upper === 'STITCH') throw new SyntaxFault(first, 'STITCH 只能是独立指令，不能附加在运动行（§50.1.1）');
  if (first.upper === 'SEARCH') {
    if (motionType === 'S') throw new SyntaxFault(first, 'Search 不支持样条运动（§65.9）');
    cursor.take(); cursor.expect('['); cursor.eat('-');
    const axis = cursor.take();
    if (!['X', 'Y', 'Z'].includes(axis.upper)) throw new SyntaxFault(axis, 'Search 方向必须是 X/Y/Z 或 -X/-Y/-Z');
    cursor.expect(']');
    if (!cursor.done && !/^GP[1-8]$/.test(cursor.token.upper)) throw new SyntaxFault(cursor.token, 'Search 必须是此运动的最后一个附加选项（§39.5）');
    optionNotice(report, first, '39.5', 'Touch Sensing'); return true;
  }
  if (first.upper === 'SOFTFLOAT') {
    if (motionType === 'S') throw new SyntaxFault(first, '样条附加选项不包含 SOFTFLOAT（§65.9）');
    cursor.take(); softfloatSchedule(cursor, report, first); optionNotice(report, first, '3.1', 'Joint Softfloat（笛卡尔Softfloat不支持此运动附加形式）'); return true;
  }
  const match = /^(CTV|PSPD|AP_LD|RT_LD)(R|\d+(?:E\d+)?)?$/.exec(first.upper);
  if (!match) return false;
  cursor.take();
  const kind = match[1], suffix = match[2];
  if (['AP_LD', 'RT_LD'].includes(kind) && motionType !== 'L') throw new SyntaxFault(first, 'Linear Distance 仅用于 L（§14.1.2）');
  if (kind === 'CTV' && motionType === 'S') throw new SyntaxFault(first, 'CTV 不支持样条运动（§65.9）');
  const min = kind === 'CTV' ? -100 : kind === 'PSPD' ? -Infinity : 0;
  const max = kind === 'CTV' ? 100 : Infinity;
  const integer = kind === 'CTV' || kind === 'PSPD';
  if (suffix === 'R') {
    if (kind === 'PSPD') throw new SyntaxFault(first, 'PSPD /09 §14.3 给出整数常量，未给出间接寄存器形式');
    register(cursor, true);
  } else if (suffix) {
    let value = suffix;
    if (cursor.token?.kind === 'number' && cursor.token.text.startsWith('.') && cursor.token.from === first.to) value += cursor.take().text;
    const amount = Number(value);
    if (!Number.isFinite(amount) || integer && !Number.isSafeInteger(amount) || amount < min || amount > max) throw new SyntaxFault(first, kind + ' 数值超出语法范围');
  } else if (kind !== 'PSPD' && cursor.is('R')) register(cursor);
  else literal(cursor, integer, min, max);
  if (kind === 'AP_LD' || kind === 'RT_LD') {
    const incompatible = topLevelTokens(cursor.tokens).find(token => token.upper === 'MAX_SPEED' || /^CTV(?:R|\d+)?$/.test(token.upper));
    if (incompatible) throw new SyntaxFault(incompatible, 'Linear Distance 不能与 max_speed 或 Continuous Turn 同用（§14.1.2）');
  }
  optionNotice(report, first, kind === 'CTV' ? '4.3' : kind === 'PSPD' ? '14.3' : '14.1', kind === 'CTV' ? 'Continuous Rotation' : 'Advanced Constant Path');
  return true;
}
/** A speed production, not a trailing motion option. */
export function parseFanucOptionalMotionSpeed(cursor: Cursor, motionType: string, report: FanucOptionalReport): boolean {
  if (!cursor.is('MAX_SPEED')) return false;
  const first = cursor.take();
  if (motionType !== 'L') throw new SyntaxFault(first, 'max_speed 仅用于 L（§14.4）');
  optionNotice(report, first, '14.4', 'Max Speed'); return true;
}

const entry = (name: string, signature: string, section: string, description: string, insertText = signature): LanguageEntry => ({
  name, kind: 'instruction', signature, description, category: 'Optional /09 §' + section,
  documentationUrl: manual, requiresOption: '所述功能选件及对应现场配置（§' + section + '）',
  insertText: insertText.replace(/<(\w+)>/g, (_, name: string) => '$' + '{' + name + '}'),
  parameters: optionalParameters[name],
});
const optionalParameters: Record<string, LanguageEntry['parameters']> = {
  SOFTFLOAT: [{ name: 'schedule', type: 'positive integer', description: '已配置的柔顺计划；手册说明通常10个，更高编号需核对配置；END形式不带编号。' }],
  CTV: [{ name: 'percent', type: 'integer -100..100 | R[]', description: '连续旋转速度相对轴最大速度的比例，负号表示相反方向。' }],
  'COL GUARD ADJUST': [
    { name: 'groups', type: 'GP1,2:', description: '可选且不重复的运动组编号；指定组掩码时必须提供灵敏度。', optional: true },
    { name: 'sensitivity', type: 'integer 1..200 | R[]', description: '灵敏度百分比；省略时读取碰撞检测设置中指定的寄存器。', optional: true },
  ],
  RESUME_PROG: [{ name: 'slot', type: 'integer 1..5', description: '恢复程序登记槽；2至5需要多任务。' }, { name: 'program', type: 'program name', description: '固定程序名；不接受CALL参数列表。' }],
  MAINT_PROG: [{ name: 'slot', type: 'integer 1..5', description: '维护程序登记槽。' }, { name: 'program', type: 'program name', description: '固定维护程序名。' }],
  AP_LD: [{ name: 'distance', type: 'nonnegative number | R[]', description: '终点前要求保持直线的距离，单位毫米。' }],
  RT_LD: [{ name: 'distance', type: 'nonnegative number | R[]', description: '起点后要求保持直线的距离，单位毫米。' }],
  PSPD: [{ name: 'percent', type: 'integer', description: '100为原处理速度；此节未给出控制器完整上下限，实际可用值需核对。' }],
  PS: [
    { name: 'distance', type: 'signed number mm', description: '相对运动终点的触发距离，负值表示终点之前。' },
    { name: 'time', type: 'signed number sec', description: '按运动速度折算成距离的时间偏移。' },
    { name: 'condition', type: '(boolean expression)', description: '可选IF条件，禁止AR[]。', optional: true },
    { name: 'assignment', type: 'mixed-logic assignment | TC_ONLINE(condition)', description: '只接受本节列出的I/O、标量寄存器、PR分量及系统变量；不会实际执行。' },
  ],
  'Sample Start': [{ name: 'schedule', type: 'integer 1..5', description: 'Data Monitor的五个已配置采样计划之一。' }],
  STITCH: [{ name: 'schedule', type: 'integer 1..20', description: 'Stitch Data List 中的条件编号；只接受独立指令，END 形式不带编号。' }],
  TORQ_LIMIT: [{ name: 'percent', type: 'number 0..100 %', description: '语法给出0.0至100.0；实际配置的有效下限与电机保护仍由控制器约束。' }],
  'Search Start': [{ name: 'schedule', type: 'integer 1..32 | R[]', description: 'Touch Sensing计划号。' }, { name: 'positionRegister', type: 'PR[integer | R[]]', description: '存储测量位置或偏移的完整位置寄存器，不能使用PR分量。' }],
  'Touch Offset': [{ name: 'positionRegister', type: 'PR[integer | R[]]', description: '包含Touch Sensing位置偏移的完整PR。' }],
  Search: [{ name: 'axis', type: 'X | Y | Z | -X | -Y | -Z', description: 'Touch Frame中的搜索方向；必须放在此运动的最后一个附加选项。' }],
};
export const FANUC_OPTIONAL_ENTRIES: readonly LanguageEntry[] = [
  ...FANUC_LASER_ENTRIES,
  entry('SOFTFLOAT', 'SOFTFLOAT[schedule] | SOFTFLOAT END', '3.1', '启用指定柔顺计划或结束；通常10个计划。关节柔顺还支持运动附加形式，笛卡尔柔顺不支持该附加形式。', 'SOFTFLOAT[<schedule>]'),
  entry('FOLLOW UP', 'FOLLOW UP', '3.1', '将当前受外力影响的位置作为后续保持位置；只检查语法，不模拟外力。'),
  entry('CTV', 'CTV -100..100 | CTV R[index]', '4.3', '连续旋转的附加速度，百分比可正负或由R间接给出；不可单独作为指令。', 'CTV<percent>'),
  entry('COL GUARD ADJUST', 'COL GUARD ADJUST [GP1,2: ][1..200 | R[index]]', '8.4.2', '临时更改碰撞检测灵敏度；无参数时使用设置菜单指定寄存器，指定组掩码时必须带参数。', 'COL GUARD ADJUST <sensitivity>'),
  entry('RESUME_PROG', 'RESUME_PROG[1..5:comment]=program', '6.4', '定义恢复程序；编号2至5还依赖多任务配置。', 'RESUME_PROG[<slot>]=<PROGRAM>'),
  entry('MAINT_PROG', 'MAINT_PROG[1..5:comment]=program', '6.6', '定义维护程序；程序类型、组掩码及运行状态限制需对应控制器验证。', 'MAINT_PROG[<slot>]=<PROGRAM>'),
  entry('CLEAR_RESUME_PROG', 'CLEAR_RESUME_PROG', '6.4', '清除当前程序登记的恢复程序。'),
  entry('RETURN_PATH_DSBL', 'RETURN_PATH_DSBL', '6.5', '恢复程序中禁用沿原路径返回；手册示例另出现RESUME_PATH_DSBL拼写，需要控制器核对。'),
  entry('AP_LD', 'AP_LD distance | AP_LDR[index]', '14.1', 'L运动的接近直线距离，非负毫米值或R寄存器；不与max_speed或CTV同用。', 'AP_LD<distance>'),
  entry('RT_LD', 'RT_LD distance | RT_LDR[index]', '14.1', 'L运动的撤离直线距离，非负毫米值或R寄存器。', 'RT_LD<distance>'),
  entry('PSPD', 'PSPD integer', '14.3', '路径处理速度倍率，100保持原倍率；手册此节未指定控制器的完整数值边界。', 'PSPD<percent>'),
  entry('max_speed', 'L P[index] max_speed FINE|CNTn', '14.4', '替代L速度字段的最大速度模式，不是后置运动选项。', 'max_speed'),
  entry('PS', 'PS signedDistance mm signedTime sec,[IF(condition),]assignment', '16', '路径触发的混合逻辑赋值或TC_ONLINE；禁止AR，条件与公式有20运算上限；不执行I/O。', 'PS -<distance>mm +<time>sec,DO[<output>]=(ON)'),
  entry('Sample Start', 'Sample Start[1..5]', '21.3', '按指定Data Monitor计划开始采样；并发采样限制需控制器确认。', 'Sample Start[<schedule>]'),
  entry('Sample End', 'Sample End', '21.3', '结束当前Data Monitor采样。'),
  entry('STITCH', 'STITCH[1..20]', '50.1.1–50.1.2', '开始指定条件的间歇密封或激光焊接；独立指令，不是运动附加选项，不模拟信号时序或设备。', 'STITCH[<schedule>]'),
  entry('STITCH END', 'STITCH END', '50.1.1', '结束当前 Stitch 区间；实际设备输出和区间运动约束仍由控制器验证。'),
  entry('TORQ_LIMIT', 'TORQ_LIMIT 0.0..100.0%', '27.1', '设置已配置轴的转矩上限；这里不模拟电机电流或安全性能。', 'TORQ_LIMIT <percent>%'),
  entry('Search Start', 'Search Start[schedule | R[]] PR[index | R[]]', '39.5', '开始触碰搜索，使用计划中的速度和触碰坐标系并保存至指定PR。', 'Search Start[<schedule>] PR[<register>]'),
  entry('Search End', 'Search End', '39.5', '结束一次Touch Sensing搜索。'),
  entry('Touch Offset', 'Touch Offset PR[index | R[]]', '39.5', '对后续程序点应用Touch Sensing得到的位置偏移。', 'Touch Offset PR[<register>]'),
  entry('Touch Offset End', 'Touch Offset End', '39.5', '结束Touch Sensing位置偏移。'),
  entry('Search', 'Search[X|Y|Z|-X|-Y|-Z]', '39.5', '运动附加搜索方向；该运动的示教位置/速度不代表真实搜索终点/速度。', 'Search[<axis>]'),
];
