import type { SourceDiagnostic } from '../types';
import type { FanucToken } from './fanuc-parser';

/** ASCII envelope productions cross-checked with B-83284EN/09 §8.10.5.
 * Generated size/date metadata is parsed, never rewritten or recomputed. */
export function validateFanucAttribute(tokens: readonly FanucToken[], report: (token: FanucToken | undefined, message: string, code?: string, severity?: SourceDiagnostic['severity']) => void): void {
  if (!tokens.length) return;
  const [name, separator, ...values] = tokens;
  if (name.kind !== 'word' || !['=', ':'].includes(separator?.upper)) { report(name, '程序属性需要名称和 = 或 : 分隔符', 'fanuc.attribute'); return; }
  const fail = (message: string) => report(name, message, 'fanuc.attribute');
  const natural = (token: FanucToken | undefined) => token?.kind === 'number' && Number.isSafeInteger(Number(token.text)) && Number(token.text) >= 0;
  if (['PROG_SIZE', 'VERSION', 'LINE_COUNT', 'MEMORY_SIZE'].includes(name.upper)) {
    if (separator.upper !== '=' || values.length !== 1 || !natural(values[0])) fail(`${name.text} 需要单个非负整数`);
  } else if (name.upper === 'COMMENT') {
    if (separator.upper !== '=' || values.length !== 1 || values[0].kind !== 'string' || !values[0].text.startsWith('"')) fail('COMMENT 需要双引号字符串');
    else if (values[0].text.length - 2 > 16) report(name, 'Basic /09 §8.10.5 的 COMMENT 长度为 16；更长注释需核对实际控制器版本', 'fanuc.version', 'warning');
  } else if (name.upper === 'OWNER' || name.upper === 'FILE_NAME') {
    if (separator.upper !== '=' || values.length > 1 || name.upper === 'OWNER' && !values.length || values.some(token => token.kind !== 'word' && token.kind !== 'string')) fail(`${name.text} 需要名称；FILE_NAME 也可为空`);
  } else if (name.upper === 'PROTECT') {
    if (separator.upper !== '=' || values.length !== 1 || !['READ_WRITE', 'READ'].includes(values[0].upper)) fail('PROTECT 需要 READ_WRITE 或 READ');
  } else if (name.upper === 'DEFAULT_GROUP') {
    const groups = values.filter((_, index) => index % 2 === 0);
    if (separator.upper !== '=' || values.length % 2 !== 1 || groups.length < 1 || groups.length > 8 || values.some((token, index) => index % 2 ? token.upper !== ',' : !['1', '*'].includes(token.upper))) fail('DEFAULT_GROUP 需要 1 至 8 个以逗号分隔的 1 或 *');
  } else if (name.upper === 'CONTROL_CODE') {
    if (separator.upper !== '=' || values.length !== 2 || values.some(token => !/^[01]{8}$/.test(token.text))) fail('CONTROL_CODE 需要两个八位二进制字段');
  } else if (name.upper === 'CREATE' || name.upper === 'MODIFIED') {
    // A source file may contain xx in a year placeholder as in the official example.
    const text = values.map(token => token.text).join('');
    if (separator.upper !== '=' || !/^DATE(?:\d{2,4}|xx)-\d{2}-\d{2}TIME\d{2}:\d{2}:\d{2}$/i.test(text)) fail(`${name.text} 需要 DATE yy-mm-dd TIME hh:mm:ss`);
  } else if (name.upper === 'TCD') {
    if (separator.upper !== ':' || !values.length) { fail('TCD 需要 : 和任务属性列表'); return; }
    const fields = new Set<string>();
    for (let at = 0; at < values.length; at += 4) {
      const key = values[at];
      if (!['STACK_SIZE', 'TASK_PRIORITY', 'TIME_SLICE', 'BUSY_LAMP_OFF', 'ABORT_REQUEST', 'PAUSE_REQUEST'].includes(key.upper)) { report(key, '未知 TCD 属性，参数尚未验证', 'fanuc.unsupported-attribute', 'warning'); continue; }
      if (fields.has(key.upper) || values[at + 1]?.upper !== '=' || !natural(values[at + 2]) || at + 3 < values.length && values[at + 3].upper !== ',') fail('TCD 需要不重复的 name=integer，以逗号分隔');
      fields.add(key.upper);
    }
    if (values.length % 4 !== 3) fail('TCD 属性列表不完整或有尾逗号');
  } else if (name.upper !== 'LOCAL_REGISTERS') report(name, `尚未验证属性 ${name.text} 的产生式，已保留原文`, 'fanuc.unsupported-attribute', 'warning');
}
