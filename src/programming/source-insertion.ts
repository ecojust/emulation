import { formatProgramPoint, getProgramProfile, renderProgramTemplate, type ProgramTemplatePoint } from '../workspace/program-templates';
import type { SourceInsertion } from './RobotCodeEditor';

/** Keep UTF-16 offsets while hiding string/comment contents from structural searches. */
function structure(source: string, brand: string): string {
  const out = source.split(''); let quoted = false, comment = false, statementStart = true;
  for (let i = 0; i < source.length; i++) {
    const c = source[i];
    if (c === '\ufeff') { out[i] = ' '; continue; }
    if (c === '\n' || c === '\r') { if (brand !== 'FANUC') comment = false; statementStart = true; continue; }
    if (comment) { out[i] = ' '; if (brand === 'FANUC' && c === ';') { comment = false; statementStart = true; } continue; }
    if (quoted) {
      out[i] = ' ';
      if (c === '\\' && brand === 'ABB' && i + 1 < source.length) { out[++i] = ' '; continue; }
      if (c === '"') { if (source[i + 1] === '"') out[++i] = ' '; else quoted = false; }
      continue;
    }
    if (c === '"') { quoted = true; out[i] = ' '; continue; }
    if ((brand === 'ABB' && c === '!') || (brand === 'Yaskawa' && c === "'")
      || (brand === 'FANUC' && statementStart && (c === '!' || source.slice(i, i + 2) === '--'))) {
      comment = true; out[i] = ' '; continue;
    }
    if (brand === 'FANUC' && c === ';') statementStart = true;
    else if (!/[\s\d:]/.test(c)) statementStart = false;
  }
  return out.join('');
}
function ordinal(mask: string, brand: string): number {
  const expression = brand === 'FANUC' ? /\bP\s*\[\s*(\d+)/gi : brand === 'Yaskawa' ? /\bC(\d{5})\b/gi : /\bP(\d+)\b/gi;
  const used = new Set([...mask.matchAll(expression)].map(match => Number(match[1]) + Number(brand === 'Yaskawa')));
  const maximum = brand === 'Yaskawa' ? 9998 : 99999;
  for (let number = 1; number <= maximum; number++) if (!used.has(number)) return number;
  throw new Error('当前源码的目标点编号已用完。');
}
function afterLine(source: string, start: number): number {
  const newline = /\r\n|\r|\n/.exec(source.slice(start));
  return newline ? start + newline.index + newline[0].length : source.length;
}
function statementPosition(source: string, mask: string, from: number, to: number, begin: number, end: number) {
  if (from >= begin && to <= end) {
    if (to > from) return { from, to };
    const lineStart = Math.max(source.lastIndexOf('\n', from - 1), source.lastIndexOf('\r', from - 1)) + 1;
    const newline = /\r\n|\r|\n/.exec(source.slice(from));
    const lineEnd = newline ? from + newline.index : -1;
    if (!mask.slice(Math.max(begin, lineStart), lineEnd < 0 ? end : lineEnd).trim()) return { from: Math.max(begin, lineStart), to: Math.max(begin, lineStart) };
    const next = lineEnd < 0 ? end : Math.min(end, afterLine(source, from));
    return { from: next, to: next };
  }
  return { from: end, to: end };
}
function finish(changes: SourceInsertion['changes'], statement: SourceInsertion['changes'][number], name: string): SourceInsertion {
  changes.sort((a, b) => a.from - b.from);
  let delta = 0;
  for (const change of changes) {
    if (change === statement) return { changes, cursor: change.from + delta + change.insert.length, message: `已插入当前仿真姿态 ${name}。` };
    delta += change.insert.length - (change.to - change.from);
  }
  return { changes, message: `已插入当前仿真姿态 ${name}。` };
}
export function insertCurrentSourcePose(modelId: string, source: string, from: number, to: number, snapshot: Omit<ProgramTemplatePoint, 'pointName'>): SourceInsertion {
  if (!Number.isInteger(from) || !Number.isInteger(to) || from < 0 || to < from || to > source.length) throw new Error('源码光标位置已失效，请重试。');
  const profile = getProgramProfile(modelId), mask = structure(source, profile.brand), separator = /\r\n|\n|\r/.exec(source)?.[0] ?? '\n';
  const pointName = 'P' + String(ordinal(mask, profile.brand)).padStart(3, '0');
  const point = { ...snapshot, pointName }, formatted = formatProgramPoint(modelId, point);
  if (!source.trim()) {
    const insert = (source.startsWith('\ufeff') ? '\ufeff' : '') + renderProgramTemplate(modelId, [point]).code.replace(/\n/g, separator);
    return { changes: [{ from: 0, to: source.length, insert }], cursor: insert.indexOf(formatted.instruction) + formatted.instruction.length, message: `已创建程序并记录 ${formatted.reference}。` };
  }
  if (profile.brand === 'ABB') {
    // RAPID is token based: modules, routine signatures and their bodies may
    // share a line or span many lines. Never match structural words in strings.
    const modules = [...mask.matchAll(/\bMODULE\s+[A-Za-z\u00c0-\u00d6\u00d8-\u00f6\u00f8-\u00ff][\w\u00c0-\u00d6\u00d8-\u00f6\u00f8-\u00ff]*/gi)].flatMap(match => {
      const headerEnd = match.index! + match[0].length;
      const closing = /\bENDMODULE\b/i.exec(mask.slice(headerEnd));
      return closing ? [{ start: match.index!, headerEnd, end: headerEnd + closing.index }] : [];
    });
    const module = modules.find(item => from >= item.start && to <= item.end) ?? modules[0];
    if (!module) throw new Error('请先建立完整的 MODULE / ENDMODULE，或在场景中将 Path 转换为程序。');
    const routines = [...mask.slice(module.headerEnd, module.end).matchAll(/\b(?:LOCAL\s+)?(PROC|FUNC|TRAP)\b/gi)].flatMap(match => {
      const start = module.headerEnd + match.index!;
      const kind = match[1].toUpperCase();
      let begin: number;
      if (kind === 'TRAP') {
        const name = /^\s+[A-Za-zÀ-ÖØ-öø-ÿ][\wÀ-ÖØ-öø-ÿ]*/.exec(mask.slice(start + match[0].length));
        if (!name) return [];
        begin = start + match[0].length + name[0].length;
      } else {
        const open = mask.indexOf('(', start + match[0].length);
        if (open < 0 || open >= module.end) return [];
        let depth = 1; begin = open + 1;
        while (begin < module.end && depth) {
          if (mask[begin] === '(') depth++;
          if (mask[begin] === ')') depth--;
          begin++;
        }
        if (depth) return [];
      }
      const closing = new RegExp('\\bEND' + kind + '\\b', 'i').exec(mask.slice(begin, module.end));
      if (!closing) return [];
      const end = begin + closing.index, headerEnd = begin;
      // Local declarations must remain before every executable statement.
      while (true) {
        const data = /^\s*(?:LOCAL\s+)?(?:VAR|CONST|PERS)\b/i.exec(mask.slice(begin, end));
        if (!data) break;
        const semi = mask.indexOf(';', begin + data[0].length);
        if (semi < 0 || semi >= end) return [];
        begin = semi + 1;
      }
      // An insertion outside a routine belongs to its ordinary execution body,
      // never to ERROR/UNDO/BACKWARD merely because they precede ENDPROC.
      const handler = /\b(?:BACKWARD|ERROR|UNDO)\b/i.exec(mask.slice(begin, end));
      return [{ start, headerEnd, begin, end, bodyEnd: handler ? begin + handler.index : end, kind,
        main: /^(?:LOCAL\s+)?PROC\s+main\s*\(/i.test(mask.slice(start, headerEnd)) }];
    });
    const routine = routines.find(item => from >= item.headerEnd && to <= item.end)
      ?? routines.find(item => item.main) ?? routines.find(item => item.kind === 'PROC') ?? routines[0];
    const newlineBefore = (offset: number) => offset && !/[\r\n]/.test(source[offset - 1]) ? separator : '';
    if (!routine) {
      if (/\b(?:PROC|FUNC|TRAP)\b/i.test(mask.slice(module.headerEnd, module.end))) throw new Error('当前例程结构不完整，请先修正例程参数和结束语句。');
      const insert = `${newlineBefore(module.end)}  ${formatted.declaration}${separator}  PROC main()${separator}    ${formatted.instruction}${separator}  ENDPROC${separator}`;
      const edit = { from: module.end, to: module.end, insert }; return finish([edit], edit, formatted.reference);
    }
    const declaration = { from: routines[0].start, to: routines[0].start, insert: newlineBefore(routines[0].start) + '  ' + formatted.declaration + separator };
    const position = statementPosition(source, mask, from, to, routine.begin, routine.bodyEnd);
    const statement = { ...position, insert: newlineBefore(position.from) + '    ' + formatted.instruction + separator };
    return finish([declaration, statement], statement, formatted.reference);
  }
  if (profile.brand === 'FANUC') {
    const mn = /^\/MN[\t ]*\r?$/im.exec(mask), pos = /^\/POS[\t ]*\r?$/im.exec(mask), end = /^\/END\b/im.exec(mask);
    if (!mn || !pos || !end || !(mn.index < pos.index && pos.index < end.index)) throw new Error('请先建立 /MN、/POS 和 /END 分区，或在场景中将 Path 转换为程序。');
    const begin = afterLine(source, mn.index);
    const position = statementPosition(source, mask, from, to, begin, pos.index);
    // Unnumbered LS instruction lines are valid; preserve all existing row numbers.
    const statement = { ...position, insert: ': ' + formatted.instruction + separator };
    const declaration = { from: end.index, to: end.index, insert: formatted.declaration.replace(/\n/g, separator) + separator };
    return finish([statement, declaration], statement, formatted.reference);
  }
  const pos = /^\/\/POS[\t ]*\r?$/im.exec(mask), inst = /^\/\/INST[\t ]*\r?$/im.exec(mask);
  const end = inst ? /^END[\t ]*\r?$/im.exec(mask.slice(inst.index)) : null;
  if (!pos || !inst || !end || pos.index >= inst.index) throw new Error('请先建立 //POS、//INST 和 END 分区，或在场景中将 Path 转换为程序。');
  const positionHeaders = mask.slice(pos.index, inst.index);
  const lastFrame = [...positionHeaders.matchAll(/^\/\/\/POSTYPE[\t ]+(\w+)[\t ]*\r?$/gim)].at(-1)?.[1].toUpperCase();
  const lastFormat = [...positionHeaders.matchAll(/^\/\/\/(RECTAN|PULSE)\b/gim)].at(-1)?.[1].toUpperCase();
  if (lastFrame !== 'BASE' || lastFormat !== 'RECTAN') throw new Error('当前仿真姿态使用 BASE 笛卡尔坐标；请使用 BASE / RECTAN 作业，脉冲或用户坐标需要控制器标定。');
  const closing = inst.index + end.index, begin = afterLine(source, inst.index);
  const position = statementPosition(source, mask, from, to, begin, closing);
  const declaration = { from: inst.index, to: inst.index, insert: formatted.declaration.replace(/\n/g, separator) + separator };
  const statement = { ...position, insert: formatted.instruction + separator };
  const edits = [declaration, statement];
  const count = /^\/\/\/NPOS[\t ]+(\d+)([^\r\n]*)/im.exec(mask.slice(pos.index, inst.index));
  if (!count) throw new Error('位置区缺少 NPOS 计数，请先补全作业头。');
  if (count) {
    const original = Number(count[1]), next = original + 1;
    const start = pos.index + count.index + count[0].indexOf(count[1]);
    edits.push({ from: start, to: start + count[1].length, insert: String(next) });
  }
  return finish(edits, statement, formatted.reference);
}
