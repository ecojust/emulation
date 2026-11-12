import type { RobotLanguageDefinition } from '../types';
import { FANUC_ENTRIES, FANUC_SOURCES } from './fanuc-catalog';
import { parseFanucLs } from './fanuc-parser';

export const fanucLanguage: RobotLanguageDefinition = {
  id: 'fanuc-ls', brand: 'FANUC', name: 'FANUC TP / LS', dialect: 'R-30iB TP ASCII LS，Basic B-83284EN/09；已列 Optional /09 扩展（仍有语法缺项）', extensions: ['ls'],
  lexical: { lineComment: '!', commentTerminator: ';', stringEscape: 'none', caseInsensitive: true,
    keywords: [...new Set([...FANUC_ENTRIES.map(entry => entry.name), 'THEN', 'TO', 'DOWNTO', 'TIMEOUT', 'CONDITION', 'ON', 'OFF', 'AND', 'OR', 'DIV', 'MOD', 'LPOS', 'JPOS', 'CONFIG', 'UF', 'UT', 'GP1', 'GP2'])],
    types: ['R', 'PR', 'SR', 'AR', 'P', 'DI', 'DO', 'RI', 'RO', 'GI', 'GO', 'AI', 'AO', 'F'] },
  entries: FANUC_ENTRIES, sources: FANUC_SOURCES,
  coverage: [
    'LS 区段、原始 UTF-16 位置、带／不带行号的多行分号指令、字符串和注释；未知源码保留。',
    '按数值、布尔、字符串、完整位置与码垛数据区分表达式；普通算术、混合逻辑、嵌套索引及位置分量分别检查。',
    'J/L/C/A/S、C 经过点附加选项；IF/FOR/SELECT、CALL 参数、RUN、WAIT、Cond WHEN、标签和寄存器／I/O 赋值。',
    '多运动组 P 定义、笛卡尔／关节形式、独立外轴、CONFIG 和未初始化坐标；局部标签与位置引用检查。',
    'Basic /APPL 的 Handling/Spot/Arc/Dispense 属性与 B/BX/E/EX 码垛 CONFIG、BOTTOM、PATTERN、ROUTE 结构。',
    '已列 Optional /09 的柔顺、路径切换、恢复、Touch、采样、扭矩限制和激光具名参数，分别标注所需选件。',
    '补全包含签名、参数、片段、说明、来源及版本／选件限制。',
  ],
  limitations: [
    '非空 POINT_LOGIC 子数据、E/EX 全点排列枚举及辅助 BOTTOM 键尚缺已核定的 ASCII 序列化样例；需要对应控制器导出的 LS，不能据示教器画面猜造格式。',
    'STORAGE 元数据与包含额外左右构型位的 CONFIG 尚缺已核定的 LS 格式；其他应用专用语法需选定对应手册和版本。完整章节清单及具体证据见覆盖说明。',
    '宏、系统变量、外部程序、动态寄存器、应用选件参数及多组轴配置没有完整语义验证；未知指令明确标记。',
    '编辑器分析不执行 TP 控制流、I/O、CNT、Skip、跟踪或多任务，不把未实现语义当作无操作。',
    '保存 LS 源码不等于生成二进制 TP；实际编译／加载仍需匹配的 FANUC 工具链或 ASCII Upload。',
  ],
  analyze(source) { const { diagnostics, symbols, folds } = parseFanucLs(source); return { diagnostics, symbols, folds }; },
  analyzeTemplate(source) { const { diagnostics, symbols, folds } = parseFanucLs(source, { templateParameters: true }); return { diagnostics, symbols, folds }; },
};
