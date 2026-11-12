import type { RobotLanguageDefinition } from '../types';
import { informEntries, informIndexSource } from './inform-catalog';
import { analyzeInform } from './inform-parser';
import { informOptionSources } from './inform-options';

export const informLanguage: RobotLanguageDefinition = {
  id: 'inform', brand: 'Yaskawa', name: 'INFORM', dialect: 'YRC1000 expanded · 外部 JBI / 178649-1CD revision 10', extensions: ['.jbi'],
  lexical: { lineComment: "'", caseInsensitive: true, stringEscape: 'none',
    keywords: [...new Set([...informEntries.map(entry => entry.name), 'ON', 'OFF', 'START', 'TO', 'STEP', 'JOB', 'VJ', 'V', 'VR', 'VE', 'NWAIT', 'PL', 'ACC', 'DEC'])],
    types: ['B', 'I', 'D', 'R', 'S', 'P', 'BP', 'EX', 'LB', 'LI', 'LD', 'LR', 'LS', 'LP', 'LBP', 'LEX'] },
  entries: informEntries,
  sources: [
    { title: informIndexSource.title, url: informIndexSource.pdfUrl, revision: informIndexSource.manual, license: '厂商文档仅作语法事实参考；正文与 PDF 未分发' },
    { title: 'INFORM Header Fields', url: 'https://knowledge.motoman.com/hc/en-us/articles/21577951275031-INFORM-Header-Fields' },
    ...informOptionSources,
  ],
  coverage: [
    '主手册 117 个指令/附加语法条目均有对应生产式、参数形状检查及合法/最小非法样例矩阵；117 项包括附加项，不是 117 条独立指令。',
    '外部 JBI 分区、DATE/ATTR/FRAME/GROUP/LVARS/RCONF、脉冲/笛卡尔位置记录、计数与位置引用、标签和作业调用参数。',
    'IFTHENEXP/ELSEIFEXP/WHILEEXP、SWITCH 首行 CASE、FOR/NEXT、嵌套/折叠；数值表达式、类型化关系条件、数组和 I/O 引用。',
    'I/O、算术/字符串/矩阵、系统变量/文件/寄存器/参数、平移、对话框、弧焊/点焊/搬运指令的参数分支；区分 P/PX、全局/局部及标量类型。',
    '各运动指令分别检查目标、可用标签、标签顺序和互斥组；解析 ENWAIT、UNTIL/CAPTURE、TRAP、+SFT/+SFTON3D。',
    '独立 Search rev1 的 SRCH：输入编号/位条件、可选延时、必填距离、变量类型与常量范围；Sensor rev3 的 ASRCH 常量参数、方向与顺序。',
    'Sensor rev3 的 ACORON/ACOROF/ACORCH 及 SACOR 变体、AOVRON/AOVROF、GETSFT、SCALIB：参数分支、可选项组合、常量范围和位置目标。',
  ],
  limitations: [
    '公开 Sensor HW1483394 rev3 未给通道/阈值等标量参数的变量类型，也没有主手册所提 HPSRCH 的语法；这些形式明确标记未验证。',
    '尚未取得 YRC1000 NSRCH 所引用的 HW1483380 完整参数图；公开 YRC1000micro 文档属于不同控制器，不拿来冒充本版本支持。',
    '已核查协调 HW1484042 rev4，但其参数概要没有完整外部 JBI 序列化和组合产生式；+MOV*/+IMOV 仍告警，不把后续参数视为已通过。',
    '实现仍未完整校验每个标签的常量范围/单位、各控制组组合、局部变量容量、条件文件与跨作业符号存在性；主目录矩阵不是全控制器语法认证。',
    '弧焊/点焊/搬运等语法可分析，但应用选件、资源权限、I/O/工具标定、控制器版本及执行语义需现场配置；不会执行这些源码。',
    '当前分析 expanded 外部 JBI；不混用示教器 IF(...)THEN 等显示语法。UTF-8 源文件与控制器编码/字节长度兼容是独立验证事项。',
  ],
  analyze: analyzeInform,
  analyzeTemplate: (source, fileName) => analyzeInform(source, fileName, { templateParameters: true }),
};
