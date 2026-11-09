import type { RobotLanguageDefinition } from '../types';
import { krlEntries } from './krl-catalog';
import { analyzeKrl } from './krl-parser';

export const krlLanguage: RobotLanguageDefinition = {
  id: 'krl', brand: 'KUKA', name: 'KRL', dialect: 'KSS 8.x 核心语言编辑 · 未绑定机器人型号', extensions: ['.src', '.dat'],
  lexical: { lineComment: ';', stringEscape: 'backslash', caseInsensitive: true,
    keywords: [...krlEntries.filter(entry => entry.kind !== 'type').map(entry => entry.name), 'AND', 'OR', 'EXOR', 'NOT', 'B_AND', 'B_OR', 'B_EXOR', 'B_NOT', 'TO', 'STEP', 'IN', 'OUT', 'WITH', 'C_DIS', 'C_ORI', 'C_VEL', 'C_PTP', 'C_SPL'],
    types: krlEntries.filter(entry => entry.kind === 'type').map(entry => entry.name) },
  entries: krlEntries,
  sources: [
    { title: 'KUKA SystemSoftware 产品与控制器版本', url: 'https://www.kuka.com/en-de/products/robotics-systems/software/system-software/kuka_systemsoftware' },
    { title: 'KRL for Vim：词法事实交叉检查，未复制实现', url: 'https://github.com/KnoP-01/krl-for-vim/blob/7af6b64f2045c0b0b576189d01bbbb2ad17d8077/syntax/krl.vim', revision: '7af6b64f2045c0b0b576189d01bbbb2ad17d8077', license: 'MIT；本项目未分发其代码' },
  ],
  coverage: ['SRC/DAT 例程与数据模块、声明、数组/结构常量、表达式、函数调用、标签及跳转。', 'IF/SWITCH/FOR/WHILE/REPEAT/LOOP 的嵌套与折叠；基础运动与 SPLINE/PTP_SPLINE 段结构。', '中断声明/控制、WAIT、核心语句；源码大小、嵌套与诊断数量有界。'],
  limitations: ['这是独立实现的 KSS 8.x 核心编辑分析，不是完整 KSS 编译器，也不保证所有 8.x 修订语法。', '未完成类型/作用域/跨文件链接、系统变量与函数全目录、技术包及高级 TRIGGER/样条参数语义；未建模语句产生警告。', '未复制 LGPL 的旧 ANTLR KRL grammar；现成高亮词表不作为完整语法证明。', '当前没有 KUKA 机器人模型、控制器后处理或 KRL 运动仿真。'],
  analyze: analyzeKrl,
};
