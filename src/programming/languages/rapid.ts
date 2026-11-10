import type { LanguageEntry, LanguageParameter, RobotLanguageDefinition } from '../types';
import catalog from './rapid-catalog.json';
import { analyzeRapid, rapidReservedWords, type RapidVersion } from './rapid-parser';

/** Original explanations of common operations; signatures are separately sourced language facts. */
const explanations: Record<string, string> = {
  movel: '让工具中心沿直线到达 robtarget；同时控制姿态、速度、转角区和工具／工件坐标。',
  movej: '使用关节协调运动到达 robtarget；TCP 的空间轨迹通常不是直线。',
  moveabsj: '使用给定的机器人与外轴关节角运动到 jointtarget。',
  moveabsl: '以关节角指定终点，同时要求 TCP 按直线路径运动。',
  movec: '以圆弧中间点和终点定义 TCP 的圆弧运动。',
  movepnp: '使用适合拾取与放置的路径方式执行运动；具体可用性取决于控制器和选件。',
  searchl: '沿直线路径搜索输入信号，并记录触发时的位置。',
  searchc: '沿圆弧路径搜索输入信号，并记录触发时的位置。',
  searchp: '搜索指定过程信号，并输出搜索命中位置。',
  accset: '调整后续机器人运动所使用的加速度及加速度变化设置。',
  velset: '为后续运动设置速度倍率与最高 TCP 速度。',
  speedrefresh: '更新正在执行运动的速度倍率。',
  confj: '设置关节运动是否检查目标的机器人配置。',
  confl: '设置直线运动是否检查配置连续性。',
  singarea: '选择奇异区域的运动处理模式。',
  circpathmode: '选择圆弧运动时的工具姿态处理方式。',
  pathaccset: '限制路径方向和法向上的加速度。',
  stopmove: '停止机器人运动，保留按规定恢复路径的机会。',
  startmove: '恢复先前通过运动停止机制暂停的机器人运动。',
  clearpath: '清除当前路径队列；恢复流程应同时遵守控制器的路径规则。',
  storepath: '保存当前路径状态，供中断处理或后续恢复使用。',
  restorepath: '恢复之前保存的路径状态。',
  startmoveretry: '恢复运动后重试产生错误的指令。',
  pathlengthstart: '开始累计路径长度。', pathlengthstop: '暂停路径长度累计。', pathlengthreset: '清零累计路径长度。', pathlengthget: '读取累计的路径长度。',
  stop: '暂停程序执行；这不是安全急停接口。', exit: '终止当前程序执行。', return: '从当前例程返回；函数可同时返回值。',
  waittime: '等待指定秒数；可选择等待机器人已经到位。',
  waitdi: '等待数字输入达到给定状态，可设置超时处理。',
  waitdo: '等待数字输出达到给定状态。',
  waitai: '等待模拟输入满足比较条件。', waitao: '等待模拟输出满足比较条件。',
  waitgi: '等待组输入满足给定条件。', waitgo: '等待组输出满足给定条件。',
  waituntil: '等待布尔条件成立，可结合超时和到位条件。',
  waitrob: '等待机器人满足到位或停止等运动状态。',
  setdo: '为数字输出信号写入 0 或 1；可选择延时或同步更新。',
  set: '将数字输出置为 1。', reset: '将数字输出置为 0。',
  setao: '写入模拟输出信号值。', setgo: '写入一组数字输出所表示的数值。',
  pulsdo: '在数字输出上产生指定时长的脉冲。',
  aliasio: '把 RAPID 的信号变量绑定到已配置的 I/O 信号。',
  aliasioreset: '解除信号变量当前的别名绑定。',
  iodevicedisable: '禁用指定 I/O 设备的通信。', iodeviceenable: '启用指定 I/O 设备的通信。',
  tpwrite: '向示教器文本窗口写入文字及可选的数值／姿态数据。',
  tperase: '清空示教器文本窗口。', tpreadnum: '在示教器请求用户输入数值，并保存结果。',
  tpreadfk: '显示示教器功能键选项，并读取用户选择。',
  uimessagebox: '显示交互消息框并返回用户选择。', uilistview: '显示列表选择界面并返回所选项。',
  uialphaentry: '请求用户输入字符串。', uinumentry: '请求用户输入 num 数值。', uidnumentry: '请求用户输入 dnum 数值。',
  uinumtune: '显示可增减调整的 num 数值输入界面。', uidnumtune: '显示可增减调整的 dnum 数值输入界面。',
  uimsgwrite: '向用户界面写入消息，并可附带图标或交互条件。',
  connect: '把中断标识关联到 TRAP 例程。',
  isignaldi: '为数字输入的触发条件建立中断监视。',
  isignaldo: '为数字输出的触发条件建立中断监视。',
  itimer: '建立按时间触发的中断。', idelete: '删除指定中断定义。',
  iwatch: '恢复监视指定中断。', isleep: '暂时停止监视指定中断。',
  ienabl: '启用中断响应。', idisable: '暂时禁止中断响应。',
  raise: '把错误交给当前或上层错误处理程序。', retry: '在错误处理后重试失败的指令。', trynext: '从错误处理中转到失败指令的下一条继续。',
  errwrite: '报告带上下文的错误信息。', errraise: '生成指定错误并进入错误处理流程。',
  bookerrnum: '为应用分配可在错误处理中使用的错误号。',
  break: '退出最内层 FOR 或 WHILE 循环；RobotWare 7 核心语法。',
  continue: '跳过当前循环剩余语句，进入下一次迭代；RobotWare 7 核心语法。',
  offs: '对 robtarget 的位置增加 XYZ 偏移，并保留其余目标数据。',
  reltool: '在工具坐标方向上偏移或旋转目标姿态。',
  calcjointt: '根据目标姿态、工具和工件数据计算关节目标；使用控制器自身的机器人模型。',
  calcrobt: '根据关节目标、工具和工件数据计算 robtarget。',
  crobt: '读取当前机器人 TCP 目标数据。', cjointt: '读取当前机器人与外轴关节值。',
  cpos: '读取当前 TCP 的位置分量。', corient: '读取当前 TCP 的方向四元数。',
  ctool: '读取当前工具数据。', cwobj: '读取当前工件坐标数据。',
  posemult: '复合两个位置和方向变换。', poseinv: '求位置和方向变换的逆。',
  posevect: '利用位姿变换一个位置向量。', orientzyx: '由 Z、Y、X 角建立方向四元数。',
  eulerzyx: '从方向四元数取得指定轴的 ZYX 欧拉角分量。',
  dotprod: '计算向量点积。', crossprod: '计算两个向量的叉积。', distance: '计算两位置之间的距离。',
  abs: '返回数值的绝对值。', sqrt: '返回平方根。', pow: '计算乘方。', exp: '计算指数。', log: '计算对数。',
  sin: '计算正弦；RAPID 的角度参数使用度。', cos: '计算余弦；角度使用度。', tan: '计算正切；角度使用度。',
  asin: '计算反正弦并以度返回角度。', acos: '计算反余弦并以度返回角度。', atan: '计算反正切并以度返回角度。', atan2: '按两坐标分量计算带象限信息的反正切角。',
  round: '按指定精度舍入数值。', trunc: '截取数值的小数部分到指定精度。', min: '返回较小的数值。', max: '返回较大的数值。', rand: '生成伪随机数。',
  dim: '读取数组某一维的长度。', present: '检查例程的可选参数是否在本次调用中提供。',
  typeof: '查询数据类型。', type: '查询数据对象的类型信息。',
  strlen: '返回字符串长度。', strpart: '提取字符串中指定范围的子串。',
  strfind: '按指定字符集合在字符串中查找位置。', strmatch: '在字符串中搜索匹配模式。',
  strmap: '按映射表替换字符串中的字符。', strorder: '比较两个字符串的排序顺序。',
  strformat: '把提供的参数替换到格式字符串中。', strtoval: '把字符串解析到指定类型的数据变量中。',
  valtostr: '把 RAPID 数据值转换成字符串。', numtostr: '把 num 转换成指定精度的字符串。', dnumtostr: '把 dnum 转换成指定精度的字符串。',
  jsonfromval: '把 RAPID 数据转换成 JSON 表示。', jsontoval: '把 JSON 内容转换到 RAPID 数据变量。',
  matrixadd: '计算两个矩阵的和。', matrixsub: '计算两个矩阵的差。', matrixmult: '计算矩阵乘积。',
  matrixinverse: '计算矩阵的逆。', matrixtranspose: '转置矩阵。', matrixreset: '按给定设置初始化或清空矩阵。',
  fitline: '从点集合拟合直线。', fitplane: '从点集合拟合平面。', fitsphere: '从点集合拟合球面。',
  open: '打开文件或通信设备，建立读写句柄。', close: '关闭已打开的文件或通信设备。',
  readstr: '从文件或设备读取字符串。', readnum: '从文件或设备读取数值。',
  write: '向文件或通信设备写入数据。', readbin: '读取二进制字节。', writebin: '写入二进制字节。',
  rewind: '把文件读写位置移回起点。', seek: '移动文件读写位置。',
  socketcreate: '创建套接字资源。', socketconnect: '连接到远端套接字。', socketbind: '把套接字绑定到本地地址和端口。',
  socketlisten: '让套接字等待客户端连接。', socketaccept: '接收等待中的客户端连接。',
  socketsend: '向连接的套接字发送数据。', socketreceive: '从套接字接收数据。', socketclose: '关闭套接字。',
  clkreset: '把时钟计数清零。', clkstart: '启动时钟计时。', clkstop: '停止时钟计时。', clkread: '读取时钟已经累计的时间。',
  cdate: '读取系统当前日期。', ctime: '读取系统当前时间。',
  load: '把指定程序模块加载到当前任务。', unload: '从当前任务卸载指定模块。', save: '保存程序模块。',
  actunit: '激活指定机械单元。', deactunit: '停用指定机械单元。',
  robtarget: 'TCP 位置、方向、机器人配置和外轴值组成的笛卡尔目标记录。',
  jointtarget: '机器人各关节值与外轴值组成的关节目标记录。',
  pos: '由 x、y、z 三个分量组成的位置记录，机器人位置通常使用 mm。',
  orient: '按 q1、q2、q3、q4 排列的方向四元数记录，其中 q1 是标量分量。',
  pose: '位置 trans 与方向 rot 组合形成的坐标变换。',
  confdata: '记录 ABB 机器人配置区间和构型选择信息。',
  robjoint: '机器人六轴的关节值记录。', extjoint: '外部机械轴的关节值记录。',
  speeddata: '平移、旋转和外轴运动速度的组合记录。',
  zonedata: '用于连续过渡的到点区域设置；fine 代表停止点。',
  stoppointdata: '停止点的到位判据与相关设置。',
  tooldata: '工具是否由机器人持有、工具坐标以及负载数据。',
  wobjdata: '工件、用户坐标系和对象坐标系的配置记录。',
  loaddata: '负载质量、重心、惯量及方向信息。',
  num: 'RAPID 常用数值类型；精度与范围由控制器定义。', dnum: '双精度数值类型。', bool: 'TRUE 或 FALSE 的布尔值。',
  string: '字符串值；字节数、字符集和最大长度按 RobotWare 版本核对。', switch: '表示调用中是否提供某个可选开关参数。',
  signaldi: '数字输入信号引用。', signaldo: '数字输出信号引用。', signalai: '模拟输入信号引用。', signalao: '模拟输出信号引用。',
  signalgi: '组数字输入信号引用。', signalgo: '组数字输出信号引用。',
  intnum: '中断标识。', errnum: '表示应用或系统错误标识的数值类型。', clock: '计时器状态。', iodev: '文件或通信设备句柄。', socketdev: '套接字句柄。',
};

const families: readonly [RegExp, string][] = [
  [/^cam/i, '相机作业、参数或视觉结果接口'], [/^cap/i, '连续工艺路径与过程控制接口'],
  [/^trigg/i, '沿机器人路径同步触发信号或过程事件'], [/^syncmove/i, '协调多任务机器人同步运动'],
  [/^soft/i, '机器人柔顺运动设置'], [/^ind/i, '机械轴独立运动控制'], [/^corr/i, '路径修正接口'],
  [/^wz/i, '世界区域监控和区域条件设置'], [/^testsign/i, '测试信号采集接口'],
  [/^safetycontroller/i, '安全控制器信息查询接口'], [/^socket/i, '套接字通信接口'],
  [/^rawbytes|^packraw|^unpackraw|^clearraw/i, '原始字节数据处理'], [/^bytes|^packbytes|^unpackbytes/i, '字节缓冲区数据处理'],
  [/^bit/i, '按位数据操作'], [/^iobus|^iodevice|^io|^getgroup/i, 'I/O 设备和信号管理'],
  [/^si|^dropsensor|^supsync|^synctosensor/i, '传感器与外部过程接口'],
  [/^prx/i, '外部位置记录接口'], [/^stindgun/i, '焊枪独立轴接口'],
  [/^tool|^defacc|^defdframe|^defframe|^defrob|^defsframe/i, '工具、坐标系或标定计算'],
  [/^str|^text/i, '字符串与文本资源处理'], [/^ui/i, '示教器交互界面接口'],
  [/^get|^is|^current|^task|^rob|^opmode|^exec|^progmem|^remaining/i, '控制器状态或数据查询'],
];

type CatalogEntry = typeof catalog.profiles['7.21']['entries'][number];
function explain(entry: CatalogEntry): string {
  const known = explanations[entry.name.toLowerCase()];
  if (known) return known;
  const family = families.find(([pattern]) => pattern.test(entry.name))?.[1];
  const required = entry.parameters.filter(p => !p.optional).map(p => `${p.name}（${p.type}）`).join('、');
  const returns = 'returns' in entry && entry.returns ? `返回 ${entry.returns} 类型的结果。` : '';
  if (entry.kind === 'datatype') return `${entry.name} 是控制器定义的数据类型，用于相应系统接口的数据声明；结构和预定义值见该版本参考页。`;
  return `${family ? family + '：' : ''}${entry.name} ${entry.kind === 'function' ? '用于表达式求值' : '作为独立语句调用'}。${required ? `必需输入／输出参数为 ${required}。` : '没有必需的位置参数。'}${returns}`;
}

function parameterHelp(parameter: CatalogEntry['parameters'][number]): LanguageParameter {
  const isSwitch = 'switch' in parameter && parameter.switch;
  const alternative = 'alt' in parameter && parameter.alt;
  return { name: parameter.name, type: parameter.type, optional: parameter.optional,
    description: `${parameter.optional ? '可选' : '必需'}${isSwitch ? '开关，不附带 := 值' : `${parameter.type} 参数`}${alternative ? '；与前一参数互为替代形式' : ''}。` };
}

function callSnippet(entry: CatalogEntry): string | undefined {
  if (entry.kind === 'datatype' || !entry.signature.toLowerCase().startsWith(entry.name.toLowerCase())
    || rapidReservedWords.includes(entry.name.toUpperCase()) || ['label', 'comment', 'proccall'].includes(entry.name.toLowerCase())) return undefined;
  const args = entry.parameters.filter(p => !p.optional && !('alt' in p && p.alt)).map(p => '${' + p.name + '}').join(', ');
  if (entry.kind === 'function') return `${entry.name}(${args})`;
  return `${entry.name}${args ? ' ' + args : ''};`;
}

const keywordExplanations: Record<string, string> = {
  MODULE: '声明模块；模块内按类型、数据、例程顺序组织。', ENDMODULE: '结束当前模块。',
  PROC: '声明不返回值的过程例程。', ENDPROC: '结束过程例程。', FUNC: '声明具有返回类型的函数例程。', ENDFUNC: '结束函数例程。',
  TRAP: '声明处理中断的例程。', ENDTRAP: '结束中断处理例程。',
  RECORD: '声明由命名成员组成的记录类型。', ENDRECORD: '结束记录类型。', ALIAS: '为已有数据类型声明别名。',
  VAR: '声明可修改的变量。', PERS: '声明在规定加载和任务生命周期内保留值的数据。', CONST: '声明带初始值的常量。',
  LOCAL: '把模块中的声明限制为本模块可见。', TASK: '声明任务作用域的数据。', INOUT: '把形参作为可读写的调用者数据引用。',
  IF: '按布尔条件执行分支；支持完整分支和单条简单语句形式。', THEN: '开始 IF 或 ELSEIF 的分支语句。',
  ELSEIF: '添加另一条件分支。', ELSE: '添加未命中前面条件时执行的分支。', ENDIF: '结束条件分支。',
  FOR: '在指定范围内迭代循环变量。', FROM: '指定 FOR 循环的初始值。', TO: '指定 FOR 循环的结束值。', STEP: '指定 FOR 循环的步长。', ENDFOR: '结束 FOR 循环。',
  WHILE: '只要条件成立就重复执行循环体。', DO: '开始 FOR 或 WHILE 循环体。', ENDWHILE: '结束 WHILE 循环。',
  TEST: '根据表达式值选择 CASE 分支。', CASE: '列出一个或多个匹配值。', DEFAULT: '处理未命中 CASE 的情况。', ENDTEST: '结束 TEST 分支。',
  BACKWARD: '定义过程逆向执行时的处理语句。', ERROR: '开始例程的错误处理部分。', UNDO: '定义例程被非正常退出时的清理语句。',
  GOTO: '跳转到当前例程中的标签。', WITH: '在 CONNECT 中指定目标 TRAP。',
  TRUE: '布尔真值。', FALSE: '布尔假值。',
  SYSMODULE: '把模块标记为系统模块。', NOSTEPIN: '限制调试器进入该模块例程。', VIEWONLY: '标记模块为仅查看。', READONLY: '标记模块为只读。', NOVIEW: '限制模块源码查看。',
};
const snippets: LanguageEntry[] = [
  { name: 'module', kind: 'snippet', signature: 'MODULE name … ENDMODULE', description: '插入包含 main 过程的新模块。', insertText: 'MODULE ${ModuleName}\n  PROC main()\n    ${}\n  ENDPROC\nENDMODULE' },
  { name: 'proc', kind: 'snippet', signature: 'PROC name() … ENDPROC', description: '插入过程声明。', insertText: 'PROC ${name}()\n  ${}\nENDPROC' },
  { name: 'func', kind: 'snippet', signature: 'FUNC type name() … ENDFUNC', description: '插入带返回值的函数。', insertText: 'FUNC ${num} ${name}()\n  RETURN ${value};\nENDFUNC' },
  { name: 'if', kind: 'snippet', signature: 'IF condition THEN … ENDIF', description: '插入条件分支。', insertText: 'IF ${condition} THEN\n  ${}\nENDIF' },
  { name: 'for', kind: 'snippet', signature: 'FOR i FROM first TO last DO … ENDFOR', description: '插入范围循环。', insertText: 'FOR ${i} FROM ${1} TO ${10} DO\n  ${}\nENDFOR' },
  { name: 'while', kind: 'snippet', signature: 'WHILE condition DO … ENDWHILE', description: '插入条件循环。', insertText: 'WHILE ${condition} DO\n  ${}\nENDWHILE' },
  { name: 'test', kind: 'snippet', signature: 'TEST value CASE … ENDTEST', description: '插入多路值分支。', insertText: 'TEST ${value}\nCASE ${1}:\n  ${}\nDEFAULT:\nENDTEST' },
  { name: 'trap', kind: 'snippet', signature: 'TRAP name … ENDTRAP', description: '插入中断例程。', insertText: 'TRAP ${name}\n  ${}\nENDTRAP' },
  { name: 'record', kind: 'snippet', signature: 'RECORD name … ENDRECORD', description: '插入记录类型。', insertText: 'RECORD ${name}\n  ${num} ${member};\nENDRECORD' },
];

export function createRapidLanguage(version: RapidVersion): RobotLanguageDefinition {
  const profile = catalog.profiles[version];
  const keywords = [...rapidReservedWords.filter(k => version !== '6.16' || !['BREAK', 'CONTINUE'].includes(k)), 'TASK'];
  const entries: LanguageEntry[] = profile.entries.map(entry => ({ name: entry.name, kind: entry.kind === 'datatype' ? 'type' : entry.kind as 'instruction' | 'function',
    signature: entry.signature || entry.name, description: explain(entry), parameters: entry.parameters.map(parameterHelp), insertText: callSnippet(entry),
    category: entry.kind === 'datatype' ? '数据类型' : entry.kind === 'function' ? '函数／运算符' : '指令／语句',
    documentationUrl: `${profile.url}#search=${encodeURIComponent(entry.name)}`,
    availableSince: `已核对 ${profile.version} · ${profile.id} rev ${profile.revision} §${entry.section}`,
  }));
  const names = new Set(entries.map(e => e.name.toUpperCase()));
  for (const keyword of keywords) if (!names.has(keyword)) entries.push({ name: keyword, kind: 'keyword', signature: keyword, description: keywordExplanations[keyword] ?? explanations[keyword.toLowerCase()] ?? `${keyword} 是 RAPID 核心语法关键字。` });
  entries.push(...snippets);
  return {
    id: 'rapid', brand: 'ABB', name: 'RAPID', dialect: profile.version,
    extensions: version === '7.21' ? ['modx', 'sysx', 'mod', 'sys'] : ['mod', 'sys'],
    lexical: { lineComment: '!', keywords, types: profile.entries.filter(e => e.kind === 'datatype').map(e => e.name), stringEscape: 'double-quote', caseInsensitive: true },
    entries,
    sources: [
      { title: `${profile.version} 指令、函数与数据类型`, url: profile.url, revision: `${profile.id} revision ${profile.revision}` },
      { title: `${version === '7.21' ? 'RobotWare 7.21' : 'RobotWare 6.16.03'} RAPID kernel`, url: `https://search.abb.com/library/Download.aspx?DocumentID=${version === '7.21' ? '3HAC065039-001' : '3HAC050946-001'}&LanguageCode=en&Action=Launch`, revision: version === '7.21' ? 'K' : 'L' },
      ...(version === '7.21' ? [{ title: 'RobotWare 7 模块格式与 UTF-8 编码', url: 'https://search.abb.com/library/Download.aspx?DocumentID=3HAC065040-001&LanguageCode=en&Action=Launch', revision: 'T · §1.1.3 pp19–20' }] : []),
      { title: 'Signatures-only extractor, adapted with documented corrections', url: 'https://github.com/ichbinmeraj/abb-rws-vscode/tree/c213a07746432b359aa3b8c6ecfbf1b4e8b9c545', revision: 'c213a07746432b359aa3b8c6ecfbf1b4e8b9c545', license: 'Apache-2.0' },
    ],
    coverage: [
      `参考手册目录核对：${profile.count}/${profile.count} 个指令、函数／运算符、数据类型条目。`,
      '核心语法：词法、模块、记录／别名、声明、数组、例程、参数、表达式、全部结构化语句、动态调用、错误／回退／清理处理和编辑占位符。',
      '补全包含手册签名、参数类型／可选项、来源版本、原创说明和结构化代码模板；识别当前文件的声明。',
    ],
    limitations: [
      '离线核心语法分析不等于 ABB 编译器验收；未实现跨模块类型推导、任务／I/O 配置、控制器选件和运行时语义验证。',
      '目录覆盖列明版本的主参考手册；EGM、焊接等另行发布的选件应用手册及用户自定义库不属于此完整性计数。',
      '签名来自有界结构提取及定向核对；说明为原创简述，复杂条件、单位和限制应查对应版本文档。',
      '仅分析纯文本模块；不解析加密模块或 RobotStudio 项目容器。允许编辑中的 EOF 注释并对未填写占位符给出警告。',
    ],
    analyze: (source, fileName) => {
      const result = analyzeRapid(source, version);
      const modern = /\.(?:modx|sysx)$/i.test(fileName ?? '');
      if (version === '6.16' && modern) {
        result.diagnostics.push({ from: 0, to: Math.min(1, source.length), severity: 'warning', code: 'rapid.file-version',
          message: '.modx/.sysx 是 RobotWare 7.1 起使用的 UTF-8 模块格式，与当前 RobotWare 6 配置不兼容；请核对目标版本和文件编码。原文已保留。' });
      } else if (version === '7.21' && modern && source.charCodeAt(0) === 0xfeff) {
        result.diagnostics.push({ from: 0, to: 1, severity: 'warning', code: 'rapid.utf8-bom',
          message: '当前 .modx/.sysx 带 UTF-8 BOM；ABB 模块格式说明要求转换为不带 BOM 的 UTF-8。请核对并移除 BOM 后再交付控制器，编辑器不会自动删除原始字节。' });
      } else if (version === '7.21' && /\.(?:mod|sys)$/i.test(fileName ?? '')) {
        const nonAscii = /[^\x00-\x7f]/u.exec(source);
        if (nonAscii) result.diagnostics.push({ from: nonAscii.index, to: nonAscii.index + nonAscii[0].length,
          severity: 'warning', code: 'rapid.legacy-encoding',
          message: '此文件按 UTF-8 保存并含非 ASCII 字符，但旧 .mod/.sys 扩展名不声明 UTF-8 编码。RobotWare 7 请另存为不带 BOM 的 .modx/.sysx，避免字符被错误解释；原文和文件名已保留。' });
      }
      return result;
    },
  };
}

export const rapidLanguage = createRapidLanguage('7.21');
