import index from '../../../vendor/robot-languages/inform/instruction-index.json';
import type { LanguageEntry } from '../types';
import { informOptionEntries } from './inform-options';

/** Public interface facts; explanations and signatures are independently written. */
const rows = `
DOUT|DOUT OT#/OG#/OGH#/OGU# value|写入数字输出或输出组。|I/O
DIN|DIN destination input|把数字输入状态读入整数变量。|I/O
WAIT|WAIT input comparison [T=seconds]|等待输入条件，可设置等待时间。|I/O
PULSE|PULSE output [groupValue] [T=seconds] [IFEXPRESS condition]|产生定时数字输出脉冲。|I/O
AOUT|AOUT channel value|设置模拟输出。|I/O
ARATION|ARATION AO#(number) [BV=value] [V=speed] [OFV=value] [FINE DELAY=time]|启用与运动速度关联的模拟输出比例。|I/O
ARATIOF|ARATIOF channel|取消模拟输出速度比例。|I/O
ANTOUT|ANTOUT AT#(number) state [ANT=time/ANTDIS=distance]|按预先配置调整数字输出相对运动到点的时机。|I/O
JUMP|JUMP *label [IF condition]|跳转到本作业内标签；也支持手册规定的间接目标。|控制
CALL|CALL JOB:name [UF#(frame)] [IF/IFEXPRESS condition] [(arg1,...,arg8)]|调用作业；变量传参与独立控制依赖对应配置。|控制
TIMER|TIMER T=seconds|暂停作业推进指定时间。|控制
*(LABEL)|*label|定义同一作业中的跳转标签。|控制
’(COMMENT)|'comment|从单引号开始记录行注释。|控制
RET|RET [value] [IF condition]|返回调用作业，并可返回一个值。|控制
NOP|NOP|空操作，通常作为作业指令区起始行。|控制
PAUSE|PAUSE [IF condition]|暂停作业执行，需控制器继续操作。|控制
CWAIT|CWAIT|等待此前无等待运动完成。|控制
MSG|MSG "text"|显示提示文本。|控制
INPUT|INPUT destination|提示操作者输入数值并写入全局数值变量。|控制
ADVINIT|ADVINIT|初始化提前处理控制。|控制
ADVSTOP|ADVSTOP|停止提前处理。|控制
PRINT|PRINT ["format"] [variable]|向示教器终端显示可选的字符串和变量。|控制
CLS|CLS|清除相应显示输出。|控制
ABORT|ABORT [IF condition]|中止当前作业调用链。|控制
SETUALM|SETUALM alarm ["message"] subcode [SMODE=value]|设置用户报警。|控制
DIALOG|DIALOG destination DIALSB dialogLayout|显示对话框，把按钮结果写入变量；布局由附加项指定。|控制
SWITCH|SWITCH variable CASE value|开始多分支选择，配合 CASE/DEFAULT/ENDSWITCH。|结构
CASE|CASE value|定义 SWITCH 分支条件。|结构
DEFAULT|DEFAULT|定义 SWITCH 的默认分支。|结构
WHILE|WHILEEXP condition|外部 JBI 使用 WHILEEXP，配合 ENDWHILE。|结构
FOR|FOR Ivariable[START]=start TO end [STEP increment]|建立计数循环，使用 NEXT variable 结束。|结构
IFTHEN|IFTHENEXP condition|外部 JBI 使用 IFTHENEXP，配合 ELSEIFEXP/ELSE/ENDIF。|结构
ELSEIF|ELSEIFEXP condition|外部 JBI 的附加条件分支。|结构
ELSE|ELSE|定义 IFTHENEXP 的剩余条件分支。|结构
SETTM|SETTM TMnumber TSTART/TSTOP/TRESET/value|设置时间相关变量。|控制
LATESTJOB|LATESTJOB|请求应用回放期间编辑的作业数据。|控制
CLEAR|CLEAR variable count/ALL / CLEAR STACK/QUE/MPF/systemVariable|清零指定变量或连续变量区间。|运算
INC|INC variable|变量加一。|运算
DEC|DEC variable|变量减一。|运算
SET|SET destination expression|将表达式或变量值赋给目标变量。|运算
ADD|ADD destination expression|对目标变量执行加法。|运算
SUB|SUB destination expression|对目标变量执行减法。|运算
MUL|MUL destination expression|对目标变量执行乘法。|运算
DIV|DIV destination expression|对目标变量执行除法。|运算
CNVRT|CNVRT PXdestination PXsource BF/RF/TF/UF#/MTF [TL#(tool)]|按工具和坐标系标签转换位置数据。|位置运算
AND|AND destination expression|执行按位与运算。|运算
OR|OR destination expression|执行按位或运算。|运算
NOT|NOT destination source|将源数据按位取反后写入目标变量。|运算
XOR|XOR destination expression|执行按位异或。|运算
MFRAME|MFRAME UF#(frame) PXorigin PXx PXy / Pposition BF/UUF#|由三个位置数据建立用户坐标系。|位置运算
SETE|SETE position element value|设置位置变量的指定分量。|位置运算
GETE|GETE destination position element|读取位置变量的指定分量。|位置运算
GETS|GETS destination systemVariable|读取控制器系统变量。|运算
SQRT|SQRT destination value|计算平方根。|运算
SIN|SIN destination angle|计算正弦值；角度约定按手册。|运算
COS|COS destination angle|计算余弦值。|运算
ATAN|ATAN destination value|计算源数值的反正切并保存结果。|运算
MULMAT|MULMAT destination left right|执行位置矩阵乘法。|位置运算
INVMAT|INVMAT destination source|计算位置矩阵的逆变换。|位置运算
GETPOS|GETPOS destination STEP#(number) [READCTP destination]|读取指定位置数据。|位置运算
VAL|VAL destination text|将字符串转换为数值。|字符串
VAL2STR|VAL2STR destination value|把数值转换为字符串。|字符串
ASC|ASC destination text|取得字符编码数值。|字符串
CHR$|CHR$ destination code|把字符编码转换为字符串。|字符串
MID$|MID$ destination text start length|截取字符串区间。|字符串
LEN|LEN destination text|取得字符串长度。|字符串
CAT$|CAT$ destination left right|连接两个字符串并保存结果。|字符串
STRSTR|STRSTR Bdestination Stext pattern [Bstart]|查找子串位置。|字符串
GETARG|GETARG destination IARG#(number)|读取调用作业传入的参数。|控制
GETNAME|GETNAME destination item|读取指定对象的名称。|控制
SETFILE|SETFILE conditionFile (element) value|修改指定工艺条件文件的一个数值元素。|文件
GETFILE|GETFILE destination conditionFile (element)|读取工艺条件文件的指定元素。|文件
SETREG|SETREG MREG#(number) value [LOW/HIGH] [REGNUM=count]|写入控制器寄存器。|寄存器
GETREG|GETREG destination MREG#(number) [LOW/HIGH] [REGNUM=count]|读取控制器寄存器。|寄存器
GETPRM|GETPRM destination KIND=type PRMNO=number [GRPNO=group/MAX/MIN]|读取控制器参数；访问权限取决于系统配置。|参数
SETPRM|SETPRM KIND=type PRMNO=number [GRPNO=group] value|设置控制器参数；编辑器不会执行此操作。|参数
MOVJ|MOVJ target VJ=percent [motion tags]|关节插补到目标位置。|运动
MOVL|MOVL target V=mmPerSecond [motion tags]|TCP 直线插补；外轴与运动附加项由控制器配置决定。|运动
MOVC|MOVC target V=mmPerSecond [motion tags]|圆弧插补指令，依赖相邻圆弧目标。|运动
MOVS|MOVS target V=mmPerSecond [motion tags]|样条插补，依赖相邻目标和控制器能力。|运动
IMOV|IMOV position V=mmPerSecond [coordinate tags]|按位置增量运动。|运动
SPEED|SPEED VJ=percent [V=mmPerSecond]|设置后续运动速度条件。|运动
REFP|REFP number target|登记供其他指令引用的参考点。|运动
SFTON|SFTON position [coordinate tags]|启用平移偏移。|偏移
SFTOF|SFTOF|取消平移偏移。|偏移
MSHIFT|MSHIFT PXdestination coordinate PXreference PXactual|计算位置偏移量。|偏移
SFTON3D|SFTON3D [RBn] position|启用三维位置与姿态偏移。|偏移
SFTOF3D|SFTOF3D|取消三维偏移。|偏移
IF|IF condition|附加于允许的指令，条件成立时执行。|附加项
UNTIL|UNTIL condition|运动或相关指令的条件终止附加项。|附加项
ENWAIT|ENWAIT T=seconds / DIS=distance|附加等待控制项，须结合所属指令。|附加项
DIALSB|DIALSB Bx By Smessage [Bx By Sbutton]|对话框相关附加项。|附加项
IFEXPRESS|IFEXPRESS condition|手册对扩展条件表达式的语法说明项。|表达式
FOREXPRESS|FOREXPRESS condition|附加在 WAIT 指令内，等待复合条件成立。|表达式
LOGICEXP|left ANDEXP right / left OREXP right|外部 JBI 逻辑表达式语法说明项。|表达式
RELEXPR|left comparison right|关系表达式语法说明项。|表达式
TRAP|TRAP *label|运动超限时中断当前运动并跳到指定本地标签。|附加项
+SFTON3D|+SFTON3D position|运动指令附加的三维偏移。|附加项
+SFT|+SFT position|运动指令附加的位置偏移。|附加项
CAPTURE|CAPTURE destination|仅用于 UNTIL 中，条件成立时记录当前笛卡尔位置。|附加项
ARCON|ARCON [weld tags]|启动弧焊过程。|弧焊
ARCOF|ARCOF [weld tags]|结束弧焊过程。|弧焊
VWELD|VWELD value|设置焊接电压指令值。|弧焊
AWELD|AWELD value|设置焊接电流。|弧焊
ARCSET|ARCSET [weld tags]|设置弧焊条件。|弧焊
WVON|WVON [weaving tags]|启用焊接摆动。|弧焊
WVOF|WVOF|关闭焊接摆动。|弧焊
ARCCTS|ARCCTS AC=value AV=value [DIS=distance]|在运动起始区间渐变焊接电流与电压。|弧焊
ARCCTE|ARCCTE AC=value AV=value [DIS=distance]|在运动结束区间渐变焊接电流与电压。|弧焊
HAND|HAND number state [ALL]|控制搬运夹具。|搬运
HSEN|HSEN number ON/OFF [T=seconds/FOREVER]|处理搬运传感器条件。|搬运
TOOLON|TOOLON|启用通用作业工具。|通用作业
TOOLOF|TOOLOF|关闭通用作业工具。|通用作业
GUNCL|GUNCL GUN#(number) T=seconds [ATT=seconds]|控制点焊焊枪闭合。|点焊
SPOT|SPOT GUN#(number) MODE=mode WTM=condition [ATT=time]|执行点焊过程。|点焊
STROKE|STROKE GUN#(number) LONG/SHORT|设置点焊焊枪行程。|点焊
STRWAIT|STRWAIT GUN#(number) LONG/SHORT|等待焊枪行程条件。|点焊
`.trim().split('\n').map(row => row.split('|'));
const details = new Map(rows.map(([name, signature, description, category]) => [name, { signature, description, category }]));
const specialSnippets: Record<string, string> = {
  MOVJ: 'MOVJ ${C00000} VJ=${10}', MOVL: 'MOVL ${C00000} V=${100}', MOVC: 'MOVC ${C00000} V=${100}',
  DOUT: 'DOUT OT#(${1}) ${ON}', DIN: 'DIN ${B000} IN#(${1})', WAIT: 'WAIT IN#(${1})=${ON}',
  TIMER: 'TIMER T=${1.00}', SET: 'SET ${I000} ${0}', CALL: 'CALL JOB:${JOBNAME}', JUMP: 'JUMP *${LABEL}',
  FOR: 'FOR ${I000}=${0} TO ${9}\n\t${}\nNEXT I000',
  SWITCH: 'SWITCH ${I000} CASE ${0}\n\t${}\nDEFAULT\nENDSWITCH',
};
export const informInstructionEntries: readonly LanguageEntry[] = index.entries.map(fact => {
  const row = details.get(fact.name)!;
  const requiresOption = ['弧焊', '搬运', '点焊', '通用作业'].includes(row.category) ? `${row.category}应用配置` : undefined;
  return { name: fact.name, kind: ['附加项', '表达式'].includes(row.category) ? 'keyword' : 'instruction', ...row,
    insertText: specialSnippets[fact.name] ?? fact.name,
    parameters: row.signature.split(' ').slice(1).filter(part => !part.includes('/') && part !== '[IF' && part !== 'condition]').map(part => ({ name: part.replace(/[\[\]]/g, ''), description: part.startsWith('[') ? '可选项；具体类型和允许组合见本指令官方语法图。' : '操作数；允许的变量类型及范围见本指令官方语法图。', optional: part.startsWith('[') })),
    documentationUrl: fact.url, availableSince: 'YRC1000 INFORM 178649-1CD revision 10', requiresOption };
});
const control = [
  ['IFTHENEXP', 'IFTHENEXP condition', '外部 JBI 条件块。', 'IFTHENEXP ${I000}=0\n\t${}\nENDIF'],
  ['ELSEIFEXP', 'ELSEIFEXP condition', '外部 JBI 条件块的附加分支。'],
  ['WHILEEXP', 'WHILEEXP condition', '外部 JBI 条件循环。', 'WHILEEXP ${I000}<10\n\t${}\nENDWHILE'],
  ['ENDIF', 'ENDIF', '结束条件块。'], ['ENDWHILE', 'ENDWHILE', '结束条件循环。'],
  ['ENDSWITCH', 'ENDSWITCH', '结束多分支选择。'], ['NEXT', 'NEXT variable', '结束 FOR 循环。'],
  ['END', 'END', '结束当前作业。'], ['ANDEXP', 'left ANDEXP right', '连接外部 JBI 的逻辑与条件。'],
  ['OREXP', 'left OREXP right', '连接外部 JBI 的逻辑或条件。'],
];
export const informEntries: readonly LanguageEntry[] = [...informInstructionEntries, ...informOptionEntries, ...control.map(([name, signature, description, insertText]) => ({ name, signature, description, insertText, kind: 'keyword' as const, category: '外部结构语法', documentationUrl: `${index.pdfUrl}#page=44`, availableSince: 'YRC1000 expanded' }))];
export const informIndexSource = index;
