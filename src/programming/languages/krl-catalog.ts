import type { LanguageEntry } from '../types';

/** Original descriptions of KRL core constructs; this is not a controller keyword inventory. */
const data = `
DEF|DEF routine(parameters)|定义过程，以 END 结束。|例程
DEFFCT|DEFFCT type function(parameters)|定义有返回值函数，以 ENDFCT 结束。|例程
DEFDAT|DEFDAT name [PUBLIC]|开始数据模块，以 ENDDAT 结束。|模块
END|END|结束过程。|例程
ENDFCT|ENDFCT|结束函数。|例程
ENDDAT|ENDDAT|结束数据模块。|模块
DECL|DECL type name [= value]|声明变量；数据初始化位置由所在模块决定。|声明
GLOBAL|GLOBAL declaration|扩大声明的可见范围。|声明
CONST|CONST type name = value|声明常量数据。|声明
PUBLIC|DEFDAT name PUBLIC|使数据模块具有公开可见性。|声明
EXT|EXT routine(parameterTypes)|声明外部过程接口。|声明
EXTFCT|EXTFCT type function(parameterTypes)|声明外部函数接口。|声明
STRUC|STRUC type componentDeclarations|定义结构类型。|类型
ENUM|ENUM type values|定义枚举类型。|类型
SIGNAL|SIGNAL name inputOrOutput [TO last]|为单个或一组数字信号定义名称。|I/O
IMPORT|IMPORT type name IS path|导入其他模块数据，具体可见性由控制器验证。|声明
IF|IF condition THEN|开始条件分支。|控制
THEN|IF condition THEN|分隔条件和分支体。|控制
ELSE|ELSE|定义剩余条件分支。|控制
ENDIF|ENDIF|结束条件分支。|控制
SWITCH|SWITCH expression|开始多分支选择。|控制
CASE|CASE values|定义 SWITCH 值分支。|控制
DEFAULT|DEFAULT|定义 SWITCH 默认分支。|控制
ENDSWITCH|ENDSWITCH|结束 SWITCH。|控制
FOR|FOR variable = start TO end [STEP increment]|开始计数循环。|控制
ENDFOR|ENDFOR|结束 FOR。|控制
WHILE|WHILE condition|开始条件循环。|控制
ENDWHILE|ENDWHILE|结束 WHILE。|控制
REPEAT|REPEAT|开始后置条件循环。|控制
UNTIL|UNTIL condition|结束 REPEAT，条件成立时退出。|控制
LOOP|LOOP|开始无条件循环。|控制
ENDLOOP|ENDLOOP|结束 LOOP。|控制
EXIT|EXIT|退出当前循环。|控制
GOTO|GOTO label|跳转到本例程标签。|控制
RETURN|RETURN [value]|返回调用者；函数返回值类型仍需类型检查。|控制
HALT|HALT|暂停程序解释器。|控制
CONTINUE|CONTINUE|控制提前运行停止行为，不等于循环 continue。|控制
WAIT|WAIT FOR condition / WAIT SEC duration|按条件或秒数等待。|控制
INTERRUPT|[GLOBAL] INTERRUPT DECL number WHEN condition DO routine()|声明中断或使用 ON/OFF/ENABLE/DISABLE 控制中断。|中断
RESUME|RESUME|恢复中断相关程序执行。|中断
BRAKE|BRAKE [F]|在中断处理中触发制动。|中断
TRIGGER|TRIGGER WHEN DISTANCE=value DELAY=value DO action [PRIO=priority]|设置与运动关联的触发动作；PATH/ONSTART 形式须按系统版本核验。|触发
PTP|PTP target [C_PTP]|关节运动到位置目标。|运动
PTP_REL|PTP_REL offset [C_PTP]|相对关节运动。|运动
LIN|LIN target [C_DIS/C_ORI/C_VEL]|笛卡尔直线运动。|运动
LIN_REL|LIN_REL offset [C_DIS/C_ORI/C_VEL] [#BASE/#TOOL]|相对直线运动。|运动
CIRC|CIRC auxiliary, target [CA=angle] [approximation]|经辅助点的圆弧运动。|运动
CIRC_REL|CIRC_REL auxiliary, target [CA=angle] [approximation]|相对圆弧运动。|运动
SPLINE|SPLINE [WITH parameters]|开始连续轨迹样条块。|样条
PTP_SPLINE|PTP_SPLINE [WITH parameters]|开始关节样条块。|样条
ENDSPLINE|ENDSPLINE [approximation]|结束样条块。|样条
SPL|SPL target [WITH parameters]|样条段目标。|样条
SLIN|SLIN target [WITH parameters]|直线样条段目标。|样条
SCIRC|SCIRC auxiliary, target [WITH parameters]|圆弧样条段目标。|样条
SPTP|SPTP target [WITH parameters]|关节样条段目标。|样条
SPL_REL|SPL_REL offset [WITH parameters]|相对样条段目标。|样条
SLIN_REL|SLIN_REL offset [WITH parameters]|相对直线样条段目标。|样条
SCIRC_REL|SCIRC_REL auxiliary, target [WITH parameters]|相对圆弧样条段目标。|样条
SPTP_REL|SPTP_REL offset [WITH parameters]|相对关节样条段目标。|样条
TIME_BLOCK|TIME_BLOCK START/PART/END parameters|样条时间块；参数和选件版本须复核。|样条
CONST_VEL|CONST_VEL START/END parameters|定义样条恒速区间。|样条
ANIN|ANIN ON destination = input / ANIN OFF destination|连续模拟输入关联。|I/O
ANOUT|ANOUT ON output = value / ANOUT OFF output|连续模拟输出关联。|I/O
BOOL|BOOL|布尔类型。|类型
INT|INT|整数类型。|类型
REAL|REAL|浮点数类型。|类型
CHAR|CHAR|字符类型；字符数组表示字符串。|类型
FRAME|FRAME {X,Y,Z,A,B,C}|空间位姿结构，不含机器人配置。|类型
POS|POS {X,Y,Z,A,B,C,S,T}|包含姿态配置的笛卡尔位置。|类型
E6POS|E6POS {X,Y,Z,A,B,C,S,T,E1..E6}|包含外轴数据的笛卡尔位置。|类型
AXIS|AXIS {A1..A6}|机器人关节角结构。|类型
E6AXIS|E6AXIS {A1..A6,E1..E6}|包含外轴的关节结构。|类型
TRUE|TRUE|布尔真。|常量
FALSE|FALSE|布尔假。|常量
ABS|ABS(value)|绝对值。|函数
SIN|SIN(angle)|角度制正弦。|函数
COS|COS(angle)|角度制余弦。|函数
TAN|TAN(angle)|角度制正切。|函数
ASIN|ASIN(value)|反正弦。|函数
ACOS|ACOS(value)|反余弦。|函数
ATAN2|ATAN2(y,x)|由两个分量计算角度。|函数
SQRT|SQRT(value)|平方根。|函数
TRUNC|TRUNC(value)|截取整数部分。|函数
STRLEN|STRLEN(text[])|取得字符串长度。|函数
STRCOMP|STRCOMP(left[],right[],caseMode)|比较字符串。|函数
STRCLEAR|STRCLEAR(text[])|清空字符数组。|函数
STRCOPY|STRCOPY(destination[],source[])|复制字符串。|函数
STRADD|STRADD(destination[],source[])|连接字符串。|函数
CREAD|CREAD(handle,state,mode,timeout,offset,format,values)|按通信通道读取数据；签名随系统能力核验。|通信
CWRITE|CWRITE(handle,state,mode,format,values)|向通信通道写入数据。|通信
SREAD|SREAD(text[],state,offset,format,values)|解析格式化字符串。|字符串
SWRITE|SWRITE(text[],state,offset,format,values)|构造格式化字符串。|字符串
`.trim().split('\n').map(row => row.split('|'));
const snippets: Record<string, string> = {
  DEF: 'DEF ${Main}()\n  ${}\nEND', DEFFCT: 'DEFFCT ${REAL} ${Calculate}()\n  RETURN ${0.0}\nENDFCT',
  DEFDAT: 'DEFDAT ${Main} PUBLIC\n  ${}\nENDDAT', IF: 'IF ${condition} THEN\n  ${}\nENDIF',
  FOR: 'FOR ${i}=1 TO ${10}\n  ${}\nENDFOR', WHILE: 'WHILE ${condition}\n  ${}\nENDWHILE',
  LOOP: 'LOOP\n  ${}\nENDLOOP', REPEAT: 'REPEAT\n  ${}\nUNTIL ${condition}',
  SWITCH: 'SWITCH ${value}\nCASE ${1}\n  ${}\nDEFAULT\nENDSWITCH',
  SPLINE: 'SPLINE\n  SLIN ${target}\nENDSPLINE', PTP: 'PTP ${target}', LIN: 'LIN ${target}',
};
export const krlEntries: readonly LanguageEntry[] = data.map(([name, signature, description, category]) => ({
  name, signature, description, category, kind: category === '类型' ? 'type' : category === '函数' ? 'function' : 'keyword',
  insertText: snippets[name] ?? name, availableSince: 'KSS 8.x 核心语法参考；具体版本须核验',
  documentationUrl: 'https://www.kuka.com/en-de/products/robotics-systems/software/system-software/kuka_systemsoftware',
}));
