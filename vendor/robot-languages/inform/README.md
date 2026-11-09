# INFORM 来源与覆盖

默认语言版本为 YRC1000 INFORM expanded 外部 `.JBI` 文件；依据 178649-1CD / RE-CKI-A466 revision 10（422 页）。`instruction-index.json` 仅记录公开目录的指令名、分组和页码链接，共 117 项，含附加语法项，不等于 117 条独立可执行指令。它不包含所有控制器技术选件。

- [官方手册入口](https://knowledge.motoman.com/hc/en-us/articles/4407425435927-YRC1000-INFORM-LANGUAGE)；§1.6 为指令语法图，§1.7.4 为外部文件序列化格式，§2 为指令与参数。
- [官方 JBI Header Fields](https://knowledge.motoman.com/hc/en-us/articles/21577951275031-INFORM-Header-Fields)：头部、位置记录、GROUP、LVARS、RCONF。
- 厂商手册正文、图、示例及 PDF 未随应用分发；目录中的名称及页码为接口事实。中文说明、解析器及测试独立编写，沿用项目 MIT 许可。
- 调研过 `tshikina/YaskawaInformSupport@f0e87861e987461ee9ecd6455f4a0f918ce4df4b`（BSD-3-Clause）和 `andyprv/yaskawa-inform@ec442d1c6b3fde04e3ac27dcd9796c2ed2c72031`（MIT）。未复制它们的实现；这些高亮/局部检查器不是完整厂商编译器。

## 参数语法覆盖

[grammar-coverage.json](grammar-coverage.json) 按官方目录顺序列出全部 117 项，每项记录实际解析的生产式、官方页码、独立编写的合法样例、最小非法近邻和剩余边界。[对应测试](../../../tests/programming-inform-manual.test.ts) 真正运行这些样例，不能只靠名称存在而通过。示例都作为外部 `.JBI` 的指令区内容，附加项放入合法父指令；它们不是可直接部署的控制器作业。

| 范围 | 已实现的语法检查 |
| --- | --- |
| 文件与记录 | `/JOB`、`//NAME`、`//POS`、`//INST` 顺序，头部字段，NPOS/实际记录计数，C/BC/EC/P/BP/EX 数值与轴数量，脉冲整数，位置引用 |
| 结构控制 | 外部 IFTHENEXP/ELSEIFEXP/WHILEEXP，SWITCH 首行 CASE，CASE/DEFAULT，FOR/NEXT 变量匹配与嵌套；本地标签定义/引用、JUMP LABEL/JOB/JET、CALL 入参与条件、RET |
| 运算与资源 | SET 数值表达式/同类位置/字符串/LOGICEXP 分支，算术与位运算，字符串函数，MULMAT/INVMAT，MFRAME 的两种构造，P/PX 区分，GETARG 局部目标，条件文件/系统变量/寄存器/参数的标签和参数分支 |
| I/O 与操作 | 数字位/组/用户组的数据类型，WAIT/FOREXPRESS，PULSE、模拟输出、ARATION FINE DELAY、ANTOUT、报警、DIALOG 一至五个布局记录及其首按钮要求 |
| 运动与偏移 | MOVJ/MOVL/MOVC/MOVS/IMOV 分别允许的目标和标签、顺序/互斥组、IMOV 坐标系，ENWAIT、UNTIL CAPTURE、TRAP、本地/附加平移与三维平移 |
| 已核定选件 | Search rev1 的 SRCH 输入/状态/时间/距离类型、顺序和常量范围；Sensor rev3 的 ASRCH、ACOR*/SACOR*、AOVR*、GETSFT、SCALIB 参数分支及常量范围 |
| 工艺指令 | 弧焊条件文件/数值两分支、电源号、摆动/渐变、HAND/HSEN、TOOLON/TOOLOF、焊枪闭合/单枪双枪 SPOT/行程 |

两处容易混淆的外部格式已按手册处理：`SWITCH I003 CASE 2` 的首个 CASE 与 SWITCH 在同一行；FOR 的 `I003=2`（物理页 126）与 `I003 START=2`（物理页 15、260 的数组示例）均有官方依据，当前两种都接收。IFTHEN/ELSEIF/WHILE 则使用 §1.7.4 规定的外部 EXP 形式，不能套用示教器显示形式。

## 独立选件手册的核查结果

[option-research.json](option-research.json) 记录真实取得的官方 PDF URL、修订、大小、SHA-256、已查章节和缺失信息。[选件测试](../../../tests/programming-inform-options.test.ts) 检查合法形式以及参数缺失、类型、边界、顺序、父指令与后续标签。所有实现均为原创；厂商 PDF 只供核对，没有打包分发。

| 语法 | 取得的资料及实际状态 |
| --- | --- |
| SRCH | 已取得 HW1483392 / 178657-1CD rev1；按 §3.1.2–3.1.3 实现 MOVL/IMOV 附加语法。DIS 必填，T 可选，保留后续 ACC/DEC 等标签的校验。 |
| ASRCH | 已取得 HW1483394 / 179329-1CD rev3；按 §6.3 实现 MOVL 的常量形式。其图未规定变量类型，变量形式会明确标记未验证。 |
| ACOR*/SACOR*、AOVR*、GETSFT、SCALIB | 按 Sensor rev3 的 §3.4、§4.2、§5.3、§7.1 实现十条独立指令的参数分支。成对 TCH#/TV、可选条件文件/目标、互斥分支与常量范围均检查；文档未给出的标量变量类型不冒充已验证。 |
| HPSRCH | 主 INFORM rev10 指向上述 Sensor 手册，但公开 rev3 全部 53 页没有此项语法或参数；缺该指令的实际版本规范，保留未验证警告。 |
| NSRCH | 官方目录与 Knowledge Center 查询未取得准确的 YRC1000 HW1483380。已取得的 YRC1000micro HW1484520 使用 NSRCHON/NSRCHOF，属于不同控制器与产生式，不能替代本版本支持。 |
| +MOVJ / +MOVL / +MOVC / +MOVS / +IMOV | 已取得 HW1484042 / 178660-1CD rev4；§9.1 是参数概要，没有完整外部 JBI 序列化与组合产生式。具体核查及缺项见 [coordinated-research.md](coordinated-research.md)，仍标记未验证。 |

这些仍缺规范的形式产生 `inform-option` 警告，**没有把警告之后的参数当成校验通过**。新增 Search/Sensor 指令单独计入选件条目，不篡改主目录 117 项的统计，也不能据此声称覆盖其他所有工艺选件。协调 SMOVL 父指令的外部序列化同样没有纳入已验证范围。

主手册已给出的 SPDL、MT/MTE、+SFTON3D 等参数形状可以分析，但是否安装伺服浮动、无限轴、多层焊等选件以及是否适用于当前作业控制组仍需要实际控制器配置。弧焊运行细节另见 RE-CSO-A052；这些限制属于配置/执行语义，不等同于上述缺失的参数语法。

## 已知实现边界

语法矩阵覆盖每个目录条目的指定生产式，不是穷尽所有标签取值组合的证明。当前没有完整检查所有常量范围/缩放单位、控制组适用表、局部变量容量、工艺文件/工具/I/O 资源存在性、跨作业名称链接或目标控制器版本。未知头部/属性/运动标签及未建模选件会产生明确诊断；编辑分析没有控制器执行能力，也不会证明碰撞或运动安全。

扫描和诊断偏移以 JavaScript UTF-16 单元计，保留 BOM/CRLF 的源码位置。分析限制为 1 MiB、单行 20,000 个词法项、128 层嵌套、300 条诊断；达到限制返回明确错误。保存 UTF-8 源文件不等同于输出控制器所需的 Shift-JIS/字节长度兼容文件。
