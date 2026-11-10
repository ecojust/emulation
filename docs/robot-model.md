# IRB 1100 标称运动学模型

本项目支持 ABB IRB 1100-4/0.58 和 IRB 1100-4/0.475，默认使用 0.58 m 型号。模型基座底面中心是运动学局部原点；默认安装时与世界原点重合。世界坐标右手系、Z 向上；机械零位的手臂朝向为基座 +X。位置单位为 mm，关节角为度，四元数采用 ABB 的 `[w, x, y, z]` 顺序。

这里的 DH 表由官方标称尺寸及关节正向推导，不是 ABB 发布的 DH 表，也不是某台机器人的个体校准参数。求解使用 `tool0` 法兰中心、`wobj0` 与基座／世界重合，不包含夹具长度、外部轴、安装倾角或 Absolute Accuracy 补偿。

## 可复查资料

1. [ABB Product specification — IRB 1100，3HAC064993-001](https://search.abb.com/library/Download.aspx?DocumentID=3HAC064993-001&LanguageCode=en&DocumentPartId=&Action=Launch)，本次核对 Revision V：第 30–31 页为两款机器人零位尺寸，第 60–61 页为工作范围及轴 2／3 位置表，第 62 页为关节范围，第 66 页为最大关节速度。页码指 PDF 印刷页码。
2. [ABB Product manual — IRB 1100，3HAC064992-001](https://search.abb.com/library/Download.aspx?DocumentID=3HAC064992-001&LanguageCode=en&DocumentPartId=&Action=Launch)，本次核对 Revision X，第 686 页、6.2.2 节为各轴正转方向图。
3. [ABB Technical reference manual — RAPID Instructions, Functions and Data types，3HAC050917-001](https://search.abb.com/library/Download.aspx?DocumentID=3HAC050917-001&LanguageCode=en&DocumentPartId=&Action=Launch)，本次核对 Revision AC：`orient` 定义 scalar-first 四元数；`tooldata` 的预定义 `tool0` 位于法兰中心且相对 wrist frame 为恒等变换。该手册的姿态示例不是 IRB 1100 机械零位的标定证据。
4. [RobotComponents IRB1100_4_058](https://github.com/RobotComponents/RobotComponents/blob/5776b1e07005361cf02c9f485dfa6ba07b7bb492/RobotComponents.ABB.Presets/Robots/IRB1100_4_058.cs)、[IRB1100_4_0475](https://github.com/RobotComponents/RobotComponents/blob/5776b1e07005361cf02c9f485dfa6ba07b7bb492/RobotComponents.ABB.Presets/Robots/IRB1100_4_0475.cs) 及 [RobotKinematicParameters](https://github.com/RobotComponents/RobotComponents/blob/5776b1e07005361cf02c9f485dfa6ba07b7bb492/RobotComponents.ABB/Definitions/RobotKinematicParameters.cs)：固定到本次核对的提交 `5776b1e`，仅作为几何尺寸、正轴及法兰坐标方向的独立交叉核对；没有复制其 GPL 代码或资源。法兰 X／Y 方向采用此交叉核对结果，不声称已由 ABB 控制器逐姿态认证。

ABB 的下载地址可能更新文档修订版；核对时请先确认版本，再比对尺寸和页码。现成外观网格的来源、许可、固定提交和哈希见 [模型资源说明](model-assets.md)。社区网格的 URDF 轴符号、近似角度和惯性参数不参与运动学计算。

## 标准 DH 与坐标约定

采用 **standard DH**，不是 modified DH：

`A_i = Rz(q_i + offset_i) · Tz(d_i) · Tx(a_i) · Rx(alpha_i)`

`F_i = A_1 · … · A_i`，基座 `F_0 = I`。全部 `thetaSign = +1`；矩阵在代码中按行优先存储。下表的 L 和 F 随型号变化。

| 轴 i | a (mm) | d (mm) | alpha (°) | offset (°) |
| --- | ---: | ---: | ---: | ---: |
| 1 | 0 | 327 | -90 | 0 |
| 2 | L | 0 | 0 | -90 |
| 3 | 10 | 0 | -90 | 0 |
| 4 | 0 | F | 90 | 0 |
| 5 | 0 | 0 | -90 | 0 |
| 6 | 0 | 64 | 0 | 180 |

| 型号 | L 上臂 (mm) | F 前臂 (mm) | 标称臂展 (mm) | 额定载荷 (kg) |
| --- | ---: | ---: | ---: | ---: |
| IRB 1100-4/0.58 | 280 | 300 | 580 | 4 |
| IRB 1100-4/0.475 | 225 | 250 | 475 | 4 |

轴 6 的固定 180° 偏移使最后一帧直接表示法兰 `tool0`；它不是对用户输入的轴 6 再加一圈运动。所有关节为零时，法兰局部 X 指向世界 -Z，局部 Y 指向世界 +Y，局部 Z 指向世界 +X，即姿态 `[√0.5, 0, √0.5, 0]`。帧 4／5 的原点是球腕中心，帧 6 的原点是法兰中心，二者相距 64 mm，不能混用。

## 机械零位与关节范围

官方机械零位为 `[0, 0, 0, 0, 0, 0]`。此时的六条关节正轴依次为世界 `+Z, +Y, +Y, +X, +Y, +X`。例如轴 2 正转会让上臂向 +X 倾斜，轴 3 正转会让前臂朝下转动。

| 零位位置 | 0.58 m 型号 (mm) | 0.475 m 型号 (mm) |
| --- | --- | --- |
| 基座 | [0, 0, 0] | [0, 0, 0] |
| 肩／轴 2 中心 | [0, 0, 327] | [0, 0, 327] |
| 肘／轴 3 中心 | [0, 0, 607] | [0, 0, 552] |
| 球腕中心 | [300, 0, 617] | [250, 0, 562] |
| tool0 法兰中心 | [364, 0, 617] | [314, 0, 562] |

机械零位的轴 4 和轴 6 共线，属于腕奇异位形；可显示，但不能把它当作已经验证的正常 MoveL 起点。代码的 `readyJoints = [0, 20, -30, 0, 40, 0]` 是本应用提供的可调示例，不是厂家的零位或推荐待机点。

| 轴 | 最小角度 (°) | 最大角度 (°) | 0.58 m 最大速度 (°/s) | 0.475 m 最大速度 (°/s) |
| --- | ---: | ---: | ---: | ---: |
| 1 | -230 | 230 | 460 | 460 |
| 2 | -115 | 113 | 360 | 380 |
| 3 | -205 | 55 | 280 | 280 |
| 4 | -230 | 230 | 560 | 560 |
| 5 | -125 | 120 | 420 | 420 |
| 6 | -400 | 400 | 750 | 750 |

采用官方默认轴 6 范围；不假设用户已在控制器上扩大范围。最大速度是规格上限，不代表本应用已模拟加速度、转矩、载荷或控制器的时间最优轨迹。

## 独立几何验证及资料差异

`tests/model.test.ts` 用官方尺寸检查两款机器人的机械零位、法兰姿态与正轴，并用第 60–61 页的工作范围图／表核验 18 个非零关节组合的腕心坐标。表内关节角只保留一位小数，坐标也经舍入，比较容差为 1 mm。测试值不由本项目 FK 或 IK 重新生成，避免正逆解同时出错却互相通过。

核对中发现两处需保留说明的源资料差异：

- 第 60–61 页表题为“Positions at wrist center”，但 `pos0` 的 `[314, 562]`／`[364, 617]` 实际对应法兰中心，而不是相距 64 mm 的球腕中心。零位按尺寸图的法兰位置验证，其余 `pos1`–`pos9` 按腕心验证。
- 第 60 页 0.475 m 型号 `pos2` 的表格写 X=53.8 mm，同页尺寸图标为 **153.8 mm**；q2=9.7°、q3=55° 配合官方尺寸得到约 153.7904 mm。第 62 页俯视图也有 R53.8 标注。这里使用与官方尺寸及侧面图一致的 **153.8 mm** 作为该点的检查值，保留此冲突，不为匹配表格而修改机械参数。

模型复现的是公开标称几何及关节规则；离散 MoveL 检查不包含碰撞、驱动动力学或个体校准，也不等同于 RobotStudio／真实控制器的验收。
