# 按型号绑定的仿真程序模板

模板语言来自已核对的型号目录，不提供脱离机器人型号的品牌切换。目前两款 ABB 使用 RAPID、两款 FANUC 使用 LS、两款 Yaskawa 使用 INFORM。目录没有可加载 KUKA 型号，因此不声称支持 KUKA 程序。

模板用于查看示教点和仿真程序结构，不是完整的控制器文件导出器。六个型号已支持按全部程序行预规划后运行仿真，但运行结果与模板导出是独立边界：整条程序的模板始终标记为不可导出；已有的独立 ABB 单段导出仍须由 Rust 重新验证。FANUC、Yaskawa 控制器文件导出继续返回未支持。

| 品牌 | 目标点表示 | 直线指令 | 尚未具备的控制器数据 |
| --- | --- | --- | --- |
| ABB | `CONST robtarget P001 := [[x,y,z],[w,x,y,z],[cf1,cf4,cf6,cfx],[9E9,…]];` | `MoveL P001, [speed,500,5000,1000], fine, tool0\WObj:=wobj0;` | 完整程序的控制器后处理／验收；已有仿真工具和工件坐标约定不等于实机校准 |
| FANUC | `/POS` 内 `P[1]` 的 `GP1` 和 `J1…J6` 关节位置 | `/MN` 内 `L P[1] speedmm/sec FINE ;` | `{{UF}}`、`{{UT}}`，URDF 与控制器之间的关节零位和耦合映射 |
| Yaskawa | `///POSTYPE BASE`、`///RECTAN` 下 `C00000=x,y,z,Rx,Ry,Rz` | `MOVL C00000 V=speed` | `{{TOOL}}`、逐点 `{{RCONF_P001}}`，控制器相对作业能力及坐标标定 |

FANUC 采用关节位置记录，因此不凭几何外观猜测 `N/F`、`U/D`、`T/B` 配置。Yaskawa 采用笛卡尔记录，不用未知的每度脉冲数生成 `PULSE`。这些模板中的占位符有意保留缺失信息，不能作为可直接导入控制器的模块。

位置以毫米保存；内部姿态使用 `[w,x,y,z]`。FANUC 的 W/P/R 和 Yaskawa 的 Rx/Ry/Rz 按 `R = Rz(Rz) Ry(Ry) Rx(Rx)` 表示，单位为度。欧拉角奇异处选择等价表示，测试独立使用 Three.js 重建旋转，覆盖正负 90° 俯仰与四元数正负等价性。

程序点拥有全程序内稳定编号：`P001` 对应 FANUC `P[1]`、Yaskawa `C00000`。插入和删除不会复用已有点号；转换为 Path 只建立引用。模板按原程序行顺序生成指令和点定义，并用同一个编号函数生成引用，不能对某个 Path 子集重新编号。

删除后模板可能包含稀疏编号。本阶段没有验证各控制器版本对目标编号上限、稀疏编号或 ASCII 导入选项的限制；未来实机后处理必须核对这些约束并同时重写指令引用和定义，不能只压缩点表。

## 编辑与运行

程序页示教模式按当前型号显示品牌指令和目标点表示；代码模式可“从示教生成”完整源码，用户确认后替换当前文件并可撤销。源码不随示教变化自动覆盖，详见[离线编程](offline-programming.md)。右键插入当前姿态会复制六轴关节、位置、四元数和速度；ABB额外要求有效robconf，其他品牌必须没有ABB配置。多选转Path只改变对象归属，程序执行仍按原行序。

“运行程序”先检查全部运动段，成功后才切入播放。勾选“循环运行”还须验证末点到首点的闭合段，后续只重复闭合段和程序内各段，不重复运行前姿态到首点的接近段。运行中不可增删或重分组，停止保留当前显示姿态；失败不会跳过某段继续播放。

仿真规划不解析模板中的代码或占位符，而是读取结构化程序点并调用该型号的运动规划器。因此Yaskawa保留待配置的RCONF／TOOL并不妨碍URDF仿真，也不能据此认为模板已经成为可执行控制器作业。Path点右键“移动至此”只是单独的关节姿态定位，不会把模板或完整运动标记为已验证。

## 可复查的语法依据

- ABB 沿用项目现有 RAPID `robtarget` / `speeddata` / `MoveL` 格式及独立原生后处理器；见 [rapid.ts](../src/robot/rapid.ts) 与 [ABB 后处理说明](program-formats.md)。
- [FANUC 直线指令接受 joint-defined P 的参考实现](https://github.com/ros-industrial/robodk_postprocessors/blob/d7e6c1c07758d67d2906cfd638049bdff88cca72/Fanuc_RJ3.py#L345-L362)：`pose is None` 时先生成关节目标，随后仍输出 `L`；[关节目标语法](https://github.com/ros-industrial/robodk_postprocessors/blob/d7e6c1c07758d67d2906cfd638049bdff88cca72/Fanuc_RJ3.py#L570-L599)。
- [Yaskawa 作业头和属性](https://github.com/ros-industrial/robodk_postprocessors/blob/d7e6c1c07758d67d2906cfd638049bdff88cca72/Motoman.py#L188-L224)、[笛卡尔、RCONF 与脉冲记录的区别](https://github.com/ros-industrial/robodk_postprocessors/blob/d7e6c1c07758d67d2906cfd638049bdff88cca72/Motoman.py#L540-L631)。该参考实现的关节记录依赖 `PULSES_X_DEG`，本项目没有引入其比例或假定默认配置。

这些链接是公开后处理器实现的语法依据，不是本项目已完成厂商控制器验收的证明。格式化代码独立编写，没有复制或运行所引用的后处理器。
