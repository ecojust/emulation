# 多品牌运动程序与模块导出

最终方向是生成不同品牌的机器人运动程序文件。机器人几何、逆解和路径规划，与控制器语言的后处理分开；不能将 ABB 的 `MoveL` 改名后，就称为兼容其他品牌。

当前已实现六个目录型号的结构化示教点、Path分组、全程序运动学预规划与循环仿真，以及按品牌生成源码模板。另有[离线源码编辑](offline-programming.md)、明文源文件导入／原文保存、静态语法分析和按型号保存的多文件草稿。源码尚不反向生成场景轨迹或驱动仿真；经过运动复验的控制器文件导出仍限于ABB独立单段，场景／示教项目也尚未整体持久化。

已完成的控制器后处理是 **ABB IRB1100 → RAPID `.mod` 单段模块**。FANUC／Yaskawa已有真实URDF模型、六轴数值直线仿真与LS／INFORM模板，但其控制器后处理尚未适配；KUKA当前目录没有可加载型号。选择未支持的控制器导出目标会明确返回 `unsupported_controller`，不会把模板占位符保存成可用模块。

| 品牌 | 目标标识 | 文件方向 | 本次状态 |
| --- | --- | --- | --- |
| ABB | `abb_rapid` | RAPID `.mod` | 两款 IRB1100 的单段 MoveL 模块生成已实现 |
| 发那科 FANUC | `fanuc_ls` | ASCII `.ls`，与二进制 `.tp` 分开 | 有LS关节点仿真模板；控制器导出未适配，不输出文件 |
| 库卡 KUKA | `kuka_krl` | KRL `.src`／`.dat` | 未适配，不输出文件 |
| 安川 YASKAWA | `yaskawa_inform` | INFORM `.jbi` | 有INFORM笛卡尔仿真模板；控制器导出未适配，不输出文件 |

表中扩展名只是对应后处理的格式方向，不代表特定控制器已支持导入。包括 ABB 在内，尚未对某个 RobotWare／KSS／FANUC／YASKAWA 控制器版本完成编译或实机认证。FANUC ASCII 导入选项、KUKA S/T 姿态、安川坐标／脉冲模式等都需要按具体型号与控制器确认。

## 仿真程序与模板

应用按型号隔离会话程序。ABB点保留robconf；FANUC点显示关节定义的`P[n]`与`L`，不制造ABB或笛卡尔CONFIG；Yaskawa点显示`Cxxxxx`与`MOVL`，未确定的TOOL／RCONF保留占位而不生成脉冲。模板指令与点定义使用同一稳定编号；Path分组不改变原行序。语法依据、待配置字段和编号边界见[品牌模板说明](program-templates.md)。

程序运行先从当前姿态检查全部段，开启循环时连同末点到首点一起检查，通过后才开始动画；后续循环不重复初始接近段。停止取消规划／动画并保留当前姿态。规划成功仅说明这些离散运动学约束通过，未验证完整路径碰撞、自碰撞、动态或控制器可执行性，不会自动开启整个程序的文件导出。

## Rust 数据通路

`crates/robot-core/src/program.rs` 提供以下真实类型与接口：

1. `ProgramExportRequest` 包含模块名、控制器目标和原始 `PlanRequest`，不接受客户端提供的“成功结果”或代码正文。
2. 后处理注册器 `postprocessor_for` 返回 `ProgramPostprocessor`，检查该后处理是否接受机器人型号。
3. Rust 原生规划器重新验证起始关节、目标、奇异区域、关节限位及整段采样路径。取消或任何规划失败都不生成文件。
4. 内部构造 `ValidatedProgram`，封装通用 `MotionProgram`。它没有公开构造器，也不能从 JSON 直接反序列化。
5. `ProgramPostprocessor` 的实际返回类型为 `ProgramBundle`，包含一个或多个 `ProgramFile`；KUKA 这类需要源代码和数据文件的后处理无需挤入单文件接口。`AbbRapidPostprocessor` 当前生成一个 `.mod` 文件，包含建议文件名、正文、格式和必要说明。桌面层负责文件选择及写入。

通用接口是 `export_program_bundle`／`export_program_bundle_with_cancel`。当前桌面还可使用 `export_program`／`export_program_with_cancel` 这一单文件便捷接口，它只在文件包恰好有一个文件时成功，不会静默丢弃其他文件。

`MotionProgram` 明确带有结构版本、机器人型号／品牌、未取模的起始关节角、工具／参考坐标系和运动指令。当前 `Linear` 表示直线运动、TCP 速度和精确停止；位置为 mm、角度为度、四元数为数学上统一的 `[w,x,y,z]`。品牌特有配置放入显式的 `ControllerConfiguration`，当前变体为 `AbbConfData`；它不是通用于各品牌的四个配置整数。

这条控制器导出数据通路当前只接受两款六轴IRB1100，IR也只实际导出一段直线。另一路通用URDF规划器已用于四款FANUC／Yaskawa的六轴运动仿真，不会因此绕过后处理器的型号和控制器能力检查。后续增加其他品牌导出、关节数量、圆弧、轨迹过渡、工具偏移、工件坐标、外轴或IO，需要扩展相应类型与验证，不能直接复用现有ABB配置假设。

## ABB 模块内容与起点

导出内容包括：

- `MODULE`／`ENDMODULE` 与 `PROC main()`／`ENDPROC`。
- 本地 `jointtarget` 起点、重新求解的 `robtarget` 终点和显式 `speeddata`。
- `CJointT()` 逐轴起点检查：实际轴角与仿真起点相差超过 **0.01°** 时，`TPWrite` 提示并 `RETURN`，不执行后面的运动。
- `ConfL \On` 和 `MoveL ...,fine,tool0\WObj:=wobj0;`。

**模块没有自动回零或定位到起点的 `MoveAbsJ`。** 从任意当前姿态到计划起点的路径未被当前 MoveL 检查覆盖，不能悄悄追加。运行者需要先在相应工作站确认并准备起点，模块入口仅检查前置条件。轴 6 的完整转数保留，例如 360° 与 0° 不作为同一起点。

0.01° 是本应用的入口比较容差，并非 ABB 给出的校准或安全保证；实际机器的几何、工具、工作对象及控制器配置仍需核对。`main` 是模块入口，向已有任务加载时也需要处理已有同名主例程，应用不会自动合并控制器上的程序。

输出采用 ASCII 内容、CRLF 换行，模块名限制为 1–32 个英文字母、数字或下划线，首字符为英文字母，并拒绝 RAPID 关键字、导出模块内部标识符和 Windows 保留文件名。位置保留至 0.001 mm、四元数至 9 位小数、起始关节至 6 位小数；速度使用 `[TCP速度,500,5000,1000]`，不拼造不存在的 `v123` 常量。未配置的六个外轴均为 `9E9`。

## 资料与验证边界

ABB 官方依据为 [3HAC050917-001 Revision AC](https://search.abb.com/library/Download.aspx?DocumentID=3HAC050917-001&LanguageCode=en&DocumentPartId=&Action=Launch)：`CJointT` 第 1224–1225 页、`jointtarget` 第 1691–1692 页、`RETURN` 第 625 页。MoveL、配置、坐标与几何的其余依据见 [RAPID 约定](rapid.md) 和 [机器人模型](robot-model.md)。

原生程序测试验证两型号模块中的目标、配置、速度和起始转数，与原生规划结果一致；检查起点判断位于任何运动之前；覆盖不可达／奇异／无效／取消请求、不支持的品牌／型号、模块名注入及序列化边界。测试不是 RAPID 编译器，也不替代 RobotStudio、碰撞检查、动态仿真或实际控制器验收。

后续接入品牌时应一起提供：明确的机器人几何和正轴、与控制器一致的姿态／配置映射、对应版本的后处理、真实模块样例和控制器编译验证结果。应用能力列表只有在这些相应实现落实后才启用导出。
