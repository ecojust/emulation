# KRL 来源和边界

本项目 KRL 编辑支持是原创的 KSS 8.x 核心语法分析，不是 KUKA 编译器。当前没有 KUKA 模型、KRL 仿真或控制器文件后处理。

核对了公开编辑器中的语言事实：

- [KnoP-01/krl-for-vim](https://github.com/KnoP-01/krl-for-vim/tree/7af6b64f2045c0b0b576189d01bbbb2ad17d8077)，MIT；含现代 SPLINE、SLIN、SPTP、SCIRC 词法。
- [RoboDK/Industrial-Robots-VSCode](https://github.com/RoboDK/Industrial-Robots-VSCode/tree/6b1e43c3ce604fa1774533eb6b6437621376da46)，MIT；仅高亮，不作为完整 parser。
- 调研发现 `antlr/grammars-v4/kuka/krl.g4` 为 LGPL-3.0-or-later，来源说明为 2011 年逆向工程，缺少现代样条语法。**未复制、转换或打包该 grammar。**

本项目代码与中文说明独立编写，沿用项目 MIT 许可；未分发第三方实现或厂商手册正文。覆盖例程、数据模块、声明、表达式、结构/数组、核心控制结构及基本运动语法。技术包、完整内建函数/系统变量、类型检查、控制器选件和高级触发参数未全覆盖，分析器返回明确警告。不能将无语法错误视作控制器编译/执行通过。
