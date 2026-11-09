# 机器人型号目录

点击顶部当前型号打开目录，按 ABB、FANUC、KUKA、Yaskawa 分组。选择已内置型号直接加载；新型号首次下载，展示进度并允许取消、失败重试。全部文件校验和网格解析成功后才替换当前机器人；取消或失败保留原场景与原规划结果。

| 型号 | 资源方式 | 当前能力 |
| --- | --- | --- |
| ABB IRB 1100-4/0.475 | 已内置 | 官方标称DH、IK、MoveL、TCP Jog、端点包围盒预检、RAPID导出 |
| ABB IRB 1100-4/0.58 | 已内置 | 同上，使用自己的尺寸与网格 |
| FANUC CRX-10iA/L | 首次下载 | URDF网格、关节调整、工具坐标与正运动学 |
| FANUC R-2000iB/165F | 首次下载 | 同上，包含源模型的两个附加配重部件 |
| Yaskawa MOTOMAN GP7 | 首次下载 | URDF网格、关节调整、工具坐标与正运动学 |
| Yaskawa MOTOMAN GP110 | 首次下载 | 同上 |
| KUKA | 当前来源没有资源 | 不提供虚构的可加载型号 |

模型库来自 [work-r-labs/robots](https://github.com/work-r-labs/robots)，固定版本为 `ab6aa489855256de07ba46c7b45887fdbc42b607`。该版本实际包含 ABB、FANUC、Kawasaki、Yaskawa，未包含 KUKA。目录按实际审查过的资源建立，不能从仓库 README 的品牌数量推断型号可用。IRB1010发现归一化URDF与网格轴向不一致，暂未加入。

新增型号没有套用IRB1100求解器。切换后右侧显示该型号的关节名称、角度范围，左侧显示对应工具坐标；未实现的MoveL、TCP拖拽求解、端点干涉与程序导出明确标为尚未适配。危险区仍保留在共享工作区并可编辑，但不输出新型号的干涉结论。切回IRB1100后恢复其功能，旧运动结果失效。

![按品牌选择与下载型号](images/robot-catalog.png)

![安川GP110关节调整后的实际工作区](images/yaskawa-gp110.png)

## 坐标与显示

URDF的 `xyz` 为米、`rpy` 为弧度，固定轴旋转顺序为 `Rz(yaw) Ry(pitch) Rx(roll)`。可动轴在各自关节坐标系中旋转，树按父子关系遍历，保留固定关节和侧分支。实际网格变换为：

```
世界到 link 的变换 × visual origin × mesh scale
```

FANUC采用毫米网格配合URDF的缩放与视觉原点，Yaskawa采用link局部米制网格。不能统一乘0.001，也不能使用IRB1100共同零位网格的补偿方法。场景基座放在世界原点，世界Z向上；界面工具坐标转换为mm，关节角显示为度。源URDF是模型预览资料，不是厂商控制器的个体标定参数。R2000的balancer按源URDF固定分支显示，未实现真实平衡连杆约束或动力学。

加载器只接受目录内的STL引用及fixed／revolute／continuous关节；拒绝DTD、外部资源、路径穿越、mimic、无效数值、循环与断开的树。完整网格准备成功后再交付给场景，切换时释放旧GPU资源，并按新模型的实际尺寸调整相机。

## 下载与缓存

[src/catalog/catalog.json](../src/catalog/catalog.json)保存型号、固定URL、逐文件字节数、Git blob SHA-1和SHA-256。应用不接受用户传入的任意下载URL或文件路径。

桌面由Rust下载到应用数据目录下的模型缓存。暂存文件在完全校验后原子提交；取消和错误清理当前暂存，完整旧缓存保留。再次使用会重新验证文件，完整缓存不需要网络。加载只向受限Tauri资源协议开放已验证的具体文件。

浏览器参考模式以相同目录校验文件，使用IndexedDB按型号和来源版本缓存；它与桌面缓存互不共享。清除浏览器站点数据会删除浏览器缓存。缓存损坏时重新下载；网络或存储失败会显示错误，不切换成残缺模型。

目录生成脚本会下载审查过的资源到临时目录，再核对原始Git对象、STL顶点与来源记录：

```sh
node scripts/generate-model-catalog.mjs --check
```

已有来源缓存时可加 `--offline`。四款新增机器人的STL不加入应用安装包，避免所有用户预先下载全部资源。

## 许可来源

Workr集合仓库的MIT声明不替代原始模型声明。ABB／FANUC保留Toshinori Kitamura的MIT原文；Yaskawa的来源包声明BSD-3-Clause，保留原文package.xml、逐网格来源映射和标准BSD条款模板，并明确上游未提供独立版权文件，不猜写版权持有人。详见[原始模型声明](../vendor/work-r-labs/model-notices/README.md)。
