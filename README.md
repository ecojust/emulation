<div align="center">
  <h1>Emulation</h1>
  <p>ABB IRB 1100 · 三维机器人运动学工作台</p>
</div>

在三维场景中输入目标位置或姿态，求解 IRB 1100 的关节角与 ABB `robconf`，检查从当前起点出发的直线运动，再生成 RAPID `MoveL` 并预览姿态。

机器人基座固定在世界原点。项目复用现成的 IRB 1100 分段 CAD 网格，尺寸、机械零位、关节正向与运动范围按 ABB 官方资料校验。

![IRB 1100 现成模型、目标姿态、路径与生成的 RAPID](docs/images/workspace.png)

## 运行

需要 Node.js **22.12+**（推荐 Node.js 24）及支持 WebGL2 的现代浏览器；macOS 和 Windows 使用相同工程。

```sh
npm ci
npm run dev
```

打开终端输出的本地网址，默认 `http://localhost:5173`。计算和模型加载均在本机执行，不需要 Python、RobotStudio 或机器人连接。

生产构建与本地查看：

```sh
npm run build
npm run preview
```

`dist/` 可以部署到静态 HTTP 服务；由于使用模块和 Web Worker，应通过 HTTP 打开，不直接双击 HTML 文件。

## 使用流程

1. 选择 IRB 1100-4/0.58 或 IRB 1100-4/0.475，观察基座原点及机械零位。两种型号均使用自己的现成网格和尺寸。
2. 设定起始关节角，或选择界面的示例起始姿态。官方机械零位与应用示例分开：全零位的轴5为0°，处于腕部奇异点，不能被当成已验证的直线起点。
3. 输入毫米单位的 XYZ；可让系统选择姿态，也可输入固定姿态或完整 ABB `robtarget`。ABB 四元数使用 `[w,x,y,z]` 顺序；欧拉角遵循 `OrientZYX(Rz,Ry,Rx)`。
4. 求解后查看关节角、`[cf1,cf4,cf6,cfx]`、位置／姿态误差和路径结果。其他终点逆解只作参考，不能代替整段路径验证。
5. 播放直线姿态预览，复制生成的 `MoveL`。修改目标、起点或型号后，原来的指令会失效，需要重新求解。

## 模型与运动学

- 官方几何来源：ABB 产品规格 **3HAC064993-001**；标准 DH 按其标称几何和关节约定推导。完整参数、零位及官方工作空间核对见 [机器人参数](docs/robot-model.md)。
- 现成外观来源：[Workr Robot Asset Library](https://github.com/work-r-labs/robots)，MIT 许可。两种型号分别包含基座和六个关节部件；原始字节、固定版本与校验值见 [模型资源说明](docs/model-assets.md)。这些是社区分发的现成资产，并非 ABB 官方认证的仿真包。
- RAPID／配置依据：ABB 技术参考 **3HAC050917-001**，包括 `confdata`、四元数、`robtarget`、`ConfL` 与 `MoveL`。详见 [RAPID 与配置约定](docs/rapid.md)。
- 场景使用右手坐标系，世界 Z 轴向上；基座在 `[0,0,0]`。公开位置单位为 mm、关节角为度。使用 `tool0` 的法兰中心和 `wobj0`，没有隐含工具偏移或工件坐标变换。
- 正运动学、逆运动学和显示姿态使用同一条 DH 链。现成网格从各自的共同机械零位坐标变换到实时关节姿态。
- 解析逆解适用于这里的两款 IRB 1100；自动姿态先尝试保持起始方向，再搜索有限组方向，结果不宣称全局最优，也不是任意 DH 表的通用求解器。
- MoveL 按不超过 5 mm／2° 的间隔采样，检查 cfx 不变、关节限位及连续性。相邻采样任一轴变化超过 15° 时保守拒绝；接近奇异点的部分路径可能需要调整起点或方向。

可达结果是基于标称几何、关节范围和离散路径检查的**运动学仿真**。它不包含环境／自碰撞、负载、制动、个体校准或完整控制器动态验证；播放用时也不代表真实控制器节拍。生成的 RAPID 需在对应 RobotStudio／控制器配置中复核，不能将这里的预览当作实机执行证明。

## 开发与检查

```sh
npm run check
npm test
npm run build
```

实际浏览器回归使用 Playwright：

```sh
npx playwright install chromium
npm run test:ui
```

已有 Chromium 时可设置 `EMULATION_CHROMIUM` 指向其可执行文件，避免重复下载。CI 同时检查数学模型、界面、模型加载和构建产物。

| 目录 | 内容 |
| --- | --- |
| `src/robot/` | 型号、标准 DH、正逆运动学、配置和直线路径规划 |
| `src/scene/` | Three.js 场景、现成机器人模型、姿态与轨迹显示 |
| `public/models/` | 两款 IRB 1100 的原始分段 STL |
| `vendor/work-r-labs/` | 上游许可、固定来源和逐文件摘要 |
| `tests/` | 数值与浏览器回归 |
| `history/` | 用户原话、验证与交接记录 |

项目采用 [MIT 许可证](LICENSE)。生产构建包含应用许可证、前端依赖声明及模型来源许可。接手开发先读 [AGENTS.md](AGENTS.md) 和 [历史索引](history/INDEX.md)。
