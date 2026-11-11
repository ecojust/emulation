# 局域网示教器连接模块

本模块仅提供仿真客户端与手机之间的连接，不包含真实机器人连接、运动求解或文件访问能力。`PendantServer::new()`不监听网络，只有桌面显式调用`start()`才在本机随机IPv4端口启动服务；`stop()`撤销会话并停止监听。桌面代码负责在关闭示教器、退出项目和退出应用时停止模块。

固定HTTP资源来自`src/pendant/mobile/`，随原生客户端编译，无外部CDN。二维码URL的fragment持有随机256位配对凭据，凭据仅在内存中使用，不写入HTTP路径或日志。HTTP／WebSocket的Host与Origin须匹配启动时枚举的本机地址。扫码设备需与电脑处于能互通的局域网；本模块使用局域网HTTP／WebSocket，没有云端中转或互联网端口映射。

## 协议与生命周期

WebSocket路径`/ws`，消息为JSON：

- 手机先发送`auth`（`token`），服务端返回`ready`（`sessionId`、整数`epoch`）。配对凭据在创建后10分钟内可用于建立连接；既有控制会话最多8小时。重新配对会撤销旧会话。
- 手机发送`input`：`epoch`、严格递增`sequence`、`mode`（`linear`／`joint`）、`space`（`world`／`tool`）、6个`axes`值、`speed`、`active`。轴分量为[-1,1]，速度比例为[0,1]；直线模式只用前三个分量。停手必须发送`active:false`且所有分量为零。
- 每个epoch首先需要一个零输入，之后才能开始新的按住手势。服务端发送`stop`（新`epoch`、`message`）时，手机必须松开当前指针、发送新epoch的零输入，之后重新按住。旧epoch排队消息不会恢复移动。
- 电脑每200毫秒以内调用`publish_state(sessionId,state)`更新真实机器人、当前关节、TCP、可用原因和移动状态。服务端以`state`消息转发给手机；此通道使用替换式watch槽，不累计旧状态。
- 350毫秒未收到当前手势输入、800毫秒未收到电脑状态、电脑不可控制、连接断开、模式在手势中改变或会话关闭时，立即撤销当前输入并发出零输入。电脑桥接层还需拒绝过时`receivedAtMs`、旧epoch，并取消正在求解的旧任务；模块不会直接修改机器人数据。

同一时刻只允许一个已认证控制端。最多16个TCP连接、4个待认证WebSocket连接，HTTP空闲5秒超时；报文大小、消息频率和写等待均有界；不接受任意远程命令或文件路径。配对信息中含有凭据，因此不要打印完整`PendantSession`、二维码URL或认证报文。

## 验证

```sh
cargo test -p emulation-desktop --no-default-features --features model-cache,pendant --locked
cargo build -p emulation-desktop --example pendant-check --no-default-features --features pendant --locked
```

真实TCP／WebSocket测试覆盖资源路由、Origin／Host、拒绝错误凭据、单控制端、初始释放、超时后的旧帧、桌面断联、会话替换和错误／重放输入。`pendant-check`是显式调用的本地集成工具：将配对会话写入新建的临时私有文件，从stdin接收桌面状态，stdout只输出无凭据的输入事件。应用安装验证另通过`smoke_drive`在实际WebView连接流程中发送有限手势；桌面调用入口必须受`--ui-smoke-test`约束。

## 依赖来源

版本由根目录`Cargo.lock`固定。新增直接依赖均通过crates.io取得，未复制或改写第三方源码：

| 依赖 | 用途 | 上游与许可 |
| --- | --- | --- |
| axum | HTTP与WebSocket服务 | [tokio-rs/axum](https://github.com/tokio-rs/axum)，MIT |
| tokio-tungstenite | 实装检查与真实套接字测试客户端 | [snapview/tokio-tungstenite](https://github.com/snapview/tokio-tungstenite)，MIT |
| if-addrs | 枚举本机IPv4接口 | [messense/if-addrs](https://github.com/messense/if-addrs)，MIT／BSD-3-Clause |
| getrandom | 操作系统随机配对凭据 | [rust-random/getrandom](https://github.com/rust-random/getrandom)，MIT／Apache-2.0 |
| futures-util | 有界WebSocket读写协作 | [rust-lang/futures-rs](https://github.com/rust-lang/futures-rs)，MIT／Apache-2.0 |

Tokio为项目既有依赖，本模块启用其网络、定时器和异步运行时功能。`pendant`为独立Cargo feature，桌面默认包含它，纯数值核心无需网络组件。
