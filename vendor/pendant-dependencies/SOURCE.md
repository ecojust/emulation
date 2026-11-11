# 示教器原生依赖许可来源

本目录保存独立示教器模块新增的15个Rust运行时crate版本，以及模块直接使用的既有`getrandom`和`futures-util`的原始许可文件。范围以本次新增依赖为准；不是整个应用的完整依赖清单。

比较基线：`4d664b0eb8e9e8a247e4685b2caaaaf22bce88eb`的`Cargo.lock`。每个版本均取自当前锁文件对应的crates.io `.crate`归档，归档SHA-256与锁文件核对一致后，直接复制其中的许可／版权文件，未改写内容。Windows专用的`windows-sys 0.59.0`也包含在内。

`SOURCE.json`记录固定版本、上游仓库、原始归档URL、归档摘要以及每个复制文件的摘要。许可表达式为原crate元数据；双许可文件全部保留，`matchit`的MIT与BSD-3-Clause两份许可也全部保留。打包时应连同这些文件分发。

| crate | 版本 | 关系 | 原始许可 | 上游 |
| --- | --- | --- | --- | --- |
| [axum](https://crates.io/crates/axum/0.8.9) | 0.8.9 | 直接 | MIT | [tokio-rs/axum](https://github.com/tokio-rs/axum) |
| [axum-core](https://crates.io/crates/axum-core/0.5.6) | 0.5.6 | 间接 | MIT | [tokio-rs/axum](https://github.com/tokio-rs/axum) |
| [base64](https://crates.io/crates/base64/0.22.1) | 0.22.1 | 间接 | MIT OR Apache-2.0 | [marshallpierce/rust-base64](https://github.com/marshallpierce/rust-base64) |
| [data-encoding](https://crates.io/crates/data-encoding/2.11.1) | 2.11.1 | 间接 | MIT | [ia0/data-encoding](https://github.com/ia0/data-encoding) |
| [futures-util](https://crates.io/crates/futures-util/0.3.34) | 0.3.34 | 直接 | MIT OR Apache-2.0 | [rust-lang/futures-rs](https://github.com/rust-lang/futures-rs) |
| [getrandom](https://crates.io/crates/getrandom/0.3.4) | 0.3.4 | 直接 | MIT OR Apache-2.0 | [rust-random/getrandom](https://github.com/rust-random/getrandom) |
| [httpdate](https://crates.io/crates/httpdate/1.0.3) | 1.0.3 | 间接 | MIT OR Apache-2.0 | [pyfisch/httpdate](https://github.com/pyfisch/httpdate) |
| [if-addrs](https://crates.io/crates/if-addrs/0.13.4) | 0.13.4 | 直接 | MIT OR BSD-3-Clause | [messense/if-addrs](https://github.com/messense/if-addrs) |
| [matchit](https://crates.io/crates/matchit/0.8.4) | 0.8.4 | 间接 | MIT AND BSD-3-Clause | [ibraheemdev/matchit](https://github.com/ibraheemdev/matchit) |
| [ppv-lite86](https://crates.io/crates/ppv-lite86/0.2.21) | 0.2.21 | 间接 | MIT OR Apache-2.0 | [cryptocorrosion/cryptocorrosion](https://github.com/cryptocorrosion/cryptocorrosion) |
| [rand](https://crates.io/crates/rand/0.9.5) | 0.9.5 | 间接 | MIT OR Apache-2.0 | [rust-random/rand](https://github.com/rust-random/rand) |
| [rand_chacha](https://crates.io/crates/rand_chacha/0.9.0) | 0.9.0 | 间接 | MIT OR Apache-2.0 | [rust-random/rand](https://github.com/rust-random/rand) |
| [rand_core](https://crates.io/crates/rand_core/0.9.5) | 0.9.5 | 间接 | MIT OR Apache-2.0 | [rust-random/rand](https://github.com/rust-random/rand) |
| [sha1](https://crates.io/crates/sha1/0.10.7) | 0.10.7 | 间接 | MIT OR Apache-2.0 | [RustCrypto/hashes](https://github.com/RustCrypto/hashes) |
| [tokio-tungstenite](https://crates.io/crates/tokio-tungstenite/0.29.0) | 0.29.0 | 直接 | MIT | [snapview/tokio-tungstenite](https://github.com/snapview/tokio-tungstenite) |
| [tungstenite](https://crates.io/crates/tungstenite/0.29.0) | 0.29.0 | 间接 | MIT OR Apache-2.0 | [snapview/tungstenite-rs](https://github.com/snapview/tungstenite-rs) |
| [windows-sys](https://crates.io/crates/windows-sys/0.59.0) | 0.59.0 | 间接 | MIT OR Apache-2.0 | [microsoft/windows-rs](https://github.com/microsoft/windows-rs) |

只保留必要的许可和版权声明，不将第三方构建缓存或实现源码复制到仓库。
