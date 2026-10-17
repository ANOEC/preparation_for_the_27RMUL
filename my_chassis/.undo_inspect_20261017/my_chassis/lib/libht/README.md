# LibHT

平台无关的 C++ 库集合，可以独立于机器人工程、LibXR、XRobot 和 RTOS 构建。
自有目录使用小写；第三方内部的 `Eigen/` 等公开路径保持上游大小写。

| 目录 | 内容 | 依赖 |
| --- | --- | --- |
| `algorithm/` | PID、LESO、LADRC、RLS、Wave | 标准库；RLS 额外使用 Eigen |
| `protocol/rmlink/` | 字节流协议、会话、时间同步、类型化载荷 | CRC、Serialization |
| `utility/crc/` | CRC 软件算法和可选 STM32 后端 | 标准库；硬件后端由消费者提供设备头 |
| `utility/serialization/` | 固定长度载荷 codec | 自带 zpp/PFR |
| `lib/eigen/` | 固定内容的 Eigen 源码快照 | 不从其他框架寻找头文件 |

## 原生 XMake

依赖 XMake 3.0+、C++20/23 编译器。测试需要 pkg-config、GoogleTest、Python；
Arm 编译探针需要 PATH 中的 `arm-none-eabi-g++`。测试默认关闭，
只构建库时无需 GoogleTest；通过 `--tests=y` 显式启用测试。
从包含 LibHT 的工程根目录执行（`-P` 明确选择库工程）：

```sh
xmake f -P lib/libht -m debug --tests=y --wire_backend=zpp --wire_standard=cxx20
xmake build -P lib/libht rmlink
xmake test -P lib/libht -j 4 -g tests
python3 lib/libht/tests/run_matrix.py --output build/libht-matrix
python3 lib/libht/tests/test_standalone.py --output build/libht-isolation
```

在独立 checkout 中将 `-P lib/libht` 改为 `-P .`。不需要消费者提供 LibXR 头文件或宏。
XMake 定义 `rmlink` 静态库以及 `crc`、`serialization`、`pid`、`leso`、`ladrc`、`rls`、
`wave` 头文件目标。`build.lua` 为原生消费者提供共享 codec 的包含目录和后端定义，
不引入其他框架。库内测试/基准使用自有样本类型；真实机器人载荷由消费者另行验证。
每个有编译单元的 target 构建后会更新 `build/<target>/<mode>/compile_commands.json`，
仅包含自己的实际编译命令。纯头文件和 phony 目标不生成数据库；无需手动导出，不提交到 Git。

## 独立 CMake 消费者

现有 CMake 入口继续保留，目录路径统一使用小写：

```cmake
add_subdirectory(path/to/libht libht)
target_link_libraries(application PRIVATE LibHT::PID LibHT::RLS LibHT::rmlink)
```

目标名和 C++ 命名空间保持原名。CMake 开关为 `LIBHT_BUILD_ALGORITHM`、
`LIBHT_BUILD_CRC`、`LIBHT_BUILD_SERIALIZATION`、`LIBHT_BUILD_RMLINK`，默认均为 ON。
`LIBHT_BUILD_RLS` 默认 ON；启用 RLS 却缺少 Eigen 会报错，不会静默跳过。
独立库测试仍可通过各目录的 `tests/CMakeLists.txt` 构建。

## Eigen 来源与配置

默认使用 `lib/eigen`，快照内容摘要与版本记录见 `lib/versions.json`。
当前本地快照没有可核实的上游 commit，不把版本字符串当成上游提交锁。
保留完整源码、许可证和快照摘要即可按内容复现。

消费者可以显式设置 CMake 的 `LIBHT_EIGEN_DIR` 或 XMake 的 `--eigen_dir`。
同一固件使用多个依赖 Eigen 的库时，由消费者统一版本、对齐/向量化等配置宏。
LibHT 不自动查找其他库内部的 Eigen，也不强制修改消费者的 Eigen 宏。

## 平台集成边界

RMLink 通过 `TransportOps` 注入时间、发送和中止行为。
LibXR UART/Topic 适配器、CDC 示例和集成测试已经迁出，由消费者工程维护；
它们不是 LibHT 的公开头文件或构建输入。不提供旧适配器包含路径的兼容转发。

CRC 的 STM32 后端属于可选硬件加速，保留在 `utility/crc/adapters/stm32`。
消费者显式加入源文件和设备头；CRC 核心不包含 STM32 设备头。
