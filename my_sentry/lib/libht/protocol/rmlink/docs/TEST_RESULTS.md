# 验证结果

验证日期：2026-09-20。

## x86_64 Linux 主机

- 环境：项目 Nix Flake，GCC 15.2.0，GoogleTest 1.17.0。
- 独立 RMLink CMake 配置、编译和 CTest 通过。
- 40 个 GoogleTest 用例与 1 个文档示例，共 41/41 通过。
- 覆盖固定向量、解析恢复、会话/同步/START/关闭、DATA/LOG、队列/限速、心跳、重连、分配守卫和双向 1000 fps 模拟。
- 1000 fps 场景为内存替身模拟，不代表实际 USB 吞吐。

## STM32H723 交叉构建

- 环境：项目 Nix Flake，Arm Toolchain for Embedded Clang 22.1.0，`ATfE-Debug`。
- `h7_bsp.elf` 构建成功，RMLink 三个核心源文件参与编译。
- FLASH 183,492 B / 768 KiB（23.33%）。
- DTCMRAM 95,112 B / 128 KiB（72.56%）。
- 未烧录硬件，未运行实际 CDC/LibUSB 后端。

## 未验证平台

macOS 主机构建和真实 USB 时间精度未在本次运行中执行，不宣称通过。物理链路 P99 目标仍按 `INTEGRATION_MEASUREMENT.md` 待集成测量。

## 2026-09-21 LibXR 接入验证

- 原有独立核心测试重新运行，41/41 通过。
- 新增 `tests/libxr` 使用实际 LibXR 读写端口与假硬件，11/11 通过。
- 覆盖较长 uptime 后握手、最大帧双向传输、同步/异步完成、Busy/Full、失败完成、
  完成超时、清理后重连、末字节软件时间戳、正常关闭、周期同步和并发接收回调。
- 新测试先复现了两个核心错误：Device 进入 Active 未初始化接收存活时刻；Host 在
  Active 中拒绝合法周期同步响应。分别补齐初始化与状态判断后测试通过。
- Flake / ARM LLVM 22.1.0 / Debug 构建通过，开启 `RMLINK_BUILD_CDC_EXAMPLE=ON`。
  额外设置 `CMAKE_EXE_LINKER_FLAGS_DEBUG=-Wl,--undefined=RMLinkCdcExampleTask`，
  强制把未启动的示例及其适配器代码链接进 ELF，避免仅检查头文件或未使用的静态库。
- 此验证 ELF 的 FLASH 为 206,372 B，DTCMRAM 为 95,128 B，RAM_D3 为 1,888 B。
  这是整个固件的静态占用，不代表任务启动后 Core 队列与任务栈的堆使用量。
- 示例默认不编译、不启动；未烧录硬件，实际 USB 热插拔、吞吐及 P99 时间精度仍待实测。

## LibXR 4e96701 与 VQF Demo 更新验证

- 使用新版公开端口接口：`Capacity()`、`GetReadQueue()`、`GetWriteQueue()`。
- RMLink RX 改为任务读取已排队字节，接收时间是出队后的软件时刻；不挂起无法取消的异步读。
  停止硬件和端点后，复位确认使用 `FailFront()` 清退 TX、`ClearQueuedData()` 清空 RX。
- 适配器迁移后 12/12 测试通过，包括停止硬件后清理残留软件队列。
- `User/threads/rmlink_vqf.cpp` 已在 app_main 启动，VQF 载荷及原始时间戳测试 2/2 通过。
- RTTLogger 公开接口迁移测试 6/6、RemoteControl 接收与解码测试 20/20 通过。
- Flake / ARM LLVM 22.1.0 的 Debug 整包构建通过，独立 CDC 示例编译选项关闭。
  VQF Demo 与 RTT 新接口实现已实际链接进 ELF。
- 当前 FLASH 206,132 B，DTCMRAM 99,256 B，RAM_D3 1,888 B；未烧录或验证物理链路。
