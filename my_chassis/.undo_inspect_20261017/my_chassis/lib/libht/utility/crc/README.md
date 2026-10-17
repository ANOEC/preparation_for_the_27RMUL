# CRC 校验库

参数化 CRC 的纯头文件 C++20 实现，供协议栈和固件模块共用。算法以编译期参数描述，软件路径使用编译期生成的查表，可选的硬件后端在运行期透明加速且不改变结果。

库不依赖 LibXR、RTOS、HAL 或任何硬件抽象，可在主机上独立构建和测试。

## 快速开始

```cpp
#include "crc/crc.hpp"

const std::uint16_t header_crc = crc::Compute<crc::Crc16Ibm3740>(header_bytes);
const std::uint32_t frame_crc = crc::Compute<crc::Crc32IsoHdlc>(frame_bytes);
```

`Compute` 同时接受 `std::span<const std::byte>` 和 `std::span<const std::uint8_t>`，不分配内存、不抛异常，可在常量求值上下文中使用。

CMake 接入：

```cmake
include(FetchContent)
FetchContent_Declare(crc SOURCE_DIR "<path>/Middlewares/Third_Party/LibHT/utility/crc")
FetchContent_MakeAvailable(crc)
target_link_libraries(your_target PRIVATE crc::crc)
```

## 设计要点

- **算法参数集中定义**：每种算法由 `Spec<Width, Poly, Init, RefIn, RefOut, XorOut>` 完整描述，预设是类型别名。同一工程里多种 CRC 不会因为函数名相近而混用。
- **查表在编译期生成**，放在只读段，不需要运行期初始化，也没有懒初始化的线程竞争。
- **硬件后端可选且透明**：进程级注册至多一个后端；后端缺失、不支持该算法、正被其他执行上下文占用，或处于常量求值时，一律回退软件实现。调用永不阻塞，中断中也可安全调用，任一路径结果逐位一致。

## 文档

- [接口说明](./docs/API.md)
- [算法预设与校验值](./docs/PRESETS.md)
- [STM32 硬件后端](./docs/STM32.md)

## 主机验证

```sh
cmake -S Middlewares/Third_Party/LibHT/utility/crc/tests -B /tmp/crc-build -G Ninja
cmake --build /tmp/crc-build --parallel
ctest --test-dir /tmp/crc-build --output-on-failure
```

测试覆盖标准校验值、编译期求值、两种字节视图、与逐位参考实现的随机对拍，以及用假后端验证的注册和回退行为。
