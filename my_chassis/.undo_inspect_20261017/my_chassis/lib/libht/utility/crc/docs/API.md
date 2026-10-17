# 接口说明

公开入口为 `include/crc/crc.hpp`，命名空间 `crc`。

## 算法描述

```cpp
template <std::uint8_t Width, std::uint32_t Poly, std::uint32_t Init,
          bool RefIn, bool RefOut, std::uint32_t XorOut>
struct Spec;
```

六个参数即 Rocksoft 模型。`Width` 取 8、16 或 32，`Poly` 和 `Init` 按非反射（MSB 在前）形式给出，与 CRC 目录的惯例一致。`Spec::Value` 由宽度推导为 `std::uint8_t`、`std::uint16_t` 或 `std::uint32_t`。参数超出宽度会在编译期断言失败。

预设见[算法预设与校验值](./PRESETS.md)。需要新算法时直接写 `Spec`，不必修改库：

```cpp
using Crc16Modbus = crc::Spec<16, 0x8005, 0xFFFF, true, true, 0x0000>;
```

## 计算

```cpp
template <typename S> constexpr typename S::Value Compute(std::span<const std::byte>) noexcept;
template <typename S> constexpr typename S::Value Compute(std::span<const std::uint8_t>) noexcept;
```

两个重载对同一段数据结果相同。契约：不分配内存、不抛异常、可常量求值、不阻塞、可在中断中调用。

一次调用处理一段连续字节。库不提供 Init/Update/Final 式的增量接口，因为增量计算跨调用持有硬件会阻塞其他调用方，而现有使用场景都是对整段缓冲一次性计算。需要拼接多段时，先把数据放进连续缓冲。

编译期求值：

```cpp
static_assert(crc::Compute<crc::Crc32IsoHdlc>(std::span(kFixedBytes)) == 0xCBF43926U);
```

常量求值一定走软件表，因此 `constexpr` 函数调用 `Compute` 后仍然可以常量求值。

直接调用软件实现（绕过后端分发，测试或对拍时有用）：

```cpp
const auto value = crc::Crc32IsoHdlc::Software(bytes);
```

## 硬件后端

```cpp
struct Params {
  std::uint8_t width; std::uint32_t poly; std::uint32_t init;
  bool refin; bool refout; std::uint32_t xorout;
};

struct Backend {
  void* context;
  bool (*supports)(void* context, const Params&) noexcept;
  std::uint32_t (*compute)(void* context, const Params&, std::span<const std::byte>) noexcept;
};

void InstallBackend(const Backend* backend) noexcept;
const Backend* InstalledBackend() noexcept;
```

后端是函数指针结构而非虚基类，因此可以放在只读段，不引入虚表和 RTTI。

`supports` 按完整算法参数判断，而不是按一个封闭的预设枚举，所以新增 `Spec` 不需要改动任何后端。`compute` 必须返回已应用 `xorout` 的最终值；它只在库已独占后端时被调用，自身不需要加锁，同样不得阻塞、分配或抛异常。

### 注册约定

`InstallBackend` 最多注册一个后端，只应在启动阶段、任何使用 CRC 的并发执行开始之前调用一次。传 `nullptr` 恢复纯软件路径，测试清理时可用。

指针以 release 写入、acquire 读取，所以即使违反约定在运行期替换后端，也只会影响某次调用走硬件还是软件，不会影响结果。后端对象的生命周期必须覆盖整个注册期间，通常是静态存储。

### 分发顺序

```
常量求值上下文                -> 软件表
未注册后端                    -> 软件表
supports() 返回 false         -> 软件表
互斥标志抢占失败               -> 软件表     (永不等待)
否则                          -> 后端计算
```

互斥用 `std::atomic_flag::test_and_set`，抢不到立即回退，不排队、不等待。这带来三个性质：

- **不会阻塞**，因此没有优先级反转。
- **中断中可安全调用**。中断打断正在使用硬件的线程时，中断内抢占失败并走软件，被打断线程的硬件状态不受破坏。
- **结果与调用路径无关**。硬件和软件结果必须逐位一致，这是后端的实现义务，也是可测试的性质。

代价是硬件后端存在时并非每次调用都走硬件。这只影响性能，不影响正确性。

## 线程与中断安全

`Compute` 本身无共享可变状态，任意上下文可并发调用。唯一的共享状态是后端指针和互斥标志，两者都是无锁原子量，在 Cortex-M7 上由 LDREX/STREX 实现。

需要注意的是：后端自己的 `compute` 实现通常操作独占硬件，库保证同一时刻只有一个调用进入它。如果后端还被库以外的代码使用（例如直接调用 `crc::stm32::RunSelfTest()`），那部分互斥由调用方负责。
