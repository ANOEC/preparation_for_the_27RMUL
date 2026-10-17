# 算法预设与校验值

## 参数

| 预设 | 宽度 | Poly | Init | RefIn | RefOut | XorOut |
|---|---:|---|---|:-:|:-:|---|
| `Crc8MaximReferee` | 8 | `0x31` | `0xFF` | 是 | 是 | `0x00` |
| `Crc16Mcrf4xx` | 16 | `0x1021` | `0xFFFF` | 是 | 是 | `0x0000` |
| `Crc16Ibm3740` | 16 | `0x1021` | `0xFFFF` | 否 | 否 | `0x0000` |
| `Crc32IsoHdlc` | 32 | `0x04C11DB7` | `0xFFFFFFFF` | 是 | 是 | `0xFFFFFFFF` |

`Poly` 和 `Init` 均为非反射形式。反射算法的查表由库内部自行取反射多项式，调用方不需要预先反转。

## 校验值

`check` 是 ASCII 输入 `123456789` 的结果，是 CRC 目录的标准比对方式；`empty` 是零长度输入的结果。测试以 `static_assert` 在编译期断言这些值。

| 预设 | check | empty | 查表占用 |
|---|---|---|---:|
| `Crc8MaximReferee` | `0x0B` | `0xFF` | 256 B |
| `Crc16Mcrf4xx` | `0x6F91` | `0xFFFF` | 512 B |
| `Crc16Ibm3740` | `0x29B1` | `0xFFFF` | 512 B |
| `Crc32IsoHdlc` | `0xCBF43926` | `0x00000000` | 1 KB |

查表为 `inline constexpr`，按 `<宽度, 多项式, 输入反射>` 去重，进入只读段。四个预设全部实例化时共占 2.3 KB flash。

## 别名与使用方

| 预设 | 常见别名 | 本仓库使用方 |
|---|---|---|
| `Crc8MaximReferee` | 无标准名；见下 | 裁判系统帧头 CRC8 |
| `Crc16Mcrf4xx` | CRC-16/MCRF4XX | 裁判系统整帧 CRC16 |
| `Crc16Ibm3740` | CRC-16/CCITT-FALSE、CRC-16/AUTOSAR | RMLink 帧头、图传遥控帧 |
| `Crc32IsoHdlc` | CRC-32、CRC-32/ADCCP、zlib crc32 | RMLink 整帧 |

## 关于 `Crc8MaximReferee`

这个名字不是 CRC 目录里的标准条目，因为它的参数组合没有标准名。它采用 CRC-8/MAXIM 的多项式（`0x31` 反射后为 `0x8C`），但初值是 `0xFF` 而不是标准 CRC-8/MAXIM-DOW 的 `0x00`。两者对 `123456789` 的结果分别是 `0x0B` 和 `0xA1`，不能混用。

该变体来自 RoboMaster 裁判系统协议附录一的 `CRC8_TAB` 与 `CRC8_INIT`。`Modules/RefereeSystem/tests/referee_appendix_reference.hpp` 逐字复刻了附录一的表和函数，作为独立于本库实现的参考，避免"用自己的实现验证自己"。

## 新增预设

直接写 `Spec`，不需要修改库：

```cpp
using Crc16Modbus = crc::Spec<16, 0x8005, 0xFFFF, true, true, 0x0000>;
using Crc32C = crc::Spec<32, 0x1EDC6F41, 0xFFFFFFFF, true, true, 0xFFFFFFFF>;
```

新增后建议同时补一条标准校验值断言。硬件后端按 `Params` 判断支持情况，因此不需要为新预设改动后端；不被支持时自动走软件。

任意宽度和反射组合都应与逐位参考实现一致，测试里包含两组非预设参数（含 `RefIn != RefOut` 的混合反射）以覆盖这条通路。
