# VOFA+ FireWater 模块

本模块通过 USART10 向 VOFA+ 发送和接收 FireWater CSV 文本帧。核心实现不依赖
具体 BSP，云台工程通过 LibXR UART 适配器接入。

## 当前工程配置

- 端口：USART10，VOFA+ 独占；
- 串口参数：115200、8 数据位、无校验、1 停止位、无流控；
- 引脚：PE3 为 TX，PE2 为 RX；
- LibXR Terminal 已通过 `terminal_source: ''` 关闭；
- UART5 仍由遥控器使用；
- USART10 收发缓冲区均为 128 字节。

接线时，USB 转串口模块 RX 接 PE3，TX 接 PE2，并连接 GND。必须使用 3.3 V TTL
电平。

## FireWater 帧格式

发送接口默认生成如下文本：

```text
data:1.25,-2.5,42\n
```

格式为 `前缀:CSV数据\n`。换行符不可省略；前缀可以通过参数修改，也可以传入空
字符串来发送纯 CSV。`image` 是 FireWater 图片协议的保留前缀，模块不会将它用作
普通采样数据前缀。

浮点数使用 6 位有效数字输出。核心格式化缓冲区为 256 字节，但当前 USART10 的
单次发送容量为 128 字节，因此最终文本帧必须不超过 128 字节。帧太长时会返回
`IO_ERROR`，可以减少通道数、缩短前缀或增大 USART10 缓冲区。

## 初始化与访问

`app_main.cpp` 已在任务启动前完成初始化：

```cpp
UserApp::Debug::InitializeVofaPlus(usart10);
```

普通任务通过以下接口获取实例：

```cpp
#include "app/debug/vofa_plus_libxr.hpp"

auto* vofa = UserApp::Debug::GetVofaPlus();
```

实例在 `all_tasks_start()` 前初始化，因此正常启动后指针不为空。不要在中断上下文
调用 `Send()` 或 `Receive()`。

## 发送波形

```cpp
auto* vofa = UserApp::Debug::GetVofaPlus();
if (vofa != nullptr)
{
  const float channels[] = {target_angle, actual_angle, motor_output};
  const auto result = vofa->Send(channels, 3);  // data:...,...,...\n
}
```

自定义前缀：

```cpp
vofa->Send(channels, 3, "gimbal");  // gimbal:...,...,...\n
vofa->Send(channels, 3, "");        // ...,...,...\n
```

## 接收参数

VOFA+ 可以发送以下任意一种形式：

```text
set:10.5,0.25\n
10.5,0.25\n
```

固件任务中非阻塞读取：

```cpp
auto* vofa = UserApp::Debug::GetVofaPlus();
float params[2];
if (vofa != nullptr &&
    vofa->Receive(params, 2) == debug::vofa_plus::VofaPlus::Result::OK)
{
  kp = params[0];
  kd = params[1];
}
```

`Receive()` 遇到换行才解析一帧。数据不足时返回 `NO_FRAME`；字段不是合法浮点数、
字段数不一致或存在多余内容时返回 `PARSE_ERROR`。只有返回 `OK` 时才应使用结果。

## VOFA+ 上位机设置

1. 选择 USB 转串口模块对应的 COM 口。
2. 设置 115200、8N1、无流控。
3. 协议引擎选择 `FireWater`。
4. 连接串口，在数据区确认出现 `data` 或自定义前缀对应的通道。
5. 波形图绑定对应通道。

## 状态码

| 状态 | 含义 |
| --- | --- |
| `OK` | 文本帧已提交发送，或成功解析一帧 |
| `NO_FRAME` | 尚未收到完整的换行结尾帧 |
| `BUSY` | 同一方向正在被另一个任务调用 |
| `INVALID_ARGUMENT` | 参数无效，或使用了保留前缀 `image` |
| `FRAME_TOO_LARGE` | FireWater 文本超过模块的 256 字节格式化缓冲区 |
| `PARSE_ERROR` | 接收帧不是约定数量的合法 CSV 浮点数据 |
| `IO_ERROR` | UART 不可用、队列容量不足或 LibXR 读写失败 |

## 使用限制

- FireWater 文本格式比 JustFloat 占用更多带宽和格式化时间，建议降低发送频率。
- 发送周期应根据通道数和 115200 波特率确定，通常可先从 10～100 Hz 开始。
- 同一个 USART10 接收端口不能再由终端或其他任务读取。
- 若只需要绘图，可以仅调用 `Send()`。

## 云台速度环正弦测试

工程已接入速度环测试模式，配置位于
`src/config/gimbal/params/gimbal.toml` 的 `[velocity_test]`：

```toml
[velocity_test]
enabled = true
axis = 0
frequency_hz = 0.5
amplitude_rad_per_sec = 1.0
offset_rad_per_sec = 0.0
telemetry_period_ms = 10
```

`axis` 对应关系：

| axis | 被测电机 |
| --- | --- |
| `0` | 小 yaw，GM6020 |
| `1` | 大 yaw，DM J4310 |
| `2` | pitch，DM J4310 |

默认测试小 yaw。右侧 DT7 开关拨到中间或上方后开始输出正弦速度目标；拨到下方、
遥控器离线或电机/IMU 反馈超时都会立即输出零力矩。测试模式下只有被测电机输出
力矩，另外两个电机输出零。

VOFA+ 每 10 ms 收到一帧：

```text
speed_loop:目标角速度,实际角速度,指令力矩,反馈力矩\n
```

四个通道的单位依次是 `rad/s`、`rad/s`、`N·m`、`N·m`。实际角速度使用与正常
串级控制相同的反馈：小 yaw 和 pitch 来自 IMU，大 yaw 由 IMU 与小 yaw 编码器
速度组合得到；“反馈力矩”是根据电机反馈电流和力矩常数换算的估算值，并非独立
力矩传感器的直接测量值。

修改正弦幅值、频率或测试轴后重新构建固件即可。完成速度环调参后应将
`enabled` 改回 `false`，恢复原来的位置—速度串级控制。
