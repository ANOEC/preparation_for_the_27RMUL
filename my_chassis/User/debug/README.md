# RTT Topic Debug

本工具通过 RTT 命令查看话题的在线状态、更新间隔和最新数据。默认超过
`500 ms` 没有更新即判定为 `OFFLINE`。

## 管理监视话题

话题注册表只需要编辑 `topic_monitor.toml`。每个话题使用一段 `[[topic]]`：

```toml
[[topic]]
id = "torque"
name = "/yaw/torque_cmd"
type = "float"
printer = "PrintTorque"
```

- `id`：生成的 C++ 成员名，必须唯一，并且只能使用字母、数字和下划线，不能以数字开头。
- `name`：发布方使用的完整话题名，是否带 `/` 必须与发布方完全一致，且不能重复。
- `type`：话题的 C++ 数据类型，必须与发布方完全一致。
- `printer`：`TopicDebugApp` 中负责输出该类型数据的静态函数。

增加话题时复制一个同类型条目，修改唯一的 `id` 和实际 `name` 即可。删除话题时删除
对应的整段 `[[topic]]`。构建时 CMake 会自动运行 `generate_topic_monitor.py`，生成
`generated/topic_monitor.hpp`；不要手工编辑生成文件。

当前可以直接复用的类型与打印函数如下：

| type | printer | 用途 |
| --- | --- | --- |
| `RemoteData` | `PrintRemote` | DT7 遥控器数据 |
| `ChassisCommand` | `PrintChassisCommand` | 底盘命令 |
| `MotorFeedback` | `PrintMotorFeedback` | DJI 电机反馈 |
| `ImuVector` | `PrintImuVector` | 三轴 IMU 向量 |
| `float` | `PrintTorque` | 浮点转矩值 |

如果要监视新的数据类型，还需在 `topic_debug_app.hpp` 中包含其定义、声明对应的打印函数，
并在 `topic_debug_app.cpp` 中实现打印函数；之后同类型话题的增删仍然只改 TOML。

也可以不经过 CMake，手工检查生成结果：

```powershell
python User/debug/generate_topic_monitor.py
```

生成器会检查缺失字段、未知字段、非法 C++ 标识符、重复 `id` 和重复话题名。文件内容
没有变化时不会改写生成文件，避免触发无意义的重新编译。

## 编译

```powershell
cmake --build --preset Debug --parallel 4
```

只要 `topic_monitor.toml` 或生成脚本发生变化，生成步骤会先于固件编译执行。配置错误会
直接终止构建，并显示具体条目和字段。

## RTT 使用方法

连接 RTT 后输入 `topic help`。可用命令：

```text
topic list                 列出所有注册话题及在线状态
topic once <name>          打印一次状态和最新值
topic echo <name>          周期打印状态和最新值
topic stop                 停止周期打印
topic timeout <ms>         修改离线判定时间
topic period <ms>          修改周期打印间隔
```

例如：

```text
topic list
topic once /remote
topic echo bmi088_gyro
topic period 100
topic stop
```

新话题在收到第一帧数据前显示 `OFFLINE, never updated`，收到数据后才显示 `ONLINE`
和实际值。

使用 DAPLink进行烧录

```powershell
openocd `
  -f interface/cmsis-dap.cfg `
  -c "transport select swd" `
  -f target/stm32h7x.cfg `
  -c "adapter speed 1000" `
  -c "program build/Debug/my_chassis.elf verify reset exit"
```

使用 DAPLink 与 OpenOCD 时，可先启动 RTT 服务：

```powershell
openocd `
  -f interface/cmsis-dap.cfg `
  -c "transport select swd" `
  -f target/stm32h7x.cfg `
  -c "adapter speed 1000" `
  -c "init" `
  -c "reset run" `
  -c "rtt setup 0x20000000 0x20000 {SEGGER RTT}" `
  -c "rtt start" `
  -c "rtt server start 9090 0"
```

连接 RTT Topic Debug，保持刚才的 OpenOCD 窗口运行，再打开第二个 PowerShell：

```powershell
python -c "import telnetlib; t=telnetlib.Telnet('127.0.0.1',9090); t.interact()"
```

连接成功后按一次回车，然后输入：

```powershell
topic help
```
