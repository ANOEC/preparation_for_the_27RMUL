# SYS_ID 串口同步保存与 VOFA 转发

`capture_sys_id.py` 独占 STM32 串口，将同一份数据：

1. 原样写入 `logs/serial_raw_*.log`；
2. 将 `gimbal_sys_id` 行解析为 `logs/gimbal_sys_id_*.csv`；
3. 原样转发到本机 TCP，供 VOFA+ 实时显示。

这样不会向 STM32 内部 Flash 高频写数据。CSV 每收到一行就刷新，即使中途按
`Ctrl+C`，已经接收的数据也保留。

## 使用

先关闭 VOFA 的串口连接，查询串口号：

```powershell
python tools/capture_sys_id.py --list-ports
```

假设设备是 `COM7`：

```powershell
python tools/capture_sys_id.py --port COM7 --baud 115200
```

然后在 VOFA+ 中：

- 接口选择 TCP Client；
- 地址填写 `127.0.0.1`；
- 端口填写 `1347`；
- 协议引擎保持原先解析 `gimbal_sys_id:` 所用的设置；
- 点击连接。

串口不能再由 VOFA 直接打开，否则采集器会报告端口被占用。VOFA 发出的命令会由
采集器反向转发到 STM32。

停止时在采集器终端按 `Ctrl+C`。默认文件保存在项目根目录的 `logs/`，该目录已被
Git 忽略。可以通过 `--output-dir` 和 `--listen-port` 修改保存目录及 TCP 端口。
