# RMLink 接口与后端接入

公开入口为 `include/rmlink/rmlink.hpp`。

## 后端注入

`TransportOps` 包含上下文指针、`now_us`、`send` 和 `abort`。`send` 必须满足：

- `Busy`：未消费、未持有任何字节；Core 可稍后重试。
- `Accepted`：后台持有整条稳定帧，完成后调用 `NotifySendComplete(token, Complete)`。
- `Failed`：传输不可继续；Core 进入 `ResetRequired`，后台中止后调用 `NotifyTransportAborted()`。

发送完成只表示后台释放缓冲，不表示对端已接收。同步后台也必须在 `send` 返回后再提交完成通知，避免重入。

## 驱动顺序

Host 调用 `Begin()`，双方持续把串行字节与该批末字节时间送入 `Receive()`，并周期调用 `Poll()`。Device 不调用 `Begin()`，收到 HELLO 后被动建立。`Close()` 停止新业务并清空尚未交给后台的业务/日志；已 Accepted 的帧完成后才发送 CLOSE。

## 业务接口

`SubmitData(stream_id, payload, event_us)` 复制 0-255 字节载荷。返回 `Ok` 只表示固定 FIFO 已接纳，后续仍可能因拥塞或断线丢弃。流必须在 `Config::streams` 中静态注册；满实时 FIFO 丢最旧未发送项，序号已消耗。

类型化入口要求共享 `::wire::Supported<T>`、可默认构造、无异常复制/赋值及
`::wire::MaxEncodedSize<T> <= 255`。普通类型自动固定编码，特殊格式使用外部 codec，
有界变长输入由共享 min/max 检查。Core 使用 checked Encode 后的实际长度入队，
TypedStream 使用 checked Decode，错误计数增加且不回调。旧成员协议、传输 selector
及 payload 专属结果类型已删除，所有消费者迁移到共享 Endian/Result/元数据。
codec 和回调必须不分配、不阻塞、不抛异常；编码最大暂存有界，不等于零栈开销。
FIFO 挤旧及 Topic 批次合并策略保持，不能据此承诺交互事件可靠转发。

Device 可调用 `SubmitLog(level, utf8)`。合法超长文本在 UTF-8 字符边界截断，flags bit0 标记截断；Host 调用会返回 `WrongRole`。调度顺序为控制、满足限速的 DATA、LOG。日志令牌预算按完整线上字节计算。

## 查询

- `statistics()`：会话、队列、重试、同步、心跳和传输累计统计，重连保留。
- `parse_statistics()`：头 CRC、整帧 CRC、半帧超时和语义错误。
- `time_mapping()`：双方起点、Device 减 Host 偏移、RTT、年龄、代次和状态。
- `MapDeviceEvent()`：按原始相对时间映射 Device/Host 事件时刻，不使用接收时刻替代采样时刻。
- `fixed_memory_bytes()`：初始化后固定容量元素的可核对内存量。
