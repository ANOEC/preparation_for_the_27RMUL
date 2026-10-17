# RMLink 实时协议核心

RMLink 是运行在有序字节流之上的 C++20 双端协议核心。它负责固定分帧、会话、四时间戳同步、双向实时 DATA、Device 到 Host 的 LOG、关闭和异常清理。核心不绑定 CDC、LibUSB、串口或 RTOS，平台适配由消费者工程提供。帧校验依赖同样不绑定 LibXR 的纯头文件 CRC 库 `../../utility/crc`。CMake 目标为 `LibHT::rmlink`（静态库），由 LibHT 伞形目录统一定义，也可以单独 `add_subdirectory` 本目录使用。

## 引入方式

```cmake
add_subdirectory(path/to/LibHT)   # 或单独 add_subdirectory 本目录
target_link_libraries(<consumer> PUBLIC LibHT::rmlink)
```

协议核心不包含平台适配器，消费者通过 `TransportOps` 接入自己的传输和时间实现。

## 集成边界

- 调用方注入 64 位单调微秒时钟、非阻塞整帧发送和传输中止操作。
- 同一实例的 `Begin`、`Receive`、`Poll`、完成通知、提交接口及用户回调必须在同一执行上下文串行调用，不允许从回调重入 Core。
- ISR 或其他线程只能通知外部执行器，由执行器串行调用 Core。
- `SendResult::Accepted` 后，后台可分段写出帧，但必须保持字节连续；在匹配完成标识或中止确认前不得继续引用其他 Core 发送缓冲。
- 收到 `ResetRequired` 后必须停止旧输入，取消旧传输，清空后台残留，再调用 `NotifyTransportAborted()`。此前不能开始新会话。
- 二进制 span、日志文本和类型化对象引用仅在回调期间有效，需要长期保存时由业务复制。

## 时序约束

DATA/LOG 的事件时间来自完整本地单调时钟，再减本次会话起点编码为 32 位微秒。事件不得早于会话起点或晚于提交时刻；会话持续超过一小时会进入清理，不实现时间戳回绕还原。Host 的映射状态可能为 `Unavailable`、`Valid` 或 `Stale`，过期映射仍可显式查询，但不能当作当前有效估计。

## 有界内存

构造 Core 时会复制流注册并分配固定队列、控制槽和同步样本槽。构造完成后的协议路径不扩容。`fixed_memory_bytes()` 返回 Core 对象及其固定容量元素的可核对字节数，不包含分配器元数据、后台库和用户回调内存。默认实时 FIFO 深度为 8，Device 日志 FIFO 深度为 4，控制容量至少为 2。

## 文档

- [线协议](./docs/PROTOCOL.md)
- [接口与后端接入](./docs/API.md)
- [P99 时间对齐实测](./docs/INTEGRATION_MEASUREMENT.md)
- [验证结果](./docs/TEST_RESULTS.md)
- [二进制和结构体示例](examples/basic.cpp)

## 主机验证

```sh
cmake -S lib/libht/protocol/rmlink/tests -B /tmp/rmlink-build -G Ninja
cmake --build /tmp/rmlink-build --parallel
ctest --test-dir /tmp/rmlink-build --output-on-failure
```

测试使用可控时钟和内存传输替身，不证明物理链路满足吞吐或 P99 精度目标。

## 共享载荷 codec

类型化入口直接调用 `::wire::Encode/Decode`，普通聚合自动固定编码，特殊或有界变长
格式由外部 `::wire::Codec<T>` 定义。`StructuredPayload<T>` 要求可默认构造、无异常
复制/赋值和 `::wire::MaxEncodedSize<T> <= 255`，容量在暂存实例化前检查。
Core 按最大长度预留空间，只将 Result.bytes 的实际前缀入队；TypedStream 只在
完整成功解码后回调，失败计入 decode_errors。0 字节固定类型可表达，超过 255
的类型（例如裁判 ClientCommand 300 字节）被拒绝；不提供分片或旧成员接口适配。

业务载荷命名空间为 `::wire`，帧协议仍为 `rmlink::wire`。独立消费者使用 C++20，
通过 `LibHT::serialization` 传播后端依赖，构建时选择 zpp/PFR。固定、Schema 和
有界外部 codec 示例见 `../../utility/serialization/README.md`，库示例在
`examples/basic.cpp` 中编译，变长接线由 `tests/shared_codec_tests.cpp` 验证。

当前实时 FIFO 满时挤掉最旧未发项并返回成功。消费者的 LibXR Topic Pump 每批只
保留最后一条；变长 codec 不改变这两层策略，不提供事件保序、重传或可靠转发。
