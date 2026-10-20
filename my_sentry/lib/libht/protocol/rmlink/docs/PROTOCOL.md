# RMLink v1 线协议

所有多字节整数按小端逐字节编码，不使用 packed struct 或主机对象表示。

| 偏移 | 字段 | 长度 | 说明 |
|---:|---|---:|---|
| 0 | Magic | 2 | `52 4D` |
| 2 | Type | 1 | 消息类型 |
| 3 | SessionId | 1 | 1-255，0 保留 |
| 4 | StreamId | 2 | DATA 为静态流 ID，其他类型为 0 |
| 6 | Sequence | 1 | 模 256 序号或控制关联号 |
| 7 | PayloadLen | 1 | 0-255 |
| 8 | Timestamp | 4 | 会话相对微秒 |
| 12 | HeaderCRC | 2 | CRC-16/IBM-3740，覆盖 0-11 |
| 14 | Payload | N | 类型载荷 |
| 14+N | FrameCRC | 4 | CRC-32/ISO-HDLC，覆盖头和载荷 |

固定开销为 18 字节，最大帧为 273 字节。HeaderCRC 参数为 poly `0x1021`、init `0xFFFF`、非反射、xorout 0；FrameCRC 参数为 poly `0x04C11DB7`、init/xorout `0xFFFFFFFF`、输入输出反射。两种 CRC 均小端存储。实现由共享库 `Middlewares/Third_Party/LibHT/utility/crc` 提供，分别对应 `crc::Crc16Ibm3740` 与 `crc::Crc32IsoHdlc`；固件注册硬件 CRC 后端时结果不变。

| Type | 名称 | 方向 | 载荷 |
|---:|---|---|---|
| 0x01 | HELLO | Host 到 Device | version:u16, token:u64, host_epoch:u64 |
| 0x02 | HELLO_ACK | Device 到 Host | version:u16, token:u64, device_epoch:u64 |
| 0x03 | SYNC_REQ | Host 到 Device | t1:u64 |
| 0x04 | SYNC_RESP | Device 到 Host | t1:u64, t2:u64, t3:u64 |
| 0x05 | START | Host 到 Device | 空 |
| 0x06 | START_ACK | Device 到 Host | 空 |
| 0x07 | CLOSE | 双向 | reason:u8 |
| 0x08 | CLOSE_ACK | 双向 | 空 |
| 0x09 | REJECT | 应答 | request_type:u8, reason:u8, version:u16, token:u64 |
| 0x10 | DATA | 双向 | 0-255 字节业务编码 |
| 0x11 | LOG | Device 到 Host | level:u8, flags:u8, UTF-8 文本 |

增量解析器不依赖驱动读边界。头 CRC 或整帧 CRC 失败后从候选起点下一字节重新扫描；默认 100 ms 的候选总期限不能被零散输入续期。完整帧的接收时间是末字节所属输入批次的时间。

DATA 不提供业务 ACK、重传、分片或去重。只有 Active 会话中当前 SessionId 的合法 DATA/LOG 刷新接收心跳；未知 StreamId 仍证明链路存活，但不会触发业务回调。
