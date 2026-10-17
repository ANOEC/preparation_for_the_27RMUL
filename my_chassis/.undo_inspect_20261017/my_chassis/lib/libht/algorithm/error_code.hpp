#pragma once

#include <cstdint>

namespace Algorithm
{
/**
 * @brief Algorithm library error codes.
 *
 * Platform-independent error enumeration for algorithm initialization and
 * parameter validation.
 */
enum class ErrorCode : int8_t
{
  PENDING = 1,        ///< 等待中 | Pending
  OK = 0,             ///< 操作成功 | Operation successful
  FAILED = -1,        ///< 操作失败 | Operation failed
  INIT_ERR = -2,      ///< 初始化错误 | Initialization error
  ARG_ERR = -3,       ///< 参数错误 | Argument error
  STATE_ERR = -4,     ///< 状态错误 | State error
  SIZE_ERR = -5,      ///< 尺寸错误 | Size error
  CHECK_ERR = -6,     ///< 校验错误 | Check error
  NOT_SUPPORT = -7,   ///< 不支持 | Not supported
  NOT_FOUND = -8,     ///< 未找到 | Not found
  NO_RESPONSE = -9,   ///< 无响应 | No response
  NO_MEM = -10,       ///< 内存不足 | Insufficient memory
  NO_BUFF = -11,      ///< 缓冲区不足 | Insufficient buffer
  TIMEOUT = -12,      ///< 超时 | Timeout
  EMPTY = -13,        ///< 为空 | Empty
  FULL = -14,         ///< 已满 | Full
  BUSY = -15,         ///< 忙碌 | Busy
  PTR_NULL = -16,     ///< 空指针 | Null pointer
  OUT_OF_RANGE = -17  ///< 超出范围 | Out of range
};

}  // namespace Algorithm
