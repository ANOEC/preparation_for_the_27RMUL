#include "vofa_plus.hpp"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace debug::vofa_plus
{
namespace
{

class AtomicUseGuard
{
 public:
  explicit AtomicUseGuard(std::atomic<bool>& flag) noexcept
      : flag_(flag), acquired_(!flag_.exchange(true, std::memory_order_acquire))
  {
  }

  ~AtomicUseGuard()
  {
    if (acquired_)
    {
      flag_.store(false, std::memory_order_release);
    }
  }

  [[nodiscard]] bool Acquired() const noexcept { return acquired_; }

 private:
  std::atomic<bool>& flag_;
  bool acquired_;
};

}  // namespace

VofaPlus::VofaPlus(Transport transport) noexcept : transport_(transport) {}

VofaPlus::Result VofaPlus::Send(const float* data, std::size_t count,
                               const char* prefix) noexcept
{
  if (!CanSend())
  {
    return Result::IO_ERROR;
  }
  if ((data == nullptr && count != 0) || prefix == nullptr ||
      std::strcmp(prefix, "image") == 0)
  {
    return Result::INVALID_ARGUMENT;
  }
  if (count > kMaxFloatCount)
  {
    return Result::FRAME_TOO_LARGE;
  }

  AtomicUseGuard guard(send_in_use_);
  if (!guard.Acquired())
  {
    return Result::BUSY;
  }

  std::size_t frame_size = 0;
  if (prefix[0] != '\0')
  {
    const int prefix_size =
        std::snprintf(reinterpret_cast<char*>(tx_buffer_), kFrameCapacity, "%s:", prefix);
    if (prefix_size < 0 || static_cast<std::size_t>(prefix_size) >= kFrameCapacity)
    {
      return Result::FRAME_TOO_LARGE;
    }
    frame_size = static_cast<std::size_t>(prefix_size);
  }

  for (std::size_t i = 0; i < count; ++i)
  {
    const char* format = (i == 0) ? "%.6g" : ",%.6g";
    const std::size_t remaining = kFrameCapacity - frame_size;
    const int value_size = std::snprintf(
        reinterpret_cast<char*>(tx_buffer_ + frame_size), remaining, format,
        static_cast<double>(data[i]));
    if (value_size < 0 || static_cast<std::size_t>(value_size) >= remaining)
    {
      return Result::FRAME_TOO_LARGE;
    }
    frame_size += static_cast<std::size_t>(value_size);
  }

  if (frame_size + 1 > kFrameCapacity)
  {
    return Result::FRAME_TOO_LARGE;
  }
  tx_buffer_[frame_size++] = '\n';

  return transport_.write(transport_.context, tx_buffer_, frame_size)
             ? Result::OK
             : Result::IO_ERROR;
}

VofaPlus::Result VofaPlus::Receive(float* data, std::size_t count) noexcept
{
  if (!CanReceive())
  {
    return Result::IO_ERROR;
  }
  if (data == nullptr && count != 0)
  {
    return Result::INVALID_ARGUMENT;
  }
  if (count > kMaxFloatCount)
  {
    return Result::FRAME_TOO_LARGE;
  }

  AtomicUseGuard guard(receive_in_use_);
  if (!guard.Acquired())
  {
    return Result::BUSY;
  }

  while (true)
  {
    if (transport_.available(transport_.context) == 0)
    {
      return Result::NO_FRAME;
    }

    std::uint8_t byte = 0;
    if (!transport_.read(transport_.context, &byte, 1))
    {
      return Result::IO_ERROR;
    }

    if (discard_rx_line_)
    {
      if (byte == '\n')
      {
        discard_rx_line_ = false;
      }
      continue;
    }

    if (byte != '\n')
    {
      if (rx_size_ + 1 >= kFrameCapacity)
      {
        rx_size_ = 0;
        discard_rx_line_ = true;
        return Result::FRAME_TOO_LARGE;
      }
      rx_buffer_[rx_size_++] = byte;
      continue;
    }

    while (rx_size_ != 0 && rx_buffer_[rx_size_ - 1] == '\r')
    {
      --rx_size_;
    }
    rx_buffer_[rx_size_] = '\0';

    char* begin = reinterpret_cast<char*>(rx_buffer_);
    if (char* separator = std::strrchr(begin, ':'); separator != nullptr)
    {
      begin = separator + 1;
    }

    bool parsed = true;
    for (std::size_t i = 0; i < count; ++i)
    {
      while (std::isspace(static_cast<unsigned char>(*begin)) != 0)
      {
        ++begin;
      }
      char* end = nullptr;
      data[i] = std::strtof(begin, &end);
      if (end == begin)
      {
        parsed = false;
        break;
      }
      begin = end;
      while (std::isspace(static_cast<unsigned char>(*begin)) != 0)
      {
        ++begin;
      }
      if (i + 1 < count)
      {
        if (*begin != ',')
        {
          parsed = false;
          break;
        }
        ++begin;
      }
    }

    while (std::isspace(static_cast<unsigned char>(*begin)) != 0)
    {
      ++begin;
    }
    if (*begin != '\0')
    {
      parsed = false;
    }

    rx_size_ = 0;
    return parsed ? Result::OK : Result::PARSE_ERROR;
  }
}

}  // namespace debug::vofa_plus
