#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace debug::vofa_plus
{

static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
              "VOFA+ FireWater float API requires 32-bit IEEE-754 float");

class VofaPlus
{
 public:
  static constexpr std::size_t kFrameCapacity = 256;
  static constexpr std::size_t kMaxFloatCount = (kFrameCapacity - 1) / 2;

  enum class Result : std::uint8_t
  {
    OK,
    NO_FRAME,
    BUSY,
    INVALID_ARGUMENT,
    FRAME_TOO_LARGE,
    PARSE_ERROR,
    IO_ERROR,
  };

  using WriteCallback =
      bool (*)(void* context, const std::uint8_t* data, std::size_t size);
  using AvailableCallback = std::size_t (*)(void* context);
  using ReadCallback = bool (*)(void* context, std::uint8_t* data, std::size_t size);

  struct Transport
  {
    void* context = nullptr;
    WriteCallback write = nullptr;
    AvailableCallback available = nullptr;
    ReadCallback read = nullptr;
  };

  explicit VofaPlus(Transport transport) noexcept;

  VofaPlus(const VofaPlus&) = delete;
  VofaPlus& operator=(const VofaPlus&) = delete;

  [[nodiscard]] Result Send(const float* data, std::size_t count,
                            const char* prefix = "data") noexcept;
  [[nodiscard]] Result Receive(float* data, std::size_t count) noexcept;

  [[nodiscard]] constexpr bool CanSend() const noexcept
  {
    return transport_.write != nullptr;
  }

  [[nodiscard]] constexpr bool CanReceive() const noexcept
  {
    return transport_.available != nullptr && transport_.read != nullptr;
  }

 private:
  Transport transport_;
  std::uint8_t tx_buffer_[kFrameCapacity]{};
  std::uint8_t rx_buffer_[kFrameCapacity]{};
  std::size_t rx_size_ = 0;
  bool discard_rx_line_ = false;
  std::atomic<bool> send_in_use_{false};
  std::atomic<bool> receive_in_use_{false};
};

}  // namespace debug::vofa_plus
