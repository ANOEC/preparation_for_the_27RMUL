#pragma once

#include "rmlink/types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace rmlink::wire {

inline constexpr std::byte kMagic0{0x52};
inline constexpr std::byte kMagic1{0x4D};

struct Header {
  MessageType type{MessageType::Data};
  std::uint8_t session_id{};
  std::uint16_t stream_id{};
  std::uint8_t sequence{};
  std::uint8_t payload_length{};
  std::uint32_t timestamp_us{};
};

struct FrameView {
  Header header{};
  std::span<const std::byte> payload{};
  std::uint64_t receive_end_us{};
};

struct EncodeResult {
  Status status{Status::InvalidArgument};
  std::size_t bytes{};
};

struct ParseStatistics {
  std::uint64_t frames{};
  std::uint64_t header_crc_errors{};
  std::uint64_t frame_crc_errors{};
  std::uint64_t partial_frame_timeouts{};
  std::uint64_t semantic_errors{};
  std::uint64_t noise_bytes{};
};

using FrameCallback = void (*)(void *context, const FrameView &frame) noexcept;

[[nodiscard]] std::uint16_t HeaderCrc(
    std::span<const std::byte> bytes) noexcept;
[[nodiscard]] std::uint32_t FrameCrc(
    std::span<const std::byte> bytes) noexcept;

[[nodiscard]] EncodeResult Encode(
    Header header, std::span<const std::byte> payload,
    std::span<std::byte> output) noexcept;

[[nodiscard]] bool DecodeHeader(std::span<const std::byte> encoded,
                                Header &header) noexcept;
[[nodiscard]] bool IsSemanticallyValid(const FrameView &frame,
                                       Role receiver) noexcept;
[[nodiscard]] bool IsValidUtf8(std::span<const std::byte> text) noexcept;

class Parser {
public:
  Parser(Role receiver, std::uint64_t partial_timeout_us,
         FrameCallback callback, void *callback_context) noexcept;

  void Feed(std::span<const std::byte> bytes,
            std::uint64_t receive_end_us) noexcept;
  void Poll(std::uint64_t now_us) noexcept;
  void Reset() noexcept;

  [[nodiscard]] const ParseStatistics &statistics() const noexcept {
    return statistics_;
  }

private:
  void Push(std::byte value, std::uint64_t receive_time_us) noexcept;
  void ProcessCandidate() noexcept;
  void RescanAfterFailure() noexcept;
  void DropPrefix(std::size_t count) noexcept;
  [[nodiscard]] std::size_t ExpectedSize() const noexcept;

  Role receiver_;
  std::uint64_t partial_timeout_us_;
  FrameCallback callback_;
  void *callback_context_;
  std::array<std::byte, kMaxFrameSize> candidate_{};
  std::array<std::uint64_t, kMaxFrameSize> receive_times_{};
  std::size_t candidate_size_{};
  ParseStatistics statistics_{};
};

} // namespace rmlink::wire
