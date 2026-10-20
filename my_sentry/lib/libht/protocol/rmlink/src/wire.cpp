#include "rmlink/wire.hpp"

#include <algorithm>

#include "crc/crc.hpp"

namespace rmlink::wire {
namespace {

std::uint16_t ReadU16(const std::byte *input) noexcept {
  return static_cast<std::uint16_t>(std::to_integer<std::uint8_t>(input[0])) |
         static_cast<std::uint16_t>(
             std::to_integer<std::uint8_t>(input[1]) << 8U);
}

std::uint32_t ReadU32(const std::byte *input) noexcept {
  std::uint32_t value = 0;
  for (std::size_t index = 0; index < 4; ++index) {
    value |= static_cast<std::uint32_t>(
                 std::to_integer<std::uint8_t>(input[index]))
             << (index * 8U);
  }
  return value;
}

void WriteU16(std::byte *output, std::uint16_t value) noexcept {
  output[0] = static_cast<std::byte>(value & 0xFFU);
  output[1] = static_cast<std::byte>((value >> 8U) & 0xFFU);
}

void WriteU32(std::byte *output, std::uint32_t value) noexcept {
  for (std::size_t index = 0; index < 4; ++index) {
    output[index] = static_cast<std::byte>((value >> (index * 8U)) & 0xFFU);
  }
}

bool IsKnownType(MessageType type) noexcept {
  switch (type) {
  case MessageType::Hello:
  case MessageType::HelloAck:
  case MessageType::SyncReq:
  case MessageType::SyncResp:
  case MessageType::Start:
  case MessageType::StartAck:
  case MessageType::Close:
  case MessageType::CloseAck:
  case MessageType::Reject:
  case MessageType::Data:
  case MessageType::Log:
    return true;
  }
  return false;
}

bool HasExactPayload(const FrameView &frame, std::size_t size) noexcept {
  return frame.payload.size() == size;
}

} // namespace

std::uint16_t HeaderCrc(std::span<const std::byte> bytes) noexcept {
  return crc::Compute<crc::Crc16Ibm3740>(bytes);
}

std::uint32_t FrameCrc(std::span<const std::byte> bytes) noexcept {
  return crc::Compute<crc::Crc32IsoHdlc>(bytes);
}

EncodeResult Encode(Header header, std::span<const std::byte> payload,
                    std::span<std::byte> output) noexcept {
  if (payload.size() > kMaxPayloadSize) {
    return {Status::PayloadTooLarge, 0};
  }
  const auto encoded_size = kFrameOverhead + payload.size();
  if (output.size() < encoded_size || header.session_id == 0) {
    return {Status::InvalidArgument, 0};
  }

  header.payload_length = static_cast<std::uint8_t>(payload.size());
  output[0] = kMagic0;
  output[1] = kMagic1;
  output[2] = static_cast<std::byte>(header.type);
  output[3] = static_cast<std::byte>(header.session_id);
  WriteU16(output.data() + 4, header.stream_id);
  output[6] = static_cast<std::byte>(header.sequence);
  output[7] = static_cast<std::byte>(header.payload_length);
  WriteU32(output.data() + 8, header.timestamp_us);
  WriteU16(output.data() + 12, HeaderCrc(output.first(12)));
  std::copy(payload.begin(), payload.end(), output.begin() + kHeaderSize);
  WriteU32(output.data() + kHeaderSize + payload.size(),
           FrameCrc(output.first(kHeaderSize + payload.size())));
  return {Status::Ok, encoded_size};
}

bool DecodeHeader(std::span<const std::byte> encoded, Header &header) noexcept {
  if (encoded.size() < kHeaderSize || encoded[0] != kMagic0 ||
      encoded[1] != kMagic1 || ReadU16(encoded.data() + 12) !=
                                    HeaderCrc(encoded.first(12))) {
    return false;
  }
  header.type = static_cast<MessageType>(
      std::to_integer<std::uint8_t>(encoded[2]));
  header.session_id = std::to_integer<std::uint8_t>(encoded[3]);
  header.stream_id = ReadU16(encoded.data() + 4);
  header.sequence = std::to_integer<std::uint8_t>(encoded[6]);
  header.payload_length = std::to_integer<std::uint8_t>(encoded[7]);
  header.timestamp_us = ReadU32(encoded.data() + 8);
  return true;
}

bool IsValidUtf8(std::span<const std::byte> text) noexcept {
  std::size_t index = 0;
  while (index < text.size()) {
    const auto first = std::to_integer<std::uint8_t>(text[index]);
    if (first <= 0x7FU) {
      ++index;
      continue;
    }
    std::size_t continuation_count = 0;
    std::uint32_t codepoint = 0;
    if (first >= 0xC2U && first <= 0xDFU) {
      continuation_count = 1;
      codepoint = first & 0x1FU;
    } else if (first >= 0xE0U && first <= 0xEFU) {
      continuation_count = 2;
      codepoint = first & 0x0FU;
    } else if (first >= 0xF0U && first <= 0xF4U) {
      continuation_count = 3;
      codepoint = first & 0x07U;
    } else {
      return false;
    }
    if (index + continuation_count >= text.size()) {
      return false;
    }
    for (std::size_t offset = 1; offset <= continuation_count; ++offset) {
      const auto continuation =
          std::to_integer<std::uint8_t>(text[index + offset]);
      if ((continuation & 0xC0U) != 0x80U) {
        return false;
      }
      codepoint = (codepoint << 6U) | (continuation & 0x3FU);
    }
    if ((continuation_count == 2 && codepoint < 0x800U) ||
        (continuation_count == 3 && codepoint < 0x10000U) ||
        (codepoint >= 0xD800U && codepoint <= 0xDFFFU) ||
        codepoint > 0x10FFFFU) {
      return false;
    }
    index += continuation_count + 1;
  }
  return true;
}

bool IsSemanticallyValid(const FrameView &frame, Role receiver) noexcept {
  const auto &header = frame.header;
  if (!IsKnownType(header.type) || header.session_id == 0 ||
      header.payload_length != frame.payload.size()) {
    return false;
  }
  if (header.type != MessageType::Data && header.stream_id != 0) {
    return false;
  }

  switch (header.type) {
  case MessageType::Hello:
    return receiver == Role::Device && HasExactPayload(frame, 18);
  case MessageType::HelloAck:
    return receiver == Role::Host && HasExactPayload(frame, 18);
  case MessageType::SyncReq:
    return receiver == Role::Device && HasExactPayload(frame, 8);
  case MessageType::SyncResp:
    return receiver == Role::Host && HasExactPayload(frame, 24);
  case MessageType::Start:
    return receiver == Role::Device && frame.payload.empty();
  case MessageType::StartAck:
    return receiver == Role::Host && frame.payload.empty();
  case MessageType::Close:
    return HasExactPayload(frame, 1) &&
           std::to_integer<std::uint8_t>(frame.payload[0]) <= 2;
  case MessageType::CloseAck:
    return frame.payload.empty();
  case MessageType::Reject: {
    if (!HasExactPayload(frame, 12)) {
      return false;
    }
    const auto request_type =
        static_cast<MessageType>(std::to_integer<std::uint8_t>(frame.payload[0]));
    const auto reason = std::to_integer<std::uint8_t>(frame.payload[1]);
    return (request_type == MessageType::Hello ||
            request_type == MessageType::Start ||
            request_type == MessageType::Close) &&
           reason >= 1 && reason <= 4;
  }
  case MessageType::Data:
    return true;
  case MessageType::Log:
    if (receiver != Role::Host || frame.payload.size() < 2) {
      return false;
    }
    return std::to_integer<std::uint8_t>(frame.payload[0]) <= 5 &&
           (std::to_integer<std::uint8_t>(frame.payload[1]) & 0xFEU) == 0 &&
           IsValidUtf8(frame.payload.subspan(2));
  }
  return false;
}

Parser::Parser(Role receiver, std::uint64_t partial_timeout_us,
               FrameCallback callback, void *callback_context) noexcept
    : receiver_(receiver), partial_timeout_us_(partial_timeout_us),
      callback_(callback), callback_context_(callback_context) {}

void Parser::Feed(std::span<const std::byte> bytes,
                  std::uint64_t receive_end_us) noexcept {
  Poll(receive_end_us);
  for (const auto value : bytes) {
    Push(value, receive_end_us);
  }
}

void Parser::Poll(std::uint64_t now_us) noexcept {
  while (candidate_size_ != 0 &&
         (now_us < receive_times_[0] ||
          now_us - receive_times_[0] > partial_timeout_us_)) {
    ++statistics_.partial_frame_timeouts;
    RescanAfterFailure();
    ProcessCandidate();
    if (candidate_size_ == 1 &&
        (now_us < receive_times_[0] ||
         now_us - receive_times_[0] > partial_timeout_us_)) {
      candidate_size_ = 0;
    }
  }
}

void Parser::Reset() noexcept { candidate_size_ = 0; }

void Parser::Push(std::byte value, std::uint64_t receive_time_us) noexcept {
  if (candidate_size_ == 0) {
    if (value == kMagic0) {
      candidate_[0] = value;
      receive_times_[0] = receive_time_us;
      candidate_size_ = 1;
    } else {
      ++statistics_.noise_bytes;
    }
    return;
  }

  if (candidate_size_ == 1 && value != kMagic1) {
    if (value == kMagic0) {
      receive_times_[0] = receive_time_us;
    } else {
      candidate_size_ = 0;
      ++statistics_.noise_bytes;
    }
    return;
  }

  candidate_[candidate_size_] = value;
  receive_times_[candidate_size_] = receive_time_us;
  ++candidate_size_;
  ProcessCandidate();
}

std::size_t Parser::ExpectedSize() const noexcept {
  if (candidate_size_ < kHeaderSize) {
    return kHeaderSize;
  }
  return kFrameOverhead + std::to_integer<std::uint8_t>(candidate_[7]);
}

void Parser::ProcessCandidate() noexcept {
  while (candidate_size_ >= kHeaderSize) {
    Header header{};
    if (!DecodeHeader(
            std::span<const std::byte>(candidate_.data(), kHeaderSize),
            header)) {
      ++statistics_.header_crc_errors;
      RescanAfterFailure();
      continue;
    }
    const auto expected_size = ExpectedSize();
    if (candidate_size_ < expected_size) {
      return;
    }
    const auto encoded =
        std::span<const std::byte>(candidate_.data(), expected_size);
    const auto expected_crc = ReadU32(candidate_.data() + expected_size - 4);
    if (expected_crc != FrameCrc(encoded.first(expected_size - 4))) {
      ++statistics_.frame_crc_errors;
      RescanAfterFailure();
      continue;
    }
    FrameView frame{header,
                    encoded.subspan(kHeaderSize, header.payload_length),
                    receive_times_[expected_size - 1]};
    if (!IsSemanticallyValid(frame, receiver_)) {
      ++statistics_.semantic_errors;
    } else {
      ++statistics_.frames;
      if (callback_ != nullptr) {
        callback_(callback_context_, frame);
      }
    }
    DropPrefix(expected_size);
  }
}

void Parser::RescanAfterFailure() noexcept {
  std::size_t next = candidate_size_;
  for (std::size_t index = 1; index + 1 < candidate_size_; ++index) {
    if (candidate_[index] == kMagic0 && candidate_[index + 1] == kMagic1) {
      next = index;
      break;
    }
  }
  if (next == candidate_size_) {
    if (candidate_size_ != 0 && candidate_[candidate_size_ - 1] == kMagic0) {
      candidate_[0] = kMagic0;
      receive_times_[0] = receive_times_[candidate_size_ - 1];
      candidate_size_ = 1;
    } else {
      candidate_size_ = 0;
    }
    return;
  }
  DropPrefix(next);
}

void Parser::DropPrefix(std::size_t count) noexcept {
  if (count >= candidate_size_) {
    candidate_size_ = 0;
    return;
  }
  std::move(candidate_.begin() + static_cast<std::ptrdiff_t>(count),
            candidate_.begin() + static_cast<std::ptrdiff_t>(candidate_size_),
            candidate_.begin());
  std::move(receive_times_.begin() + static_cast<std::ptrdiff_t>(count),
            receive_times_.begin() + static_cast<std::ptrdiff_t>(candidate_size_),
            receive_times_.begin());
  candidate_size_ -= count;
}

} // namespace rmlink::wire
