#pragma once

#include "wire/codec.hpp"

// A complete message owns its tail. Only the six-byte header and used content
// cross the wire; the in-memory length is inferred from the frame boundary.
struct BoundedMessage
{
  std::array<std::uint16_t, 3> header{};
  std::size_t length{};
  std::array<std::byte, 112> content{};
};

template <> struct wire::Codec<BoundedMessage>
{
  static constexpr std::size_t MinEncodedSize = 6;
  static constexpr std::size_t MaxEncodedSize = 118;

  static constexpr wire::Result Encode(const BoundedMessage& value,
                                       std::span<std::byte> output,
                                       wire::Endian endian) noexcept
  {
    if (value.length > value.content.size()) return {};
    const auto size = MinEncodedSize + value.length;
    if (output.size() < size) return {wire::Status::OutputTooSmall, 0};
    const auto result = wire::Codec<decltype(value.header)>::Encode(
        value.header, output.first(MinEncodedSize), endian);
    if (!result) return result;
    for (std::size_t i = 0; i < value.length; ++i)
      output[MinEncodedSize + i] = value.content[i];
    return {wire::Status::Ok, size};
  }

  static constexpr wire::Result Decode(std::span<const std::byte> input,
                                       BoundedMessage& value,
                                       wire::Endian endian) noexcept
  {
    if (input.size() < MinEncodedSize || input.size() > MaxEncodedSize)
      return {wire::Status::InvalidLength, 0};
    const auto result = wire::Codec<decltype(value.header)>::Decode(
        input.first(MinEncodedSize), value.header, endian);
    if (!result) return result;
    value.length = input.size() - MinEncodedSize;
    value.content.fill(std::byte{});
    for (std::size_t i = 0; i < value.length; ++i)
      value.content[i] = input[MinEncodedSize + i];
    return {wire::Status::Ok, input.size()};
  }
};
