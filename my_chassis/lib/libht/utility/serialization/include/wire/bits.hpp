#pragma once

#include "wire/codec.hpp"

namespace wire
{
// Lsb0: bit zero is the low bit of a byte, first field bit is least significant.
// Msb0: bit zero is the high bit of a byte, first field bit is most significant.
// These describe bit streams, independently of Endian on numeric fields.
enum class BitOrder { Lsb0, Msb0 };

namespace detail
{
constexpr bool BitRange(std::size_t size, std::size_t offset,
                        std::size_t width) noexcept
{
  if (width == 0 || width > 64 ||
      offset > std::numeric_limits<std::size_t>::max() - (width - 1)) return false;
  const auto first = offset / 8;
  const auto count = (offset % 8 + width + 7) / 8;
  return first <= size && count <= size - first;
}

constexpr bool BitValue(std::uint64_t value, std::size_t width) noexcept
{
  return width == 64 || value < (std::uint64_t{1} << width);
}
} // namespace detail

constexpr Result ReadBits(std::span<const std::byte> input, std::size_t offset,
                          std::size_t width, std::uint64_t& value,
                          BitOrder order = BitOrder::Lsb0) noexcept
{
  if (!detail::BitRange(input.size(), offset, width)) return {Status::InvalidLength, 0};
  std::uint64_t temporary = 0;
  for (std::size_t i = 0; i < width; ++i)
  {
    const auto position = offset + i;
    const auto bit = order == BitOrder::Lsb0 ? position % 8 : 7 - position % 8;
    const auto shift = order == BitOrder::Lsb0 ? i : width - 1 - i;
    temporary |= std::uint64_t((std::to_integer<unsigned>(input[position / 8]) >> bit) & 1) << shift;
  }
  value = temporary;
  return {Status::Ok, (offset % 8 + width + 7) / 8};
}

constexpr Result WriteBits(std::span<std::byte> output, std::size_t offset,
                           std::size_t width, std::uint64_t value,
                           BitOrder order = BitOrder::Lsb0) noexcept
{
  if (!detail::BitRange(output.size(), offset, width)) return {Status::InvalidLength, 0};
  if (!detail::BitValue(value, width)) return {Status::InvalidValue, 0};
  for (std::size_t i = 0; i < width; ++i)
  {
    const auto position = offset + i;
    const auto bit = order == BitOrder::Lsb0 ? position % 8 : 7 - position % 8;
    const auto shift = order == BitOrder::Lsb0 ? i : width - 1 - i;
    const auto mask = std::byte(1u << bit);
    output[position / 8] = (output[position / 8] & ~mask) |
                          (((value >> shift) & 1) ? mask : std::byte{});
  }
  return {Status::Ok, (offset % 8 + width + 7) / 8};
}

constexpr Result SignExtend(std::uint64_t bits, std::size_t width,
                            std::int64_t& value) noexcept
{
  if (width == 0 || width > 64) return {Status::InvalidLength, 0};
  if (!detail::BitValue(bits, width)) return {Status::InvalidValue, 0};
  if (width < 64 && (bits & (std::uint64_t{1} << (width - 1))))
    bits |= ~((std::uint64_t{1} << width) - 1);
  value = std::bit_cast<std::int64_t>(bits);
  return {Status::Ok, 0};
}
} // namespace wire
