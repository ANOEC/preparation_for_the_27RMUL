#include "wire/codec.hpp"

enum class Mode : std::uint8_t { Off, On };
struct Classic { std::uint32_t sequence; std::uint32_t flags; };
struct Gimbal
{
  bool a, b, c;
  std::array<float, 3> velocity;
  Mode mode, state;
  std::uint8_t progress;
  std::array<std::uint16_t, 4> status;
  float first_angle, second_angle;
};
struct Chassis { bool alive; std::array<float, 3> acceleration, gyro, euler; };
struct Vqf { std::array<float, 10> fields; };
struct Nested { Classic packet; std::array<bool, 3> flags; };
static_assert(wire::EncodedSize<Classic> == 8);
static_assert(wire::EncodedSize<Gimbal> == 34);
static_assert(wire::EncodedSize<Chassis> == 37);
static_assert(wire::EncodedSize<Vqf> == 40);
static_assert(wire::EncodedSize<Nested> == 11);
constexpr bool Check()
{
  std::array<std::byte, 11> bytes{};
  Nested value{{0x01020304, 0x05060708}, {true, false, true}};
  if (!wire::Encode(value, bytes)) return false;
  bytes[8] = std::byte{2};
  bytes[10] = std::byte{255};
  Nested decoded{};
  return wire::Decode(bytes, decoded) && decoded.packet.sequence == 0x01020304 &&
         decoded.flags[0] && !decoded.flags[1] && decoded.flags[2];
}
static_assert(Check());

#include "bounded_payload.hpp"
struct VariableFirst { BoundedMessage message; std::uint8_t tail; };
struct VariableLast { std::uint8_t head; BoundedMessage message; };
struct VariableSchema { BoundedMessage message; };
template <> struct wire::Schema<VariableSchema>
{
  static constexpr auto Fields = std::tuple{&VariableSchema::message};
};
struct Huge { std::uint8_t local; };
template <> struct wire::Codec<Huge>
{
  static constexpr std::size_t MinEncodedSize = std::numeric_limits<std::size_t>::max() - 1;
  static constexpr std::size_t MaxEncodedSize = MinEncodedSize;
  static wire::Result Encode(const Huge&, std::span<std::byte>, wire::Endian) noexcept;
  static wire::Result Decode(std::span<const std::byte>, Huge&, wire::Endian) noexcept;
};
struct SumOverflow { Huge a, b; };
struct SchemaOverflow { Huge a, b; };
template <> struct wire::Schema<SchemaOverflow>
{
  static constexpr auto Fields = std::tuple{&SchemaOverflow::a, &SchemaOverflow::b};
};
struct MissingBounds {};
template <> struct wire::Codec<MissingBounds> {};
struct ReversedBounds {};
template <> struct wire::Codec<ReversedBounds> : wire::Codec<BoundedMessage>
{
  static constexpr std::size_t MinEncodedSize = 119;
  static wire::Result Encode(const ReversedBounds&, std::span<std::byte>, wire::Endian) noexcept;
  static wire::Result Decode(std::span<const std::byte>, ReversedBounds&, wire::Endian) noexcept;
};
struct Unbounded {};
template <> struct wire::Codec<Unbounded> : wire::Codec<ReversedBounds>
{
  static constexpr std::size_t MinEncodedSize = 0;
  static constexpr std::size_t MaxEncodedSize = std::numeric_limits<std::size_t>::max();
  static wire::Result Encode(const Unbounded&, std::span<std::byte>, wire::Endian) noexcept;
  static wire::Result Decode(std::span<const std::byte>, Unbounded&, wire::Endian) noexcept;
};
template <class T> concept HasFixedQuery = requires { wire::EncodedSize<T>; };
static_assert(wire::Supported<BoundedMessage> && !wire::FixedSize<BoundedMessage>);
static_assert(wire::MinEncodedSize<BoundedMessage> == 6);
static_assert(wire::MaxEncodedSize<BoundedMessage> == 118);
static_assert(!HasFixedQuery<BoundedMessage>);
static_assert(wire::FixedSize<Classic> && wire::MinEncodedSize<Classic> == 8 && wire::MaxEncodedSize<Classic> == 8);
static_assert(!wire::Supported<VariableFirst> && !wire::Supported<VariableLast>);
static_assert(!wire::Supported<std::array<BoundedMessage, 2>>);
static_assert(!wire::Supported<VariableSchema>);
static_assert(wire::FixedSize<Huge>);
static_assert(!wire::Supported<SumOverflow> && !wire::Supported<SchemaOverflow>);
static_assert(!wire::Supported<std::array<Huge, 2>>);
static_assert(!wire::Supported<MissingBounds> && !wire::Supported<ReversedBounds> && !wire::Supported<Unbounded>);
constexpr bool CheckBounded()
{
  BoundedMessage message{{0x1234, 0x5678, 0x9abc}, 20, {}}, decoded{};
  std::array<std::byte, 26> bytes{};
  const auto encoded = wire::Encode(message, bytes);
  return encoded && encoded.bytes == 26 && wire::Decode(bytes, decoded) &&
         decoded.header == message.header && decoded.length == 20;
}
static_assert(CheckBounded());

#include "wire/bits.hpp"
constexpr bool CheckBits()
{
  std::array<std::byte, 2> bytes{};
  std::uint64_t bits{};
  std::int64_t value{};
  return wire::WriteBits(bytes, 0, 11, 0x400, wire::BitOrder::Msb0) &&
         bytes[0] == std::byte{0x80} && bytes[1] == std::byte{} &&
         wire::ReadBits(bytes, 0, 11, bits, wire::BitOrder::Msb0) &&
         wire::SignExtend(bits, 11, value) && value == -1024;
}
static_assert(CheckBits());
