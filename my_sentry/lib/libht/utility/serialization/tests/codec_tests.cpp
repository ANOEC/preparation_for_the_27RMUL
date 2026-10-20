#include "wire/codec.hpp"
#include <gtest/gtest.h>
#include <atomic>
#include <cstdlib>
#include <new>
#include <vector>

namespace
{
std::atomic<std::size_t> allocation_count{};
bool count_allocations = false;
}
void* operator new(std::size_t size)
{
  if (count_allocations) ++allocation_count;
  if (void* result = std::malloc(size ? size : 1)) return result;
  std::abort();
}
void* operator new[](std::size_t size) { return ::operator new(size); }
void operator delete(void* pointer) noexcept { std::free(pointer); }
void operator delete[](void* pointer) noexcept { std::free(pointer); }
void operator delete(void* pointer, std::size_t) noexcept { std::free(pointer); }
void operator delete[](void* pointer, std::size_t) noexcept { std::free(pointer); }

struct Basic
{
  std::uint8_t tag{};
  std::uint32_t sequence{};
  bool ready{};
  std::array<std::uint16_t, 2> samples{};
};
struct Child { std::uint16_t id{}; bool valid{}; };
struct Nested { bool active{}; std::array<bool, 3> switches{}; std::array<Child, 2> entries{}; std::uint32_t counter{}; };
struct Selected { std::uint16_t first{}; std::uint8_t local{}; std::uint16_t last{}; };
struct Packed { std::uint8_t low{}, high{}; };
struct Composite { std::uint16_t id{}; Packed flags{}; Selected selected{}; bool alive{}; };
struct InheritedCustom { std::uint8_t value{}; };
struct NestedInherited { InheritedCustom item{}; std::uint8_t tail{}; };

template <> struct wire::Codec<InheritedCustom> : wire::Codec<std::uint8_t>
{
  static wire::Result Encode(const InheritedCustom& value, std::span<std::byte> bytes, wire::Endian) noexcept
  {
    bytes[0] = std::byte(value.value ^ 255);
    return {wire::Status::Ok, 1};
  }
  static wire::Result Decode(std::span<const std::byte> bytes, InheritedCustom& value, wire::Endian) noexcept
  {
    value.value = std::to_integer<std::uint8_t>(bytes[0]) ^ 255;
    return {wire::Status::Ok, 1};
  }
};

template <> struct wire::Schema<Selected>
{
  static constexpr auto Fields = std::tuple{&Selected::last, &Selected::first};
};
// Deliberately also register a schema: the explicit codec must win.
template <> struct wire::Schema<Packed>
{
  static constexpr auto Fields = std::tuple{&Packed::low, &Packed::high};
};
template <> struct wire::Codec<Packed>
{
  static constexpr std::size_t MinEncodedSize = 1;
  static constexpr std::size_t MaxEncodedSize = MinEncodedSize;
  static wire::Result Encode(const Packed& value, std::span<std::byte> bytes, wire::Endian) noexcept
  {
    bytes[0] = std::byte{0xee}; // Verify a failed codec cannot corrupt the caller's output.
    if (value.low > 15 || value.high > 15) return {};
    bytes[0] = std::byte(value.low | (value.high << 4));
    return {wire::Status::Ok, 1};
  }
  static wire::Result Decode(std::span<const std::byte> bytes, Packed& value, wire::Endian) noexcept
  {
    const auto byte = std::to_integer<std::uint8_t>(bytes[0]);
    value.low = byte & 15;
    if (byte == 255) return {};
    value.high = byte >> 4;
    return {wire::Status::Ok, 1};
  }
};

namespace
{
struct Pointer { int* field{}; };
struct Dynamic { std::vector<int> field; };
struct Constant { const int field{}; };
struct Reference { int& field; };
union Union { int number; float real; };
static_assert(!wire::Supported<Pointer>);
static_assert(!wire::Supported<Dynamic>);
static_assert(!wire::Supported<Constant>);
static_assert(!wire::Supported<Reference>);
static_assert(!wire::Supported<Union>);
static_assert(wire::EncodedSize<Basic> == 10);
static_assert(wire::EncodedSize<Nested> == 14);
static_assert(wire::EncodedSize<Selected> == 4);
static_assert(wire::EncodedSize<Packed> == 1);
static_assert(wire::EncodedSize<Composite> == 8);

TEST(Wire, IndependentGoldenVectors)
{
  const Basic input{0x11, 0x11223344, true, {0x5566, 0x7788}};
  const std::array little{std::byte{0x11}, std::byte{0x44}, std::byte{0x33}, std::byte{0x22},
                          std::byte{0x11}, std::byte{1}, std::byte{0x66}, std::byte{0x55},
                          std::byte{0x88}, std::byte{0x77}};
  const std::array big{std::byte{0x11}, std::byte{0x11}, std::byte{0x22}, std::byte{0x33},
                       std::byte{0x44}, std::byte{1}, std::byte{0x55}, std::byte{0x66},
                       std::byte{0x77}, std::byte{0x88}};
  for (const auto endian : {wire::Endian::Little, wire::Endian::Big})
  {
    std::array<std::byte, 10> output{};
    ASSERT_TRUE(wire::Encode(input, output, endian));
    EXPECT_EQ(output, endian == wire::Endian::Little ? little : big);
    Basic decoded{};
    ASSERT_TRUE(wire::Decode(endian == wire::Endian::Little ? little : big, decoded, endian));
    EXPECT_EQ(decoded.sequence, input.sequence);
    EXPECT_EQ(decoded.samples, input.samples);
    EXPECT_TRUE(decoded.ready);
  }
}

TEST(Wire, BufferChecksAndSentinels)
{
  std::array<std::byte, 12> bytes{};
  bytes.fill(std::byte{0xaa});
  const auto original = bytes;
  Basic value{1, 2, true, {3, 4}};
  EXPECT_EQ(wire::Encode(value, std::span(bytes).first(9)).status, wire::Status::OutputTooSmall);
  EXPECT_EQ(bytes, original);
  ASSERT_TRUE(wire::Encode(value, bytes));
  EXPECT_EQ(bytes[10], std::byte{0xaa});
  EXPECT_EQ(bytes[11], std::byte{0xaa});
  Basic target{7, 8, false, {9, 10}};
  EXPECT_EQ(wire::Decode(bytes, target).status, wire::Status::InvalidLength);
  EXPECT_EQ(target.sequence, 8u);
  EXPECT_EQ(wire::Decode(std::span(bytes).first(9), target).bytes, 0u);
  EXPECT_EQ(target.samples, (std::array<std::uint16_t, 2>{9, 10}));
}

TEST(Wire, NoncanonicalNestedBooleans)
{
  for (const auto flag : {0u, 1u, 2u, 255u})
  {
    std::array<std::byte, 14> bytes{std::byte{0}, std::byte{0}, std::byte{0}, std::byte{0},
        std::byte{0x34}, std::byte{0x12}, std::byte{0}, std::byte{0x78}, std::byte{0x56},
        std::byte{0}, std::byte{1}, std::byte{2}, std::byte{3}, std::byte{4}};
    for (const auto index : {0, 1, 2, 3, 6, 9}) bytes[index] = std::byte(flag);
    const auto original = bytes;
    Nested value{};
    ASSERT_TRUE(wire::Decode(bytes, value));
    EXPECT_EQ(bytes, original);
    EXPECT_EQ(value.active, flag != 0);
    for (auto entry : value.entries) EXPECT_EQ(entry.valid, flag != 0);
    EXPECT_EQ(value.entries[0].id, 0x1234);
    EXPECT_EQ(value.entries[1].id, 0x5678);
    EXPECT_EQ(value.counter, 0x04030201u);
    std::array<std::byte, 14> output{};
    ASSERT_TRUE(wire::Encode(value, output));
    for (const auto index : {0, 1, 2, 3, 6, 9}) EXPECT_EQ(output[index], std::byte(flag != 0));
  }
}

template <class Float, class Bits>
void CheckFloat(Bits pattern)
{
  const auto value = std::bit_cast<Float>(pattern);
  for (const auto endian : {wire::Endian::Little, wire::Endian::Big})
  {
    std::array<std::byte, sizeof(Float)> bytes{};
    ASSERT_TRUE(wire::Encode(value, bytes, endian));
    for (std::size_t i = 0; i < sizeof(Float); ++i)
    {
      const auto shift = (endian == wire::Endian::Little ? i : sizeof(Float) - 1 - i) * 8;
      EXPECT_EQ(bytes[i], std::byte((pattern >> shift) & 255));
    }
    Float output{};
    ASSERT_TRUE(wire::Decode(bytes, output, endian));
    EXPECT_EQ(std::bit_cast<Bits>(output), pattern);
  }
}
TEST(Wire, FloatAndDoubleSpecialBits)
{
  for (std::uint32_t bits : {0x80000000u, 1u, 0x7f800000u, 0xff800000u, 0x7fc12345u})
    CheckFloat<float>(bits);
  for (std::uint64_t bits : {0x8000000000000000ull, 1ull, 0x7ff0000000000000ull, 0x7ff8123456789abcull})
    CheckFloat<double>(bits);
}

TEST(Wire, SpecialCodecPriorityAndFailureTransaction)
{
  std::array bytes{std::byte{0xaa}};
  EXPECT_EQ(wire::Encode(Packed{16, 2}, bytes).status, wire::Status::InvalidValue);
  EXPECT_EQ(bytes[0], std::byte{0xaa});
  ASSERT_TRUE(wire::Encode(Packed{3, 4}, bytes));
  EXPECT_EQ(bytes[0], std::byte{0x43});
  Packed target{7, 8};
  bytes[0] = std::byte{255};
  EXPECT_FALSE(wire::Decode(bytes, target));
  EXPECT_EQ(target.low, 7);
  EXPECT_EQ(target.high, 8);
}

TEST(Wire, SchemaOrderAndUnselectedField)
{
  std::array<std::byte, 4> bytes{};
  ASSERT_TRUE(wire::Encode(Selected{0x0102, 99, 0x0304}, bytes));
  EXPECT_EQ(bytes, (std::array{std::byte{4}, std::byte{3}, std::byte{2}, std::byte{1}}));
  Selected target{0, 55, 0};
  ASSERT_TRUE(wire::Decode(bytes, target));
  EXPECT_EQ(target.first, 0x0102);
  EXPECT_EQ(target.last, 0x0304);
  EXPECT_EQ(target.local, 55);
}

TEST(Wire, InheritedExternalCodecStillOverridesBulkBackend)
{
  std::array<std::byte, 2> bytes{};
  ASSERT_TRUE(wire::Encode(NestedInherited{{0x34}, 7}, bytes));
  EXPECT_EQ(bytes, (std::array{std::byte{0xcb}, std::byte{7}}));
  NestedInherited value{};
  ASSERT_TRUE(wire::Decode(bytes, value));
  EXPECT_EQ(value.item.value, 0x34);
  EXPECT_EQ(value.tail, 7);
}

TEST(Wire, NestedExtensionsAndFailedDecode)
{
  const Composite input{0x1234, {3, 4}, {0x0102, 55, 0x0304}, true};
  std::array<std::byte, 8> bytes{};
  ASSERT_TRUE(wire::Encode(input, bytes));
  EXPECT_EQ(bytes, (std::array{std::byte{0x34}, std::byte{0x12}, std::byte{0x43},
      std::byte{4}, std::byte{3}, std::byte{2}, std::byte{1}, std::byte{1}}));
  Composite target{99, {7, 8}, {5, 66, 6}, false};
  bytes[2] = std::byte{255};
  EXPECT_FALSE(wire::Decode(bytes, target));
  EXPECT_EQ(target.id, 99);
  EXPECT_EQ(target.flags.low, 7);
  bytes[2] = std::byte{0x43};
  bytes[7] = std::byte{255};
  ASSERT_TRUE(wire::Decode(bytes, target));
  EXPECT_EQ(target.selected.local, 66);
  EXPECT_TRUE(target.alive);
}

TEST(Wire, SignedBoundary)
{
  std::array<std::byte, 4> bytes{};
  std::int32_t value = std::numeric_limits<std::int32_t>::min();
  ASSERT_TRUE(wire::Encode(value, bytes));
  EXPECT_EQ(bytes, (std::array{std::byte{0}, std::byte{0}, std::byte{0}, std::byte{0x80}}));
  std::int32_t output{};
  ASSERT_TRUE(wire::Decode(bytes, output));
  EXPECT_EQ(output, value);
}

TEST(Wire, NoRuntimeAllocation)
{
  Nested input{true, {true, false, true}, {{{1, true}, {2, false}}}, 42}, output{};
  std::array<std::byte, 14> bytes{};
  allocation_count = 0;
  count_allocations = true;
  const auto encoded = wire::Encode(input, bytes);
  const auto decoded = wire::Decode(bytes, output);
  count_allocations = false;
  ASSERT_TRUE(encoded);
  ASSERT_TRUE(decoded);
  EXPECT_EQ(allocation_count.load(), 0u);
}
} // namespace

#include "bounded_payload.hpp"

// A hostile formatting codec writes first and then reports invalid results.
struct FaultyResult { std::uint8_t fault{}; };
template <> struct wire::Codec<FaultyResult>
{
  static constexpr std::size_t MinEncodedSize = 2;
  static constexpr std::size_t MaxEncodedSize = 8;
  static wire::Result Report(std::uint8_t fault, std::size_t size) noexcept
  {
    switch (fault)
    {
      case 0: return {wire::Status::InvalidValue, 7};
      case 1: return {wire::Status::Ok, 1};
      case 2: return {wire::Status::Ok, 9};
      default: return {wire::Status::Ok, size - 1};
    }
  }
  static wire::Result Encode(const FaultyResult& value, std::span<std::byte> bytes, wire::Endian) noexcept
  {
    bytes[0] = std::byte{0xee};
    return Report(value.fault, bytes.size());
  }
  static wire::Result Decode(std::span<const std::byte> bytes, FaultyResult& value, wire::Endian) noexcept
  {
    value.fault = 99;
    return Report(std::to_integer<std::uint8_t>(bytes[0]), bytes.size());
  }
};

namespace
{
TEST(Wire, BoundedActualLengthAndGoldenHeader)
{
  for (const auto endian : {wire::Endian::Little, wire::Endian::Big})
  {
    for (std::size_t length : {0u, 1u, 20u, 112u})
    {
      BoundedMessage message{{0x1234, 0x5678, 0x9abc}, length, {}};
      message.content.fill(std::byte{0x5a});
      std::array<std::byte, 120> bytes;
      bytes.fill(std::byte{0xaa});
      const auto encoded = wire::Encode(message, std::span(bytes).first(6 + length), endian);
      ASSERT_TRUE(encoded);
      EXPECT_EQ(encoded.bytes, 6 + length);
      const std::array little{std::byte{0x34}, std::byte{0x12}, std::byte{0x78}, std::byte{0x56}, std::byte{0xbc}, std::byte{0x9a}};
      const std::array big{std::byte{0x12}, std::byte{0x34}, std::byte{0x56}, std::byte{0x78}, std::byte{0x9a}, std::byte{0xbc}};
      const auto& header = endian == wire::Endian::Little ? little : big;
      for (std::size_t i = 0; i < 6; ++i) EXPECT_EQ(bytes[i], header[i]);
      for (std::size_t i = 6; i < encoded.bytes; ++i) EXPECT_EQ(bytes[i], std::byte{0x5a});
      for (std::size_t i = encoded.bytes; i < bytes.size(); ++i) EXPECT_EQ(bytes[i], std::byte{0xaa});
      BoundedMessage decoded{};
      ASSERT_TRUE(wire::Decode(std::span(bytes).first(encoded.bytes), decoded, endian));
      EXPECT_EQ(decoded.header, message.header);
      EXPECT_EQ(decoded.length, length);
      for (std::size_t i = 0; i < length; ++i) EXPECT_EQ(decoded.content[i], std::byte{0x5a});
      const auto snapshot = bytes;
      const auto short_result = wire::Encode(message, std::span(bytes).first(encoded.bytes - 1), endian);
      EXPECT_EQ(short_result.status, wire::Status::OutputTooSmall);
      EXPECT_EQ(short_result.bytes, 0u);
      EXPECT_EQ(bytes, snapshot);
    }
  }
}

TEST(Wire, BoundedLimitsAndLongThenShort)
{
  BoundedMessage value{{1, 2, 3}, 113, {}};
  std::array<std::byte, 119> bytes;
  bytes.fill(std::byte{0xaa});
  const auto original = bytes;
  EXPECT_EQ(wire::Encode(value, bytes).bytes, 0u);
  EXPECT_EQ(bytes, original);
  value.length = 112;
  for (auto input : {std::span(bytes).first(5), std::span(bytes).first(bytes.size())})
  {
    EXPECT_EQ(wire::Decode(input, value).status, wire::Status::InvalidLength);
    EXPECT_EQ(value.length, 112u);
    EXPECT_EQ(value.header, (std::array<std::uint16_t, 3>{1, 2, 3}));
  }
  ASSERT_TRUE(wire::Decode(std::span(bytes).first(118), value));
  EXPECT_EQ(value.length, 112u);
  ASSERT_TRUE(wire::Decode(std::span(bytes).first(7), value));
  EXPECT_EQ(value.length, 1u);
  EXPECT_EQ(value.content[0], std::byte{0xaa});
  for (std::size_t i = 1; i < value.content.size(); ++i) EXPECT_EQ(value.content[i], std::byte{});
}

TEST(Wire, InvalidSuccessAndPartialFailureAreTransactional)
{
  for (const std::uint8_t fault : {0, 1, 2})
  {
    std::array<std::byte, 10> bytes;
    bytes.fill(std::byte{0xaa});
    const auto original = bytes;
    const auto encoded = wire::Encode(FaultyResult{fault}, bytes);
    EXPECT_FALSE(encoded);
    EXPECT_EQ(encoded.bytes, 0u);
    EXPECT_EQ(bytes, original);
  }
  for (const std::uint8_t fault : {0, 1, 2, 3})
  {
    std::array<std::byte, 8> bytes{};
    bytes[0] = std::byte{fault};
    const auto original = bytes;
    FaultyResult value{42};
    const auto decoded = wire::Decode(bytes, value);
    EXPECT_FALSE(decoded);
    EXPECT_EQ(decoded.bytes, 0u);
    EXPECT_EQ(value.fault, 42u);
    EXPECT_EQ(bytes, original);
  }
}

TEST(Wire, BoundedCallsDoNotAllocate)
{
  BoundedMessage input{{1, 2, 3}, 20, {}}, output{};
  std::array<std::byte, 26> bytes{};
  allocation_count = 0;
  count_allocations = true;
  const auto encoded = wire::Encode(input, bytes);
  const auto decoded = wire::Decode(bytes, output);
  count_allocations = false;
  EXPECT_TRUE(encoded);
  EXPECT_TRUE(decoded);
  EXPECT_EQ(allocation_count.load(), 0u);
}
} // namespace

#include "wire/bits.hpp"
namespace
{
TEST(WireBits, IndependentMotorAndRemoteVectors)
{
  const std::array motor{std::byte{0x12}, std::byte{0x34}, std::byte{0x56}, std::byte{0x78}, std::byte{0x9a}, std::byte{0xbc}, std::byte{0xde}, std::byte{0xf0}};
  std::array<std::byte, 8> encoded{};
  const std::array<std::uint64_t, 5> values{0x1234, 0x567, 0x89a, 0xbcd, 0xef0};
  std::size_t offset = 0;
  for (std::size_t i = 0; i < values.size(); ++i)
  {
    const auto width = i == 0 ? 16 : 12;
    std::uint64_t value{};
    ASSERT_TRUE(wire::ReadBits(motor, offset, width, value, wire::BitOrder::Msb0));
    EXPECT_EQ(value, values[i]);
    ASSERT_TRUE(wire::WriteBits(encoded, offset, width, values[i], wire::BitOrder::Msb0));
    offset += width;
  }
  EXPECT_EQ(encoded, motor);
  const std::array remote{std::byte{0x00}, std::byte{0x04}, std::byte{0x20}, std::byte{0x00}, std::byte{0x01}, std::byte{0x08}};
  std::array<std::byte, 6> channels{};
  for (std::size_t i = 0; i < 4; ++i)
  {
    std::uint64_t value{};
    ASSERT_TRUE(wire::ReadBits(remote, i * 11, 11, value));
    EXPECT_EQ(value, 1024u);
    ASSERT_TRUE(wire::WriteBits(channels, i * 11, 11, 1024));
  }
  EXPECT_EQ(channels, remote);
}

TEST(WireBits, InvalidAccessPreservesAllData)
{
  std::array<std::byte, 9> bytes;
  bytes.fill(std::byte{0xa5});
  const auto original = bytes;
  for (const auto order : {wire::BitOrder::Lsb0, wire::BitOrder::Msb0})
  {
    for (const auto offset : {0u, 1u, 7u, 8u})
    {
      for (const auto width : {0u, 65u, 73u})
      {
        EXPECT_FALSE(wire::WriteBits(bytes, offset, width, 0, order));
        std::uint64_t result = 42;
        EXPECT_FALSE(wire::ReadBits(bytes, offset, width, result, order));
        EXPECT_EQ(result, 42u);
        EXPECT_EQ(bytes, original);
      }
    }
    for (const auto offset : {std::size_t{72}, std::numeric_limits<std::size_t>::max()})
    {
      EXPECT_FALSE(wire::WriteBits(bytes, offset, 2, 0, order));
      std::uint64_t result = 42;
      EXPECT_FALSE(wire::ReadBits(bytes, offset, 2, result, order));
      EXPECT_EQ(result, 42u);
    }
    EXPECT_FALSE(wire::WriteBits(bytes, 3, 11, 2048, order));
    EXPECT_EQ(bytes, original);
    ASSERT_TRUE(wire::WriteBits(bytes, 3, 11, 0, order));
    // Bits surrounding the field are preserved for both numbering conventions.
    EXPECT_EQ(bytes[0] & (order == wire::BitOrder::Lsb0 ? std::byte{7} : std::byte{0xe0}),
              original[0] & (order == wire::BitOrder::Lsb0 ? std::byte{7} : std::byte{0xe0}));
    bytes = original;
  }
}

TEST(WireBits, SignedAndFullWidthBoundaries)
{
  for (const std::size_t width : {11, 56, 64})
  {
    const std::uint64_t sign = std::uint64_t{1} << (width - 1);
    const auto maximum = sign - 1;
    const auto mask = width == 64 ? std::numeric_limits<std::uint64_t>::max() : (sign * 2 - 1);
    for (const auto bits : {std::uint64_t{0}, maximum, sign, mask})
    {
      std::int64_t decoded = 42;
      ASSERT_TRUE(wire::SignExtend(bits, width, decoded));
      if (bits == 0) EXPECT_EQ(decoded, 0);
      else if (bits == maximum) EXPECT_EQ(decoded, static_cast<std::int64_t>(maximum));
      else if (bits == sign) EXPECT_EQ(decoded, -static_cast<std::int64_t>(maximum) - 1);
      else EXPECT_EQ(decoded, -1);
      std::array<std::byte, 9> bytes{};
      for (auto order : {wire::BitOrder::Lsb0, wire::BitOrder::Msb0})
      {
        ASSERT_TRUE(wire::WriteBits(bytes, 7, width, bits, order));
        std::uint64_t value{};
        ASSERT_TRUE(wire::ReadBits(bytes, 7, width, value, order));
        EXPECT_EQ(value, bits);
      }
    }
  }
  std::int64_t value = 42;
  EXPECT_FALSE(wire::SignExtend(0, 0, value));
  EXPECT_FALSE(wire::SignExtend(0, 65, value));
  EXPECT_FALSE(wire::SignExtend(2048, 11, value));
  EXPECT_EQ(value, 42);
}

TEST(WireBits, PreparedCallsDoNotAllocate)
{
  std::array<std::byte, 9> bytes{};
  std::uint64_t bits{};
  std::int64_t value{};
  allocation_count = 0;
  count_allocations = true;
  const auto write = wire::WriteBits(bytes, 7, 64, std::numeric_limits<std::uint64_t>::max());
  const auto read = wire::ReadBits(bytes, 7, 64, bits);
  const auto sign = wire::SignExtend(bits, 64, value);
  count_allocations = false;
  EXPECT_TRUE(write && read && sign);
  EXPECT_EQ(value, -1);
  EXPECT_EQ(allocation_count.load(), 0u);
}
} // namespace
