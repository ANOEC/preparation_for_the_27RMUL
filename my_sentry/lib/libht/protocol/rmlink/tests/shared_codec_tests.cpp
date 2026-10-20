#include <gtest/gtest.h>
#include "rmlink/codec.hpp"

#include "../../../utility/serialization/tests/bounded_payload.hpp"
struct SpecialPacket { std::uint16_t value{}; };
template <> struct wire::Codec<SpecialPacket>
{
  static constexpr std::size_t MinEncodedSize = 2, MaxEncodedSize = 2;
  static wire::Result Encode(const SpecialPacket& value, std::span<std::byte> output, wire::Endian endian) noexcept
  {
    return wire::Codec<std::uint16_t>::Encode(static_cast<std::uint16_t>(value.value ^ 0xffff), output, endian);
  }
  static wire::Result Decode(std::span<const std::byte> input, SpecialPacket& value, wire::Endian endian) noexcept
  {
    std::uint16_t raw{};
    const auto result = wire::Codec<std::uint16_t>::Decode(input, raw, endian);
    if (result) value.value = raw ^ 0xffff;
    return result;
  }
};
TEST(SharedRMLinkCodec, ExternalFormatAndGoldenBytes)
{
  std::array<std::byte, 4> bytes{};
  bytes.fill(std::byte{0xaa});
  const auto encoded = wire::Encode(SpecialPacket{0x1234}, bytes);
  ASSERT_TRUE(encoded);
  EXPECT_EQ(encoded.bytes, 2u);
  EXPECT_EQ(bytes, (std::array{std::byte{0xcb}, std::byte{0xed}, std::byte{0xaa}, std::byte{0xaa}}));
  SpecialPacket value{};
  ASSERT_TRUE(wire::Decode(std::span(bytes).first(2), value));
  EXPECT_EQ(value.value, 0x1234);
}

namespace
{
struct Ordinary { std::uint16_t value{}; bool ready{}; };
struct Received
{
  unsigned calls{};
  Ordinary value{};
  static void OnData(void* context, const Ordinary& value, const rmlink::EventMetadata&) noexcept
  {
    auto& self = *static_cast<Received*>(context);
    self.value = value;
    ++self.calls;
  }
};
static_assert(rmlink::StructuredPayload<Ordinary>);
struct Maximum { std::array<std::byte, 255> bytes{}; };
struct Oversized { std::array<std::byte, 256> bytes{}; };
static_assert(rmlink::StructuredPayload<Maximum>);
static_assert(!rmlink::StructuredPayload<Oversized>);

TEST(SharedRMLinkCodec, OrdinaryDecodeAndRejectedExtraInput)
{
  Received received;
  rmlink::TypedStream<Ordinary> stream(42, &received, &Received::OnData);
  const auto registration = stream.registration();
  std::array bytes{std::byte{0x34}, std::byte{0x12}, std::byte{2}, std::byte{0xaa}};
  registration.on_receive(registration.context, std::span(bytes).first(3), {});
  ASSERT_EQ(received.calls, 1u);
  EXPECT_EQ(received.value.value, 0x1234);
  EXPECT_TRUE(received.value.ready);
  registration.on_receive(registration.context, bytes, {});
  EXPECT_EQ(received.calls, 1u);
  EXPECT_EQ(stream.decode_errors(), 1u);
  EXPECT_EQ(received.value.value, 0x1234);
}
} // namespace

namespace
{
struct BoundedReceived
{
  unsigned calls{};
  BoundedMessage value{};
  rmlink::EventMetadata metadata{};
  static void OnData(void* context, const BoundedMessage& value, const rmlink::EventMetadata& metadata) noexcept
  {
    auto& self = *static_cast<BoundedReceived*>(context);
    self.value = value; self.metadata = metadata; ++self.calls;
  }
};
static_assert(rmlink::StructuredPayload<BoundedMessage>);
static_assert(rmlink::StructuredPayload<std::array<std::byte, 0>>);
TEST(SharedRMLinkCodec, BoundedLengthsFailureAndLongThenShort)
{
  BoundedReceived received{};
  rmlink::TypedStream<BoundedMessage> stream(43, &received, &BoundedReceived::OnData);
  const auto registration = stream.registration();
  std::array<std::byte, 119> bytes{};
  for (const std::size_t length : {26, 118, 7})
  {
    registration.on_receive(registration.context, std::span(bytes).first(length), {});
    EXPECT_EQ(received.value.length, length - 6);
  }
  EXPECT_EQ(received.calls, 3u);
  registration.on_receive(registration.context, std::span(bytes).first(5), {});
  registration.on_receive(registration.context, bytes, {});
  EXPECT_EQ(received.calls, 3u);
  EXPECT_EQ(stream.decode_errors(), 2u);
  EXPECT_EQ(received.value.length, 1u);
}
} // namespace

TEST(SharedRMLinkCodec, EmptyAndMaximumFixedTypes)
{
  rmlink::TypedStream<std::array<std::byte, 0>> empty(90, nullptr, nullptr);
  rmlink::TypedStream<std::array<std::byte, 255>> maximum(91, nullptr, nullptr);
  const auto empty_registration = empty.registration();
  empty_registration.on_receive(empty_registration.context, {}, {});
  EXPECT_EQ(empty.decode_errors(), 0u);
  std::array<std::byte, 255> bytes{};
  const auto maximum_registration = maximum.registration();
  maximum_registration.on_receive(maximum_registration.context, bytes, {});
  EXPECT_EQ(maximum.decode_errors(), 0u);
  maximum_registration.on_receive(maximum_registration.context, std::span(bytes).first(254), {});
  EXPECT_EQ(maximum.decode_errors(), 1u);
}
