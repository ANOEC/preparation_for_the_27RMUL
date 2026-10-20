#include "wire/codec.hpp"
#include "reference_can.hpp"
#include "payloads.hpp"
#include "../tests/bounded_payload.hpp"

using CurrentGimbal = WireBenchmark::GimbalToChassis;
using CurrentChassis = WireBenchmark::ChassisToGimbal;

#define WIRE_RESOURCE_PAIR(Name, Type, Size) \
extern "C" __attribute__((noinline)) int Name##_encode(const Type* value, std::byte* data, std::size_t length) { \
  return static_cast<int>(wire::Encode(*value, {data, length}).status); \
} \
extern "C" __attribute__((noinline)) int Name##_decode(const std::byte* data, std::size_t length, Type* value) { \
  return static_cast<int>(wire::Decode({data, length}, *value).status); \
} \
static_assert(wire::FixedSize<Type> && wire::MinEncodedSize<Type> == Size && wire::MaxEncodedSize<Type> == Size)

WIRE_RESOURCE_PAIR(classic, Classic, 8);
WIRE_RESOURCE_PAIR(gimbal, CurrentGimbal, 34);
WIRE_RESOURCE_PAIR(chassis, CurrentChassis, 37);
WIRE_RESOURCE_PAIR(vqf, Vqf, 40);

// The preserved hand-written reference uses the same capacity/transaction boundary.
#define WIRE_REFERENCE_PAIR(Name, Type, Size) \
extern "C" __attribute__((noinline)) int Name##_encode(const Type* value, std::byte* data, std::size_t length) { \
  if (length < Size) return 1; \
  std::array<std::byte, Size> temporary{}; \
  if (value->Serialize(temporary.data(), WireReference::DataEndian::LittleEndian) != Size) return 3; \
  std::memcpy(data, temporary.data(), Size); return 0; \
} \
extern "C" __attribute__((noinline)) int Name##_decode(const std::byte* data, std::size_t length, Type* value) { \
  if (length != Size) return 2; \
  auto temporary = Type::Deserialize(data, WireReference::DataEndian::LittleEndian); \
  *value = temporary; return 0; \
}
WIRE_REFERENCE_PAIR(reference_gimbal, WireReference::GimbalToChassis, 34)
WIRE_REFERENCE_PAIR(reference_chassis, WireReference::ChassisToGimbal, 37)

extern "C" __attribute__((noinline)) int reference_classic_encode(const Classic* value, std::byte* data, std::size_t length)
{
  if (length < 8) return 1;
  std::array<std::byte, 8> temporary{};
  WireReference::detail::SerializeField(temporary.data(), value->sequence, WireReference::DataEndian::LittleEndian);
  WireReference::detail::SerializeField(temporary.data() + 4, value->flags, WireReference::DataEndian::LittleEndian);
  std::memcpy(data, temporary.data(), 8);
  return 0;
}
extern "C" __attribute__((noinline)) int reference_classic_decode(const std::byte* data, std::size_t length, Classic* value)
{
  if (length != 8) return 2;
  *value = {WireReference::detail::DeserializeField<std::uint32_t>(data, WireReference::DataEndian::LittleEndian),
            WireReference::detail::DeserializeField<std::uint32_t>(data + 4, WireReference::DataEndian::LittleEndian)};
  return 0;
}
extern "C" __attribute__((noinline)) int reference_vqf_encode(const Vqf* value, std::byte* data, std::size_t length)
{
  if (length < 40) return 1;
  std::array<std::byte, 40> temporary{};
  for (unsigned i = 0; i < 10; ++i)
    WireReference::detail::SerializeField(temporary.data() + i * 4,
        std::bit_cast<std::uint32_t>(value->fields[i]), WireReference::DataEndian::LittleEndian);
  std::memcpy(data, temporary.data(), 40);
  return 0;
}
extern "C" __attribute__((noinline)) int reference_vqf_decode(const std::byte* data, std::size_t length, Vqf* value)
{
  if (length != 40) return 2;
  Vqf temporary{};
  for (unsigned i = 0; i < 10; ++i)
    temporary.fields[i] = std::bit_cast<float>(WireReference::detail::DeserializeField<std::uint32_t>(
        data + i * 4, WireReference::DataEndian::LittleEndian));
  *value = temporary;
  return 0;
}

static_assert(wire::MinEncodedSize<BoundedMessage> == 6 && wire::MaxEncodedSize<BoundedMessage> == 118);
extern "C" __attribute__((noinline)) wire::Result bounded_encode(
    const BoundedMessage* value, std::byte* data, std::size_t length)
{
  return wire::Encode(*value, {data, length});
}
extern "C" __attribute__((noinline)) wire::Result bounded_decode(
    const std::byte* data, std::size_t length, BoundedMessage* value)
{
  return wire::Decode({data, length}, *value);
}
