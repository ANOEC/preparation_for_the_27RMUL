#include "wire/codec.hpp"
#include "bounded_payload.hpp"

struct Telemetry { std::uint16_t sequence{}; bool alive{}; std::array<float, 3> velocity{}; };
struct Selected { std::uint16_t first{}; std::uint8_t local{}; std::uint16_t last{}; };
template <> struct wire::Schema<Selected>
{
  static constexpr auto Fields = std::tuple{&Selected::last, &Selected::first};
};
struct Flags { std::uint8_t low{}, high{}; };
template <> struct wire::Codec<Flags>
{
  static constexpr std::size_t MinEncodedSize = 1;
  static constexpr std::size_t MaxEncodedSize = MinEncodedSize;
  static constexpr wire::Result Encode(const Flags& value, std::span<std::byte> bytes,
                                       wire::Endian) noexcept
  {
    if (bytes.size() < 1) return {wire::Status::OutputTooSmall, 0};
    if (value.low > 15 || value.high > 15) return {};
    bytes[0] = std::byte(value.low | (value.high << 4));
    return {wire::Status::Ok, 1};
  }
  static constexpr wire::Result Decode(std::span<const std::byte> bytes, Flags& value,
                                       wire::Endian) noexcept
  {
    if (bytes.size() != 1) return {wire::Status::InvalidLength, 0};
    const auto byte = std::to_integer<std::uint8_t>(bytes[0]);
    value = {static_cast<std::uint8_t>(byte & 15), static_cast<std::uint8_t>(byte >> 4)};
    return {wire::Status::Ok, 1};
  }
};
static_assert(wire::EncodedSize<Telemetry> == 15);
static_assert(wire::EncodedSize<Selected> == 4);
int main()
{
  std::array<std::byte, 15> bytes{};
  Telemetry input{42, true, {1, 2, 3}}, output{};
  if (!wire::Encode(input, bytes) || !wire::Decode(bytes, output)) return 1;
  if (output.sequence != 42 || !output.alive || output.velocity != input.velocity) return 1;
  BoundedMessage message{{1, 2, 3}, 20, {}}, received{};
  std::array<std::byte, 26> actual{};
  const auto encoded = wire::Encode(message, actual);
  return encoded && encoded.bytes == 26 && wire::Decode(actual, received) && received.length == 20 ? 0 : 1;
}
