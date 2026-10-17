#include "dwt_probe.hpp"
#include "wire/codec.hpp"
#include "payloads.hpp"
#include "reference_can.hpp"
#include "../tests/bounded_payload.hpp"

using Payload = WireBenchmark::GimbalToChassis;
extern "C" wire::Result bounded_encode(const BoundedMessage*, std::byte*, std::size_t);
extern "C" wire::Result bounded_decode(const std::byte*, std::size_t, BoundedMessage*);
extern "C" int gimbal_encode(const Payload*, std::byte*, std::size_t);
extern "C" int gimbal_decode(const std::byte*, std::size_t, Payload*);
extern "C" int reference_gimbal_encode(const WireReference::GimbalToChassis*, std::byte*, std::size_t);
extern "C" int reference_gimbal_decode(const std::byte*, std::size_t, WireReference::GimbalToChassis*);
namespace
{
volatile std::uint32_t& Register(std::uintptr_t address)
{
  return *reinterpret_cast<volatile std::uint32_t*>(address);
}
void Barrier() { asm volatile("dsb sy\nisb sy" ::: "memory"); }
std::uint32_t Cycles() { Barrier(); return Register(0xe0001004); }
} // namespace

bool WireBenchmarkDwtInit()
{
  Register(0xe000edfc) = Register(0xe000edfc) | (1u << 24); // DEMCR.TRCENA
  Register(0xe0001fb0) = 0xc5acce55; // Cortex-M7 DWT lock access
  if (Register(0xe0001000) & (1u << 25)) return false; // NOCYCCNT
  Register(0xe0001004) = 0;
  Register(0xe0001000) = Register(0xe0001000) | 1u;
  Barrier();
  return true;
}

template <class T, auto encode, auto decode>
static WireDwtResult Run(std::uint32_t iterations, bool noncanonical_bool)
{
  WireDwtResult result{};
  if (iterations == 0 || iterations > 5000) return result;
  T input{}, output{};
  std::array<std::byte, 34> bytes{};
  const auto begin_encode = Cycles();
  for (std::uint32_t i = 0; i < iterations; ++i)
  {
    ++input.game_progress;
    if (encode(&input, bytes.data(), bytes.size()) != 0) return result;
    result.checksum += std::to_integer<unsigned>(bytes[i % bytes.size()]);
  }
  result.encode_cycles = Cycles() - begin_encode;
  if (noncanonical_bool) bytes[0] = std::byte{255};
  const auto begin_decode = Cycles();
  for (std::uint32_t i = 0; i < iterations; ++i)
  {
    bytes[17] ^= std::byte{1};
    if (decode(bytes.data(), bytes.size(), &output) != 0) return result;
    result.checksum += output.game_progress + output.referee_alive;
  }
  result.decode_cycles = Cycles() - begin_decode;
  result.iterations = iterations;
  result.valid = true;
  return result;
}

WireDwtResult WireBenchmarkDwtRun(std::uint32_t iterations, bool noncanonical_bool)
{
  return Run<Payload, gimbal_encode, gimbal_decode>(iterations, noncanonical_bool);
}
WireDwtResult WireBenchmarkDwtRunReference(std::uint32_t iterations, bool noncanonical_bool)
{
  return Run<WireReference::GimbalToChassis, reference_gimbal_encode, reference_gimbal_decode>(iterations, noncanonical_bool);
}

WireDwtResult WireBenchmarkDwtRunBounded(std::uint32_t iterations, std::uint32_t content_length)
{
  WireDwtResult result{};
  if (iterations == 0 || iterations > 5000 || content_length > 112) return result;
  BoundedMessage input{{1, 2, 3}, content_length, {}}, output{};
  std::array<std::byte, wire::MaxEncodedSize<BoundedMessage>> bytes{};
  const auto size = 6 + content_length;
  const auto begin_encode = Cycles();
  for (std::uint32_t i = 0; i < iterations; ++i)
  {
    ++input.header[0];
    const auto encoded = bounded_encode(&input, bytes.data(), bytes.size());
    if (!encoded || encoded.bytes != size) return result;
    result.checksum += std::to_integer<unsigned>(bytes[i % size]);
  }
  result.encode_cycles = Cycles() - begin_encode;
  const auto begin_decode = Cycles();
  for (std::uint32_t i = 0; i < iterations; ++i)
  {
    bytes[0] ^= std::byte{1};
    if (!bounded_decode(bytes.data(), size, &output)) return result;
    result.checksum += output.header[0] + output.length;
  }
  result.decode_cycles = Cycles() - begin_decode;
  result.iterations = iterations;
  result.valid = true;
  return result;
}
