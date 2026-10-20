#pragma once
#include <cstdint>

struct WireDwtResult
{
  std::uint32_t encode_cycles{}, decode_cycles{}, iterations{}, checksum{};
  bool valid{};
};
bool WireBenchmarkDwtInit();
WireDwtResult WireBenchmarkDwtRun(std::uint32_t iterations, bool noncanonical_bool = false);
WireDwtResult WireBenchmarkDwtRunReference(std::uint32_t iterations, bool noncanonical_bool = false);

WireDwtResult WireBenchmarkDwtRunBounded(std::uint32_t iterations, std::uint32_t content_length);
