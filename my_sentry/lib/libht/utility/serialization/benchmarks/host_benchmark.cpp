#include "resource_probe.cpp"
#include <chrono>
#include <iostream>

namespace
{
volatile std::uint64_t sink = 0;
constexpr unsigned Iterations = 100000;
template <class T> void Mutate(T& value)
{
  if constexpr (requires { value.sequence; }) ++value.sequence;
  else if constexpr (requires { value.game_progress; }) ++value.game_progress;
  else if constexpr (requires { value.imu_alive; }) value.imu_alive = !value.imu_alive;
  else value.fields[0] = static_cast<float>(static_cast<unsigned>(value.fields[0]) + 1);
}
template <class T> std::uint64_t Observe(const T& value, unsigned index)
{
  if constexpr (requires { value.sequence; }) return value.sequence;
  else if constexpr (requires { value.game_progress; }) return value.game_progress + value.referee_alive;
  else if constexpr (requires { value.imu_alive; }) return value.imu_alive + std::bit_cast<std::uint32_t>(value.imu_acc[index % 3]);
  else return std::bit_cast<std::uint32_t>(value.fields[index % 10]);
}
bool first_row = true;
template <class T, std::size_t N, class Encode, class Decode>
void Run(const char* name, Encode encode, Decode decode, bool noncanonical = false)
{
  T input{}, output{};
  std::array<std::byte, N> bytes{};
  std::uint64_t checksum = 0;
  auto start = std::chrono::steady_clock::now();
  for (unsigned i = 0; i < Iterations; ++i)
  {
    Mutate(input);
    if (encode(&input, bytes.data(), bytes.size()) != 0) std::abort();
    checksum += std::to_integer<unsigned>(bytes[i % N]);
  }
  const auto encoding = std::chrono::steady_clock::now() - start;
  if (noncanonical) bytes[0] = std::byte{255};
  start = std::chrono::steady_clock::now();
  for (unsigned i = 0; i < Iterations; ++i)
  {
    bytes[N - 1] ^= std::byte{1};
    if (decode(bytes.data(), bytes.size(), &output) != 0) std::abort();
    checksum += Observe(output, i);
  }
  const auto decoding = std::chrono::steady_clock::now() - start;
  sink = checksum;
  if (!first_row) std::cout << ',';
  first_row = false;
  std::cout << "{\"name\":\"" << name << "\",\"size\":" << N
            << ",\"noncanonical_bool\":" << (noncanonical ? "true" : "false")
            << ",\"encode_ns_per_call\":"
            << std::chrono::duration<double, std::nano>(encoding).count() / Iterations
            << ",\"decode_ns_per_call\":"
            << std::chrono::duration<double, std::nano>(decoding).count() / Iterations
            << ",\"checksum\":" << checksum << '}';
}
} // namespace

int main()
{
#if defined(WIRE_BACKEND_PFR)
  constexpr auto backend = "pfr";
#else
  constexpr auto backend = "zpp";
#endif
  std::cout << "{\"backend\":\"" << backend << "\",\"iterations\":" << Iterations
            << ",\"clock\":\"host steady_clock\",\"rows\":[";
  Run<Classic, 8>("classic", classic_encode, classic_decode);
  Run<Classic, 8>("reference_classic", reference_classic_encode, reference_classic_decode);
  Run<CurrentGimbal, 34>("gimbal", gimbal_encode, gimbal_decode);
  Run<WireReference::GimbalToChassis, 34>("reference_gimbal", reference_gimbal_encode, reference_gimbal_decode);
  Run<CurrentGimbal, 34>("gimbal_noncanonical", gimbal_encode, gimbal_decode, true);
  Run<WireReference::GimbalToChassis, 34>("reference_gimbal_noncanonical", reference_gimbal_encode, reference_gimbal_decode, true);
  Run<CurrentChassis, 37>("chassis", chassis_encode, chassis_decode);
  Run<WireReference::ChassisToGimbal, 37>("reference_chassis", reference_chassis_encode, reference_chassis_decode);
  Run<Vqf, 40>("vqf", vqf_encode, vqf_decode);
  Run<Vqf, 40>("reference_vqf", reference_vqf_encode, reference_vqf_decode);
  std::cout << "]}\n";
}
