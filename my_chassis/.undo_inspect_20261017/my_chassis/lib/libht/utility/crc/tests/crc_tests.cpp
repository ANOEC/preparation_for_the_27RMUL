#include "crc/crc.hpp"

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <random>
#include <span>
#include <string_view>
#include <thread>
#include <vector>

namespace
{

constexpr std::string_view kCheck = "123456789";

template <std::size_t N>
constexpr std::array<std::uint8_t, N> Bytes(std::string_view text)
{
  std::array<std::uint8_t, N> bytes{};
  for (std::size_t i = 0; i < N; ++i)
  {
    bytes[i] = static_cast<std::uint8_t>(text[i]);
  }
  return bytes;
}

constexpr auto kCheckBytes = Bytes<9>(kCheck);

static_assert(crc::Compute<crc::Crc8MaximReferee>(std::span(kCheckBytes)) == 0x0B);
static_assert(crc::Compute<crc::Crc16Mcrf4xx>(std::span(kCheckBytes)) == 0x6F91);
static_assert(crc::Compute<crc::Crc16Ibm3740>(std::span(kCheckBytes)) == 0x29B1);
static_assert(crc::Compute<crc::Crc32IsoHdlc>(std::span(kCheckBytes)) == 0xCBF43926U);
static_assert(crc::Compute<crc::Crc32IsoHdlc>(std::span<const std::uint8_t>{}) == 0);

// Bit-at-a-time Rocksoft model used as an independent reference.
std::uint32_t Reference(const crc::Params& p, std::span<const std::uint8_t> data)
{
  const std::uint32_t mask = p.width == 32 ? 0xFFFFFFFFU : ((1U << p.width) - 1U);
  const std::uint32_t top = 1U << (p.width - 1U);
  const auto reflect = [](std::uint32_t value, unsigned width) {
    std::uint32_t result = 0;
    for (unsigned bit = 0; bit < width; ++bit)
    {
      result = (result << 1U) | ((value >> bit) & 1U);
    }
    return result;
  };
  std::uint32_t reg = p.init;
  for (auto byte : data)
  {
    const std::uint32_t in = p.refin ? reflect(byte, 8) : byte;
    reg ^= in << (p.width - 8U);
    for (int bit = 0; bit < 8; ++bit)
    {
      reg = (reg & top) != 0U ? ((reg << 1U) ^ p.poly) & mask : (reg << 1U) & mask;
    }
  }
  if (p.refout)
  {
    reg = reflect(reg, p.width);
  }
  return (reg ^ p.xorout) & mask;
}

template <typename S>
void ExpectMatchesReference()
{
  std::mt19937 rng(0x5EEDU);
  std::uniform_int_distribution<int> byte(0, 255);
  for (std::size_t size = 0; size <= 309; ++size)
  {
    std::vector<std::uint8_t> data(size);
    for (auto& value : data)
    {
      value = static_cast<std::uint8_t>(byte(rng));
    }
    ASSERT_EQ(crc::Compute<S>(std::span<const std::uint8_t>(data)),
              Reference(S::kParams, data))
        << "size " << size;
  }
}

// Fake backend: computes in software, can be held busy and records overlap.
struct FakeBackend
{
  bool supported{true};
  std::atomic<int> calls{};
  std::atomic<int> in_flight{};
  std::atomic<int> max_in_flight{};
  std::atomic<bool> hold{};
  std::atomic<bool> entered{};
  std::chrono::microseconds delay{};

  static bool Supports(void* context, const crc::Params&) noexcept
  {
    return static_cast<FakeBackend*>(context)->supported;
  }

  static std::uint32_t Compute(void* context, const crc::Params& params,
                               std::span<const std::byte> data) noexcept
  {
    auto& self = *static_cast<FakeBackend*>(context);
    ++self.calls;
    const int now = ++self.in_flight;
    int seen = self.max_in_flight.load();
    while (now > seen && !self.max_in_flight.compare_exchange_weak(seen, now))
    {
    }
    self.entered = true;
    while (self.hold)
    {
      std::this_thread::yield();
    }
    if (self.delay.count() > 0)
    {
      std::this_thread::sleep_for(self.delay);
    }
    const auto* bytes = reinterpret_cast<const std::uint8_t*>(data.data());
    const auto value = Reference(params, {bytes, data.size()});
    --self.in_flight;
    return value;
  }

  crc::Backend Descriptor() { return {this, &Supports, &Compute}; }
};

class BackendTest : public ::testing::Test
{
 protected:
  void TearDown() override { crc::InstallBackend(nullptr); }
};

}  // namespace

TEST(CrcPresets, StandardCheckValues)
{
  const auto data = std::span<const std::uint8_t>(kCheckBytes);
  EXPECT_EQ(crc::Compute<crc::Crc8MaximReferee>(data), 0x0B);
  EXPECT_EQ(crc::Compute<crc::Crc16Mcrf4xx>(data), 0x6F91);
  EXPECT_EQ(crc::Compute<crc::Crc16Ibm3740>(data), 0x29B1);
  EXPECT_EQ(crc::Compute<crc::Crc32IsoHdlc>(data), 0xCBF43926U);
}

TEST(CrcPresets, EmptyInput)
{
  EXPECT_EQ(crc::Compute<crc::Crc32IsoHdlc>(std::span<const std::byte>{}), 0U);
  EXPECT_EQ(crc::Compute<crc::Crc16Ibm3740>(std::span<const std::byte>{}), 0xFFFF);
  EXPECT_EQ(crc::Compute<crc::Crc8MaximReferee>(std::span<const std::byte>{}), 0xFF);
}

TEST(CrcPresets, RefereeAppendixVectors)
{
  constexpr std::array<std::uint8_t, 4> header{0xA5, 0x0D, 0x00, 0x01};
  EXPECT_EQ(crc::Compute<crc::Crc8MaximReferee>(std::span(header)), 0x8D);
  std::array<std::uint8_t, 32> payload{};
  for (std::size_t i = 0; i < payload.size(); ++i)
  {
    payload[i] = static_cast<std::uint8_t>(i);
  }
  EXPECT_EQ(crc::Compute<crc::Crc8MaximReferee>(std::span<const std::uint8_t>(payload)), 0x21);
  EXPECT_EQ(crc::Compute<crc::Crc16Mcrf4xx>(std::span<const std::uint8_t>(payload)), 0xAC1B);
}

TEST(CrcPresets, ByteAndUint8ViewsAgree)
{
  std::array<std::byte, 9> bytes{};
  for (std::size_t i = 0; i < bytes.size(); ++i)
  {
    bytes[i] = static_cast<std::byte>(kCheckBytes[i]);
  }
  EXPECT_EQ(crc::Compute<crc::Crc32IsoHdlc>(std::span<const std::byte>(bytes)),
            crc::Compute<crc::Crc32IsoHdlc>(std::span(kCheckBytes)));
  EXPECT_EQ(crc::Compute<crc::Crc8MaximReferee>(std::span<const std::byte>(bytes)),
            crc::Compute<crc::Crc8MaximReferee>(std::span(kCheckBytes)));
}

TEST(CrcPresets, MatchBitwiseReference)
{
  ExpectMatchesReference<crc::Crc8MaximReferee>();
  ExpectMatchesReference<crc::Crc16Mcrf4xx>();
  ExpectMatchesReference<crc::Crc16Ibm3740>();
  ExpectMatchesReference<crc::Crc32IsoHdlc>();
  // Mixed reflection is outside the presets but part of the model.
  ExpectMatchesReference<crc::Spec<16, 0x8005, 0x1234, true, false, 0x00FF>>();
  ExpectMatchesReference<crc::Spec<32, 0x1EDC6F41, 0x0, false, true, 0x0>>();
}

TEST_F(BackendTest, NoBackendUsesSoftware)
{
  ASSERT_EQ(crc::InstalledBackend(), nullptr);
  EXPECT_EQ(crc::Compute<crc::Crc32IsoHdlc>(std::span(kCheckBytes)), 0xCBF43926U);
}

TEST_F(BackendTest, InstalledBackendComputes)
{
  FakeBackend fake;
  const auto descriptor = fake.Descriptor();
  crc::InstallBackend(&descriptor);
  EXPECT_EQ(crc::Compute<crc::Crc16Mcrf4xx>(std::span(kCheckBytes)), 0x6F91);
  EXPECT_EQ(fake.calls, 1);
}

TEST_F(BackendTest, UnsupportedSpecFallsBack)
{
  FakeBackend fake;
  fake.supported = false;
  const auto descriptor = fake.Descriptor();
  crc::InstallBackend(&descriptor);
  EXPECT_EQ(crc::Compute<crc::Crc16Ibm3740>(std::span(kCheckBytes)), 0x29B1);
  EXPECT_EQ(fake.calls, 0);
}

TEST_F(BackendTest, BusyBackendFallsBackWithoutWaiting)
{
  FakeBackend fake;
  fake.hold = true;
  const auto descriptor = fake.Descriptor();
  crc::InstallBackend(&descriptor);

  std::uint32_t holder_result = 0;
  std::thread holder([&] {
    holder_result = crc::Compute<crc::Crc32IsoHdlc>(std::span(kCheckBytes));
  });
  while (!fake.entered)
  {
    std::this_thread::yield();
  }

  // The backend is held: this call must return from software immediately.
  EXPECT_EQ(crc::Compute<crc::Crc8MaximReferee>(std::span(kCheckBytes)), 0x0B);
  EXPECT_EQ(fake.calls, 1);

  fake.hold = false;
  holder.join();
  EXPECT_EQ(holder_result, 0xCBF43926U);
}

TEST_F(BackendTest, ResetToNullRestoresSoftware)
{
  FakeBackend fake;
  const auto descriptor = fake.Descriptor();
  crc::InstallBackend(&descriptor);
  crc::InstallBackend(nullptr);
  EXPECT_EQ(crc::Compute<crc::Crc16Ibm3740>(std::span(kCheckBytes)), 0x29B1);
  EXPECT_EQ(fake.calls, 0);
}

TEST_F(BackendTest, ConcurrentCallsAreExclusiveAndCorrect)
{
  FakeBackend fake;
  fake.delay = std::chrono::microseconds(50);
  const auto descriptor = fake.Descriptor();
  crc::InstallBackend(&descriptor);

  constexpr int kThreads = 8;
  constexpr int kIterations = 200;
  std::atomic<int> mismatches{};
  std::vector<std::thread> threads;
  const auto start = std::chrono::steady_clock::now();
  for (int t = 0; t < kThreads; ++t)
  {
    threads.emplace_back([&, t] {
      std::vector<std::uint8_t> data(static_cast<std::size_t>(17 + t * 37));
      for (std::size_t i = 0; i < data.size(); ++i)
      {
        data[i] = static_cast<std::uint8_t>(i * 7 + static_cast<std::size_t>(t));
      }
      const auto view = std::span<const std::uint8_t>(data);
      const auto expected = crc::Crc32IsoHdlc::Software(view);
      for (int i = 0; i < kIterations; ++i)
      {
        if (crc::Compute<crc::Crc32IsoHdlc>(view) != expected)
        {
          ++mismatches;
        }
      }
    });
  }
  for (auto& thread : threads)
  {
    thread.join();
  }
  const auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_EQ(mismatches, 0);
  EXPECT_EQ(fake.max_in_flight, 1);
  EXPECT_GT(fake.calls, 0);
  // Blocking on the backend would serialize every call behind the 50 us delay.
  EXPECT_LT(fake.calls, kThreads * kIterations);
  EXPECT_LT(elapsed, std::chrono::seconds(5));
}
