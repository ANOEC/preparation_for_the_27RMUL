#include "rmlink/types.hpp"
#include "rmlink/wire.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace {

using rmlink::MessageType;
using rmlink::Role;
using rmlink::Status;
using rmlink::wire::FrameView;
using rmlink::wire::Header;

template <std::size_t Size>
std::array<std::byte, Size> Bytes(const std::uint8_t (&values)[Size]) {
  std::array<std::byte, Size> result{};
  for (std::size_t index = 0; index < Size; ++index) {
    result[index] = static_cast<std::byte>(values[index]);
  }
  return result;
}

template <std::size_t Size>
std::array<std::byte, Size> Bytes(const char (&text)[Size + 1]) {
  std::array<std::byte, Size> result{};
  for (std::size_t index = 0; index < Size; ++index) {
    result[index] = static_cast<std::byte>(text[index]);
  }
  return result;
}

struct ReceivedFrames {
  std::vector<Header> headers;
  std::vector<std::vector<std::byte>> payloads;
  std::vector<std::uint64_t> receive_times;

  static void OnFrame(void *context, const FrameView &frame) noexcept {
    auto &self = *static_cast<ReceivedFrames *>(context);
    self.headers.push_back(frame.header);
    self.payloads.emplace_back(frame.payload.begin(), frame.payload.end());
    self.receive_times.push_back(frame.receive_end_us);
  }
};

std::vector<std::byte> Encode(Header header,
                              std::span<const std::byte> payload = {}) {
  std::array<std::byte, rmlink::kMaxFrameSize> storage{};
  const auto result = rmlink::wire::Encode(header, payload, storage);
  EXPECT_EQ(result.status, Status::Ok);
  return {storage.begin(), storage.begin() +
                              static_cast<std::ptrdiff_t>(result.bytes)};
}

TEST(WireCodecTest, MatchesCrcChecksAndFixedVectors) {
  const auto check = Bytes<9>("123456789");
  EXPECT_EQ(rmlink::wire::HeaderCrc(check), 0x29B1U);
  EXPECT_EQ(rmlink::wire::FrameCrc(check), 0xCBF43926U);

  constexpr std::uint8_t empty_values[] = {
      0x52, 0x4d, 0x10, 0x01, 0x34, 0x12, 0xfe, 0x00, 0x64,
      0x00, 0x00, 0x00, 0xf6, 0xe4, 0xc5, 0xd6, 0xc2, 0x5f};
  const auto empty_expected = Bytes(empty_values);
  const auto empty_actual = Encode(
      {MessageType::Data, 1, 0x1234, 254, 0, 100});
  EXPECT_TRUE(std::equal(empty_actual.begin(), empty_actual.end(),
                         empty_expected.begin(), empty_expected.end()));

  constexpr std::uint8_t payload_values[] = {0x00, 0x52, 0x4d, 0xff};
  constexpr std::uint8_t data_values[] = {
      0x52, 0x4d, 0x10, 0x01, 0x34, 0x12, 0xff, 0x04, 0xc8, 0x00, 0x00,
      0x00, 0x14, 0x8d, 0x00, 0x52, 0x4d, 0xff, 0x1f, 0x2f, 0x90, 0xbb};
  const auto payload = Bytes(payload_values);
  const auto data_expected = Bytes(data_values);
  const auto data_actual =
      Encode({MessageType::Data, 1, 0x1234, 255, 0, 200}, payload);
  EXPECT_TRUE(std::equal(data_actual.begin(), data_actual.end(),
                         data_expected.begin(), data_expected.end()));

  constexpr std::uint8_t start_values[] = {
      0x52, 0x4d, 0x05, 0x01, 0x00, 0x00, 0x02, 0x00, 0xf0,
      0x49, 0x02, 0x00, 0x56, 0xdf, 0xb2, 0xf6, 0x82, 0xe6};
  const auto start_expected = Bytes(start_values);
  const auto start_actual =
      Encode({MessageType::Start, 1, 0, 2, 0, 150000});
  EXPECT_TRUE(std::equal(start_actual.begin(), start_actual.end(),
                         start_expected.begin(), start_expected.end()));
}

TEST(WireCodecTest, EnforcesPayloadAndFrameSizeBoundaries) {
  std::array<std::byte, rmlink::kMaxFrameSize + 1> output{};
  std::array<std::byte, 255> maximum{};
  std::array<std::byte, 256> too_large{};
  const Header header{MessageType::Data, 1, 7, 0, 0, 0};

  const auto empty = rmlink::wire::Encode(header, {}, output);
  EXPECT_EQ(empty.status, Status::Ok);
  EXPECT_EQ(empty.bytes, 18U);
  const auto full = rmlink::wire::Encode(header, maximum, output);
  EXPECT_EQ(full.status, Status::Ok);
  EXPECT_EQ(full.bytes, 273U);
  const auto rejected = rmlink::wire::Encode(header, too_large, output);
  EXPECT_EQ(rejected.status, Status::PayloadTooLarge);
  EXPECT_EQ(rejected.bytes, 0U);
}

TEST(WireParserTest, HandlesBytewiseAndCombinedInput) {
  const auto first = Encode({MessageType::Data, 1, 1, 1, 0, 10});
  constexpr std::uint8_t inner_values[] = {0x52, 0x4d, 0x11};
  const auto inner = Bytes(inner_values);
  const auto second =
      Encode({MessageType::Data, 1, 2, 2, 0, 11}, inner);
  ReceivedFrames received;
  rmlink::wire::Parser parser(Role::Host, 100'000,
                              &ReceivedFrames::OnFrame, &received);

  std::uint64_t time = 1000;
  for (const auto value : first) {
    parser.Feed(std::span<const std::byte>(&value, 1), time++);
  }
  std::vector<std::byte> combined = second;
  combined.insert(combined.end(), first.begin(), first.end());
  parser.Feed(combined, 5000);

  ASSERT_EQ(received.headers.size(), 3U);
  EXPECT_EQ(received.headers[0].stream_id, 1);
  EXPECT_EQ(received.headers[1].stream_id, 2);
  EXPECT_EQ(received.headers[2].stream_id, 1);
  ASSERT_GE(received.receive_times.size(), 2U);
  EXPECT_EQ(received.receive_times[0], 1017U);
  EXPECT_EQ(received.receive_times[1], 5000U);
  ASSERT_GE(received.payloads.size(), 2U);
  EXPECT_EQ(received.payloads[1].size(), 3U);
}

TEST(WireParserTest, RecoversFromCrcErrorsAcrossRandomChunks) {
  const auto good = Encode({MessageType::Data, 1, 9, 4, 0, 80});
  auto bad_header = good;
  bad_header[7] = std::byte{0xFE};
  auto bad_frame = good;
  bad_frame[14] ^= std::byte{0x01};

  std::vector<std::byte> bytes = bad_header;
  bytes.insert(bytes.end(), bad_frame.begin(), bad_frame.end());
  bytes.insert(bytes.end(), good.begin(), good.end());

  ReceivedFrames received;
  rmlink::wire::Parser parser(Role::Host, 100'000,
                              &ReceivedFrames::OnFrame, &received);
  std::size_t offset = 0;
  std::uint32_t random = 0x12345678U;
  while (offset < bytes.size()) {
    random = random * 1664525U + 1013904223U;
    const auto count =
        std::min<std::size_t>((random % 7U) + 1U, bytes.size() - offset);
    parser.Feed(std::span<const std::byte>(bytes).subspan(offset, count),
                10'000 + offset);
    offset += count;
  }
  EXPECT_EQ(received.headers.size(), 1U);
  EXPECT_EQ(parser.statistics().header_crc_errors, 1U);
  EXPECT_EQ(parser.statistics().frame_crc_errors, 1U);
}

TEST(WireParserTest, RecoversValidSuffixAfterCandidateTimeout) {
  auto outer = Encode({MessageType::Data, 1, 1, 0, 0, 0});
  outer[7] = std::byte{100};
  const auto crc = rmlink::wire::HeaderCrc(
      std::span<const std::byte>(outer).first(12));
  outer[12] = static_cast<std::byte>(crc & 0xFFU);
  outer[13] = static_cast<std::byte>((crc >> 8U) & 0xFFU);
  outer.resize(rmlink::kHeaderSize);
  const auto good = Encode({MessageType::Data, 1, 5, 8, 0, 123});
  outer.insert(outer.end(), good.begin(), good.end());

  ReceivedFrames received;
  rmlink::wire::Parser parser(Role::Host, 100,
                              &ReceivedFrames::OnFrame, &received);
  parser.Feed(outer, 1000);
  EXPECT_TRUE(received.headers.empty());
  parser.Poll(1101);
  ASSERT_EQ(received.headers.size(), 1U);
  EXPECT_EQ(received.headers[0].stream_id, 5);
  EXPECT_EQ(parser.statistics().partial_frame_timeouts, 1U);
}

TEST(WireParserTest, RejectsInvalidTypeSessionDirectionAndPayloads) {
  ReceivedFrames received;
  rmlink::wire::Parser device_parser(Role::Device, 1000,
                                     &ReceivedFrames::OnFrame, &received);
  const auto wrong_direction =
      Encode({MessageType::Log, 1, 0, 0, 0, 0},
             std::array{std::byte{2}, std::byte{0}});
  const auto unknown_type =
      Encode({static_cast<MessageType>(0xAA), 1, 0, 0, 0, 0});
  const auto bad_start_length =
      Encode({MessageType::Start, 1, 0, 0, 0, 0},
             std::array{std::byte{0}});
  const auto bad_reject = Encode(
      {MessageType::Reject, 1, 0, 0, 0, 0},
      std::array{std::byte{0xAA}, std::byte{1}, std::byte{1}, std::byte{0},
                 std::byte{0}, std::byte{0}, std::byte{0}, std::byte{0},
                 std::byte{0}, std::byte{0}, std::byte{0}, std::byte{0}});
  const auto bad_session = [] {
    auto frame = Encode({MessageType::Data, 1, 2, 0, 0, 0});
    frame[3] = std::byte{0};
    const auto header_crc =
        rmlink::wire::HeaderCrc(std::span<const std::byte>(frame).first(12));
    frame[12] = static_cast<std::byte>(header_crc & 0xFFU);
    frame[13] = static_cast<std::byte>((header_crc >> 8U) & 0xFFU);
    const auto frame_crc = rmlink::wire::FrameCrc(
        std::span<const std::byte>(frame).first(frame.size() - 4));
    for (std::size_t index = 0; index < 4; ++index) {
      frame[frame.size() - 4 + index] =
          static_cast<std::byte>((frame_crc >> (index * 8U)) & 0xFFU);
    }
    return frame;
  }();
  device_parser.Feed(wrong_direction, 1);
  device_parser.Feed(unknown_type, 2);
  device_parser.Feed(bad_session, 3);
  device_parser.Feed(bad_start_length, 4);
  device_parser.Feed(bad_reject, 5);
  EXPECT_TRUE(received.headers.empty());
  EXPECT_EQ(device_parser.statistics().semantic_errors, 5U);
}

TEST(WireParserTest, RecoversAfterContinuousNoise) {
  ReceivedFrames received;
  rmlink::wire::Parser parser(Role::Host, 100,
                              &ReceivedFrames::OnFrame, &received);
  std::array<std::byte, 4096> noise{};
  for (std::size_t index = 0; index < noise.size(); ++index) {
    noise[index] = index % 5 == 0 ? std::byte{0x52} : std::byte{0xA5};
  }
  for (std::size_t offset = 0; offset < noise.size(); offset += 17) {
    const auto count = std::min<std::size_t>(17, noise.size() - offset);
    parser.Feed(std::span<const std::byte>(noise).subspan(offset, count),
                1000 + offset);
  }
  const auto good = Encode({MessageType::Data, 1, 77, 0, 0, 12});
  parser.Feed(good, 6000);
  ASSERT_EQ(received.headers.size(), 1U);
  EXPECT_EQ(received.headers[0].stream_id, 77);
  EXPECT_NE(parser.statistics().noise_bytes, 0U);
}

void OnReceive(void *, std::span<const std::byte>,
               const rmlink::EventMetadata &) noexcept {}

TEST(CoreConfigTest, ValidatesRatesCapacitiesAndTiming) {
  const std::array streams{rmlink::StreamRegistration{1, nullptr, &OnReceive}};
  rmlink::Config config{};
  config.streams = streams;
  EXPECT_EQ(rmlink::ValidateConfig(config), Status::Ok);
  config.realtime_rate_limit_fps = 1001;
  EXPECT_EQ(rmlink::ValidateConfig(config), Status::InvalidConfig);
  config.realtime_rate_limit_fps = 500;
  config.control_queue_depth = 1;
  EXPECT_EQ(rmlink::ValidateConfig(config), Status::InvalidConfig);
  config.control_queue_depth = 2;
  config.timing.partial_frame_timeout_ms = 0;
  EXPECT_EQ(rmlink::ValidateConfig(config), Status::InvalidConfig);
}

} // namespace
