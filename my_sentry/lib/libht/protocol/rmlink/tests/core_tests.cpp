#include "rmlink/core.hpp"

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <new>
#include <span>
#include <vector>

namespace allocation_guard {
std::atomic<bool> enabled{false};
std::atomic<std::size_t> count{0};
} // namespace allocation_guard

void *operator new(std::size_t size) {
  if (allocation_guard::enabled.load(std::memory_order_relaxed)) {
    allocation_guard::count.fetch_add(1, std::memory_order_relaxed);
  }
  if (void *pointer = std::malloc(size); pointer != nullptr) {
    return pointer;
  }
  throw std::bad_alloc();
}

void *operator new[](std::size_t size) { return ::operator new(size); }
void operator delete(void *pointer) noexcept { std::free(pointer); }
void operator delete[](void *pointer) noexcept { std::free(pointer); }
void operator delete(void *pointer, std::size_t) noexcept { std::free(pointer); }
void operator delete[](void *pointer, std::size_t) noexcept {
  std::free(pointer);
}

namespace {

struct TypedValue {
  std::uint16_t value{};
  bool fail_encode{};
  bool fail_decode{};
};
struct InvalidTypedValue { std::array<std::byte, 256> bytes{}; };
} // namespace

template <> struct wire::Codec<TypedValue> {
  static constexpr std::size_t MinEncodedSize = 2, MaxEncodedSize = 2;
  static wire::Result Encode(const TypedValue& value, std::span<std::byte> output, wire::Endian endian) noexcept {
    if (value.fail_encode) return {};
    return wire::Codec<std::uint16_t>::Encode(value.value, output, endian);
  }
  static wire::Result Decode(std::span<const std::byte> input, TypedValue& value, wire::Endian endian) noexcept {
    if (value.fail_decode) return {};
    return wire::Codec<std::uint16_t>::Decode(input, value.value, endian);
  }
};
namespace {

static_assert(rmlink::StructuredPayload<TypedValue>);
static_assert(!rmlink::StructuredPayload<InvalidTypedValue>);

void OnReceive(void *, std::span<const std::byte>,
               const rmlink::EventMetadata &) noexcept {}

struct ReceivedEvent {
  std::size_t count{};
  rmlink::EventMetadata metadata{};
  std::vector<std::byte> payload;

  static void OnData(void *context, std::span<const std::byte> bytes,
                     const rmlink::EventMetadata &metadata) noexcept {
    auto &self = *static_cast<ReceivedEvent *>(context);
    ++self.count;
    self.metadata = metadata;
    self.payload.assign(bytes.begin(), bytes.end());
  }
};

struct ReceivedLog {
  std::size_t count{};
  std::uint8_t level{};
  std::uint8_t flags{};
  std::vector<std::byte> text;
  rmlink::EventMetadata metadata{};

  static void OnLog(void *context, std::uint8_t level, std::uint8_t flags,
                    std::span<const std::byte> text,
                    const rmlink::EventMetadata &metadata) noexcept {
    auto &self = *static_cast<ReceivedLog *>(context);
    ++self.count;
    self.level = level;
    self.flags = flags;
    self.text.assign(text.begin(), text.end());
    self.metadata = metadata;
  }
};

struct ReceivedTyped {
  std::size_t count{};
  std::uint16_t value{};

  static void OnValue(void *context, const TypedValue &value,
                      const rmlink::EventMetadata &) noexcept {
    auto &self = *static_cast<ReceivedTyped *>(context);
    ++self.count;
    self.value = value.value;
  }
};

struct BusyTransport {
  std::uint64_t now_us{1'000'000};
  std::size_t send_calls{};
  std::size_t abort_calls{};

  static std::uint64_t Now(void *context) noexcept {
    return static_cast<BusyTransport *>(context)->now_us;
  }
  static rmlink::SendResult Send(void *context,
                                 std::span<const std::byte>,
                                 std::uint64_t) noexcept {
    ++static_cast<BusyTransport *>(context)->send_calls;
    return rmlink::SendResult::Busy;
  }
  static void Abort(void *context) noexcept {
    ++static_cast<BusyTransport *>(context)->abort_calls;
  }
};

struct DelayedTransport {
  std::uint64_t now_us{2'000'000};
  std::size_t send_calls{};
  std::size_t abort_calls{};
  const std::byte *accepted_data{};
  std::size_t accepted_size{};
  std::uint64_t accepted_token{};
  std::vector<std::byte> snapshot;

  static std::uint64_t Now(void *context) noexcept {
    return static_cast<DelayedTransport *>(context)->now_us;
  }
  static rmlink::SendResult Send(void *context,
                                 std::span<const std::byte> frame,
                                 std::uint64_t token) noexcept {
    auto &self = *static_cast<DelayedTransport *>(context);
    ++self.send_calls;
    self.accepted_data = frame.data();
    self.accepted_size = frame.size();
    self.accepted_token = token;
    self.snapshot.assign(frame.begin(), frame.end());
    return rmlink::SendResult::Accepted;
  }
  static void Abort(void *context) noexcept {
    ++static_cast<DelayedTransport *>(context)->abort_calls;
  }
};

struct LinkTransport {
  std::uint64_t now_us{};
  std::size_t send_calls{};
  std::size_t abort_calls{};
  std::uint64_t accepted_token{};
  std::vector<std::byte> snapshot;

  static std::uint64_t Now(void *context) noexcept {
    return static_cast<LinkTransport *>(context)->now_us;
  }
  static rmlink::SendResult Send(void *context,
                                 std::span<const std::byte> frame,
                                 std::uint64_t token) noexcept {
    auto &self = *static_cast<LinkTransport *>(context);
    ++self.send_calls;
    self.accepted_token = token;
    self.snapshot.assign(frame.begin(), frame.end());
    return rmlink::SendResult::Accepted;
  }
  static void Abort(void *context) noexcept {
    ++static_cast<LinkTransport *>(context)->abort_calls;
  }
};

struct FixedTransport {
  std::uint64_t now_us{};
  std::size_t send_calls{};
  std::size_t abort_calls{};
  std::uint64_t accepted_token{};
  std::array<std::byte, rmlink::kMaxFrameSize> frame{};
  std::size_t frame_size{};

  static std::uint64_t Now(void *context) noexcept {
    return static_cast<FixedTransport *>(context)->now_us;
  }
  static rmlink::SendResult Send(void *context,
                                 std::span<const std::byte> frame,
                                 std::uint64_t token) noexcept {
    auto &self = *static_cast<FixedTransport *>(context);
    ++self.send_calls;
    self.accepted_token = token;
    self.frame_size = frame.size();
    std::copy(frame.begin(), frame.end(), self.frame.begin());
    return rmlink::SendResult::Accepted;
  }
  static void Abort(void *context) noexcept {
    ++static_cast<FixedTransport *>(context)->abort_calls;
  }
  [[nodiscard]] std::span<const std::byte> bytes() const noexcept {
    return std::span<const std::byte>(frame.data(), frame_size);
  }
};

bool EstablishActiveFixed(rmlink::Core &host, rmlink::Core &device,
                          FixedTransport &host_transport,
                          FixedTransport &device_transport) {
  if (host.Begin() != rmlink::Status::Ok) {
    return false;
  }
  host.Poll();
  device.Receive(host_transport.bytes(), device_transport.now_us);
  host.NotifySendComplete(host_transport.accepted_token,
                          rmlink::CompletionResult::Complete);
  device.Poll();
  host.Receive(device_transport.bytes(), host_transport.now_us);
  device.NotifySendComplete(device_transport.accepted_token,
                            rmlink::CompletionResult::Complete);
  host.Poll();
  device_transport.now_us = host_transport.now_us + 1'000'000;
  device.Receive(host_transport.bytes(), device_transport.now_us);
  host.NotifySendComplete(host_transport.accepted_token,
                          rmlink::CompletionResult::Complete);
  device.Poll();
  host.Receive(device_transport.bytes(), host_transport.now_us);
  device.NotifySendComplete(device_transport.accepted_token,
                            rmlink::CompletionResult::Complete);
  host.Poll();
  host.Poll();
  device.Receive(host_transport.bytes(), device_transport.now_us);
  device.Poll();
  host.Receive(device_transport.bytes(), host_transport.now_us);
  device.NotifySendComplete(device_transport.accepted_token,
                            rmlink::CompletionResult::Complete);
  host.NotifySendComplete(host_transport.accepted_token,
                          rmlink::CompletionResult::Complete);
  host.Poll();
  return host.state() == rmlink::State::Active &&
         device.state() == rmlink::State::Active;
}

bool EstablishActive(rmlink::Core &host, rmlink::Core &device,
                     LinkTransport &host_transport,
                     LinkTransport &device_transport) {
  if (host.Begin() != rmlink::Status::Ok) {
    return false;
  }
  host.Poll();
  if (!host.has_in_flight_frame()) {
    return false;
  }
  const auto hello = host_transport.snapshot;
  host.NotifySendComplete(host_transport.accepted_token,
                          rmlink::CompletionResult::Complete);
  device.Receive(hello, device_transport.now_us);
  device.Poll();
  if (!device.has_in_flight_frame()) {
    return false;
  }
  const auto hello_ack = device_transport.snapshot;
  device.NotifySendComplete(device_transport.accepted_token,
                            rmlink::CompletionResult::Complete);
  host.Receive(hello_ack, host_transport.now_us);

  host.Poll();
  if (!host.has_in_flight_frame()) {
    return false;
  }
  const auto sync_request = host_transport.snapshot;
  host.NotifySendComplete(host_transport.accepted_token,
                          rmlink::CompletionResult::Complete);
  device_transport.now_us = host_transport.now_us + 1'000'000;
  device.Receive(sync_request, device_transport.now_us);
  device.Poll();
  if (!device.has_in_flight_frame()) {
    return false;
  }
  const auto sync_response = device_transport.snapshot;
  device.NotifySendComplete(device_transport.accepted_token,
                            rmlink::CompletionResult::Complete);
  host.Receive(sync_response, host_transport.now_us);
  host.Poll();
  if (host.state() != rmlink::State::Starting) {
    return false;
  }

  host.Poll();
  if (!host.has_in_flight_frame()) {
    return false;
  }
  const auto start = host_transport.snapshot;
  device.Receive(start, device_transport.now_us);
  device.Poll();
  if (!device.has_in_flight_frame()) {
    return false;
  }
  const auto start_ack = device_transport.snapshot;
  host.Receive(start_ack, host_transport.now_us);
  device.NotifySendComplete(device_transport.accepted_token,
                            rmlink::CompletionResult::Complete);
  host.NotifySendComplete(host_transport.accepted_token,
                          rmlink::CompletionResult::Complete);
  host.Poll();
  return host.state() == rmlink::State::Active &&
         device.state() == rmlink::State::Active;
}

rmlink::Config MakeConfig(
    std::span<const rmlink::StreamRegistration> streams) {
  rmlink::Config config{};
  config.streams = streams;
  return config;
}

void WriteU16(std::byte *output, std::uint16_t value) {
  output[0] = static_cast<std::byte>(value & 0xFFU);
  output[1] = static_cast<std::byte>((value >> 8U) & 0xFFU);
}

void WriteU64(std::byte *output, std::uint64_t value) {
  for (std::size_t index = 0; index < 8; ++index) {
    output[index] = static_cast<std::byte>((value >> (index * 8U)) & 0xFFU);
  }
}

std::vector<std::byte> EncodeHello(std::uint16_t version,
                                   std::uint8_t session_id,
                                   std::uint8_t sequence,
                                   std::uint64_t attempt_token,
                                   std::uint64_t host_epoch_us) {
  std::array<std::byte, 18> payload{};
  WriteU16(payload.data(), version);
  WriteU64(payload.data() + 2, attempt_token);
  WriteU64(payload.data() + 10, host_epoch_us);
  std::array<std::byte, rmlink::kMaxFrameSize> encoded{};
  const auto result = rmlink::wire::Encode(
      {rmlink::MessageType::Hello, session_id, 0, sequence, 0, 0}, payload,
      encoded);
  EXPECT_EQ(result.status, rmlink::Status::Ok);
  return {encoded.begin(),
          encoded.begin() + static_cast<std::ptrdiff_t>(result.bytes)};
}

std::uint64_t ReadU64(const std::byte *input) {
  std::uint64_t value = 0;
  for (std::size_t index = 0; index < 8; ++index) {
    value |= static_cast<std::uint64_t>(
                 std::to_integer<std::uint8_t>(input[index]))
             << (index * 8U);
  }
  return value;
}

std::vector<std::byte> EncodeHelloAck(const std::vector<std::byte> &hello,
                                      std::uint8_t session_id,
                                      std::uint8_t sequence,
                                      std::uint64_t attempt_token) {
  std::array<std::byte, 18> payload{};
  WriteU16(payload.data(), rmlink::kProtocolVersion);
  WriteU64(payload.data() + 2, attempt_token);
  WriteU64(payload.data() + 10, 9'000'000);
  std::array<std::byte, rmlink::kMaxFrameSize> encoded{};
  const auto result = rmlink::wire::Encode(
      {rmlink::MessageType::HelloAck, session_id, 0, sequence, 0, 0}, payload,
      encoded);
  EXPECT_EQ(result.status, rmlink::Status::Ok);
  EXPECT_GE(hello.size(), 32U);
  return {encoded.begin(),
          encoded.begin() + static_cast<std::ptrdiff_t>(result.bytes)};
}

std::vector<std::byte> EncodeData(std::uint8_t session_id,
                                  std::uint16_t stream_id,
                                  std::uint8_t sequence,
                                  std::uint32_t timestamp_us,
                                  std::span<const std::byte> payload = {}) {
  std::array<std::byte, rmlink::kMaxFrameSize> encoded{};
  const auto result = rmlink::wire::Encode(
      {rmlink::MessageType::Data, session_id, stream_id, sequence, 0,
       timestamp_us},
      payload, encoded);
  EXPECT_EQ(result.status, rmlink::Status::Ok);
  return {encoded.begin(),
          encoded.begin() + static_cast<std::ptrdiff_t>(result.bytes)};
}

std::vector<std::byte> EncodeSyncResponse(std::uint8_t session_id,
                                          std::uint8_t sequence,
                                          std::uint64_t t1,
                                          std::uint64_t t2,
                                          std::uint64_t t3) {
  std::array<std::byte, 24> payload{};
  WriteU64(payload.data(), t1);
  WriteU64(payload.data() + 8, t2);
  WriteU64(payload.data() + 16, t3);
  std::array<std::byte, rmlink::kMaxFrameSize> encoded{};
  const auto result = rmlink::wire::Encode(
      {rmlink::MessageType::SyncResp, session_id, 0, sequence, 0, 0}, payload,
      encoded);
  EXPECT_EQ(result.status, rmlink::Status::Ok);
  return {encoded.begin(),
          encoded.begin() + static_cast<std::ptrdiff_t>(result.bytes)};
}

std::vector<std::byte> EncodeSyncRequest(std::uint8_t session_id,
                                         std::uint8_t sequence,
                                         std::uint64_t t1) {
  std::array<std::byte, 8> payload{};
  WriteU64(payload.data(), t1);
  std::array<std::byte, rmlink::kMaxFrameSize> encoded{};
  const auto result = rmlink::wire::Encode(
      {rmlink::MessageType::SyncReq, session_id, 0, sequence, 0, 0}, payload,
      encoded);
  EXPECT_EQ(result.status, rmlink::Status::Ok);
  return {encoded.begin(),
          encoded.begin() + static_cast<std::ptrdiff_t>(result.bytes)};
}

rmlink::wire::Header DecodeHeader(const std::vector<std::byte> &frame) {
  rmlink::wire::Header header{};
  EXPECT_TRUE(rmlink::wire::DecodeHeader(frame, header));
  return header;
}

TEST(CoreTransportTest, BusyDoesNotCreateInFlightTransfer) {
  const std::array streams{
      rmlink::StreamRegistration{10, nullptr, &OnReceive}};
  BusyTransport transport;
  rmlink::Core core(
      MakeConfig(streams),
      {&transport, &BusyTransport::Now, &BusyTransport::Send,
       &BusyTransport::Abort});
  EXPECT_EQ(core.initialization_status(), rmlink::Status::Ok);
  EXPECT_EQ(core.Begin(), rmlink::Status::Ok);
  core.Poll();
  ++transport.now_us;
  core.Poll();
  EXPECT_EQ(transport.send_calls, 2U);
  EXPECT_FALSE(core.has_in_flight_frame());
  EXPECT_EQ(core.statistics().transport_busy, 2U);
  EXPECT_EQ(core.state(), rmlink::State::Handshaking);
}

TEST(CoreTransportTest, AcceptedBufferStaysStableUntilMatchingCompletion) {
  const std::array streams{
      rmlink::StreamRegistration{10, nullptr, &OnReceive}};
  DelayedTransport transport;
  rmlink::Core core(
      MakeConfig(streams),
      {&transport, &DelayedTransport::Now, &DelayedTransport::Send,
       &DelayedTransport::Abort});
  EXPECT_EQ(core.Begin(), rmlink::Status::Ok);
  core.Poll();
  EXPECT_TRUE(core.has_in_flight_frame());
  EXPECT_EQ(transport.send_calls, 1U);
  EXPECT_EQ(transport.accepted_size, 36U);
  ASSERT_NE(transport.accepted_data, nullptr);
  const auto token = transport.accepted_token;

  transport.now_us += 10;
  core.Poll();
  EXPECT_EQ(transport.send_calls, 1U);
  EXPECT_TRUE(std::equal(transport.snapshot.begin(), transport.snapshot.end(),
                         transport.accepted_data));

  core.NotifySendComplete(token + 1, rmlink::CompletionResult::Complete);
  EXPECT_TRUE(core.has_in_flight_frame());
  EXPECT_EQ(core.statistics().stale_completions, 1U);
  core.NotifySendComplete(token, rmlink::CompletionResult::Complete);
  EXPECT_FALSE(core.has_in_flight_frame());
  core.Poll();
  EXPECT_EQ(transport.send_calls, 1U);
}

TEST(CoreTransportTest, CompletionTimeoutRequiresAbortConfirmation) {
  const std::array streams{
      rmlink::StreamRegistration{10, nullptr, &OnReceive}};
  DelayedTransport transport;
  auto config = MakeConfig(streams);
  config.timing.tx_completion_timeout_ms = 1;
  rmlink::Core core(
      config,
      {&transport, &DelayedTransport::Now, &DelayedTransport::Send,
       &DelayedTransport::Abort});
  EXPECT_EQ(core.Begin(), rmlink::Status::Ok);
  core.Poll();
  transport.now_us += 1001;
  core.Poll();
  EXPECT_EQ(core.state(), rmlink::State::ResetRequired);
  EXPECT_TRUE(core.has_in_flight_frame());
  EXPECT_EQ(transport.abort_calls, 1U);
  core.NotifyTransportAborted();
  EXPECT_EQ(core.state(), rmlink::State::Idle);
  EXPECT_FALSE(core.has_in_flight_frame());
  EXPECT_EQ(core.session_id(), 0);

  core.NotifySendComplete(transport.accepted_token,
                          rmlink::CompletionResult::Complete);
  EXPECT_EQ(core.state(), rmlink::State::Idle);
  EXPECT_EQ(core.statistics().stale_completions, 1U);
}

TEST(CoreTransportTest, FailedCompletionStopsSessionBeforeReuse) {
  const std::array streams{
      rmlink::StreamRegistration{10, nullptr, &OnReceive}};
  DelayedTransport transport;
  rmlink::Core core(
      MakeConfig(streams),
      {&transport, &DelayedTransport::Now, &DelayedTransport::Send,
       &DelayedTransport::Abort});
  ASSERT_EQ(core.Begin(), rmlink::Status::Ok);
  core.Poll();
  ASSERT_TRUE(core.has_in_flight_frame());
  const auto token = transport.accepted_token;
  core.NotifySendComplete(token, rmlink::CompletionResult::Failed);
  EXPECT_EQ(core.state(), rmlink::State::ResetRequired);
  EXPECT_EQ(core.statistics().transport_failures, 1U);
  EXPECT_EQ(transport.abort_calls, 1U);
  EXPECT_TRUE(core.has_in_flight_frame());
}

TEST(CoreHandshakeTest, ExchangesHelloAndPreservesEpochForDuplicate) {
  const std::array streams{
      rmlink::StreamRegistration{10, nullptr, &OnReceive}};
  DelayedTransport host_transport;
  DelayedTransport device_transport;
  device_transport.now_us = 9'000'000;
  auto host_config = MakeConfig(streams);
  auto device_config = MakeConfig(streams);
  device_config.role = rmlink::Role::Device;
  host_config.timing.sync_sample_count = 1;
  device_config.timing.sync_sample_count = 1;
  rmlink::Core host(
      host_config,
      {&host_transport, &DelayedTransport::Now, &DelayedTransport::Send,
       &DelayedTransport::Abort});
  rmlink::Core device(
      device_config,
      {&device_transport, &DelayedTransport::Now, &DelayedTransport::Send,
       &DelayedTransport::Abort});

  ASSERT_EQ(host.Begin(), rmlink::Status::Ok);
  host.Poll();
  ASSERT_FALSE(host_transport.snapshot.empty());
  const auto hello = host_transport.snapshot;
  host.NotifySendComplete(host_transport.accepted_token,
                          rmlink::CompletionResult::Complete);

  device.Receive(hello, device_transport.now_us);
  EXPECT_EQ(device.state(), rmlink::State::Syncing);
  EXPECT_EQ(device.session_id(), 1);
  device.Poll();
  ASSERT_FALSE(device_transport.snapshot.empty());
  const auto first_ack = device_transport.snapshot;
  EXPECT_EQ(DecodeHeader(first_ack).type, rmlink::MessageType::HelloAck);
  device.NotifySendComplete(device_transport.accepted_token,
                            rmlink::CompletionResult::Complete);

  host.Receive(first_ack, host_transport.now_us);
  EXPECT_EQ(host.state(), rmlink::State::Syncing);

  device_transport.now_us += 50'000;
  device.Receive(hello, device_transport.now_us);
  device.Poll();
  const auto duplicate_ack = device_transport.snapshot;
  ASSERT_GE(first_ack.size(), rmlink::kFrameOverhead);
  ASSERT_GE(duplicate_ack.size(), rmlink::kFrameOverhead);
  EXPECT_TRUE(std::equal(first_ack.begin() + rmlink::kHeaderSize,
                         first_ack.end() - 4,
                         duplicate_ack.begin() + rmlink::kHeaderSize));
  EXPECT_EQ(ReadU64(first_ack.data() + 24),
            ReadU64(duplicate_ack.data() + 24));
  device.NotifySendComplete(device_transport.accepted_token,
                            rmlink::CompletionResult::Complete);

  const auto competing = EncodeHello(1, 2, 7, 99, 1234);
  device.Receive(competing, device_transport.now_us);
  device.Poll();
  EXPECT_EQ(DecodeHeader(device_transport.snapshot).type,
            rmlink::MessageType::Reject);
  EXPECT_EQ(device.session_id(), 1);
}

TEST(CoreHandshakeTest, RejectsUnsupportedVersionWithoutTakingSession) {
  const std::array streams{
      rmlink::StreamRegistration{10, nullptr, &OnReceive}};
  DelayedTransport transport;
  auto config = MakeConfig(streams);
  config.role = rmlink::Role::Device;
  rmlink::Core device(
      config,
      {&transport, &DelayedTransport::Now, &DelayedTransport::Send,
       &DelayedTransport::Abort});

  const auto hello = EncodeHello(2, 4, 9, 123, 456);
  device.Receive(hello, transport.now_us);
  EXPECT_EQ(device.state(), rmlink::State::Idle);
  EXPECT_EQ(device.session_id(), 0);
  device.Poll();
  ASSERT_FALSE(transport.snapshot.empty());
  const auto header = DecodeHeader(transport.snapshot);
  EXPECT_EQ(header.type, rmlink::MessageType::Reject);
  EXPECT_EQ(header.session_id, 4);
  EXPECT_EQ(header.sequence, 9);
  EXPECT_EQ(header.timestamp_us, 0U);
}

TEST(CoreControlTransactionTest, IgnoresIncorrectCorrelation) {
  const std::array streams{
      rmlink::StreamRegistration{10, nullptr, &OnReceive}};
  DelayedTransport transport;
  rmlink::Core host(
      MakeConfig(streams),
      {&transport, &DelayedTransport::Now, &DelayedTransport::Send,
       &DelayedTransport::Abort});
  ASSERT_EQ(host.Begin(), rmlink::Status::Ok);
  host.Poll();
  ASSERT_GE(transport.snapshot.size(), 32U);
  const auto hello_header = DecodeHeader(transport.snapshot);
  const auto attempt_token = ReadU64(transport.snapshot.data() + 16);

  const auto wrong_sequence = EncodeHelloAck(
      transport.snapshot, hello_header.session_id,
      static_cast<std::uint8_t>(hello_header.sequence + 1), attempt_token);
  host.Receive(wrong_sequence, transport.now_us);
  EXPECT_EQ(host.state(), rmlink::State::Handshaking);

  const auto wrong_token =
      EncodeHelloAck(transport.snapshot, hello_header.session_id,
                     hello_header.sequence, attempt_token + 1);
  host.Receive(wrong_token, transport.now_us);
  EXPECT_EQ(host.state(), rmlink::State::Handshaking);
  EXPECT_EQ(host.statistics().session_semantic_errors, 2U);
}

TEST(CoreControlTransactionTest, AcceptsMatchingResponseBeforeSendCompletion) {
  const std::array streams{
      rmlink::StreamRegistration{10, nullptr, &OnReceive}};
  DelayedTransport transport;
  rmlink::Core host(
      MakeConfig(streams),
      {&transport, &DelayedTransport::Now, &DelayedTransport::Send,
       &DelayedTransport::Abort});
  ASSERT_EQ(host.Begin(), rmlink::Status::Ok);
  host.Poll();
  const auto hello_header = DecodeHeader(transport.snapshot);
  const auto attempt_token = ReadU64(transport.snapshot.data() + 16);
  const auto acknowledgement =
      EncodeHelloAck(transport.snapshot, hello_header.session_id,
                     hello_header.sequence, attempt_token);

  host.Receive(acknowledgement, transport.now_us);
  EXPECT_EQ(host.state(), rmlink::State::Handshaking);
  host.NotifySendComplete(transport.accepted_token,
                          rmlink::CompletionResult::Complete);
  host.Poll();
  EXPECT_EQ(host.state(), rmlink::State::Syncing);
}

TEST(CoreControlTransactionTest, RetriesTwiceThenRequiresReset) {
  const std::array streams{
      rmlink::StreamRegistration{10, nullptr, &OnReceive}};
  DelayedTransport transport;
  auto config = MakeConfig(streams);
  config.timing.control_response_timeout_ms = 1;
  rmlink::Core host(
      config,
      {&transport, &DelayedTransport::Now, &DelayedTransport::Send,
       &DelayedTransport::Abort});
  ASSERT_EQ(host.Begin(), rmlink::Status::Ok);

  for (std::size_t attempt = 0; attempt < 3; ++attempt) {
    host.Poll();
    ASSERT_TRUE(host.has_in_flight_frame());
    host.NotifySendComplete(transport.accepted_token,
                            rmlink::CompletionResult::Complete);
    transport.now_us += 1001;
  }
  host.Poll();
  EXPECT_EQ(transport.send_calls, 3U);
  EXPECT_EQ(host.statistics().control_retries, 2U);
  EXPECT_EQ(host.statistics().control_failures, 1U);
  EXPECT_EQ(host.state(), rmlink::State::ResetRequired);
  EXPECT_EQ(transport.abort_calls, 1U);
}

TEST(CoreControlTransactionTest, ContinuousBusyEndsAtTotalDeadline) {
  const std::array streams{
      rmlink::StreamRegistration{10, nullptr, &OnReceive}};
  BusyTransport transport;
  auto config = MakeConfig(streams);
  config.timing.control_response_timeout_ms = 1;
  config.timing.tx_completion_timeout_ms = 1;
  rmlink::Core host(
      config,
      {&transport, &BusyTransport::Now, &BusyTransport::Send,
       &BusyTransport::Abort});
  ASSERT_EQ(host.Begin(), rmlink::Status::Ok);
  host.Poll();
  transport.now_us += 6001;
  host.Poll();
  EXPECT_EQ(host.state(), rmlink::State::ResetRequired);
  EXPECT_EQ(host.statistics().control_failures, 1U);
  EXPECT_EQ(transport.abort_calls, 1U);
}

TEST(TimeContractTest, UsesFullClockAndRejectsOutOfRangeEvents) {
  std::uint32_t relative_us = 0;
  constexpr std::uint64_t epoch_us = 5'000'000'000ULL;
  EXPECT_EQ(rmlink::ToRelativeTimestamp(epoch_us, epoch_us + 100,
                                        epoch_us + 200, relative_us),
            rmlink::Status::Ok);
  EXPECT_EQ(relative_us, 100U);
  EXPECT_EQ(rmlink::ToRelativeTimestamp(epoch_us, epoch_us - 1,
                                        epoch_us + 200, relative_us),
            rmlink::Status::TimeOutOfRange);
  EXPECT_EQ(rmlink::ToRelativeTimestamp(epoch_us, epoch_us + 201,
                                        epoch_us + 200, relative_us),
            rmlink::Status::TimeOutOfRange);
  EXPECT_EQ(rmlink::ToRelativeTimestamp(epoch_us + 10, epoch_us + 9,
                                        epoch_us + 20, relative_us),
            rmlink::Status::TimeOutOfRange);
  EXPECT_EQ(rmlink::ToRelativeTimestamp(epoch_us, epoch_us,
                                        epoch_us +
                                            rmlink::kMaxSessionDurationUs + 1,
                                        relative_us),
            rmlink::Status::TimeOutOfRange);
}

TEST(CoreDataTest, CopiesPayloadAndDispatchesRegisteredStream) {
  ReceivedEvent received{};
  const std::array streams{
      rmlink::StreamRegistration{10, &received, &ReceivedEvent::OnData}};
  auto host_config = MakeConfig(streams);
  auto device_config = MakeConfig(streams);
  device_config.role = rmlink::Role::Device;
  host_config.timing.sync_sample_count = 1;
  device_config.timing.sync_sample_count = 1;
  LinkTransport host_transport{};
  LinkTransport device_transport{};
  host_transport.now_us = 1'000'000;
  device_transport.now_us = 2'000'000;
  rmlink::Core host(
      host_config,
      {&host_transport, &LinkTransport::Now, &LinkTransport::Send,
       &LinkTransport::Abort});
  rmlink::Core device(
      device_config,
      {&device_transport, &LinkTransport::Now, &LinkTransport::Send,
       &LinkTransport::Abort});
  ASSERT_TRUE(EstablishActive(host, device, host_transport, device_transport));

  std::array<std::byte, 3> payload{std::byte{1}, std::byte{2}, std::byte{3}};
  EXPECT_EQ(host.SubmitData(10, payload, host_transport.now_us),
            rmlink::Status::Ok);
  payload[0] = std::byte{9};
  host.Poll();
  ASSERT_TRUE(host.has_in_flight_frame());
  const auto encoded = host_transport.snapshot;
  const auto header = DecodeHeader(encoded);
  EXPECT_EQ(header.type, rmlink::MessageType::Data);
  EXPECT_EQ(header.sequence, 0);
  EXPECT_EQ(encoded[14], std::byte{1});
  EXPECT_EQ(encoded[15], std::byte{2});
  EXPECT_EQ(encoded[16], std::byte{3});
  host.NotifySendComplete(host_transport.accepted_token,
                          rmlink::CompletionResult::Complete);
  device.Receive(encoded, device_transport.now_us);
  ASSERT_EQ(received.count, 1U);
  EXPECT_EQ(received.metadata.stream_id, 10);
  EXPECT_EQ(received.metadata.relative_event_us, 0U);
  EXPECT_EQ(received.payload, (std::vector<std::byte>{std::byte{1},
                                                       std::byte{2},
                                                       std::byte{3}}));
}

TEST(CoreTypedTest, UsesActualEncodedLengthAndPreallocatedDecodeSlot) {
  ReceivedTyped received{};
  rmlink::TypedStream<TypedValue> typed_stream(42, &received,
                                               &ReceivedTyped::OnValue);
  const std::array registrations{typed_stream.registration()};
  auto host_config = MakeConfig(registrations);
  auto device_config = MakeConfig(registrations);
  device_config.role = rmlink::Role::Device;
  host_config.timing.sync_sample_count = 1;
  device_config.timing.sync_sample_count = 1;
  LinkTransport host_transport{};
  LinkTransport device_transport{};
  host_transport.now_us = 1'000'000;
  device_transport.now_us = 2'000'000;
  rmlink::Core host(
      host_config,
      {&host_transport, &LinkTransport::Now, &LinkTransport::Send,
       &LinkTransport::Abort});
  rmlink::Core device(
      device_config,
      {&device_transport, &LinkTransport::Now, &LinkTransport::Send,
       &LinkTransport::Abort});
  ASSERT_TRUE(EstablishActive(host, device, host_transport, device_transport));

  const TypedValue value{0x1234, false, false};
  EXPECT_EQ(host.Submit(42, value, host_transport.now_us), rmlink::Status::Ok);
  host.Poll();
  ASSERT_TRUE(host.has_in_flight_frame());
  const auto encoded = host_transport.snapshot;
  EXPECT_EQ(DecodeHeader(encoded).payload_length, 2);
  host.NotifySendComplete(host_transport.accepted_token,
                          rmlink::CompletionResult::Complete);
  device.Receive(encoded, device_transport.now_us);
  EXPECT_EQ(received.count, 1U);
  EXPECT_EQ(received.value, 0x1234);
  EXPECT_EQ(typed_stream.decode_errors(), 0U);

  const TypedValue failed{0x1234, true, false};
  EXPECT_EQ(host.Submit(42, failed, host_transport.now_us),
            rmlink::Status::EncodeFailed);
  const std::array<std::byte, 3> trailing_payload{
      std::byte{0x34}, std::byte{0x12}, std::byte{0xFF}};
  const auto trailing = EncodeData(1, 42, 2, 0, trailing_payload);
  device.Receive(trailing, device_transport.now_us);
  EXPECT_EQ(typed_stream.decode_errors(), 1U);
  EXPECT_EQ(received.count, 1U);
  const std::array<std::byte, 1> short_payload{std::byte{0x34}};
  const auto invalid = EncodeData(1, 42, 3, 0, short_payload);
  device.Receive(invalid, device_transport.now_us);
  EXPECT_EQ(typed_stream.decode_errors(), 2U);
}

TEST(CoreDataTest, RejectsUnregisteredAndDisabledDataSubmission) {
  const std::array streams{
      rmlink::StreamRegistration{10, nullptr, &OnReceive}};
  auto host_config = MakeConfig(streams);
  auto device_config = MakeConfig(streams);
  device_config.role = rmlink::Role::Device;
  host_config.timing.sync_sample_count = 1;
  device_config.timing.sync_sample_count = 1;
  LinkTransport host_transport{};
  LinkTransport device_transport{};
  host_transport.now_us = 1'000'000;
  device_transport.now_us = 2'000'000;
  rmlink::Core host(
      host_config,
      {&host_transport, &LinkTransport::Now, &LinkTransport::Send,
       &LinkTransport::Abort});
  rmlink::Core device(
      device_config,
      {&device_transport, &LinkTransport::Now, &LinkTransport::Send,
       &LinkTransport::Abort});
  EXPECT_EQ(host.SubmitData(10, {}, host_transport.now_us),
            rmlink::Status::InvalidState);
  ASSERT_TRUE(EstablishActive(host, device, host_transport, device_transport));
  EXPECT_EQ(host.SubmitData(99, {}, host_transport.now_us),
            rmlink::Status::UnknownStream);
}

TEST(CoreQueueTest, DropsOldestQueuedDataAndPreservesSequenceGaps) {
  const std::array streams{
      rmlink::StreamRegistration{10, nullptr, &OnReceive}};
  auto host_config = MakeConfig(streams);
  auto device_config = MakeConfig(streams);
  device_config.role = rmlink::Role::Device;
  host_config.timing.sync_sample_count = 1;
  device_config.timing.sync_sample_count = 1;
  host_config.realtime_queue_depth = 2;
  host_config.realtime_rate_limit_fps = 500;
  LinkTransport host_transport{};
  LinkTransport device_transport{};
  host_transport.now_us = 1'000'000;
  device_transport.now_us = 2'000'000;
  rmlink::Core host(
      host_config,
      {&host_transport, &LinkTransport::Now, &LinkTransport::Send,
       &LinkTransport::Abort});
  rmlink::Core device(
      device_config,
      {&device_transport, &LinkTransport::Now, &LinkTransport::Send,
       &LinkTransport::Abort});
  ASSERT_TRUE(EstablishActive(host, device, host_transport, device_transport));

  const std::array<std::byte, 1> first{std::byte{1}};
  const std::array<std::byte, 1> second{std::byte{2}};
  const std::array<std::byte, 1> third{std::byte{3}};
  EXPECT_EQ(host.SubmitData(10, first, host_transport.now_us),
            rmlink::Status::Ok);
  EXPECT_EQ(host.SubmitData(10, second, host_transport.now_us),
            rmlink::Status::Ok);
  EXPECT_EQ(host.SubmitData(10, third, host_transport.now_us),
            rmlink::Status::Ok);
  EXPECT_EQ(host.statistics().dropped_realtime, 1U);

  host.Poll();
  ASSERT_TRUE(host.has_in_flight_frame());
  EXPECT_EQ(DecodeHeader(host_transport.snapshot).sequence, 1);
  host.NotifySendComplete(host_transport.accepted_token,
                          rmlink::CompletionResult::Complete);
  host_transport.now_us += 2'000;
  host.Poll();
  ASSERT_TRUE(host.has_in_flight_frame());
  EXPECT_EQ(DecodeHeader(host_transport.snapshot).sequence, 2);
}

TEST(CoreQueueTest, DefaultDataAndLogDepthsAreEightAndFour) {
  const std::array streams{
      rmlink::StreamRegistration{10, nullptr, &OnReceive}};
  auto host_config = MakeConfig(streams);
  auto device_config = MakeConfig(streams);
  device_config.role = rmlink::Role::Device;
  host_config.timing.sync_sample_count = 1;
  device_config.timing.sync_sample_count = 1;
  LinkTransport host_transport{};
  LinkTransport device_transport{};
  host_transport.now_us = 1'000'000;
  device_transport.now_us = 2'000'000;
  rmlink::Core host(
      host_config,
      {&host_transport, &LinkTransport::Now, &LinkTransport::Send,
       &LinkTransport::Abort});
  rmlink::Core device(
      device_config,
      {&device_transport, &LinkTransport::Now, &LinkTransport::Send,
       &LinkTransport::Abort});
  ASSERT_TRUE(EstablishActive(host, device, host_transport, device_transport));
  for (std::size_t index = 0; index < 9; ++index) {
    EXPECT_EQ(host.SubmitData(10, {}, host_transport.now_us),
              rmlink::Status::Ok);
  }
  for (std::size_t index = 0; index < 5; ++index) {
    EXPECT_EQ(device.SubmitLog(2, {}), rmlink::Status::Ok);
  }
  EXPECT_EQ(host.statistics().dropped_realtime, 1U);
  EXPECT_EQ(device.statistics().dropped_logs, 1U);
}

TEST(CoreQueueTest, EnforcesAggregateRateInterval) {
  const std::array streams{
      rmlink::StreamRegistration{10, nullptr, &OnReceive}};
  auto host_config = MakeConfig(streams);
  auto device_config = MakeConfig(streams);
  device_config.role = rmlink::Role::Device;
  host_config.timing.sync_sample_count = 1;
  device_config.timing.sync_sample_count = 1;
  host_config.realtime_rate_limit_fps = 1000;
  LinkTransport host_transport{};
  LinkTransport device_transport{};
  host_transport.now_us = 1'000'000;
  device_transport.now_us = 2'000'000;
  rmlink::Core host(
      host_config,
      {&host_transport, &LinkTransport::Now, &LinkTransport::Send,
       &LinkTransport::Abort});
  rmlink::Core device(
      device_config,
      {&device_transport, &LinkTransport::Now, &LinkTransport::Send,
       &LinkTransport::Abort});
  ASSERT_TRUE(EstablishActive(host, device, host_transport, device_transport));

  EXPECT_EQ(host.SubmitData(10, {}, host_transport.now_us),
            rmlink::Status::Ok);
  host_transport.now_us += 1;
  EXPECT_EQ(host.SubmitData(10, {}, host_transport.now_us),
            rmlink::Status::Ok);
  host.Poll();
  ASSERT_TRUE(host.has_in_flight_frame());
  host.NotifySendComplete(host_transport.accepted_token,
                          rmlink::CompletionResult::Complete);
  host_transport.now_us += 999;
  host.Poll();
  EXPECT_FALSE(host.has_in_flight_frame());
  host_transport.now_us += 1;
  host.Poll();
  EXPECT_TRUE(host.has_in_flight_frame());
}

TEST(CoreQueueTest, OneFpsDoesNotAccumulateCatchUpCredit) {
  const std::array streams{
      rmlink::StreamRegistration{10, nullptr, &OnReceive}};
  auto host_config = MakeConfig(streams);
  auto device_config = MakeConfig(streams);
  device_config.role = rmlink::Role::Device;
  host_config.timing.sync_sample_count = 1;
  device_config.timing.sync_sample_count = 1;
  host_config.timing.sync_period_ms = 100'000;
  host_config.timing.rx_liveness_timeout_ms = 100'000;
  host_config.realtime_rate_limit_fps = 1;
  LinkTransport host_transport{};
  LinkTransport device_transport{};
  host_transport.now_us = 1'000'000;
  device_transport.now_us = 2'000'000;
  rmlink::Core host(
      host_config,
      {&host_transport, &LinkTransport::Now, &LinkTransport::Send,
       &LinkTransport::Abort});
  rmlink::Core device(
      device_config,
      {&device_transport, &LinkTransport::Now, &LinkTransport::Send,
       &LinkTransport::Abort});
  ASSERT_TRUE(EstablishActive(host, device, host_transport, device_transport));
  EXPECT_EQ(host.SubmitData(10, {}, host_transport.now_us), rmlink::Status::Ok);
  EXPECT_EQ(host.SubmitData(10, {}, host_transport.now_us), rmlink::Status::Ok);
  host_transport.now_us += 10'000'000;
  host.Poll();
  ASSERT_TRUE(host.has_in_flight_frame());
  host.NotifySendComplete(host_transport.accepted_token,
                          rmlink::CompletionResult::Complete);
  host.Poll();
  EXPECT_FALSE(host.has_in_flight_frame());
  host_transport.now_us += 999'999;
  host.Poll();
  EXPECT_FALSE(host.has_in_flight_frame());
  host_transport.now_us += 1;
  host.Poll();
  EXPECT_TRUE(host.has_in_flight_frame());
}

TEST(CoreQueueTest, MultipleStreamsShareOneFiveHundredFpsLimit) {
  const std::array streams{
      rmlink::StreamRegistration{10, nullptr, &OnReceive},
      rmlink::StreamRegistration{11, nullptr, &OnReceive}};
  auto host_config = MakeConfig(streams);
  auto device_config = MakeConfig(streams);
  device_config.role = rmlink::Role::Device;
  host_config.timing.sync_sample_count = 1;
  device_config.timing.sync_sample_count = 1;
  host_config.realtime_rate_limit_fps = 500;
  LinkTransport host_transport{};
  LinkTransport device_transport{};
  host_transport.now_us = 1'000'000;
  device_transport.now_us = 2'000'000;
  rmlink::Core host(
      host_config,
      {&host_transport, &LinkTransport::Now, &LinkTransport::Send,
       &LinkTransport::Abort});
  rmlink::Core device(
      device_config,
      {&device_transport, &LinkTransport::Now, &LinkTransport::Send,
       &LinkTransport::Abort});
  ASSERT_TRUE(EstablishActive(host, device, host_transport, device_transport));
  EXPECT_EQ(host.SubmitData(10, {}, host_transport.now_us), rmlink::Status::Ok);
  EXPECT_EQ(host.SubmitData(11, {}, host_transport.now_us), rmlink::Status::Ok);
  host.Poll();
  ASSERT_TRUE(host.has_in_flight_frame());
  EXPECT_EQ(DecodeHeader(host_transport.snapshot).stream_id, 10);
  host.NotifySendComplete(host_transport.accepted_token,
                          rmlink::CompletionResult::Complete);
  host_transport.now_us += 1'999;
  host.Poll();
  EXPECT_FALSE(host.has_in_flight_frame());
  host_transport.now_us += 1;
  host.Poll();
  ASSERT_TRUE(host.has_in_flight_frame());
  EXPECT_EQ(DecodeHeader(host_transport.snapshot).stream_id, 11);
}

TEST(CoreQueueTest, ZeroFpsRejectsDataWithoutCreatingFrames) {
  const std::array streams{
      rmlink::StreamRegistration{10, nullptr, &OnReceive}};
  auto host_config = MakeConfig(streams);
  auto device_config = MakeConfig(streams);
  device_config.role = rmlink::Role::Device;
  host_config.timing.sync_sample_count = 1;
  device_config.timing.sync_sample_count = 1;
  host_config.timing.sync_period_ms = 100'000;
  host_config.realtime_rate_limit_fps = 0;
  LinkTransport host_transport{};
  LinkTransport device_transport{};
  host_transport.now_us = 1'000'000;
  device_transport.now_us = 2'000'000;
  rmlink::Core host(
      host_config,
      {&host_transport, &LinkTransport::Now, &LinkTransport::Send,
       &LinkTransport::Abort});
  rmlink::Core device(
      device_config,
      {&device_transport, &LinkTransport::Now, &LinkTransport::Send,
       &LinkTransport::Abort});
  ASSERT_TRUE(EstablishActive(host, device, host_transport, device_transport));
  const auto sends_before = host_transport.send_calls;
  EXPECT_EQ(host.SubmitData(10, {}, host_transport.now_us),
            rmlink::Status::RateDisabled);
  host.Poll();
  EXPECT_EQ(host_transport.send_calls, sends_before);
  device_transport.now_us += 3'000'001;
  device.Poll();
  EXPECT_EQ(device.state(), rmlink::State::ResetRequired);
  EXPECT_EQ(device.statistics().heartbeat_timeouts, 1U);
}

TEST(CoreLogTest, TruncatesAtUtf8BoundaryAndDeliversOnlyFromDevice) {
  ReceivedEvent received{};
  ReceivedLog received_log{};
  const std::array streams{
      rmlink::StreamRegistration{10, &received, &ReceivedEvent::OnData}};
  auto host_config = MakeConfig(streams);
  auto device_config = MakeConfig(streams);
  device_config.role = rmlink::Role::Device;
  device_config.log_budget_bytes_per_second = 4096;
  host_config.timing.sync_sample_count = 1;
  device_config.timing.sync_sample_count = 1;
  host_config.log_context = &received_log;
  host_config.on_log = &ReceivedLog::OnLog;
  LinkTransport host_transport{};
  LinkTransport device_transport{};
  host_transport.now_us = 1'000'000;
  device_transport.now_us = 2'000'000;
  rmlink::Core host(
      host_config,
      {&host_transport, &LinkTransport::Now, &LinkTransport::Send,
       &LinkTransport::Abort});
  rmlink::Core device(
      device_config,
      {&device_transport, &LinkTransport::Now, &LinkTransport::Send,
       &LinkTransport::Abort});
  ASSERT_TRUE(EstablishActive(host, device, host_transport, device_transport));

  const std::array<std::byte, 2> invalid_utf8{std::byte{0xC3},
                                                std::byte{0x28}};
  EXPECT_EQ(device.SubmitLog(2, invalid_utf8), rmlink::Status::InvalidArgument);
  EXPECT_EQ(host.SubmitLog(2, {}), rmlink::Status::WrongRole);

  std::vector<std::byte> long_text(252, std::byte{'a'});
  long_text.push_back(std::byte{0xC3});
  long_text.push_back(std::byte{0xA9});
  long_text.push_back(std::byte{'z'});
  EXPECT_EQ(device.SubmitLog(2, long_text), rmlink::Status::Ok);
  device.Poll();
  ASSERT_TRUE(device.has_in_flight_frame());
  const auto encoded = device_transport.snapshot;
  const auto header = DecodeHeader(encoded);
  EXPECT_EQ(header.type, rmlink::MessageType::Log);
  EXPECT_EQ(std::to_integer<std::uint8_t>(encoded[15]), 1U);
  EXPECT_LE(header.payload_length, 255);
  EXPECT_TRUE(rmlink::wire::IsValidUtf8(
      std::span<const std::byte>(encoded).subspan(16, header.payload_length - 2)));
  device.NotifySendComplete(device_transport.accepted_token,
                            rmlink::CompletionResult::Complete);
  host.Receive(encoded, host_transport.now_us);
  EXPECT_EQ(host.statistics().received_log_frames, 1U);
  EXPECT_EQ(host.statistics().heartbeat_timeouts, 0U);
  ASSERT_EQ(received_log.count, 1U);
  EXPECT_EQ(received_log.level, 2);
  EXPECT_EQ(received_log.flags, 1);
  EXPECT_EQ(received_log.text.size(), 252U);
  EXPECT_EQ(received_log.metadata.device_event_us, 2'000'000U);
}

TEST(CoreSessionTest, UnknownDataRefreshesHeartbeatBeforeTimeout) {
  const std::array streams{
      rmlink::StreamRegistration{10, nullptr, &OnReceive}};
  auto host_config = MakeConfig(streams);
  auto device_config = MakeConfig(streams);
  device_config.role = rmlink::Role::Device;
  host_config.timing.sync_sample_count = 1;
  device_config.timing.sync_sample_count = 1;
  host_config.timing.sync_period_ms = 100'000;
  LinkTransport host_transport{};
  LinkTransport device_transport{};
  host_transport.now_us = 1'000'000;
  device_transport.now_us = 2'000'000;
  rmlink::Core host(
      host_config,
      {&host_transport, &LinkTransport::Now, &LinkTransport::Send,
       &LinkTransport::Abort});
  rmlink::Core device(
      device_config,
      {&device_transport, &LinkTransport::Now, &LinkTransport::Send,
       &LinkTransport::Abort});
  ASSERT_TRUE(EstablishActive(host, device, host_transport, device_transport));

  const auto unknown = EncodeData(1, 99, 0, 5);
  host.Receive(unknown, host_transport.now_us);
  EXPECT_EQ(host.statistics().unknown_streams, 1U);
  host_transport.now_us += 2'999'000;
  host.Poll();
  EXPECT_EQ(host.state(), rmlink::State::Active);
  host_transport.now_us += 2'000;
  host.Poll();
  EXPECT_EQ(host.state(), rmlink::State::ResetRequired);
  EXPECT_EQ(host.statistics().heartbeat_timeouts, 1U);
}

TEST(CoreSessionTest, OldBadAndSyncFramesDoNotRefreshHeartbeat) {
  const std::array streams{
      rmlink::StreamRegistration{10, nullptr, &OnReceive}};
  auto host_config = MakeConfig(streams);
  auto device_config = MakeConfig(streams);
  device_config.role = rmlink::Role::Device;
  host_config.timing.sync_sample_count = 1;
  device_config.timing.sync_sample_count = 1;
  host_config.timing.sync_period_ms = 100'000;
  LinkTransport host_transport{};
  LinkTransport device_transport{};
  host_transport.now_us = 1'000'000;
  device_transport.now_us = 2'000'000;
  rmlink::Core host(
      host_config,
      {&host_transport, &LinkTransport::Now, &LinkTransport::Send,
       &LinkTransport::Abort});
  rmlink::Core device(
      device_config,
      {&device_transport, &LinkTransport::Now, &LinkTransport::Send,
       &LinkTransport::Abort});
  ASSERT_TRUE(EstablishActive(host, device, host_transport, device_transport));

  const auto old_session = EncodeData(2, 10, 0, 10);
  host.Receive(old_session, host_transport.now_us);
  auto bad = EncodeData(1, 10, 0, 10);
  bad[14] ^= std::byte{1};
  host.Receive(bad, host_transport.now_us);
  const auto unsolicited_sync = EncodeSyncResponse(1, 99, 1, 2, 3);
  host.Receive(unsolicited_sync, host_transport.now_us);
  host_transport.now_us += 3'000'001;
  host.Poll();
  EXPECT_EQ(host.state(), rmlink::State::ResetRequired);
  EXPECT_EQ(host.statistics().heartbeat_timeouts, 1U);
}

TEST(CoreCloseTest, WaitsForInFlightFrameAndAcknowledgesDuplicateClose) {
  const std::array streams{
      rmlink::StreamRegistration{10, nullptr, &OnReceive}};
  auto host_config = MakeConfig(streams);
  auto device_config = MakeConfig(streams);
  device_config.role = rmlink::Role::Device;
  host_config.timing.sync_sample_count = 1;
  device_config.timing.sync_sample_count = 1;
  LinkTransport host_transport{};
  LinkTransport device_transport{};
  host_transport.now_us = 1'000'000;
  device_transport.now_us = 2'000'000;
  rmlink::Core host(
      host_config,
      {&host_transport, &LinkTransport::Now, &LinkTransport::Send,
       &LinkTransport::Abort});
  rmlink::Core device(
      device_config,
      {&device_transport, &LinkTransport::Now, &LinkTransport::Send,
       &LinkTransport::Abort});
  ASSERT_TRUE(EstablishActive(host, device, host_transport, device_transport));

  EXPECT_EQ(host.SubmitData(10, {}, host_transport.now_us), rmlink::Status::Ok);
  host.Poll();
  ASSERT_TRUE(host.has_in_flight_frame());
  EXPECT_EQ(DecodeHeader(host_transport.snapshot).type,
            rmlink::MessageType::Data);
  EXPECT_EQ(host.SubmitData(10, {}, host_transport.now_us), rmlink::Status::Ok);
  EXPECT_EQ(host.Close(), rmlink::Status::Ok);
  EXPECT_EQ(host.state(), rmlink::State::Closing);
  host.Poll();
  EXPECT_EQ(DecodeHeader(host_transport.snapshot).type,
            rmlink::MessageType::Data);

  host.NotifySendComplete(host_transport.accepted_token,
                          rmlink::CompletionResult::Complete);
  host.Poll();
  ASSERT_TRUE(host.has_in_flight_frame());
  const auto close = host_transport.snapshot;
  EXPECT_EQ(DecodeHeader(close).type, rmlink::MessageType::Close);
  device.Receive(close, device_transport.now_us);
  EXPECT_EQ(device.state(), rmlink::State::Closing);
  device.Poll();
  ASSERT_TRUE(device.has_in_flight_frame());
  const auto close_ack = device_transport.snapshot;
  EXPECT_EQ(DecodeHeader(close_ack).type, rmlink::MessageType::CloseAck);
  host.NotifySendComplete(host_transport.accepted_token,
                          rmlink::CompletionResult::Complete);
  host.Receive(close_ack, host_transport.now_us);
  device.NotifySendComplete(device_transport.accepted_token,
                            rmlink::CompletionResult::Complete);
  EXPECT_EQ(host.state(), rmlink::State::Idle);
  EXPECT_EQ(device.state(), rmlink::State::Idle);

  const auto device_sends = device_transport.send_calls;
  device.Receive(close, device_transport.now_us + 1000);
  device.Poll();
  ASSERT_TRUE(device.has_in_flight_frame());
  EXPECT_EQ(device_transport.send_calls, device_sends + 1);
  EXPECT_EQ(DecodeHeader(device_transport.snapshot).type,
            rmlink::MessageType::CloseAck);
}

TEST(CoreCloseTest, SimultaneousCloseRespondsInBothDirections) {
  const std::array streams{
      rmlink::StreamRegistration{10, nullptr, &OnReceive}};
  auto host_config = MakeConfig(streams);
  auto device_config = MakeConfig(streams);
  device_config.role = rmlink::Role::Device;
  host_config.timing.sync_sample_count = 1;
  device_config.timing.sync_sample_count = 1;
  LinkTransport host_transport{};
  LinkTransport device_transport{};
  host_transport.now_us = 1'000'000;
  device_transport.now_us = 2'000'000;
  rmlink::Core host(
      host_config,
      {&host_transport, &LinkTransport::Now, &LinkTransport::Send,
       &LinkTransport::Abort});
  rmlink::Core device(
      device_config,
      {&device_transport, &LinkTransport::Now, &LinkTransport::Send,
       &LinkTransport::Abort});
  ASSERT_TRUE(EstablishActive(host, device, host_transport, device_transport));
  ASSERT_EQ(host.Close(), rmlink::Status::Ok);
  ASSERT_EQ(device.Close(), rmlink::Status::Ok);
  host.Poll();
  device.Poll();
  const auto host_close = host_transport.snapshot;
  const auto device_close = device_transport.snapshot;
  host.NotifySendComplete(host_transport.accepted_token,
                          rmlink::CompletionResult::Complete);
  device.NotifySendComplete(device_transport.accepted_token,
                            rmlink::CompletionResult::Complete);
  host.Receive(device_close, host_transport.now_us);
  device.Receive(host_close, device_transport.now_us);
  host.Poll();
  device.Poll();
  EXPECT_EQ(DecodeHeader(host_transport.snapshot).type,
            rmlink::MessageType::CloseAck);
  EXPECT_EQ(DecodeHeader(device_transport.snapshot).type,
            rmlink::MessageType::CloseAck);
  const auto host_ack = host_transport.snapshot;
  const auto device_ack = device_transport.snapshot;
  host.NotifySendComplete(host_transport.accepted_token,
                          rmlink::CompletionResult::Complete);
  device.NotifySendComplete(device_transport.accepted_token,
                            rmlink::CompletionResult::Complete);
  host.Receive(device_ack, host_transport.now_us);
  device.Receive(host_ack, device_transport.now_us);
  EXPECT_EQ(host.state(), rmlink::State::Idle);
  EXPECT_EQ(device.state(), rmlink::State::Idle);
}

TEST(CoreStatisticsTest, CountersSurviveAbnormalCleanupAndReconnect) {
  const std::array streams{
      rmlink::StreamRegistration{10, nullptr, &OnReceive}};
  auto host_config = MakeConfig(streams);
  auto device_config = MakeConfig(streams);
  device_config.role = rmlink::Role::Device;
  host_config.timing.sync_sample_count = 1;
  device_config.timing.sync_sample_count = 1;
  host_config.realtime_queue_depth = 1;
  LinkTransport host_transport{};
  LinkTransport device_transport{};
  host_transport.now_us = 1'000'000;
  device_transport.now_us = 2'000'000;
  rmlink::Core host(
      host_config,
      {&host_transport, &LinkTransport::Now, &LinkTransport::Send,
       &LinkTransport::Abort});
  rmlink::Core device(
      device_config,
      {&device_transport, &LinkTransport::Now, &LinkTransport::Send,
       &LinkTransport::Abort});
  ASSERT_TRUE(EstablishActive(host, device, host_transport, device_transport));
  EXPECT_EQ(host.SubmitData(10, {}, host_transport.now_us), rmlink::Status::Ok);
  EXPECT_EQ(host.SubmitData(10, {}, host_transport.now_us), rmlink::Status::Ok);
  auto bad = EncodeData(1, 10, 0, 0);
  bad.back() ^= std::byte{1};
  host.Receive(bad, host_transport.now_us);
  EXPECT_EQ(host.statistics().dropped_realtime, 1U);
  EXPECT_EQ(host.parse_statistics().frame_crc_errors, 1U);
  EXPECT_EQ(host.time_mapping().state, rmlink::MappingState::Valid);

  host.NotifyTransportFailure();
  EXPECT_EQ(host.state(), rmlink::State::ResetRequired);
  host.NotifyTransportAborted();
  EXPECT_EQ(host.state(), rmlink::State::Idle);
  EXPECT_EQ(host.statistics().dropped_realtime, 1U);
  EXPECT_EQ(host.statistics().transport_failures, 1U);
  EXPECT_EQ(host.parse_statistics().frame_crc_errors, 1U);
  EXPECT_EQ(host.time_mapping().state, rmlink::MappingState::Unavailable);
  EXPECT_EQ(host.Begin(), rmlink::Status::Ok);
  EXPECT_EQ(host.session_id(), 2);
}

TEST(CoreMemoryTest, RuntimePathsDoNotAllocateAfterInitialization) {
  const std::array streams{
      rmlink::StreamRegistration{10, nullptr, &OnReceive}};
  auto host_config = MakeConfig(streams);
  auto device_config = MakeConfig(streams);
  device_config.role = rmlink::Role::Device;
  host_config.timing.sync_sample_count = 1;
  device_config.timing.sync_sample_count = 1;
  FixedTransport host_transport{};
  FixedTransport device_transport{};
  host_transport.now_us = 1'000'000;
  device_transport.now_us = 2'000'000;
  rmlink::Core host(
      host_config,
      {&host_transport, &FixedTransport::Now, &FixedTransport::Send,
       &FixedTransport::Abort});
  rmlink::Core device(
      device_config,
      {&device_transport, &FixedTransport::Now, &FixedTransport::Send,
       &FixedTransport::Abort});
  ASSERT_TRUE(
      EstablishActiveFixed(host, device, host_transport, device_transport));
  auto bad = EncodeData(1, 10, 0, 0);
  bad.back() ^= std::byte{1};

  allocation_guard::count.store(0, std::memory_order_relaxed);
  allocation_guard::enabled.store(true, std::memory_order_relaxed);
  for (std::size_t index = 0; index < 9; ++index) {
    (void)host.SubmitData(10, {}, host_transport.now_us);
  }
  host.Poll();
  device.Receive(host_transport.bytes(), device_transport.now_us);
  host.NotifySendComplete(host_transport.accepted_token,
                          rmlink::CompletionResult::Complete);
  (void)device.SubmitLog(2, {});
  device.Poll();
  host.Receive(device_transport.bytes(), host_transport.now_us);
  device.NotifySendComplete(device_transport.accepted_token,
                            rmlink::CompletionResult::Complete);
  host.Receive(bad, host_transport.now_us);
  (void)host.Close();
  host.Poll();
  device.Receive(host_transport.bytes(), device_transport.now_us);
  host.NotifySendComplete(host_transport.accepted_token,
                          rmlink::CompletionResult::Complete);
  device.Poll();
  host.Receive(device_transport.bytes(), host_transport.now_us);
  device.NotifySendComplete(device_transport.accepted_token,
                            rmlink::CompletionResult::Complete);
  (void)host.Begin();
  host.Poll();
  allocation_guard::enabled.store(false, std::memory_order_relaxed);

  EXPECT_EQ(allocation_guard::count.load(std::memory_order_relaxed), 0U);
  EXPECT_GT(host.fixed_memory_bytes(), sizeof(host));
  EXPECT_GT(device.fixed_memory_bytes(), sizeof(device));
}

TEST(CoreStressTest, SimulatesBidirectionalThousandFpsWithLogs) {
  const std::array streams{
      rmlink::StreamRegistration{10, nullptr, &OnReceive}};
  auto host_config = MakeConfig(streams);
  auto device_config = MakeConfig(streams);
  device_config.role = rmlink::Role::Device;
  host_config.timing.sync_sample_count = 1;
  device_config.timing.sync_sample_count = 1;
  host_config.timing.sync_period_ms = 100'000;
  device_config.timing.sync_period_ms = 100'000;
  host_config.realtime_rate_limit_fps = 1000;
  device_config.realtime_rate_limit_fps = 1000;
  host_config.timing.rx_liveness_timeout_ms = 10'000;
  device_config.timing.rx_liveness_timeout_ms = 10'000;
  FixedTransport host_transport{};
  FixedTransport device_transport{};
  host_transport.now_us = 1'000'000;
  device_transport.now_us = 2'000'000;
  rmlink::Core host(
      host_config,
      {&host_transport, &FixedTransport::Now, &FixedTransport::Send,
       &FixedTransport::Abort});
  rmlink::Core device(
      device_config,
      {&device_transport, &FixedTransport::Now, &FixedTransport::Send,
       &FixedTransport::Abort});
  ASSERT_TRUE(
      EstablishActiveFixed(host, device, host_transport, device_transport));

  std::array<std::byte, 255> payload{};
  payload.fill(std::byte{0xA5});
  const std::array<std::byte, 3> log_text{
      std::byte{'l'}, std::byte{'o'}, std::byte{'g'}};
  for (std::size_t frame = 0; frame < 1000; ++frame) {
    host_transport.now_us += 1000;
    device_transport.now_us += 1000;
    ASSERT_EQ(host.SubmitData(10, payload, host_transport.now_us),
              rmlink::Status::Ok);
    ASSERT_EQ(device.SubmitData(10, payload, device_transport.now_us),
              rmlink::Status::Ok);
    if (frame % 50 == 0) {
      ASSERT_EQ(device.SubmitLog(2, log_text), rmlink::Status::Ok);
    }
    host.Poll();
    device.Poll();
    ASSERT_EQ(host_transport.frame_size, rmlink::kMaxFrameSize);
    ASSERT_EQ(device_transport.frame_size, rmlink::kMaxFrameSize);
    device.Receive(host_transport.bytes(), device_transport.now_us);
    host.Receive(device_transport.bytes(), host_transport.now_us);
    host.NotifySendComplete(host_transport.accepted_token,
                            rmlink::CompletionResult::Complete);
    device.NotifySendComplete(device_transport.accepted_token,
                              rmlink::CompletionResult::Complete);
  }
  EXPECT_EQ(host.statistics().received_data_frames, 1000U);
  EXPECT_EQ(device.statistics().received_data_frames, 1000U);
  EXPECT_GT(device.statistics().dropped_logs, 0U);
  EXPECT_EQ(host.statistics().heartbeat_timeouts, 0U);
  EXPECT_EQ(device.statistics().heartbeat_timeouts, 0U);
  EXPECT_EQ(host.state(), rmlink::State::Active);
  EXPECT_EQ(device.state(), rmlink::State::Active);
}

TEST(CorePriorityTest, RealTimeDataPrecedesLogsAndBudgetCanWait) {
  const std::array streams{
      rmlink::StreamRegistration{10, nullptr, &OnReceive}};
  auto host_config = MakeConfig(streams);
  auto device_config = MakeConfig(streams);
  device_config.role = rmlink::Role::Device;
  host_config.timing.sync_sample_count = 1;
  device_config.timing.sync_sample_count = 1;
  device_config.log_budget_bytes_per_second = 1;
  LinkTransport host_transport{};
  LinkTransport device_transport{};
  host_transport.now_us = 1'000'000;
  device_transport.now_us = 2'000'000;
  rmlink::Core host(
      host_config,
      {&host_transport, &LinkTransport::Now, &LinkTransport::Send,
       &LinkTransport::Abort});
  rmlink::Core device(
      device_config,
      {&device_transport, &LinkTransport::Now, &LinkTransport::Send,
       &LinkTransport::Abort});
  ASSERT_TRUE(EstablishActive(host, device, host_transport, device_transport));

  const std::array<std::byte, 1> data{std::byte{7}};
  const std::array<std::byte, 1> log_text{std::byte{'x'}};
  EXPECT_EQ(device.SubmitLog(2, log_text), rmlink::Status::Ok);
  EXPECT_EQ(device.SubmitData(10, data, device_transport.now_us),
            rmlink::Status::Ok);
  const auto sync_request =
      EncodeSyncRequest(1, 77, host_transport.now_us);
  device.Receive(sync_request, device_transport.now_us);
  device.Poll();
  ASSERT_TRUE(device.has_in_flight_frame());
  EXPECT_EQ(DecodeHeader(device_transport.snapshot).type,
            rmlink::MessageType::SyncResp);
  device.NotifySendComplete(device_transport.accepted_token,
                            rmlink::CompletionResult::Complete);
  device.Poll();
  ASSERT_TRUE(device.has_in_flight_frame());
  EXPECT_EQ(DecodeHeader(device_transport.snapshot).type,
            rmlink::MessageType::Data);
  device.NotifySendComplete(device_transport.accepted_token,
                            rmlink::CompletionResult::Complete);
  device_transport.now_us += 2'000;
  device.Poll();
  EXPECT_FALSE(device.has_in_flight_frame());
  EXPECT_GT(device.statistics().log_budget_waits, 0U);
}

TEST(TimeContractTest, CalculatesSignedOffsetAndRejectsNegativeRtt) {
  rmlink::TimeSample sample{};
  EXPECT_EQ(rmlink::CalculateTimeSample(1000, 2500, 3500, 2000, sample),
            rmlink::Status::Ok);
  EXPECT_EQ(sample.device_minus_host_us, 1500);
  EXPECT_EQ(sample.round_trip_us, 0U);

  EXPECT_EQ(rmlink::CalculateTimeSample(1000, 500, 1500, 2000, sample),
            rmlink::Status::Ok);
  EXPECT_EQ(sample.device_minus_host_us, -500);
  EXPECT_EQ(sample.round_trip_us, 0U);

  EXPECT_EQ(rmlink::CalculateTimeSample(1000, 2500, 3500, 1500, sample),
            rmlink::Status::TimeOutOfRange);
}

TEST(CoreSessionTest, CompletesInitialSyncBeforeStartingAndDropsEarlyData) {
  ReceivedEvent received_event{};
  const std::array streams{
      rmlink::StreamRegistration{10, &received_event, &ReceivedEvent::OnData}};
  auto host_config = MakeConfig(streams);
  auto device_config = MakeConfig(streams);
  device_config.role = rmlink::Role::Device;
  host_config.timing.sync_sample_count = 8;
  device_config.timing.sync_sample_count = 8;
  host_config.timing.rx_liveness_timeout_ms = 10'000;
  LinkTransport host_transport{};
  LinkTransport device_transport{};
  host_transport.now_us = 1'000'000;
  device_transport.now_us = 2'000'000;
  rmlink::Core host(
      host_config,
      {&host_transport, &LinkTransport::Now, &LinkTransport::Send,
       &LinkTransport::Abort});
  rmlink::Core device(
      device_config,
      {&device_transport, &LinkTransport::Now, &LinkTransport::Send,
       &LinkTransport::Abort});

  ASSERT_EQ(host.Begin(), rmlink::Status::Ok);
  host.Poll();
  const auto hello = host_transport.snapshot;
  host.NotifySendComplete(host_transport.accepted_token,
                          rmlink::CompletionResult::Complete);
  device.Receive(hello, device_transport.now_us);
  device.Poll();
  const auto hello_ack = device_transport.snapshot;
  device.NotifySendComplete(device_transport.accepted_token,
                            rmlink::CompletionResult::Complete);
  host.Receive(hello_ack, host_transport.now_us);
  ASSERT_EQ(host.state(), rmlink::State::Syncing);

  for (std::size_t sample = 0; sample < 8; ++sample) {
    host.Poll();
    ASSERT_TRUE(host.has_in_flight_frame());
    const auto sync_request = host_transport.snapshot;
    host.NotifySendComplete(host_transport.accepted_token,
                            rmlink::CompletionResult::Complete);
    if (sample == 0) {
      const auto request_header = DecodeHeader(sync_request);
      const auto request_t1 = ReadU64(sync_request.data() + 14);
      const auto wrong_response = EncodeSyncResponse(
          request_header.session_id, request_header.sequence, request_t1 + 1,
          device_transport.now_us + 1'000'000,
          device_transport.now_us + 1'000'000);
      host.Receive(wrong_response, host_transport.now_us);
      EXPECT_EQ(host.statistics().session_semantic_errors, 1U);
      EXPECT_EQ(host.time_mapping().state, rmlink::MappingState::Unavailable);
    }
    device_transport.now_us = host_transport.now_us + 1'000'000;
    device.Receive(sync_request, device_transport.now_us);
    device.Poll();
    ASSERT_TRUE(device.has_in_flight_frame());
    const auto sync_response = device_transport.snapshot;
    device.NotifySendComplete(device_transport.accepted_token,
                              rmlink::CompletionResult::Complete);
    const auto receive_time = host_transport.now_us + (sample == 0 ? 1000 : 0);
    host.Receive(sync_response, receive_time);
    if (sample == 0) {
      host.Receive(sync_response, receive_time);
      EXPECT_EQ(host.statistics().session_semantic_errors, 2U);
    }
    if (sample != 7) {
      host_transport.now_us += 20'000;
    }
  }

  host.Poll();
  EXPECT_EQ(host.state(), rmlink::State::Starting);
  EXPECT_EQ(host.time_mapping().state, rmlink::MappingState::Valid);
  EXPECT_EQ(host.time_mapping().device_minus_host_us, 1'000'000);
  EXPECT_EQ(host.time_mapping().round_trip_us, 0U);

  host.Poll();
  ASSERT_TRUE(host.has_in_flight_frame());
  const auto start = host_transport.snapshot;
  device.Receive(start, device_transport.now_us);
  EXPECT_EQ(device.state(), rmlink::State::Active);
  const auto early_data = EncodeData(1, 10, 1, 123);
  host.Receive(early_data, host_transport.now_us);
  EXPECT_EQ(host.statistics().early_data_drops, 1U);
  device.Poll();
  const auto start_ack = device_transport.snapshot;
  host.Receive(start_ack, host_transport.now_us);
  EXPECT_EQ(host.statistics().early_data_drops, 1U);
  host.NotifySendComplete(host_transport.accepted_token,
                          rmlink::CompletionResult::Complete);
  host.Poll();
  EXPECT_EQ(host.state(), rmlink::State::Active);

  const auto mapping_generation = host.time_mapping().generation;
  device.Receive(start, device_transport.now_us);
  EXPECT_EQ(device.state(), rmlink::State::Active);
  device.Poll();
  EXPECT_TRUE(device.has_in_flight_frame());
  device.NotifySendComplete(device_transport.accepted_token,
                            rmlink::CompletionResult::Complete);
  EXPECT_EQ(host.time_mapping().generation, mapping_generation);

  const auto data = EncodeData(1, 10, 9, 777);
  host.Receive(data, host_transport.now_us + 5'000);
  ASSERT_EQ(received_event.count, 1U);
  EXPECT_EQ(received_event.metadata.relative_event_us, 777U);
  EXPECT_EQ(received_event.metadata.device_event_us, 2'000'777U);
  EXPECT_TRUE(received_event.payload.empty());

  std::uint64_t device_event_us = 0;
  std::uint64_t host_event_us = 0;
  rmlink::MappingState mapping_state{};
  EXPECT_EQ(host.MapDeviceEvent(777, device_event_us, host_event_us,
                                mapping_state),
            rmlink::Status::Ok);
  EXPECT_EQ(mapping_state, rmlink::MappingState::Valid);
  EXPECT_EQ(device_event_us, 2'000'777U);
  EXPECT_EQ(host_event_us, 1'000'777U);

  const auto generation_before_periodic = host.time_mapping().generation;
  host_transport.now_us += 1'000'000;
  host.Poll();
  EXPECT_TRUE(host.has_in_flight_frame());
  host.NotifySendComplete(host_transport.accepted_token,
                          rmlink::CompletionResult::Complete);
  host_transport.now_us += 5'001'000;
  host.Poll();
  EXPECT_EQ(host.time_mapping().generation, generation_before_periodic);
  EXPECT_EQ(host.time_mapping().state, rmlink::MappingState::Stale);
  EXPECT_EQ(host.time_mapping().age_us, 6'001'000U);
}

TEST(CoreSessionTest, DropsTimedOutSyncSampleAndKeepsRoundBounded) {
  const std::array streams{
      rmlink::StreamRegistration{10, nullptr, &OnReceive}};
  auto config = MakeConfig(streams);
  config.timing.control_response_timeout_ms = 1;
  config.timing.initial_sync_timeout_ms = 100;
  config.timing.sync_sample_spacing_ms = 1;
  LinkTransport transport{};
  transport.now_us = 1'000'000;
  rmlink::Core host(
      config,
      {&transport, &LinkTransport::Now, &LinkTransport::Send,
       &LinkTransport::Abort});

  ASSERT_EQ(host.Begin(), rmlink::Status::Ok);
  host.Poll();
  const auto hello = transport.snapshot;
  const auto hello_header = DecodeHeader(hello);
  const auto attempt_token = ReadU64(hello.data() + 16);
  host.NotifySendComplete(transport.accepted_token,
                          rmlink::CompletionResult::Complete);
  const auto hello_ack = EncodeHelloAck(hello, hello_header.session_id,
                                        hello_header.sequence, attempt_token);
  host.Receive(hello_ack, transport.now_us);
  ASSERT_EQ(host.state(), rmlink::State::Syncing);

  host.Poll();
  ASSERT_TRUE(host.has_in_flight_frame());
  host.NotifySendComplete(transport.accepted_token,
                          rmlink::CompletionResult::Complete);
  transport.now_us += 1001;
  host.Poll();
  EXPECT_EQ(host.statistics().sync_failures, 1U);
  EXPECT_EQ(host.state(), rmlink::State::Syncing);
  EXPECT_EQ(host.time_mapping().state, rmlink::MappingState::Unavailable);
}

} // namespace
