#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace rmlink {

inline constexpr std::uint16_t kProtocolVersion = 1;
inline constexpr std::size_t kMaxPayloadSize = 255;
inline constexpr std::size_t kHeaderSize = 14;
inline constexpr std::size_t kFrameOverhead = 18;
inline constexpr std::size_t kMaxFrameSize = 273;
inline constexpr std::uint64_t kMaxSessionDurationUs = 3'600'000'000ULL;

enum class Role : std::uint8_t { Host, Device };

enum class State : std::uint8_t {
  Idle,
  Handshaking,
  Syncing,
  Starting,
  Active,
  Closing,
  ResetRequired,
};

enum class MessageType : std::uint8_t {
  Hello = 0x01,
  HelloAck = 0x02,
  SyncReq = 0x03,
  SyncResp = 0x04,
  Start = 0x05,
  StartAck = 0x06,
  Close = 0x07,
  CloseAck = 0x08,
  Reject = 0x09,
  Data = 0x10,
  Log = 0x11,
};

enum class Status : std::uint8_t {
  Ok,
  InvalidArgument,
  InvalidConfig,
  InvalidState,
  UnknownStream,
  PayloadTooLarge,
  EncodeFailed,
  DecodeFailed,
  QueueFull,
  RateDisabled,
  TimeOutOfRange,
  WrongRole,
  TransportFailed,
};

enum class SendResult : std::uint8_t { Busy, Accepted, Failed };
enum class CompletionResult : std::uint8_t { Complete, Failed };
enum class CloseReason : std::uint8_t { Normal = 0, Cancelled = 1, Protocol = 2 };

struct EventMetadata {
  std::uint8_t session_id{};
  std::uint16_t stream_id{};
  std::uint8_t sequence{};
  std::uint32_t relative_event_us{};
  std::uint64_t receive_end_us{};
  std::uint64_t device_event_us{};
};

enum class MappingState : std::uint8_t { Unavailable, Valid, Stale };

struct TimeMapping {
  MappingState state{MappingState::Unavailable};
  std::uint8_t session_id{};
  std::uint64_t host_epoch_us{};
  std::uint64_t device_epoch_us{};
  std::int64_t device_minus_host_us{};
  std::uint64_t round_trip_us{};
  std::uint64_t updated_at_us{};
  std::uint64_t age_us{};
  std::uint64_t generation{};
};

struct TimeSample {
  std::int64_t device_minus_host_us{};
  std::uint64_t round_trip_us{};
};

using BinaryReceiveCallback = void (*)(void *context,
                                       std::span<const std::byte> payload,
                                       const EventMetadata &metadata) noexcept;

using LogReceiveCallback = void (*)(void *context, std::uint8_t level,
                                    std::uint8_t flags,
                                    std::span<const std::byte> text,
                                    const EventMetadata &metadata) noexcept;

struct StreamRegistration {
  std::uint16_t stream_id{};
  void *context{};
  BinaryReceiveCallback on_receive{};
};

struct TimingConfig {
  std::uint32_t control_response_timeout_ms{250};
  std::uint8_t control_max_retries{2};
  std::uint32_t initial_sync_timeout_ms{2000};
  std::uint8_t sync_sample_count{8};
  std::uint32_t sync_sample_spacing_ms{20};
  std::uint32_t sync_period_ms{1000};
  std::uint32_t sync_stale_ms{5000};
  std::uint32_t rx_liveness_timeout_ms{3000};
  std::uint32_t partial_frame_timeout_ms{100};
  std::uint32_t tx_completion_timeout_ms{250};
  std::uint32_t close_record_timeout_ms{1500};
};

struct Config {
  Role role{Role::Host};
  std::span<const StreamRegistration> streams{};
  void *log_context{};
  LogReceiveCallback on_log{};
  std::uint16_t realtime_rate_limit_fps{500};
  std::size_t realtime_queue_depth{8};
  std::size_t log_queue_depth{4};
  std::size_t control_queue_depth{2};
  std::uint32_t log_budget_bytes_per_second{4096};
  TimingConfig timing{};
};

struct TransportOps {
  void *context{};
  std::uint64_t (*now_us)(void *context) noexcept{};
  SendResult (*send)(void *context, std::span<const std::byte> frame,
                     std::uint64_t transfer_token) noexcept{};
  void (*abort)(void *context) noexcept{};
};

[[nodiscard]] Status ValidateConfig(const Config &config) noexcept;
[[nodiscard]] Status ToRelativeTimestamp(std::uint64_t local_epoch_us,
                                         std::uint64_t event_us,
                                         std::uint64_t now_us,
                                         std::uint32_t &relative_us) noexcept;
[[nodiscard]] Status CalculateTimeSample(std::uint64_t t1,
                                         std::uint64_t t2,
                                         std::uint64_t t3,
                                         std::uint64_t t4,
                                         TimeSample &sample) noexcept;

} // namespace rmlink
