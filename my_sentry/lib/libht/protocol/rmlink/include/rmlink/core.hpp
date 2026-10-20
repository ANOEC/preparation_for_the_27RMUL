#pragma once

#include "rmlink/codec.hpp"
#include "rmlink/types.hpp"
#include "rmlink/wire.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace rmlink {

struct Statistics {
  std::uint64_t transport_accepted{};
  std::uint64_t transport_busy{};
  std::uint64_t transport_failures{};
  std::uint64_t tx_completion_timeouts{};
  std::uint64_t stale_completions{};
  std::uint64_t session_semantic_errors{};
  std::uint64_t control_queue_drops{};
  std::uint64_t control_retries{};
  std::uint64_t control_failures{};
  std::uint64_t sync_failures{};
  std::uint64_t early_data_drops{};
  std::uint64_t received_data_frames{};
  std::uint64_t unknown_streams{};
  std::uint64_t heartbeat_timeouts{};
  std::uint64_t received_log_frames{};
  std::uint64_t dropped_realtime{};
  std::uint64_t dropped_logs{};
  std::uint64_t log_budget_waits{};
  std::uint64_t log_truncations{};
  std::uint64_t invalid_log_entries{};
};

class Core {
public:
  Core(const Config &config, TransportOps transport);
  ~Core() = default;

  Core(const Core &) = delete;
  Core &operator=(const Core &) = delete;
  Core(Core &&) = delete;
  Core &operator=(Core &&) = delete;

  [[nodiscard]] Status initialization_status() const noexcept {
    return initialization_status_;
  }
  [[nodiscard]] State state() const noexcept { return state_; }
  [[nodiscard]] std::uint8_t session_id() const noexcept {
    return session_id_;
  }
  [[nodiscard]] bool has_in_flight_frame() const noexcept {
    return in_flight_;
  }
  [[nodiscard]] std::uint64_t in_flight_token() const noexcept {
    return in_flight_token_;
  }
  [[nodiscard]] const Statistics &statistics() const noexcept {
    return statistics_;
  }
  [[nodiscard]] const wire::ParseStatistics &parse_statistics() const noexcept {
    return parser_.statistics();
  }
  [[nodiscard]] const TimeMapping &time_mapping() const noexcept {
    return mapping_;
  }
  [[nodiscard]] std::size_t fixed_memory_bytes() const noexcept;
  [[nodiscard]] Status MapDeviceEvent(std::uint32_t relative_event_us,
                                      std::uint64_t &device_event_us,
                                      std::uint64_t &host_event_us,
                                      MappingState &state) const noexcept;

  [[nodiscard]] Status Begin() noexcept;
  [[nodiscard]] Status Close(CloseReason reason = CloseReason::Normal) noexcept;
  [[nodiscard]] Status SubmitData(std::uint16_t stream_id,
                                  std::span<const std::byte> payload,
                                  std::uint64_t event_us) noexcept;
  template <StructuredPayload T>
  [[nodiscard]] Status Submit(std::uint16_t stream_id, const T &value,
                              std::uint64_t event_us) noexcept {
    std::array<std::byte, ::wire::MaxEncodedSize<T>> encoded{};
    const auto result = ::wire::Encode(value, encoded);
    if (!result) {
      return Status::EncodeFailed;
    }
    return SubmitData(stream_id,
                      std::span<const std::byte>(encoded.data(), result.bytes),
                      event_us);
  }
  [[nodiscard]] Status SubmitLog(std::uint8_t level,
                                 std::span<const std::byte> text) noexcept;
  void Receive(std::span<const std::byte> bytes,
               std::uint64_t receive_end_us) noexcept;
  void Poll() noexcept;
  void NotifySendComplete(std::uint64_t transfer_token,
                          CompletionResult result) noexcept;
  void NotifyTransportFailure() noexcept;
  void NotifyTransportAborted() noexcept;

private:
  struct StreamState {
    StreamRegistration registration{};
    std::uint8_t next_sequence{};
  };

  struct QueueSlot {
    MessageType type{MessageType::Data};
    std::array<std::byte, kMaxPayloadSize> payload{};
    std::uint8_t payload_size{};
    std::uint16_t stream_id{};
    std::uint8_t sequence{};
    std::uint64_t event_us{};
    std::uint8_t log_flags{};
  };

  struct ControlSlot {
    MessageType type{MessageType::Reject};
    std::uint8_t session_id{};
    std::uint8_t sequence{};
    std::uint32_t timestamp_us{};
    std::array<std::byte, kMaxPayloadSize> payload{};
    std::uint8_t payload_size{};
    bool sample_t3_on_submit{};
  };

  enum class ControlPhase : std::uint8_t {
    PendingSubmit,
    InFlight,
    WaitingResponse,
  };

  struct ControlTransaction {
    bool active{};
    MessageType request_type{MessageType::Hello};
    MessageType response_type{MessageType::HelloAck};
    std::uint8_t session_id{};
    std::uint8_t sequence{};
    std::array<std::byte, kMaxPayloadSize> payload{};
    std::uint8_t payload_size{};
    ControlPhase phase{ControlPhase::PendingSubmit};
    std::uint8_t accepted_attempts{};
    bool early_response{};
    std::uint64_t started_us{};
    std::uint64_t total_deadline_us{};
    std::uint64_t response_deadline_us{};
  };

  struct SyncSample {
    TimeSample value{};
    std::uint64_t received_at_us{};
  };

  static void HandleParsedFrame(void *context,
                                const wire::FrameView &frame) noexcept;
  void OnFrame(const wire::FrameView &frame) noexcept;
  void BeginHelloTransaction(std::uint64_t now_us) noexcept;
  void BeginSyncRound(std::uint64_t now_us) noexcept;
  void BeginSyncTransaction(std::uint64_t now_us) noexcept;
  void BeginStartTransaction(std::uint64_t now_us) noexcept;
  void BeginCloseTransaction(std::uint64_t now_us) noexcept;
  void AdvanceControlTransaction(std::uint64_t now_us) noexcept;
  void TrySendTransaction(std::uint64_t now_us) noexcept;
  void AcceptControlResponse() noexcept;
  void TrySendControl(std::uint64_t now_us) noexcept;
  [[nodiscard]] bool EnqueueControl(
      MessageType type, std::uint8_t session_id, std::uint8_t sequence,
      std::span<const std::byte> payload, std::uint32_t timestamp_us) noexcept;
  void HandleHello(const wire::FrameView &frame) noexcept;
  void HandleHelloAck(const wire::FrameView &frame) noexcept;
  void HandleReject(const wire::FrameView &frame) noexcept;
  void HandleSyncRequest(const wire::FrameView &frame) noexcept;
  void HandleSyncResponse(const wire::FrameView &frame) noexcept;
  void HandleStart(const wire::FrameView &frame) noexcept;
  void HandleStartAck(const wire::FrameView &frame) noexcept;
  void HandleClose(const wire::FrameView &frame) noexcept;
  void HandleCloseAck(const wire::FrameView &frame) noexcept;
  void HandleEarlyData(const wire::FrameView &frame) noexcept;
  void HandleData(const wire::FrameView &frame) noexcept;
  void HandleLog(const wire::FrameView &frame) noexcept;
  void FinishSyncRound(std::uint64_t now_us) noexcept;
  void RefreshMappingAndLiveness(std::uint64_t now_us) noexcept;
  void MaybeFinishNormalClose() noexcept;
  void FinishNormalClose() noexcept;
  [[nodiscard]] bool TrySendBusiness(std::uint64_t now_us) noexcept;
  [[nodiscard]] bool TrySendQueue(std::vector<QueueSlot> &queue,
                                  std::size_t head, std::uint64_t now_us,
                                  bool is_log) noexcept;
  [[nodiscard]] StreamState *FindStream(std::uint16_t stream_id) noexcept;
  void EnterResetRequired() noexcept;
  void ClearSession() noexcept;

  Role role_;
  TimingConfig timing_;
  TransportOps transport_;
  Status initialization_status_{Status::InvalidConfig};
  State state_{State::Idle};
  wire::Parser parser_;
  std::vector<StreamState> streams_;
  std::vector<QueueSlot> realtime_queue_;
  std::vector<QueueSlot> log_queue_;
  std::vector<ControlSlot> control_queue_;
  std::vector<SyncSample> sync_samples_;
  std::array<std::byte, kMaxFrameSize> tx_buffer_{};
  std::size_t tx_size_{};
  bool in_flight_{};
  std::uint64_t in_flight_token_{};
  std::uint64_t next_transfer_token_{1};
  std::uint64_t in_flight_started_us_{};
  bool in_flight_is_transaction_{};
  bool in_flight_is_business_{};
  bool in_flight_is_log_{};
  MessageType in_flight_type_{MessageType::Data};
  std::uint8_t session_id_{};
  std::uint8_t next_session_id_{1};
  std::uint8_t control_sequence_{};
  std::uint64_t attempt_token_{};
  std::uint64_t local_epoch_us_{};
  std::uint64_t peer_epoch_us_{};
  bool local_epoch_valid_{};
  bool hello_payload_frozen_{};
  ControlTransaction transaction_{};
  std::size_t control_head_{};
  std::size_t control_count_{};
  std::size_t sync_sample_count_{};
  std::uint64_t sync_round_deadline_us_{};
  std::uint64_t next_sync_request_us_{};
  std::uint64_t next_sync_round_us_{};
  bool initial_sync_round_{};
  std::uint64_t active_since_us_{};
  std::uint64_t last_valid_data_us_{};
  std::uint16_t realtime_rate_limit_fps_{};
  std::uint32_t log_budget_bytes_per_second_{};
  std::uint64_t last_data_accept_us_{};
  bool data_accept_seen_{};
  std::uint64_t log_budget_tokens_{};
  std::uint64_t log_budget_last_us_{};
  std::size_t realtime_head_{};
  std::size_t realtime_count_{};
  std::size_t log_head_{};
  std::size_t log_count_{};
  std::uint8_t log_sequence_{};
  void *log_context_{};
  LogReceiveCallback on_log_{};
  bool local_close_requested_{};
  bool local_close_ack_received_{};
  bool peer_close_received_{};
  bool close_ack_pending_{};
  CloseReason close_reason_{CloseReason::Normal};
  bool close_record_valid_{};
  std::uint8_t close_record_session_id_{};
  std::uint8_t close_record_sequence_{};
  std::uint64_t close_record_expires_us_{};
  TimeMapping mapping_{};
  Statistics statistics_{};
};

} // namespace rmlink
