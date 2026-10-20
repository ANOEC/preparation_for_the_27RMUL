#include "rmlink/core.hpp"

#include <algorithm>

namespace rmlink {
namespace {

void WriteU16(std::byte *output, std::uint16_t value) noexcept {
  output[0] = static_cast<std::byte>(value & 0xFFU);
  output[1] = static_cast<std::byte>((value >> 8U) & 0xFFU);
}

void WriteU64(std::byte *output, std::uint64_t value) noexcept {
  for (std::size_t index = 0; index < 8; ++index) {
    output[index] = static_cast<std::byte>((value >> (index * 8U)) & 0xFFU);
  }
}

std::uint16_t ReadU16(const std::byte *input) noexcept {
  return static_cast<std::uint16_t>(std::to_integer<std::uint8_t>(input[0])) |
         static_cast<std::uint16_t>(
             std::to_integer<std::uint8_t>(input[1]) << 8U);
}

std::uint64_t ReadU64(const std::byte *input) noexcept {
  std::uint64_t value = 0;
  for (std::size_t index = 0; index < 8; ++index) {
    value |= static_cast<std::uint64_t>(
                 std::to_integer<std::uint8_t>(input[index]))
             << (index * 8U);
  }
  return value;
}

std::uint64_t ControlBudgetUs(const TimingConfig &timing) noexcept {
  const auto attempt_us = static_cast<std::uint64_t>(
      timing.tx_completion_timeout_ms + timing.control_response_timeout_ms) *
                           1000ULL;
  return static_cast<std::uint64_t>(timing.control_max_retries + 1U) *
         attempt_us;
}

std::size_t ValidUtf8Prefix(std::span<const std::byte> text,
                            std::size_t maximum) noexcept {
  auto length = std::min(text.size(), maximum);
  while (length > 0 && !wire::IsValidUtf8(text.first(length))) {
    --length;
  }
  return length;
}

} // namespace

Core::Core(const Config &config, TransportOps transport)
    : role_(config.role), timing_(config.timing), transport_(transport),
      parser_(config.role,
              static_cast<std::uint64_t>(
                  config.timing.partial_frame_timeout_ms) *
                  1000ULL,
              &Core::HandleParsedFrame, this),
      realtime_queue_(config.realtime_queue_depth),
      log_queue_(config.role == Role::Device ? config.log_queue_depth : 0),
      control_queue_(config.control_queue_depth),
      sync_samples_(config.timing.sync_sample_count),
      realtime_rate_limit_fps_(config.realtime_rate_limit_fps),
      log_budget_bytes_per_second_(config.log_budget_bytes_per_second),
      log_budget_tokens_(std::min<std::uint64_t>(
          273, config.log_budget_bytes_per_second)),
      log_context_(config.log_context), on_log_(config.on_log) {
  if (ValidateConfig(config) != Status::Ok || transport_.now_us == nullptr ||
      transport_.send == nullptr || transport_.abort == nullptr) {
    return;
  }
  streams_.reserve(config.streams.size());
  for (const auto &registration : config.streams) {
    streams_.push_back({registration, 0});
  }
  initialization_status_ = Status::Ok;
}

std::size_t Core::fixed_memory_bytes() const noexcept {
  return sizeof(*this) + streams_.capacity() * sizeof(StreamState) +
         realtime_queue_.capacity() * sizeof(QueueSlot) +
         log_queue_.capacity() * sizeof(QueueSlot) +
         control_queue_.capacity() * sizeof(ControlSlot) +
         sync_samples_.capacity() * sizeof(SyncSample);
}

Status Core::Begin() noexcept {
  if (initialization_status_ != Status::Ok) {
    return initialization_status_;
  }
  if (role_ != Role::Host) {
    return Status::WrongRole;
  }
  if (state_ != State::Idle) {
    return Status::InvalidState;
  }

  close_record_valid_ = false;

  session_id_ = next_session_id_;
  next_session_id_ = next_session_id_ == 255
                         ? 1
                         : static_cast<std::uint8_t>(next_session_id_ + 1);
  ++attempt_token_;
  ++control_sequence_;
  hello_payload_frozen_ = false;
  transaction_ = {};
  state_ = State::Handshaking;
  return Status::Ok;
}

Status Core::Close(CloseReason reason) noexcept {
  if (state_ != State::Active && state_ != State::Syncing &&
      state_ != State::Starting) {
    return Status::InvalidState;
  }
  local_close_requested_ = true;
  local_close_ack_received_ = false;
  close_reason_ = reason;
  transaction_ = {};
  realtime_head_ = 0;
  realtime_count_ = 0;
  log_head_ = 0;
  log_count_ = 0;
  state_ = State::Closing;
  return Status::Ok;
}

Core::StreamState *Core::FindStream(std::uint16_t stream_id) noexcept {
  for (auto &stream : streams_) {
    if (stream.registration.stream_id == stream_id) {
      return &stream;
    }
  }
  return nullptr;
}

Status Core::SubmitData(std::uint16_t stream_id,
                        std::span<const std::byte> payload,
                        std::uint64_t event_us) noexcept {
  if (initialization_status_ != Status::Ok || state_ != State::Active) {
    return Status::InvalidState;
  }
  if (payload.size() > kMaxPayloadSize) {
    return Status::PayloadTooLarge;
  }
  if (realtime_rate_limit_fps_ == 0) {
    return Status::RateDisabled;
  }
  auto *stream = FindStream(stream_id);
  if (stream == nullptr) {
    return Status::UnknownStream;
  }
  const auto now_us = transport_.now_us(transport_.context);
  std::uint32_t relative_us = 0;
  if (ToRelativeTimestamp(local_epoch_us_, event_us, now_us, relative_us) !=
      Status::Ok) {
    return Status::TimeOutOfRange;
  }
  if (realtime_count_ == realtime_queue_.size()) {
    realtime_head_ = (realtime_head_ + 1) % realtime_queue_.size();
    --realtime_count_;
    ++statistics_.dropped_realtime;
  }
  const auto tail = (realtime_head_ + realtime_count_) % realtime_queue_.size();
  auto &slot = realtime_queue_[tail];
  slot = {};
  slot.type = MessageType::Data;
  slot.stream_id = stream_id;
  slot.sequence = stream->next_sequence++;
  slot.event_us = event_us;
  slot.payload_size = static_cast<std::uint8_t>(payload.size());
  std::copy(payload.begin(), payload.end(), slot.payload.begin());
  ++realtime_count_;
  (void)relative_us;
  return Status::Ok;
}

Status Core::SubmitLog(std::uint8_t level,
                       std::span<const std::byte> text) noexcept {
  if (role_ != Role::Device) {
    return Status::WrongRole;
  }
  if (initialization_status_ != Status::Ok || state_ != State::Active) {
    return Status::InvalidState;
  }
  if (level > 5 || !wire::IsValidUtf8(text)) {
    ++statistics_.invalid_log_entries;
    return Status::InvalidArgument;
  }
  const auto length = ValidUtf8Prefix(text, 253);
  const auto truncated = length != text.size();
  if (truncated) {
    ++statistics_.log_truncations;
  }
  if (log_count_ == log_queue_.size()) {
    log_head_ = (log_head_ + 1) % log_queue_.size();
    --log_count_;
    ++statistics_.dropped_logs;
  }
  const auto tail = (log_head_ + log_count_) % log_queue_.size();
  auto &slot = log_queue_[tail];
  slot = {};
  slot.type = MessageType::Log;
  slot.sequence = log_sequence_++;
  slot.event_us = transport_.now_us(transport_.context);
  slot.payload_size = static_cast<std::uint8_t>(length + 2);
  slot.payload[0] = static_cast<std::byte>(level);
  slot.payload[1] = truncated ? std::byte{1} : std::byte{0};
  std::copy(text.begin(), text.begin() + static_cast<std::ptrdiff_t>(length),
            slot.payload.begin() + 2);
  ++log_count_;
  return Status::Ok;
}

void Core::Receive(std::span<const std::byte> bytes,
                   std::uint64_t receive_end_us) noexcept {
  if (initialization_status_ != Status::Ok ||
      state_ == State::ResetRequired) {
    return;
  }
  parser_.Feed(bytes, receive_end_us);
}

void Core::Poll() noexcept {
  if (initialization_status_ != Status::Ok ||
      state_ == State::ResetRequired) {
    return;
  }
  const auto now_us = transport_.now_us(transport_.context);
  parser_.Poll(now_us);

  if (close_record_valid_ && now_us > close_record_expires_us_) {
    close_record_valid_ = false;
  }

  RefreshMappingAndLiveness(now_us);
  if (state_ == State::ResetRequired) {
    return;
  }

  if (local_epoch_valid_ &&
      (now_us < local_epoch_us_ ||
       now_us - local_epoch_us_ > kMaxSessionDurationUs)) {
    EnterResetRequired();
    return;
  }

  if (in_flight_) {
    const auto timeout_us =
        static_cast<std::uint64_t>(timing_.tx_completion_timeout_ms) * 1000ULL;
    if (now_us < in_flight_started_us_ ||
        now_us - in_flight_started_us_ > timeout_us) {
      ++statistics_.tx_completion_timeouts;
      EnterResetRequired();
    }
    return;
  }

  if (control_count_ != 0) {
    TrySendControl(now_us);
  } else if (transaction_.active) {
    AdvanceControlTransaction(now_us);
  } else if (state_ == State::Active && TrySendBusiness(now_us)) {
    return;
  } else if (role_ == Role::Host && state_ == State::Handshaking) {
    BeginHelloTransaction(now_us);
    AdvanceControlTransaction(now_us);
  } else if (role_ == Role::Host &&
             (state_ == State::Syncing || state_ == State::Active)) {
    if (sync_round_deadline_us_ == 0 &&
        (state_ == State::Syncing || now_us >= next_sync_round_us_)) {
      BeginSyncRound(now_us);
    }
    if (sync_round_deadline_us_ == 0) {
      return;
    }
    if (now_us > sync_round_deadline_us_) {
      ++statistics_.sync_failures;
      sync_round_deadline_us_ = 0;
      next_sync_round_us_ =
          now_us + static_cast<std::uint64_t>(timing_.sync_period_ms) * 1000ULL;
      if (initial_sync_round_) {
        EnterResetRequired();
      }
    } else if (sync_sample_count_ == sync_samples_.size()) {
      FinishSyncRound(now_us);
    } else if (now_us >= next_sync_request_us_) {
      BeginSyncTransaction(now_us);
      AdvanceControlTransaction(now_us);
    }
  } else if (role_ == Role::Host && state_ == State::Starting) {
    BeginStartTransaction(now_us);
    AdvanceControlTransaction(now_us);
  } else if (state_ == State::Closing && local_close_requested_) {
    BeginCloseTransaction(now_us);
    AdvanceControlTransaction(now_us);
  }
}

void Core::NotifySendComplete(std::uint64_t transfer_token,
                              CompletionResult result) noexcept {
  if (!in_flight_ || transfer_token != in_flight_token_) {
    ++statistics_.stale_completions;
    return;
  }
  if (result == CompletionResult::Failed) {
    ++statistics_.transport_failures;
    EnterResetRequired();
    return;
  }
  if (in_flight_is_transaction_ && transaction_.active &&
      transaction_.phase == ControlPhase::InFlight) {
    transaction_.phase = ControlPhase::WaitingResponse;
    transaction_.response_deadline_us =
        transport_.now_us(transport_.context) +
        static_cast<std::uint64_t>(timing_.control_response_timeout_ms) *
            1000ULL;
  }
  if (!in_flight_is_transaction_ && in_flight_type_ == MessageType::CloseAck) {
    close_ack_pending_ = false;
  }
  in_flight_is_business_ = false;
  in_flight_is_log_ = false;
  in_flight_ = false;
  in_flight_is_transaction_ = false;
  tx_size_ = 0;
  MaybeFinishNormalClose();
}

void Core::NotifyTransportFailure() noexcept {
  if (initialization_status_ == Status::Ok && state_ != State::Idle &&
      state_ != State::ResetRequired) {
    ++statistics_.transport_failures;
    EnterResetRequired();
  }
}

void Core::NotifyTransportAborted() noexcept {
  if (state_ != State::ResetRequired) {
    return;
  }
  in_flight_ = false;
  tx_size_ = 0;
  ClearSession();
  close_record_valid_ = false;
  state_ = State::Idle;
}

void Core::HandleParsedFrame(void *context,
                             const wire::FrameView &frame) noexcept {
  static_cast<Core *>(context)->OnFrame(frame);
}

void Core::OnFrame(const wire::FrameView &frame) noexcept {
  switch (frame.header.type) {
  case MessageType::Hello:
    HandleHello(frame);
    return;
  case MessageType::HelloAck:
    HandleHelloAck(frame);
    return;
  case MessageType::Reject:
    HandleReject(frame);
    return;
  case MessageType::SyncReq:
    HandleSyncRequest(frame);
    return;
  case MessageType::SyncResp:
    HandleSyncResponse(frame);
    return;
  case MessageType::Start:
    HandleStart(frame);
    return;
  case MessageType::StartAck:
    HandleStartAck(frame);
    return;
  case MessageType::Close:
    HandleClose(frame);
    return;
  case MessageType::CloseAck:
    HandleCloseAck(frame);
    return;
  case MessageType::Data:
    HandleData(frame);
    return;
  case MessageType::Log:
    HandleLog(frame);
    return;
  default:
    ++statistics_.session_semantic_errors;
    return;
  }
}

void Core::BeginHelloTransaction(std::uint64_t now_us) noexcept {
  std::array<std::byte, 18> payload{};
  WriteU16(payload.data(), kProtocolVersion);
  WriteU64(payload.data() + 2, attempt_token_);
  WriteU64(payload.data() + 10,
           hello_payload_frozen_ ? local_epoch_us_ : now_us);
  transaction_.active = true;
  transaction_.request_type = MessageType::Hello;
  transaction_.response_type = MessageType::HelloAck;
  transaction_.session_id = session_id_;
  transaction_.sequence = control_sequence_;
  transaction_.payload_size = static_cast<std::uint8_t>(payload.size());
  std::copy(payload.begin(), payload.end(), transaction_.payload.begin());
  transaction_.phase = ControlPhase::PendingSubmit;
  transaction_.accepted_attempts = 0;
  transaction_.early_response = false;
  transaction_.started_us = now_us;
  transaction_.total_deadline_us = now_us + ControlBudgetUs(timing_);
}

void Core::BeginSyncRound(std::uint64_t now_us) noexcept {
  sync_sample_count_ = 0;
  initial_sync_round_ = state_ == State::Syncing;
  sync_round_deadline_us_ =
      now_us + static_cast<std::uint64_t>(timing_.initial_sync_timeout_ms) *
                   1000ULL;
  next_sync_request_us_ = now_us;
}

void Core::BeginSyncTransaction(std::uint64_t now_us) noexcept {
  ++control_sequence_;
  transaction_ = {};
  transaction_.active = true;
  transaction_.request_type = MessageType::SyncReq;
  transaction_.response_type = MessageType::SyncResp;
  transaction_.session_id = session_id_;
  transaction_.sequence = control_sequence_;
  transaction_.payload_size = 8;
  WriteU64(transaction_.payload.data(), now_us);
  transaction_.phase = ControlPhase::PendingSubmit;
  transaction_.started_us = now_us;
  transaction_.total_deadline_us = sync_round_deadline_us_;
}

void Core::BeginStartTransaction(std::uint64_t now_us) noexcept {
  ++control_sequence_;
  transaction_ = {};
  transaction_.active = true;
  transaction_.request_type = MessageType::Start;
  transaction_.response_type = MessageType::StartAck;
  transaction_.session_id = session_id_;
  transaction_.sequence = control_sequence_;
  transaction_.phase = ControlPhase::PendingSubmit;
  transaction_.started_us = now_us;
  transaction_.total_deadline_us = now_us + ControlBudgetUs(timing_);
}

void Core::BeginCloseTransaction(std::uint64_t now_us) noexcept {
  ++control_sequence_;
  transaction_ = {};
  transaction_.active = true;
  transaction_.request_type = MessageType::Close;
  transaction_.response_type = MessageType::CloseAck;
  transaction_.session_id = session_id_;
  transaction_.sequence = control_sequence_;
  transaction_.payload_size = 1;
  transaction_.payload[0] = static_cast<std::byte>(close_reason_);
  transaction_.phase = ControlPhase::PendingSubmit;
  transaction_.started_us = now_us;
  transaction_.total_deadline_us = now_us + ControlBudgetUs(timing_);
}

void Core::AdvanceControlTransaction(std::uint64_t now_us) noexcept {
  if (!transaction_.active) {
    return;
  }
  if (now_us < transaction_.started_us ||
      now_us > transaction_.total_deadline_us) {
    ++statistics_.control_failures;
    EnterResetRequired();
    return;
  }
  if (transaction_.phase == ControlPhase::WaitingResponse) {
    if (transaction_.early_response) {
      AcceptControlResponse();
      return;
    }
    if (now_us <= transaction_.response_deadline_us) {
      return;
    }
    if (transaction_.request_type == MessageType::SyncReq) {
      ++statistics_.sync_failures;
      transaction_.active = false;
      return;
    }
    if (transaction_.accepted_attempts >= timing_.control_max_retries) {
      ++statistics_.control_failures;
      EnterResetRequired();
      return;
    }
    ++transaction_.accepted_attempts;
    ++statistics_.control_retries;
    transaction_.phase = ControlPhase::PendingSubmit;
  }
  if (transaction_.phase == ControlPhase::PendingSubmit) {
    TrySendTransaction(now_us);
  }
}

void Core::TrySendTransaction(std::uint64_t now_us) noexcept {
  if (transaction_.request_type == MessageType::Hello &&
      !hello_payload_frozen_) {
    WriteU64(transaction_.payload.data() + 10, now_us);
  } else if (transaction_.request_type == MessageType::SyncReq) {
    WriteU64(transaction_.payload.data(), now_us);
  }
  const auto timestamp = transaction_.request_type == MessageType::Hello
                             ? 0U
                             : static_cast<std::uint32_t>(
                                   now_us - local_epoch_us_);
  const wire::Header header{transaction_.request_type, transaction_.session_id,
                            0, transaction_.sequence, 0, timestamp};
  const auto encoded = wire::Encode(
      header,
      std::span<const std::byte>(transaction_.payload.data(),
                                 transaction_.payload_size),
      tx_buffer_);
  if (encoded.status != Status::Ok) {
    EnterResetRequired();
    return;
  }
  tx_size_ = encoded.bytes;
  const auto token = next_transfer_token_;
  const auto result = transport_.send(
      transport_.context,
      std::span<const std::byte>(tx_buffer_.data(), tx_size_), token);
  switch (result) {
  case SendResult::Busy:
    ++statistics_.transport_busy;
    tx_size_ = 0;
    return;
  case SendResult::Accepted:
    ++statistics_.transport_accepted;
    if (transaction_.request_type == MessageType::Hello &&
        !hello_payload_frozen_) {
      local_epoch_us_ = now_us;
      local_epoch_valid_ = true;
      hello_payload_frozen_ = true;
    } else if (transaction_.request_type == MessageType::SyncReq) {
      next_sync_request_us_ =
          now_us +
          static_cast<std::uint64_t>(timing_.sync_sample_spacing_ms) * 1000ULL;
    }
    in_flight_ = true;
    in_flight_is_transaction_ = true;
    in_flight_is_business_ = false;
    in_flight_is_log_ = false;
    in_flight_token_ = token;
    ++next_transfer_token_;
    in_flight_started_us_ = now_us;
    in_flight_type_ = transaction_.request_type;
    transaction_.phase = ControlPhase::InFlight;
    return;
  case SendResult::Failed:
    ++statistics_.transport_failures;
    EnterResetRequired();
    return;
  }
}

void Core::AcceptControlResponse() noexcept {
  transaction_.active = false;
  transaction_.phase = ControlPhase::PendingSubmit;
  if (transaction_.response_type == MessageType::HelloAck) {
    state_ = State::Syncing;
  } else if (transaction_.response_type == MessageType::StartAck) {
    state_ = State::Active;
    sync_sample_count_ = 0;
    active_since_us_ = transport_.now_us(transport_.context);
    last_valid_data_us_ = active_since_us_;
    next_sync_round_us_ =
        active_since_us_ + static_cast<std::uint64_t>(timing_.sync_period_ms) *
                               1000ULL;
  } else if (transaction_.response_type == MessageType::CloseAck) {
    local_close_ack_received_ = true;
    MaybeFinishNormalClose();
  }
}

void Core::TrySendControl(std::uint64_t now_us) noexcept {
  auto &slot = control_queue_[control_head_];
  if (slot.sample_t3_on_submit) {
    WriteU64(slot.payload.data() + 16, now_us);
  }
  auto timestamp = slot.timestamp_us;
  if (local_epoch_valid_ && slot.session_id == session_id_) {
    std::uint32_t relative_us = 0;
    if (ToRelativeTimestamp(local_epoch_us_, now_us, now_us, relative_us) ==
        Status::Ok) {
      timestamp = relative_us;
    }
  }
  const wire::Header header{slot.type, slot.session_id, 0, slot.sequence, 0,
                            timestamp};
  const auto encoded = wire::Encode(
      header,
      std::span<const std::byte>(slot.payload.data(), slot.payload_size),
      tx_buffer_);
  if (encoded.status != Status::Ok) {
    EnterResetRequired();
    return;
  }
  tx_size_ = encoded.bytes;
  const auto token = next_transfer_token_;
  const auto result = transport_.send(
      transport_.context,
      std::span<const std::byte>(tx_buffer_.data(), tx_size_), token);
  switch (result) {
  case SendResult::Busy:
    ++statistics_.transport_busy;
    tx_size_ = 0;
    return;
  case SendResult::Accepted:
    ++statistics_.transport_accepted;
    in_flight_ = true;
    in_flight_is_transaction_ = false;
    in_flight_is_business_ = false;
    in_flight_is_log_ = false;
    in_flight_token_ = token;
    ++next_transfer_token_;
    in_flight_started_us_ = now_us;
    in_flight_type_ = slot.type;
    control_head_ = (control_head_ + 1) % control_queue_.size();
    --control_count_;
    return;
  case SendResult::Failed:
    ++statistics_.transport_failures;
    EnterResetRequired();
    return;
  }
}

bool Core::TrySendBusiness(std::uint64_t now_us) noexcept {
  if (realtime_count_ != 0 && realtime_rate_limit_fps_ != 0) {
    const auto interval_us =
        (1'000'000ULL + realtime_rate_limit_fps_ - 1U) /
        realtime_rate_limit_fps_;
    if (!data_accept_seen_ || now_us >= last_data_accept_us_ + interval_us) {
      if (TrySendQueue(realtime_queue_, realtime_head_, now_us, false)) {
        return true;
      }
    }
  }
  if (log_count_ != 0) {
    return TrySendQueue(log_queue_, log_head_, now_us, true);
  }
  return false;
}

bool Core::TrySendQueue(std::vector<QueueSlot> &queue, std::size_t head,
                        std::uint64_t now_us, bool is_log) noexcept {
  if (queue.empty() || (is_log ? log_count_ : realtime_count_) == 0) {
    return false;
  }
  auto &slot = queue[head];
  std::uint32_t timestamp_us = 0;
  if (ToRelativeTimestamp(local_epoch_us_, slot.event_us, now_us,
                          timestamp_us) != Status::Ok) {
    return false;
  }

  const auto frame_size = kFrameOverhead + slot.payload_size;
  if (is_log) {
    if (log_budget_bytes_per_second_ == 0) {
      ++statistics_.log_budget_waits;
      return true;
    }
    if (now_us > log_budget_last_us_) {
      const auto elapsed_us = now_us - log_budget_last_us_;
      const auto added =
          elapsed_us * log_budget_bytes_per_second_ / 1'000'000ULL;
      log_budget_tokens_ = std::min<std::uint64_t>(
          273, log_budget_tokens_ + added);
      log_budget_last_us_ = now_us;
    }
    if (log_budget_tokens_ < frame_size) {
      ++statistics_.log_budget_waits;
      return true;
    }
  }

  const wire::Header header{slot.type, session_id_, slot.stream_id,
                            slot.sequence, 0, timestamp_us};
  const auto encoded = wire::Encode(
      header,
      std::span<const std::byte>(slot.payload.data(), slot.payload_size),
      tx_buffer_);
  if (encoded.status != Status::Ok) {
    ++statistics_.transport_failures;
    EnterResetRequired();
    return true;
  }
  tx_size_ = encoded.bytes;
  const auto token = next_transfer_token_;
  const auto result = transport_.send(
      transport_.context,
      std::span<const std::byte>(tx_buffer_.data(), tx_size_), token);
  if (result == SendResult::Busy) {
    ++statistics_.transport_busy;
    tx_size_ = 0;
    return true;
  }
  if (result == SendResult::Failed) {
    ++statistics_.transport_failures;
    EnterResetRequired();
    return true;
  }

  ++statistics_.transport_accepted;
  in_flight_ = true;
  in_flight_is_transaction_ = false;
  in_flight_is_business_ = true;
  in_flight_is_log_ = is_log;
  in_flight_token_ = token;
  ++next_transfer_token_;
  in_flight_started_us_ = now_us;
  in_flight_type_ = slot.type;
  if (is_log) {
    log_budget_tokens_ -= frame_size;
    log_head_ = (log_head_ + 1) % log_queue_.size();
    --log_count_;
  } else {
    data_accept_seen_ = true;
    last_data_accept_us_ = now_us;
    realtime_head_ = (realtime_head_ + 1) % realtime_queue_.size();
    --realtime_count_;
  }
  return true;
}

bool Core::EnqueueControl(MessageType type, std::uint8_t session_id,
                          std::uint8_t sequence,
                          std::span<const std::byte> payload,
                          std::uint32_t timestamp_us) noexcept {
  if (control_count_ == control_queue_.size() ||
      payload.size() > kMaxPayloadSize) {
    ++statistics_.control_queue_drops;
    return false;
  }
  const auto tail = (control_head_ + control_count_) % control_queue_.size();
  auto &slot = control_queue_[tail];
  slot.type = type;
  slot.session_id = session_id;
  slot.sequence = sequence;
  slot.timestamp_us = timestamp_us;
  slot.payload_size = static_cast<std::uint8_t>(payload.size());
  slot.sample_t3_on_submit = type == MessageType::SyncResp;
  std::copy(payload.begin(), payload.end(), slot.payload.begin());
  ++control_count_;
  return true;
}

void Core::HandleHello(const wire::FrameView &frame) noexcept {
  if (role_ != Role::Device) {
    ++statistics_.session_semantic_errors;
    return;
  }

  const auto version = ReadU16(frame.payload.data());
  const auto attempt_token = ReadU64(frame.payload.data() + 2);
  const auto host_epoch_us = ReadU64(frame.payload.data() + 10);
  if (version != kProtocolVersion) {
    std::array<std::byte, 12> reject{};
    reject[0] = static_cast<std::byte>(MessageType::Hello);
    reject[1] = std::byte{1};
    WriteU16(reject.data() + 2, kProtocolVersion);
    WriteU64(reject.data() + 4, attempt_token);
    (void)EnqueueControl(MessageType::Reject, frame.header.session_id,
                         frame.header.sequence, reject, 0);
    return;
  }

  if (state_ == State::Idle) {
    session_id_ = frame.header.session_id;
    attempt_token_ = attempt_token;
    peer_epoch_us_ = host_epoch_us;
    local_epoch_us_ = transport_.now_us(transport_.context);
    local_epoch_valid_ = true;
    state_ = State::Syncing;
  } else if (frame.header.session_id != session_id_ ||
             attempt_token != attempt_token_) {
    std::array<std::byte, 12> reject{};
    reject[0] = static_cast<std::byte>(MessageType::Hello);
    reject[1] = std::byte{2};
    WriteU16(reject.data() + 2, kProtocolVersion);
    WriteU64(reject.data() + 4, attempt_token);
    (void)EnqueueControl(MessageType::Reject, frame.header.session_id,
                         frame.header.sequence, reject, 0);
    return;
  }

  std::array<std::byte, 18> acknowledgement{};
  WriteU16(acknowledgement.data(), kProtocolVersion);
  WriteU64(acknowledgement.data() + 2, attempt_token_);
  WriteU64(acknowledgement.data() + 10, local_epoch_us_);
  (void)EnqueueControl(MessageType::HelloAck, session_id_,
                       frame.header.sequence, acknowledgement, 0);
}

void Core::HandleHelloAck(const wire::FrameView &frame) noexcept {
  if (role_ != Role::Host || state_ != State::Handshaking ||
      !transaction_.active || transaction_.request_type != MessageType::Hello ||
      frame.header.session_id != session_id_ ||
      frame.header.sequence != control_sequence_ ||
      ReadU16(frame.payload.data()) != kProtocolVersion ||
      ReadU64(frame.payload.data() + 2) != attempt_token_) {
    ++statistics_.session_semantic_errors;
    return;
  }
  peer_epoch_us_ = ReadU64(frame.payload.data() + 10);
  transaction_.early_response = true;
  if (!in_flight_) {
    AcceptControlResponse();
  }
}

void Core::HandleReject(const wire::FrameView &frame) noexcept {
  const auto request_type =
      static_cast<MessageType>(std::to_integer<std::uint8_t>(frame.payload[0]));
  if (role_ == Role::Host && state_ == State::Handshaking &&
      transaction_.active && transaction_.request_type == MessageType::Hello &&
      frame.header.session_id == session_id_ &&
      frame.header.sequence == control_sequence_ &&
      request_type == MessageType::Hello &&
      ReadU64(frame.payload.data() + 4) == attempt_token_) {
    EnterResetRequired();
    return;
  }
  ++statistics_.session_semantic_errors;
}

void Core::HandleSyncRequest(const wire::FrameView &frame) noexcept {
  if (role_ != Role::Device || frame.header.session_id != session_id_ ||
      (state_ != State::Syncing && state_ != State::Active)) {
    ++statistics_.session_semantic_errors;
    return;
  }
  std::array<std::byte, 24> response{};
  std::copy(frame.payload.begin(), frame.payload.end(), response.begin());
  WriteU64(response.data() + 8, frame.receive_end_us);
  if (!EnqueueControl(MessageType::SyncResp, session_id_,
                      frame.header.sequence, response, 0)) {
    ++statistics_.sync_failures;
  }
}

void Core::HandleSyncResponse(const wire::FrameView &frame) noexcept {
  if (role_ != Role::Host ||
      (state_ != State::Syncing && state_ != State::Active) ||
      !transaction_.active ||
      transaction_.request_type != MessageType::SyncReq ||
      frame.header.session_id != transaction_.session_id ||
      frame.header.sequence != transaction_.sequence ||
      ReadU64(frame.payload.data()) != ReadU64(transaction_.payload.data())) {
    ++statistics_.session_semantic_errors;
    return;
  }

  TimeSample sample{};
  const auto status = CalculateTimeSample(
      ReadU64(frame.payload.data()), ReadU64(frame.payload.data() + 8),
      ReadU64(frame.payload.data() + 16), frame.receive_end_us, sample);
  if (status != Status::Ok || sync_sample_count_ >= sync_samples_.size()) {
    ++statistics_.sync_failures;
    transaction_.active = false;
    return;
  }
  sync_samples_[sync_sample_count_++] = {sample, frame.receive_end_us};
  transaction_.early_response = true;
  if (!in_flight_) {
    AcceptControlResponse();
  }
}

void Core::HandleStart(const wire::FrameView &frame) noexcept {
  if (role_ != Role::Device || frame.header.session_id != session_id_ ||
      (state_ != State::Syncing && state_ != State::Active)) {
    ++statistics_.session_semantic_errors;
    return;
  }
  if (EnqueueControl(MessageType::StartAck, session_id_,
                     frame.header.sequence, {}, 0) &&
      state_ != State::Active) {
    state_ = State::Active;
    active_since_us_ = transport_.now_us(transport_.context);
    last_valid_data_us_ = active_since_us_;
  }
}

void Core::HandleStartAck(const wire::FrameView &frame) noexcept {
  if (role_ == Role::Host && state_ == State::Active &&
      frame.header.session_id == session_id_) {
    return;
  }
  if (role_ != Role::Host || state_ != State::Starting ||
      !transaction_.active || transaction_.request_type != MessageType::Start ||
      frame.header.session_id != transaction_.session_id ||
      frame.header.sequence != transaction_.sequence) {
    ++statistics_.session_semantic_errors;
    return;
  }
  transaction_.early_response = true;
  if (!in_flight_) {
    AcceptControlResponse();
  }
}

void Core::HandleClose(const wire::FrameView &frame) noexcept {
  const auto now_us = transport_.now_us(transport_.context);
  if (state_ == State::Idle && close_record_valid_ &&
      frame.header.session_id == close_record_session_id_ &&
      frame.header.sequence == close_record_sequence_ &&
      now_us <= close_record_expires_us_) {
    (void)EnqueueControl(MessageType::CloseAck, frame.header.session_id,
                         frame.header.sequence, {}, 0);
    return;
  }
  if (frame.header.session_id != session_id_ ||
      (state_ != State::Active && state_ != State::Syncing &&
       state_ != State::Starting && state_ != State::Closing)) {
    ++statistics_.session_semantic_errors;
    return;
  }

  peer_close_received_ = true;
  close_record_valid_ = true;
  close_record_session_id_ = frame.header.session_id;
  close_record_sequence_ = frame.header.sequence;
  close_record_expires_us_ =
      now_us + static_cast<std::uint64_t>(timing_.close_record_timeout_ms) *
                   1000ULL;
  realtime_head_ = 0;
  realtime_count_ = 0;
  log_head_ = 0;
  log_count_ = 0;
  if (!local_close_requested_) {
    transaction_ = {};
  }
  state_ = State::Closing;
  if (EnqueueControl(MessageType::CloseAck, frame.header.session_id,
                     frame.header.sequence, {}, 0)) {
    close_ack_pending_ = true;
  }
}

void Core::HandleCloseAck(const wire::FrameView &frame) noexcept {
  if (state_ != State::Closing || !local_close_requested_ ||
      !transaction_.active || transaction_.request_type != MessageType::Close ||
      frame.header.session_id != transaction_.session_id ||
      frame.header.sequence != transaction_.sequence) {
    ++statistics_.session_semantic_errors;
    return;
  }
  transaction_.early_response = true;
  if (!in_flight_) {
    AcceptControlResponse();
  }
}

void Core::HandleEarlyData(const wire::FrameView &frame) noexcept {
  if (role_ == Role::Host && state_ == State::Starting &&
      frame.header.session_id == session_id_) {
    ++statistics_.early_data_drops;
    return;
  }
  ++statistics_.session_semantic_errors;
}

void Core::HandleData(const wire::FrameView &frame) noexcept {
  if (frame.header.session_id != session_id_) {
    ++statistics_.session_semantic_errors;
    return;
  }
  if (role_ == Role::Host && state_ == State::Starting) {
    ++statistics_.early_data_drops;
    return;
  }
  if (state_ != State::Active) {
    ++statistics_.session_semantic_errors;
    return;
  }

  ++statistics_.received_data_frames;
  last_valid_data_us_ = frame.receive_end_us;
  for (const auto &stream : streams_) {
    if (stream.registration.stream_id != frame.header.stream_id) {
      continue;
    }
    if (stream.registration.on_receive != nullptr) {
      std::uint64_t device_event_us = 0;
      const auto event_epoch_us =
          role_ == Role::Host ? peer_epoch_us_ : local_epoch_us_;
      if (event_epoch_us <= UINT64_MAX - frame.header.timestamp_us) {
        device_event_us =
            event_epoch_us + static_cast<std::uint64_t>(
                                 frame.header.timestamp_us);
      }
      const EventMetadata metadata{
          frame.header.session_id,
          frame.header.stream_id,
          frame.header.sequence,
          frame.header.timestamp_us,
          frame.receive_end_us,
          device_event_us};
      stream.registration.on_receive(stream.registration.context,
                                     frame.payload, metadata);
    }
    return;
  }
  ++statistics_.unknown_streams;
}

void Core::HandleLog(const wire::FrameView &frame) noexcept {
  if (role_ != Role::Host || frame.header.session_id != session_id_) {
    ++statistics_.session_semantic_errors;
    return;
  }
  if (state_ == State::Starting) {
    ++statistics_.early_data_drops;
    return;
  }
  if (state_ != State::Active || frame.payload.size() < 2) {
    ++statistics_.session_semantic_errors;
    return;
  }
  ++statistics_.received_log_frames;
  last_valid_data_us_ = frame.receive_end_us;
  if (on_log_ == nullptr) {
    return;
  }
  std::uint64_t device_event_us = 0;
  if (peer_epoch_us_ <= UINT64_MAX - frame.header.timestamp_us) {
    device_event_us =
        peer_epoch_us_ + static_cast<std::uint64_t>(frame.header.timestamp_us);
  }
  const EventMetadata metadata{frame.header.session_id,
                               0,
                               frame.header.sequence,
                               frame.header.timestamp_us,
                               frame.receive_end_us,
                               device_event_us};
  on_log_(log_context_, std::to_integer<std::uint8_t>(frame.payload[0]),
          std::to_integer<std::uint8_t>(frame.payload[1]),
          frame.payload.subspan(2), metadata);
}

void Core::FinishSyncRound(std::uint64_t now_us) noexcept {
  if (sync_sample_count_ == 0) {
    ++statistics_.sync_failures;
    EnterResetRequired();
    return;
  }
  std::size_t best = 0;
  for (std::size_t index = 1; index < sync_sample_count_; ++index) {
    if (sync_samples_[index].value.round_trip_us <
            sync_samples_[best].value.round_trip_us ||
        (sync_samples_[index].value.round_trip_us ==
             sync_samples_[best].value.round_trip_us &&
         sync_samples_[index].received_at_us >=
             sync_samples_[best].received_at_us)) {
      best = index;
    }
  }
  mapping_.state = MappingState::Valid;
  mapping_.session_id = session_id_;
  mapping_.host_epoch_us = local_epoch_us_;
  mapping_.device_epoch_us = peer_epoch_us_;
  mapping_.device_minus_host_us =
      sync_samples_[best].value.device_minus_host_us;
  mapping_.round_trip_us = sync_samples_[best].value.round_trip_us;
  mapping_.updated_at_us = now_us;
  mapping_.age_us = 0;
  ++mapping_.generation;
  sync_sample_count_ = 0;
  sync_round_deadline_us_ = 0;
  next_sync_round_us_ =
      now_us + static_cast<std::uint64_t>(timing_.sync_period_ms) * 1000ULL;
  if (initial_sync_round_) {
    state_ = State::Starting;
  }
}

void Core::RefreshMappingAndLiveness(std::uint64_t now_us) noexcept {
  if (mapping_.state != MappingState::Unavailable) {
    if (now_us < mapping_.updated_at_us ||
        now_us - mapping_.updated_at_us >
            static_cast<std::uint64_t>(timing_.sync_stale_ms) * 1000ULL) {
      mapping_.state = MappingState::Stale;
    } else {
      mapping_.state = MappingState::Valid;
    }
    mapping_.age_us = now_us >= mapping_.updated_at_us
                          ? now_us - mapping_.updated_at_us
                          : 0;
  }
  if (state_ == State::Active &&
      (now_us < last_valid_data_us_ ||
       now_us - last_valid_data_us_ >
           static_cast<std::uint64_t>(timing_.rx_liveness_timeout_ms) *
               1000ULL)) {
    ++statistics_.heartbeat_timeouts;
    EnterResetRequired();
  }
}

void Core::MaybeFinishNormalClose() noexcept {
  if (state_ != State::Closing || in_flight_ || close_ack_pending_) {
    return;
  }
  if (local_close_requested_ && !local_close_ack_received_) {
    return;
  }
  if (!local_close_requested_ && !peer_close_received_) {
    return;
  }
  FinishNormalClose();
}

void Core::FinishNormalClose() noexcept {
  ClearSession();
  state_ = State::Idle;
}

Status Core::MapDeviceEvent(std::uint32_t relative_event_us,
                            std::uint64_t &device_event_us,
                            std::uint64_t &host_event_us,
                            MappingState &state) const noexcept {
  state = mapping_.state;
  if (state == MappingState::Unavailable || mapping_.device_epoch_us >
                                                UINT64_MAX - relative_event_us) {
    return Status::InvalidState;
  }
  device_event_us = mapping_.device_epoch_us + relative_event_us;
  if (mapping_.device_minus_host_us >= 0) {
    const auto offset = static_cast<std::uint64_t>(
        mapping_.device_minus_host_us);
    if (device_event_us < offset) {
      return Status::TimeOutOfRange;
    }
    host_event_us = device_event_us - offset;
  } else {
    const auto offset = static_cast<std::uint64_t>(
        -(mapping_.device_minus_host_us + 1)) + 1U;
    if (device_event_us > UINT64_MAX - offset) {
      return Status::TimeOutOfRange;
    }
    host_event_us = device_event_us + offset;
  }
  return Status::Ok;
}

void Core::EnterResetRequired() noexcept {
  state_ = State::ResetRequired;
  parser_.Reset();
  transport_.abort(transport_.context);
}

void Core::ClearSession() noexcept {
  parser_.Reset();
  session_id_ = 0;
  local_epoch_us_ = 0;
  peer_epoch_us_ = 0;
  local_epoch_valid_ = false;
  hello_payload_frozen_ = false;
  transaction_ = {};
  control_head_ = 0;
  control_count_ = 0;
  realtime_head_ = 0;
  realtime_count_ = 0;
  log_head_ = 0;
  log_count_ = 0;
  log_sequence_ = 0;
  data_accept_seen_ = false;
  last_data_accept_us_ = 0;
  log_budget_last_us_ = 0;
  log_budget_tokens_ = std::min<std::uint64_t>(273, log_budget_bytes_per_second_);
  local_close_requested_ = false;
  local_close_ack_received_ = false;
  peer_close_received_ = false;
  close_ack_pending_ = false;
  sync_sample_count_ = 0;
  sync_round_deadline_us_ = 0;
  next_sync_request_us_ = 0;
  active_since_us_ = 0;
  next_sync_round_us_ = 0;
  initial_sync_round_ = false;
  last_valid_data_us_ = 0;
  mapping_ = {};
  for (auto &stream : streams_) {
    stream.next_sequence = 0;
  }
}

} // namespace rmlink
