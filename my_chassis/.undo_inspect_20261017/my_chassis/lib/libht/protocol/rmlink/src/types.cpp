#include "rmlink/types.hpp"

#include <limits>

namespace rmlink {

Status ValidateConfig(const Config &config) noexcept {
  if (config.realtime_rate_limit_fps > 1000 ||
      config.realtime_queue_depth == 0 || config.control_queue_depth < 2 ||
      (config.role == Role::Device && config.log_queue_depth == 0) ||
      config.timing.control_response_timeout_ms == 0 ||
      config.timing.initial_sync_timeout_ms == 0 ||
      config.timing.sync_sample_count == 0 ||
      config.timing.sync_sample_spacing_ms == 0 ||
      config.timing.sync_period_ms == 0 || config.timing.sync_stale_ms == 0 ||
      config.timing.rx_liveness_timeout_ms == 0 ||
      config.timing.partial_frame_timeout_ms == 0 ||
      config.timing.tx_completion_timeout_ms == 0 ||
      config.timing.close_record_timeout_ms == 0 || config.streams.empty()) {
    return Status::InvalidConfig;
  }

  for (std::size_t index = 0; index < config.streams.size(); ++index) {
    const auto &stream = config.streams[index];
    if (stream.on_receive == nullptr) {
      return Status::InvalidConfig;
    }
    for (std::size_t other = 0; other < index; ++other) {
      if (config.streams[other].stream_id == stream.stream_id) {
        return Status::InvalidConfig;
      }
    }
  }
  return Status::Ok;
}

Status ToRelativeTimestamp(std::uint64_t local_epoch_us,
                           std::uint64_t event_us, std::uint64_t now_us,
                           std::uint32_t &relative_us) noexcept {
  if (now_us < local_epoch_us || event_us < local_epoch_us ||
      event_us > now_us || now_us - local_epoch_us > kMaxSessionDurationUs ||
      event_us - local_epoch_us > kMaxSessionDurationUs) {
    return Status::TimeOutOfRange;
  }
  relative_us = static_cast<std::uint32_t>(event_us - local_epoch_us);
  return Status::Ok;
}

Status CalculateTimeSample(std::uint64_t t1, std::uint64_t t2,
                           std::uint64_t t3, std::uint64_t t4,
                           TimeSample &sample) noexcept {
  constexpr auto signed_max =
      static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max());
  if (t1 > signed_max || t2 > signed_max || t3 > signed_max ||
      t4 > signed_max || t4 < t1 || t3 < t2) {
    return Status::TimeOutOfRange;
  }

  const auto host_elapsed = t4 - t1;
  const auto device_elapsed = t3 - t2;
  if (host_elapsed > signed_max || device_elapsed > signed_max ||
      device_elapsed > host_elapsed) {
    return Status::TimeOutOfRange;
  }

  const auto first = static_cast<std::int64_t>(t2) -
                     static_cast<std::int64_t>(t1);
  const auto second = static_cast<std::int64_t>(t3) -
                      static_cast<std::int64_t>(t4);
  sample.device_minus_host_us =
      first / 2 + second / 2 + (first % 2 + second % 2) / 2;
  sample.round_trip_us = host_elapsed - device_elapsed;
  return Status::Ok;
}

} // namespace rmlink
