#include "control/gimbal_sys_id.hpp"

#include <algorithm>
#include <cmath>

namespace {
constexpr float kControlPeriodSeconds =
    static_cast<float>(AppConfig::Gimbal::kControlPeriodMs) / 1000.0F;
}

GimbalSystemIdentification::GimbalSystemIdentification() { Reset(); }

const AppConfig::Gimbal::SysIdParam&
GimbalSystemIdentification::Param() const {
  if constexpr (GIMBAL_SYS_ID_AXIS == SYS_ID_AXIS_YAW) {
    return AppConfig::Gimbal::kYawSysIdParam;
  } else {
    return AppConfig::Gimbal::kPitchSysIdParam;
  }
}

float GimbalSystemIdentification::Update(
    float yaw_angle_rad, float pitch_angle_rad,
    float selected_axis_velocity_rad_per_sec) {
  const float selected_angle =
      GIMBAL_SYS_ID_AXIS == SYS_ID_AXIS_YAW ? yaw_angle_rad : pitch_angle_rad;

  if (!std::isfinite(yaw_angle_rad) || !std::isfinite(pitch_angle_rad) ||
      !std::isfinite(selected_axis_velocity_rad_per_sec)) {
    Reset();
    return 0.0F;
  }

  if (!entered_) {
    yaw_start_angle_rad_ = yaw_angle_rad;
    pitch_start_angle_rad_ = pitch_angle_rad;
    chirp_.set_param(Param().start_freq_hz, Param().end_freq_hz,
                     Param().duration_s, Param().amplitude_nm, 0.0F);
    entered_ = true;
    active_ = true;
    finished_ = false;
    safety_tripped_ = false;
    elapsed_s_ = 0.0F;
  }

  if (!active_) return 0.0F;

  const float start_angle = GIMBAL_SYS_ID_AXIS == SYS_ID_AXIS_YAW
                                ? yaw_start_angle_rad_
                                : pitch_start_angle_rad_;
  if (std::abs(selected_angle - start_angle) > Param().max_angle_rad) {
    AbortForSafety();
    return 0.0F;
  }

  chirp_.compute(kControlPeriodSeconds);
  elapsed_s_ += kControlPeriodSeconds;
  if (chirp_.is_finished()) {
    active_ = false;
    finished_ = true;
    chirp_.reset();
    return 0.0F;
  }

  return std::clamp(chirp_.get_output(), -Param().output_limit_nm,
                    Param().output_limit_nm);
}

void GimbalSystemIdentification::Reset() {
  chirp_.reset();
  entered_ = false;
  active_ = false;
  finished_ = false;
  safety_tripped_ = false;
  elapsed_s_ = 0.0F;
  yaw_start_angle_rad_ = 0.0F;
  pitch_start_angle_rad_ = 0.0F;
}

void GimbalSystemIdentification::AbortForSafety() {
  chirp_.reset();
  active_ = false;
  safety_tripped_ = true;
}

GimbalSysIdTelemetry GimbalSystemIdentification::Telemetry(
    float final_input_nm, float selected_axis_angle_rad,
    float selected_axis_velocity_rad_per_sec) const {
  return {
      .time_s = elapsed_s_,
      .input_nm = final_input_nm,
      .angle_rad = selected_axis_angle_rad,
      .angular_velocity_rad_per_sec = selected_axis_velocity_rad_per_sec,
      .axis = static_cast<uint8_t>(GIMBAL_SYS_ID_AXIS),
      .active = static_cast<uint8_t>(active_),
      .finished = static_cast<uint8_t>(finished_),
      .safety_tripped = static_cast<uint8_t>(safety_tripped_),
  };
}
