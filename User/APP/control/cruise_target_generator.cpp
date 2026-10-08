#include "cruise_target_generator.hpp"

#include <cmath>
#include <numbers>

namespace app::gimbal {

void CruiseTargetGenerator::Configure(float yaw_amplitude_rad,
                                      float yaw_frequency_hz,
                                      float pitch_amplitude_rad,
                                      float pitch_frequency_hz) {
  yaw_amplitude_rad_ = yaw_amplitude_rad;
  yaw_frequency_hz_ = yaw_frequency_hz;
  pitch_amplitude_rad_ = pitch_amplitude_rad;
  pitch_frequency_hz_ = pitch_frequency_hz;
}

void CruiseTargetGenerator::Start(float yaw_center_rad,
                                  float pitch_center_rad, uint32_t now_ms) {
  yaw_center_rad_ = yaw_center_rad;
  pitch_center_rad_ = pitch_center_rad;
  start_ms_ = now_ms;
  active_ = true;
}

void CruiseTargetGenerator::Stop() { active_ = false; }

CruiseTarget CruiseTargetGenerator::Update(uint32_t now_ms) const {
  if (!active_) {
    return {
        .yaw_angle_rad = yaw_center_rad_,
        .yaw_velocity_rad_per_sec = 0.0F,
        .pitch_angle_rad = pitch_center_rad_,
    };
  }

  // Unsigned subtraction also handles the millisecond counter wrapping.
  const float elapsed_seconds =
      static_cast<float>(now_ms - start_ms_) * 0.001F;
  constexpr float kTwoPi = 2.0F * std::numbers::pi_v<float>;
  const float yaw_angular_frequency = kTwoPi * yaw_frequency_hz_;
  const float pitch_angular_frequency = kTwoPi * pitch_frequency_hz_;
  return {
      .yaw_angle_rad =
          yaw_center_rad_ +
          yaw_amplitude_rad_ *
              std::sin(yaw_angular_frequency * elapsed_seconds),
      .yaw_velocity_rad_per_sec =
          yaw_amplitude_rad_ * yaw_angular_frequency *
          std::cos(yaw_angular_frequency * elapsed_seconds),
      .pitch_angle_rad =
          pitch_center_rad_ +
          pitch_amplitude_rad_ *
              std::sin(pitch_angular_frequency * elapsed_seconds),
  };
}

bool CruiseTargetGenerator::active() const { return active_; }

}  // namespace app::gimbal
