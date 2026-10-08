#pragma once

#include <cstdint>

namespace app::gimbal {

struct CruiseTarget {
  float yaw_angle_rad = 0.0F;
  float yaw_velocity_rad_per_sec = 0.0F;
  float pitch_angle_rad = 0.0F;
};

// Generates angle references and the analytical Yaw velocity of the
// sinusoidal trajectory. It remains independent from the feedback controller.
class CruiseTargetGenerator {
 public:
  void Configure(float yaw_amplitude_rad, float yaw_frequency_hz,
                 float pitch_amplitude_rad, float pitch_frequency_hz);
  void Start(float yaw_center_rad, float pitch_center_rad,
             uint32_t now_ms);
  void Stop();
  [[nodiscard]] CruiseTarget Update(uint32_t now_ms) const;
  [[nodiscard]] bool active() const;

 private:
  float yaw_amplitude_rad_ = 0.0F;
  float yaw_frequency_hz_ = 0.0F;
  float pitch_amplitude_rad_ = 0.0F;
  float pitch_frequency_hz_ = 0.0F;
  float yaw_center_rad_ = 0.0F;
  float pitch_center_rad_ = 0.0F;
  uint32_t start_ms_ = 0;
  bool active_ = false;
};

}  // namespace app::gimbal
