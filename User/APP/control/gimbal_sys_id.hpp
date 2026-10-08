#pragma once

#include <cstdint>

#include "algorithm/math/wave/chirp/chirp.hpp"
#include "config/app_config.hpp"

struct GimbalSysIdTelemetry {
  float time_s;
  float input_nm;
  float angle_rad;
  float angular_velocity_rad_per_sec;
  uint8_t axis;
  uint8_t active;
  uint8_t finished;
  uint8_t safety_tripped;
};

class GimbalSystemIdentification {
 public:
  GimbalSystemIdentification();

  float Update(float yaw_angle_rad, float pitch_angle_rad,
               float selected_axis_velocity_rad_per_sec);
  void Reset();
  void AbortForSafety();

  bool Entered() const { return entered_; }
  bool Active() const { return active_; }
  float YawStartAngle() const { return yaw_start_angle_rad_; }
  float PitchStartAngle() const { return pitch_start_angle_rad_; }
  GimbalSysIdTelemetry Telemetry(float final_input_nm,
                                 float selected_axis_angle_rad,
                                 float selected_axis_velocity_rad_per_sec) const;

 private:
  const AppConfig::Gimbal::SysIdParam& Param() const;

  algorithm::math::wave::chirp::Chirp chirp_;
  bool entered_ = false;
  bool active_ = false;
  bool finished_ = false;
  bool safety_tripped_ = false;
  float elapsed_s_ = 0.0F;
  float yaw_start_angle_rad_ = 0.0F;
  float pitch_start_angle_rad_ = 0.0F;
};
