#pragma once

#include <algorithm>
#include <cmath>

namespace UserApp::Gimbal
{

struct PidParameters
{
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  float deadband = 0.0f;
  float integral_limit = 0.0f;
  float output_limit = 0.0f;
};

/** Position/velocity cascade retained from the reference gimbal controller. */
class CascadeController
{
 public:
  void Configure(const PidParameters& position, const PidParameters& velocity,
                 float position_feedforward = 0.0f);
  float Compute(float position_target, float position_feedback,
                float velocity_feedback, float dt_s,
                float velocity_feedforward = 0.0f,
                float acceleration_feedforward = 0.0f);
  void Reset();

 private:
  struct PidState
  {
    PidParameters parameters{};
    float integral = 0.0f;
    float previous_error = 0.0f;
    bool initialized = false;

    float Compute(float target, float feedback, float dt_s);
    void Reset();
  };

  PidState position_{};
  PidState velocity_{};
  float position_feedforward_ = 0.0f;
  float previous_position_target_ = 0.0f;
  bool feedforward_initialized_ = false;
};

}  // namespace UserApp::Gimbal
