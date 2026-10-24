#pragma once

#include "pid/PID.hpp"

namespace UserApp::Gimbal
{

using PidParameters = PID::Param;

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
  float ComputeVelocity(float velocity_target, float velocity_feedback,
                        float dt_s, float acceleration_feedforward = 0.0f);
  void Reset();

 private:
  PID::Controller position_{PID::Param{}};
  PID::Controller velocity_{PID::Param{}};
  float position_feedforward_ = 0.0f;
  float previous_position_target_ = 0.0f;
  bool feedforward_initialized_ = false;
};

}  // namespace UserApp::Gimbal
