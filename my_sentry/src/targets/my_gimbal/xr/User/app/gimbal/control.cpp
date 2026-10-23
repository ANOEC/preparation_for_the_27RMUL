#include "app/gimbal/control.hpp"

namespace UserApp::Gimbal
{

float CascadeController::PidState::Compute(float target, float feedback, float)
{
  float error = target - feedback;
  if (std::abs(error) < parameters.deadband)
  {
    error = 0.0f;
  }

  // The imported gains are discrete, per-control-cycle gains from the
  // reference project, so integration and differentiation intentionally do
  // not scale with wall-clock dt here.
  integral += error;
  integral = std::clamp(integral, -parameters.integral_limit,
                        parameters.integral_limit);

  const float derivative = initialized ? error - previous_error : 0.0f;
  previous_error = error;
  initialized = true;

  float output = parameters.kp * error + parameters.ki * integral +
                 parameters.kd * derivative;
  if (parameters.output_limit > 0.0f)
  {
    output = std::clamp(output, -parameters.output_limit, parameters.output_limit);
  }
  return output;
}

void CascadeController::PidState::Reset()
{
  integral = 0.0f;
  previous_error = 0.0f;
  initialized = false;
}

void CascadeController::Configure(const PidParameters& position,
                                  const PidParameters& velocity,
                                  float position_feedforward)
{
  position_.parameters = position;
  velocity_.parameters = velocity;
  position_feedforward_ = position_feedforward;
  Reset();
}

float CascadeController::Compute(float position_target, float position_feedback,
                                 float velocity_feedback, float dt_s,
                                 float velocity_feedforward,
                                 float acceleration_feedforward)
{
  const float target_step = position_target - previous_position_target_;
  previous_position_target_ = position_target;
  feedforward_initialized_ = true;

  const float velocity_target =
      position_.Compute(position_target, position_feedback, dt_s) +
      velocity_feedforward + position_feedforward_ * target_step;
  return velocity_.Compute(velocity_target, velocity_feedback, dt_s) +
         acceleration_feedforward;
}

void CascadeController::Reset()
{
  position_.Reset();
  velocity_.Reset();
  previous_position_target_ = 0.0f;
  feedforward_initialized_ = false;
}

}  // namespace UserApp::Gimbal
