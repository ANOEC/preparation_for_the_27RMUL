#include "app/gimbal/control.hpp"

namespace UserApp::Gimbal
{

void CascadeController::Configure(const PidParameters& position,
                                  const PidParameters& velocity,
                                  float position_feedforward)
{
  (void)position_.setParam(position);
  (void)velocity_.setParam(velocity);
  position_feedforward_ = position_feedforward;
  Reset();
}

float CascadeController::Compute(float position_target, float position_feedback,
                                 float velocity_feedback, float dt_s,
                                 float velocity_feedforward,
                                 float acceleration_feedforward)
{
  auto next_position = position_;
  auto next_velocity = velocity_;
  const float target_step = position_target - previous_position_target_;

  if (next_position.update(position_target, position_feedback, dt_s) !=
      PID::ErrorCode::OK)
  {
    return 0.0f;
  }

  const float velocity_target = next_position.getOutput() +
                                velocity_feedforward +
                                position_feedforward_ * target_step;
  if (next_velocity.update(velocity_target, velocity_feedback, dt_s) !=
      PID::ErrorCode::OK)
  {
    return 0.0f;
  }

  position_ = next_position;
  velocity_ = next_velocity;
  previous_position_target_ = position_target;
  feedforward_initialized_ = true;
  return velocity_.getOutput() + acceleration_feedforward;
}

float CascadeController::ComputeVelocity(float velocity_target,
                                         float velocity_feedback, float dt_s,
                                         float acceleration_feedforward)
{
  auto next_velocity = velocity_;
  if (next_velocity.update(velocity_target, velocity_feedback, dt_s) !=
      PID::ErrorCode::OK)
  {
    return 0.0f;
  }

  velocity_ = next_velocity;
  return velocity_.getOutput() + acceleration_feedforward;
}

void CascadeController::Reset()
{
  position_.reset();
  velocity_.reset();
  previous_position_target_ = 0.0f;
  feedforward_initialized_ = false;
}

}  // namespace UserApp::Gimbal
