#include "gimbal_algorithm.hpp"

#include <algorithm>
#include <cmath>

namespace app::gimbal {

void GimbalAlgorithm::set_pid_params(const PID::Param& position,
                                     const PID::Param& velocity) {
  position_pid_.set_param(position);
  velocity_pid_.set_param(velocity);
}

void GimbalAlgorithm::set_ladrc1_params(float b0, float ts,
                                        float output_limit, float r) {
  velocity_ladrc_.set_param(b0, ts, output_limit, r);
}

void GimbalAlgorithm::set_ladrc2_params(float b0, float ts,
                                        float output_limit, float r) {
  position_ladrc_.set_param(b0, ts, output_limit, r);
}

void GimbalAlgorithm::set_ladrc2_observer_input(float input) {
  position_ladrc_.set_observer_input(input);
}

void GimbalAlgorithm::set_ladrc2_disturbance_compensation_limit(
    float input_limit) {
  position_ladrc_.set_disturbance_compensation_limit(input_limit);
}

void GimbalAlgorithm::set_ff_params(float position_gain) {
  position_ff_.set_gain(position_gain);
}

void GimbalAlgorithm::algorithm_compute(float position_target,
                                        float position_feedback,
                                        float velocity_feedback,
                                        float dt,
                                        float velocity_feedforward,
                                        float acceleration_feedforward) {
  position_pid_.compute(position_target, position_feedback);
  position_ff_.compute(position_target, dt);

  const float velocity_target_unclamped =
      position_pid_.out() + velocity_feedforward + position_ff_.out();
  const float velocity_target_limit = position_pid_.get_param().output_limit_;
  const float velocity_target =
      std::clamp(velocity_target_unclamped, -velocity_target_limit,
                 velocity_target_limit);
  velocity_pid_.compute(velocity_target, velocity_feedback);

  const float output_limit = velocity_pid_.get_param().output_limit_;
  const float final_output = std::clamp(
      velocity_pid_.out() + acceleration_feedforward, -output_limit,
      output_limit);

  debug_ = {
      .position_target = position_target,
      .position_feedback = position_feedback,
      .position_error = position_pid_.error(),
      .position_pid_output = position_pid_.out(),
      .velocity_target = velocity_target,
      .velocity_feedback = velocity_feedback,
      .velocity_error = velocity_pid_.error(),
      .velocity_pid_output = velocity_pid_.out(),
      .velocity_feedforward = velocity_feedforward,
      .acceleration_feedforward = acceleration_feedforward,
      .position_feedforward = position_ff_.out(),
      .final_output = final_output,
  };
}

void GimbalAlgorithm::algorithm_compute_pid_ladrc1(
    float position_target, float position_feedback, float velocity_feedback,
    float dt, float velocity_feedforward) {
  position_pid_.compute(position_target, position_feedback);
  position_ff_.compute(position_target, dt);

  const float velocity_target_unclamped =
      position_pid_.out() + velocity_feedforward + position_ff_.out();
  const float velocity_target_limit = position_pid_.get_param().output_limit_;
  const float velocity_target =
      std::clamp(velocity_target_unclamped, -velocity_target_limit,
                 velocity_target_limit);
  const float output =
      velocity_ladrc_.compute(velocity_target, velocity_feedback, dt);

  debug_ = {
      .position_target = position_target,
      .position_feedback = position_feedback,
      .position_error = position_pid_.error(),
      .position_pid_output = position_pid_.out(),
      .velocity_target = velocity_target,
      .velocity_feedback = velocity_feedback,
      .velocity_error = velocity_target - velocity_feedback,
      .velocity_pid_output = output,
      .velocity_feedforward = velocity_feedforward,
      .acceleration_feedforward = 0.0F,
      .position_feedforward = position_ff_.out(),
      .final_output = output,
      .ladrc_z1 = velocity_ladrc_.z1(),
      .ladrc_z2 = velocity_ladrc_.z2(),
      .ladrc_z3 = velocity_ladrc_.z3(),
  };
}

void GimbalAlgorithm::algorithm_compute_ladrc2(float position_target,
                                               float position_feedback,
                                               float velocity_feedback,
                                               float dt) {
  const float output =
      position_ladrc_.compute(position_target, position_feedback, dt);
  debug_ = {
      .position_target = position_target,
      .position_feedback = position_feedback,
      .position_error = position_target - position_feedback,
      .position_pid_output = 0.0F,
      .velocity_target = 0.0F,
      .velocity_feedback = velocity_feedback,
      .velocity_error = 0.0F,
      .velocity_pid_output = output,
      .velocity_feedforward = 0.0F,
      .acceleration_feedforward = 0.0F,
      .position_feedforward = 0.0F,
      .final_output = output,
      .ladrc_z1 = position_ladrc_.z1(),
      .ladrc_z2 = position_ladrc_.z2(),
      .ladrc_z3 = position_ladrc_.z3(),
  };
}

void GimbalAlgorithm::reset() {
  position_pid_.reset();
  velocity_pid_.reset();
  velocity_ladrc_.reset();
  position_ladrc_.reset();
  position_ff_.reset();
  debug_ = {};
}

float GimbalAlgorithm::out() const { return debug_.final_output; }

const GimbalAlgorithm::Debug& GimbalAlgorithm::debug() const { return debug_; }

}  // namespace app::gimbal
