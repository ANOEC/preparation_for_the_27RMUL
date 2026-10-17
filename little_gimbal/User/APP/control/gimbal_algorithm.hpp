#pragma once

#include "algorithm/control/LADRC/ladrc.hpp"
#include "algorithm/control/PID/pid.hpp"
#include "algorithm/control/forward_feedback/forward_feedback.hpp"

namespace app::gimbal {

class GimbalAlgorithm {
 public:
  struct Debug {
    float position_target = 0.0F;
    float position_feedback = 0.0F;
    float position_error = 0.0F;
    float position_pid_output = 0.0F;
    float velocity_target = 0.0F;
    float velocity_feedback = 0.0F;
    float velocity_error = 0.0F;
    float velocity_pid_output = 0.0F;
    float velocity_feedforward = 0.0F;
    float acceleration_feedforward = 0.0F;
    float position_feedforward = 0.0F;
    float final_output = 0.0F;
    float ladrc_z1 = 0.0F;
    float ladrc_z2 = 0.0F;
    float ladrc_z3 = 0.0F;
  };

  using PID = algorithm::control::pid::PID;
  using LADRC1 = algorithm::control::ladrc::LADRC<1>;
  using LADRC2 = algorithm::control::ladrc::LADRC<2>;

  GimbalAlgorithm() = default;

  void set_pid_params(const PID::Param& position, const PID::Param& velocity);
  void set_ladrc1_params(float b0, float ts, float output_limit, float r);
  void set_ladrc2_params(float b0, float ts, float output_limit, float r);
  void set_ladrc2_observer_input(float input);
  void set_ladrc2_disturbance_compensation_limit(float input_limit);
  void set_ff_params(float position_gain);
  // Existing double-loop PID baseline. Keep this path unchanged.
  void algorithm_compute(float position_target, float position_feedback,
                         float velocity_feedback, float dt,
                         float velocity_feedforward = 0.0F,
                         float acceleration_feedforward = 0.0F);
  void algorithm_compute_pid_ladrc1(float position_target,
                                    float position_feedback,
                                    float velocity_feedback, float dt,
                                    float velocity_feedforward = 0.0F);
  void algorithm_compute_ladrc2(float position_target,
                                float position_feedback,
                                float velocity_feedback, float dt);
  void reset();

  [[nodiscard]] float out() const;
  [[nodiscard]] const Debug& debug() const;

 private:
  PID position_pid_;
  PID velocity_pid_;
  LADRC1 velocity_ladrc_;
  LADRC2 position_ladrc_;
  algorithm::control::forward_feedback::ForwardFeedback position_ff_;
  Debug debug_{};
};

}  // namespace app::gimbal
