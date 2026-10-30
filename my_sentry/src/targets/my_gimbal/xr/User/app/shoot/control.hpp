#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace UserApp::Shoot
{

/**
 * PID implementation kept intentionally compatible with steering_sentry.
 *
 * The reference project's PID is discrete per control iteration: its integral is
 * `sum(error)` and its derivative is `error[k]-error[k-1]` without multiplying or
 * dividing by dt.  The existing shoot.toml gains were tuned for that exact form at
 * a 1 kHz task rate, therefore using LibHT's dt-aware PID directly would change the
 * controller by roughly three orders of magnitude.
 */
class LegacyPid
{
 public:
  struct Param
  {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float deadband = 0.0f;
    float integral_limit = 0.0f;
    float output_limit = 0.0f;
  };

  LegacyPid() = default;
  explicit LegacyPid(const Param& param) : param_(param) {}

  void SetParam(const Param& param)
  {
    param_ = param;
    Reset();
  }

  // Match the reference PID's set_param(): changing gains at run time keeps
  // the accumulated controller state.  This matters when the feeder switches
  // between velocity and position control for big-buff shots.
  void UpdateParam(const Param& param) { param_ = param; }

  float Update(float reference, float feedback)
  {
    float error = reference - feedback;
    if (std::abs(error) < param_.deadband)
    {
      error = 0.0f;
    }

    integral_ += error;
    integral_ = std::clamp(integral_, -param_.integral_limit,
                           param_.integral_limit);

    output_ = param_.kp * error + param_.ki * integral_ +
              param_.kd * (error - last_error_);
    output_ = std::clamp(output_, -param_.output_limit, param_.output_limit);
    last_error_ = error;
    return output_;
  }

  void Reset()
  {
    integral_ = 0.0f;
    last_error_ = 0.0f;
    output_ = 0.0f;
  }

  [[nodiscard]] float Output() const { return output_; }

 private:
  Param param_{};
  float integral_ = 0.0f;
  float last_error_ = 0.0f;
  float output_ = 0.0f;
};

enum class FeederControlMode
{
  VELOCITY,
  POSITION,
};

/** Cascaded feeder controller equivalent to the reference PluckAlgorithm. */
class FeederController
{
 public:
  void SetParams(const LegacyPid::Param& velocity_param,
                 const LegacyPid::Param& position_param)
  {
    velocity_param_ = velocity_param;
    position_param_ = position_param;
    velocity_pid_.SetParam(velocity_param_);
    position_pid_.SetParam(position_param_);
  }

  void SetVelocityKi(float ki)
  {
    velocity_param_.ki = ki;
    velocity_pid_.UpdateParam(velocity_param_);
  }

  void SetMode(FeederControlMode mode)
  {
    if (mode_ == mode)
    {
      return;
    }
    mode_ = mode;
    velocity_pid_.UpdateParam(velocity_param_);
    if (mode_ == FeederControlMode::POSITION)
    {
      position_pid_.UpdateParam(position_param_);
    }
    else
    {
      position_pid_.Reset();
    }
  }

  float Update(float reference, float position_feedback, float velocity_feedback)
  {
    if (mode_ == FeederControlMode::VELOCITY)
    {
      return velocity_pid_.Update(reference, velocity_feedback);
    }

    const float velocity_reference =
        position_pid_.Update(reference, position_feedback);
    return velocity_pid_.Update(velocity_reference, velocity_feedback);
  }

  void Reset()
  {
    velocity_pid_.Reset();
    position_pid_.Reset();
  }

  [[nodiscard]] FeederControlMode Mode() const { return mode_; }

 private:
  LegacyPid velocity_pid_{};
  LegacyPid position_pid_{};
  LegacyPid::Param velocity_param_{};
  LegacyPid::Param position_param_{};
  FeederControlMode mode_ = FeederControlMode::VELOCITY;
};

}  // namespace UserApp::Shoot
