#pragma once

#include <cmath>

#include "error_code.hpp"

namespace Wave
{
using ErrorCode = Algorithm::ErrorCode;

class StepWaveGenerator
{
 public:
  StepWaveGenerator() = default;

  StepWaveGenerator(float peak, float offset, float duration = 0.0f,
                    float delay_time = 0.0f)
  {
    (void)setParam(peak, offset, duration, delay_time);
  }

  ErrorCode setParam(float peak, float offset, float duration, float delay_time)
  {
    param_ = {peak, offset, duration, delay_time};
    is_param_valid_ = isParamValid(param_);
    reset();
    return is_param_valid_ ? ErrorCode::OK : ErrorCode::ARG_ERR;
  }

  ErrorCode update(float dt)
  {
    if (!is_param_valid_ || !std::isfinite(dt) || dt <= 0.0f)
    {
      return ErrorCode::ARG_ERR;
    }

    const float next_elapsed_time = state_.elapsed_time + dt;
    if (!std::isfinite(next_elapsed_time))
    {
      return ErrorCode::OUT_OF_RANGE;
    }

    state_.elapsed_time = next_elapsed_time;
    updateOutput();
    return ErrorCode::OK;
  }

  void reset()
  {
    state_ = {};
    state_.output = std::isfinite(param_.offset) ? param_.offset : 0.0f;
  }

  float getOutput() const { return state_.output; }

 private:
  struct Param
  {
    float peak = 0.0f;
    float offset = 0.0f;
    float duration = 0.0f;
    float delay_time = 0.0f;
  };

  struct State
  {
    float output = 0.0f;
    float elapsed_time = 0.0f;
  };

  static bool isParamValid(const Param& param)
  {
    return std::isfinite(param.peak) && std::isfinite(param.offset) &&
           std::isfinite(param.offset + param.peak) && std::isfinite(param.duration) &&
           param.duration >= 0.0f && std::isfinite(param.delay_time) &&
           param.delay_time >= 0.0f && std::isfinite(param.delay_time + param.duration);
  }

  void updateOutput()
  {
    const bool delay_elapsed = state_.elapsed_time >= param_.delay_time;
    const bool duration_active =
        param_.duration == 0.0f ||
        state_.elapsed_time < param_.delay_time + param_.duration;
    state_.output =
        delay_elapsed && duration_active ? param_.peak + param_.offset : param_.offset;
  }

  Param param_{};
  State state_{};
  bool is_param_valid_ = false;
};
}  // namespace Wave
