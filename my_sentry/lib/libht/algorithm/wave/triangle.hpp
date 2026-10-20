#pragma once

#include <cmath>

#include "error_code.hpp"

namespace Wave
{
using ErrorCode = Algorithm::ErrorCode;

class Triangle
{
 public:
  Triangle() = default;

  Triangle(float period, float amplitude, float offset)
  {
    (void)setParam(period, amplitude, offset);
  }

  ErrorCode setParam(float period, float amplitude, float offset)
  {
    param_ = {period, amplitude, offset};
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

    const float phase_increment = dt / param_.period;
    const float next_phase = std::fmod(state_.phase + phase_increment, 1.0f);
    if (!std::isfinite(phase_increment) || !std::isfinite(next_phase))
    {
      return ErrorCode::OUT_OF_RANGE;
    }

    state_.phase = next_phase;
    updateOutput();
    return ErrorCode::OK;
  }

  float getOutput() const { return state_.output; }

  void reset()
  {
    state_ = {};
    state_.output = std::isfinite(param_.offset) ? param_.offset : 0.0f;
  }

 private:
  struct Param
  {
    float period = 0.0f;
    float amplitude = 0.0f;
    float offset = 0.0f;
  };

  struct State
  {
    float output = 0.0f;
    float phase = 0.0f;
  };

  static bool isParamValid(const Param& param)
  {
    return std::isfinite(param.period) && param.period > 0.0f &&
           std::isfinite(param.amplitude) && param.amplitude >= 0.0f &&
           std::isfinite(param.offset) && std::isfinite(param.offset + param.amplitude) &&
           std::isfinite(param.offset - param.amplitude);
  }

  void updateOutput()
  {
    const float phase = state_.phase;
    if (phase < 0.25f)
    {
      state_.output = 4.0f * param_.amplitude * phase + param_.offset;
    }
    else if (phase < 0.75f)
    {
      state_.output = -4.0f * param_.amplitude * (phase - 0.5f) + param_.offset;
    }
    else
    {
      state_.output = 4.0f * param_.amplitude * (phase - 1.0f) + param_.offset;
    }
  }

  Param param_{};
  State state_{};
  bool is_param_valid_ = false;
};
}  // namespace Wave
