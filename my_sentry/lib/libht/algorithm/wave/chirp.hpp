#pragma once

#include <algorithm>
#include <cmath>
#include <numbers>

#include "error_code.hpp"

namespace Wave
{
using ErrorCode = Algorithm::ErrorCode;

class Chirp
{
 public:
  Chirp() = default;

  Chirp(float start_frequency, float end_frequency, float period, float amplitude,
        float offset)
  {
    (void)setParam(start_frequency, end_frequency, period, amplitude, offset);
  }

  ErrorCode setParam(float start_frequency, float end_frequency, float period,
                     float amplitude, float offset)
  {
    param_ = {start_frequency, end_frequency, period, amplitude, offset};
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
    if (state_.finished)
    {
      return ErrorCode::OK;
    }

    const float active_dt = std::min(dt, param_.period - state_.elapsed_time);
    const float frequency_slope =
        (param_.end_frequency - param_.start_frequency) / param_.period;
    const float phase_increment = 2.0f * std::numbers::pi_v<float> *
                                  (param_.start_frequency * active_dt +
                                   frequency_slope * (state_.elapsed_time * active_dt +
                                                      0.5f * active_dt * active_dt));
    const float next_phase =
        std::fmod(state_.phase + phase_increment, 2.0f * std::numbers::pi_v<float>);
    const float next_elapsed_time = state_.elapsed_time + active_dt;

    if (!std::isfinite(phase_increment) || !std::isfinite(next_phase) ||
        !std::isfinite(next_elapsed_time))
    {
      return ErrorCode::OUT_OF_RANGE;
    }

    state_.phase = next_phase;
    state_.elapsed_time = next_elapsed_time;
    if (state_.elapsed_time >= param_.period)
    {
      state_.elapsed_time = param_.period;
      state_.output = param_.offset;
      state_.finished = true;
    }
    else
    {
      state_.output = param_.amplitude * std::sin(state_.phase) + param_.offset;
    }
    return ErrorCode::OK;
  }

  float getOutput() const { return state_.output; }

  bool isFinished() const { return state_.finished; }

  void reset()
  {
    state_ = {};
    state_.output = std::isfinite(param_.offset) ? param_.offset : 0.0f;
  }

 private:
  struct Param
  {
    float start_frequency = 0.0f;
    float end_frequency = 0.0f;
    float period = 0.0f;
    float amplitude = 0.0f;
    float offset = 0.0f;
  };

  struct State
  {
    float output = 0.0f;
    float phase = 0.0f;
    float elapsed_time = 0.0f;
    bool finished = false;
  };

  static bool isParamValid(const Param& param)
  {
    return std::isfinite(param.start_frequency) && param.start_frequency >= 0.0f &&
           std::isfinite(param.end_frequency) && param.end_frequency >= 0.0f &&
           std::isfinite(param.period) && param.period > 0.0f &&
           std::isfinite(param.amplitude) && param.amplitude >= 0.0f &&
           std::isfinite(param.offset) && std::isfinite(param.offset + param.amplitude) &&
           std::isfinite(param.offset - param.amplitude);
  }

  Param param_{};
  State state_{};
  bool is_param_valid_ = false;
};
}  // namespace Wave
