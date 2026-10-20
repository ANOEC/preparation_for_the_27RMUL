#pragma once

#include <cmath>

#include "error_code.hpp"

namespace Wave
{
using ErrorCode = Algorithm::ErrorCode;

class SquareWaveGenerator
{
 public:
  SquareWaveGenerator() = default;

  SquareWaveGenerator(float peak, float offset, float frequency = 50.0f,
                      float duty_cycle = 0.5f)
  {
    (void)setParam(peak, offset, frequency, duty_cycle);
  }

  ErrorCode setParam(float peak, float offset, float frequency, float duty_cycle)
  {
    param_ = {peak, offset, frequency, duty_cycle};
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

    const float phase_increment = param_.frequency * dt;
    const float next_phase = std::fmod(state_.phase + phase_increment, 1.0f);
    if (!std::isfinite(phase_increment) || !std::isfinite(next_phase))
    {
      return ErrorCode::OUT_OF_RANGE;
    }

    state_.phase = next_phase;
    updateOutput();
    return ErrorCode::OK;
  }

  ErrorCode setFrequency(float frequency)
  {
    auto next_param = param_;
    next_param.frequency = frequency;
    if (!isParamValid(next_param))
    {
      return ErrorCode::ARG_ERR;
    }
    param_ = next_param;
    is_param_valid_ = true;
    return ErrorCode::OK;
  }

  ErrorCode setDutyCycle(float duty_cycle)
  {
    auto next_param = param_;
    next_param.duty_cycle = duty_cycle;
    if (!isParamValid(next_param))
    {
      return ErrorCode::ARG_ERR;
    }
    param_ = next_param;
    is_param_valid_ = true;
    updateOutput();
    return ErrorCode::OK;
  }

  ErrorCode setPeak(float peak)
  {
    auto next_param = param_;
    next_param.peak = peak;
    if (!isParamValid(next_param))
    {
      return ErrorCode::ARG_ERR;
    }
    param_ = next_param;
    is_param_valid_ = true;
    updateOutput();
    return ErrorCode::OK;
  }

  ErrorCode setOffset(float offset)
  {
    auto next_param = param_;
    next_param.offset = offset;
    if (!isParamValid(next_param))
    {
      return ErrorCode::ARG_ERR;
    }
    param_ = next_param;
    is_param_valid_ = true;
    updateOutput();
    return ErrorCode::OK;
  }

  void reset()
  {
    state_ = {};
    if (std::isfinite(param_.offset) && std::isfinite(param_.peak) &&
        std::isfinite(param_.duty_cycle))
    {
      updateOutput();
    }
  }

  float getOutput() const { return state_.output; }

 private:
  struct Param
  {
    float peak = 0.0f;
    float offset = 0.0f;
    float frequency = 0.0f;
    float duty_cycle = 0.0f;
  };

  struct State
  {
    float output = 0.0f;
    float phase = 0.0f;
  };

  static bool isParamValid(const Param& param)
  {
    return std::isfinite(param.peak) && param.peak >= 0.0f &&
           std::isfinite(param.offset) && std::isfinite(param.offset + param.peak) &&
           std::isfinite(param.frequency) && param.frequency > 0.0f &&
           std::isfinite(param.duty_cycle) && param.duty_cycle >= 0.0f &&
           param.duty_cycle <= 1.0f;
  }

  void updateOutput()
  {
    state_.output =
        state_.phase < param_.duty_cycle ? param_.peak + param_.offset : param_.offset;
  }

  Param param_{};
  State state_{};
  bool is_param_valid_ = false;
};
}  // namespace Wave
