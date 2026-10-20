#pragma once

#include <cmath>
#include <numbers>

#include "error_code.hpp"

namespace Wave
{
using ErrorCode = Algorithm::ErrorCode;

class SineWaveGenerator
{
 public:
  SineWaveGenerator() = default;

  SineWaveGenerator(float frequency, float amplitude = std::numbers::pi_v<float>,
                    float offset = 0.0f)
  {
    (void)setParam(frequency, amplitude, offset);
  }

  ErrorCode setParam(float frequency, float amplitude, float offset)
  {
    param_ = {frequency, amplitude, offset};
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

    const float phase_increment =
        2.0f * std::numbers::pi_v<float> * param_.frequency * dt;
    const float next_phase =
        std::fmod(state_.phase + phase_increment, 2.0f * std::numbers::pi_v<float>);
    const float next_output = param_.amplitude * std::sin(next_phase) + param_.offset;
    if (!std::isfinite(phase_increment) || !std::isfinite(next_phase) ||
        !std::isfinite(next_output))
    {
      return ErrorCode::OUT_OF_RANGE;
    }

    state_.phase = next_phase;
    state_.output = next_output;
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

  ErrorCode setAmplitude(float amplitude)
  {
    auto next_param = param_;
    next_param.amplitude = amplitude;
    if (!isParamValid(next_param))
    {
      return ErrorCode::ARG_ERR;
    }
    param_ = next_param;
    is_param_valid_ = true;
    state_.output = param_.amplitude * std::sin(state_.phase) + param_.offset;
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
    state_.output = param_.amplitude * std::sin(state_.phase) + param_.offset;
    return ErrorCode::OK;
  }

  void reset()
  {
    state_ = {};
    state_.output = std::isfinite(param_.offset) ? param_.offset : 0.0f;
  }

  float getOutput() const { return state_.output; }

  float getPhaseRadians() const { return state_.phase; }

  float getPhaseDegrees() const
  {
    return state_.phase * 180.0f / std::numbers::pi_v<float>;
  }

 private:
  struct Param
  {
    float frequency = 0.0f;
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
    return std::isfinite(param.frequency) && param.frequency >= 0.0f &&
           std::isfinite(param.amplitude) && param.amplitude >= 0.0f &&
           std::isfinite(param.offset) && std::isfinite(param.offset + param.amplitude) &&
           std::isfinite(param.offset - param.amplitude);
  }

  Param param_{};
  State state_{};
  bool is_param_valid_ = false;
};
}  // namespace Wave
