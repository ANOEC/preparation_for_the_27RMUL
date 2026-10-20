#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "error_code.hpp"

namespace Wave
{
using ErrorCode = Algorithm::ErrorCode;

class Prbs
{
 public:
  Prbs() = default;

  Prbs(float amplitude, float offset, float period, float update_interval)
  {
    (void)setParam(amplitude, offset, period, update_interval);
  }

  ErrorCode setParam(float amplitude, float offset, float period, float update_interval)
  {
    param_ = {amplitude, offset, period, update_interval};
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
    const double accumulated_time =
        static_cast<double>(state_.time_since_last_update) + active_dt;
    const double raw_step_count = std::floor(accumulated_time / param_.update_interval);
    const double reduced_step_count = std::fmod(raw_step_count, LFSR_PERIOD);
    const double remainder = std::fmod(accumulated_time, param_.update_interval);
    const float next_elapsed_time = state_.elapsed_time + active_dt;

    if (!std::isfinite(accumulated_time) || !std::isfinite(raw_step_count) ||
        !std::isfinite(reduced_step_count) || !std::isfinite(remainder) ||
        !std::isfinite(next_elapsed_time))
    {
      return ErrorCode::OUT_OF_RANGE;
    }

    auto next_state = state_;
    const auto steps = static_cast<std::uint32_t>(reduced_step_count);
    for (std::uint32_t i = 0; i < steps; ++i)
    {
      advanceLfsr(next_state.lfsr_state);
    }
    if (raw_step_count > 0.0)
    {
      next_state.output = (next_state.lfsr_state & 1U) != 0U
                              ? param_.offset + param_.amplitude
                              : param_.offset - param_.amplitude;
    }

    next_state.time_since_last_update = static_cast<float>(remainder);
    next_state.elapsed_time = next_elapsed_time;
    if (next_state.elapsed_time >= param_.period)
    {
      next_state.elapsed_time = param_.period;
      next_state.output = param_.offset;
      next_state.finished = true;
    }

    state_ = next_state;
    return ErrorCode::OK;
  }

  float getOutput() const { return state_.output; }

  bool isFinished() const { return state_.finished; }

  void reset()
  {
    state_ = {};
    state_.output = std::isfinite(param_.offset) ? param_.offset : 0.0f;
    state_.time_since_last_update =
        std::isfinite(param_.update_interval) ? param_.update_interval : 0.0f;
    state_.lfsr_state = LFSR_SEED;
  }

 private:
  static constexpr std::uint16_t LFSR_MASK = 0x07FFU;
  static constexpr std::uint16_t LFSR_SEED = LFSR_MASK;
  static constexpr double LFSR_PERIOD = 2047.0;

  struct Param
  {
    float amplitude = 0.0f;
    float offset = 0.0f;
    float period = 0.0f;
    float update_interval = 0.0f;
  };

  struct State
  {
    float output = 0.0f;
    float elapsed_time = 0.0f;
    float time_since_last_update = 0.0f;
    std::uint16_t lfsr_state = LFSR_SEED;
    bool finished = false;
  };

  static void advanceLfsr(std::uint16_t& state)
  {
    const std::uint16_t bit = ((state >> 10U) ^ (state >> 8U)) & 1U;
    state = static_cast<std::uint16_t>(((state << 1U) & LFSR_MASK) | bit);
  }

  static bool isParamValid(const Param& param)
  {
    return std::isfinite(param.amplitude) && param.amplitude >= 0.0f &&
           std::isfinite(param.offset) && std::isfinite(param.offset + param.amplitude) &&
           std::isfinite(param.offset - param.amplitude) && std::isfinite(param.period) &&
           param.period > 0.0f && std::isfinite(param.update_interval) &&
           param.update_interval > 0.0f;
  }

  Param param_{};
  State state_{};
  bool is_param_valid_ = false;
};
}  // namespace Wave
