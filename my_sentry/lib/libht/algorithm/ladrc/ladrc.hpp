#pragma once

#include <array>
#include <cmath>
#include <cstddef>

#include "leso/leso.hpp"
#include "error_code.hpp"

namespace LADRC
{
using ErrorCode = Algorithm::ErrorCode;

/**
 * @brief LADRC (Linear Active Disturbance Rejection Control) class template.
 *
 * @tparam N 是控制器的阶数，当前只支持1阶和2阶控制器, 对应于LESO的2阶和3阶观测器。
 */
template <std::size_t N>
  requires(N == 1 || N == 2)
class Controller
{
 public:
  struct Param
  {
    float b = 0.0f;               // 系统输入增益
    float omega_observer = 0.0f;  // 观测器带宽
    std::array<float, N> gain{};  // 控制器增益
  };

  struct State
  {
    float output = 0.0f;  // 控制器输出
  };

  explicit Controller(float b, float omega_controller, float omega_observer)
      : leso_(b, omega_observer), param_{b, omega_observer}
  {
    if constexpr (N == 1)
    {
      param_.gain[0] = omega_controller;
    }
    else if constexpr (N == 2)
    {
      param_.gain[0] = omega_controller * omega_controller;
      param_.gain[1] = 2.0f * omega_controller;
    }
    is_param_valid_ = isParamValid(param_);
  }

  ErrorCode setParam(const Param& param)
  {
    param_ = param;
    is_param_valid_ = isParamValid(param_);
    const auto observer_err = leso_.setParam(param_.b, param_.omega_observer);
    is_param_valid_ = is_param_valid_ && observer_err == ErrorCode::OK;
    return is_param_valid_ ? ErrorCode::OK : ErrorCode::ARG_ERR;
  }

  ErrorCode setParam(float b, float omega_controller, float omega_observer)
  {
    Param param;
    param.b = b;
    param.omega_observer = omega_observer;
    if constexpr (N == 1)
    {
      param.gain[0] = omega_controller;
    }
    else if constexpr (N == 2)
    {
      param.gain[0] = omega_controller * omega_controller;
      param.gain[1] = 2.0f * omega_controller;
    }
    return setParam(param);
  }

  ErrorCode setParam(const std::array<float, N>& gain, float b, float omega_observer)
  {
    Param param;
    param.b = b;
    param.omega_observer = omega_observer;
    param.gain = gain;
    return setParam(param);
  }

  void reset()
  {
    leso_.reset();
    state_.output = 0.0f;
  }

  ErrorCode update(float target, float feedback, float dt)
  {
    if (!std::isfinite(target) || !std::isfinite(feedback) || !std::isfinite(dt) ||
        dt <= 0.0f || !is_param_valid_)
    {
      return ErrorCode::ARG_ERR;
    }

    const auto err = leso_.update(state_.output, feedback, dt);
    if (err != ErrorCode::OK)
    {
      return err;
    }

    // Compute the control law
    const auto status = leso_.getStatus();
    const float tracking_error = target - status[0];
    float nominal_output = param_.gain[0] * tracking_error;
    if constexpr (N == 2)
    {
      nominal_output -= param_.gain[1] * status[1];
    }

    const float output = (nominal_output - status[N]) / param_.b;
    if (!std::isfinite(tracking_error) || !std::isfinite(nominal_output) ||
        !std::isfinite(output))
    {
      return ErrorCode::OUT_OF_RANGE;
    }

    state_.output = output;
    return ErrorCode::OK;
  }

  float getOutput() const { return state_.output; }

 private:
  static bool isParamValid(const Param& param)
  {
    if (!std::isfinite(param.b) || param.b == 0.0f ||
        !std::isfinite(param.omega_observer) || param.omega_observer <= 0.0f)
    {
      return false;
    }
    for (const auto& gain : param.gain)
    {
      if (!std::isfinite(gain) || gain <= 0.0f)
      {
        return false;
      }
    }

    return true;
  }

  LESO::Observer<N + 1> leso_;
  Param param_{};
  State state_{};
  bool is_param_valid_ = false;
};
}  // namespace LADRC
