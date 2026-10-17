#pragma once

#include <array>
#include <cmath>
#include <cstddef>

#include "error_code.hpp"

namespace LESO
{
using ErrorCode = Algorithm::ErrorCode;

/**
 * @brief LESO (Linear Extended State Observer) class template.
 *
 * @tparam N 是观测器的阶数，当前只支持2阶和3阶观测器。
 */
template <std::size_t N>
  requires(N == 2 || N == 3)
class Observer
{
 public:
  struct Param
  {
    std::array<float, N> beta{};  // 观测器增益参数
    float b = 0.0f;               // 系统输入增益
    float omega = 0.0f;           // 观测器带宽，自定义 beta 时为 0
  };

  struct State
  {
    std::array<float, N> status{};      // 观测器状态
    std::array<float, N> status_dot{};  // 观测器状态导数
  };

  explicit Observer(float b, float omega)
  {
    param_.b = b;
    param_.omega = omega;
    if constexpr (N == 2)
    {
      param_.beta = {2.0f * omega, omega * omega};
    }
    else
    {
      param_.beta = {3.0f * omega, 3.0f * omega * omega, omega * omega * omega};
    }
    is_param_valid_ = isParamValid(param_);
  }
  explicit Observer(const std::array<float, N>& beta, float b)
  {
    param_.beta = beta;
    param_.b = b;
    param_.omega = 0.0f;
    is_param_valid_ = isParamValid(param_);
  }

  explicit Observer(const Param& param)
      : param_(param), is_param_valid_(isParamValid(param))
  {
  }

  ErrorCode update(float u, float feedback, float dt)
  {
    if (!is_param_valid_ || !std::isfinite(u) || !std::isfinite(feedback) ||
        !std::isfinite(dt) || dt <= 0.0f)
    {
      return ErrorCode::ARG_ERR;
    }

    if (is_first_update_)
    {
      State next_state{};
      next_state.status[0] = feedback;
      state_ = next_state;
      is_first_update_ = false;
      return ErrorCode::OK;
    }

    // Update the observer state using the LESO equations
    const float error = feedback - state_.status[0];
    if (!std::isfinite(error))
    {
      return ErrorCode::OUT_OF_RANGE;
    }

    State next_state = state_;

    if constexpr (N == 3)
    {
      next_state.status_dot[0] = state_.status[1] + param_.beta[0] * error;
    }

    next_state.status_dot[N - 2] =
        state_.status[N - 1] + param_.beta[N - 2] * error + param_.b * u;
    next_state.status_dot[N - 1] = param_.beta[N - 1] * error;

    for (std::size_t i = 0; i < N; ++i)
    {
      next_state.status[i] += next_state.status_dot[i] * dt;
      if (!std::isfinite(next_state.status_dot[i]) ||
          !std::isfinite(next_state.status[i]))
      {
        return ErrorCode::OUT_OF_RANGE;
      }
    }

    state_ = next_state;
    return ErrorCode::OK;
  }

  std::array<float, N> getStatus() const { return state_.status; }

  float getDisturbingTorque() const
  {
    return is_param_valid_ ? state_.status[N - 1] / param_.b : 0.0f;
  }

  void reset()
  {
    state_ = {};
    is_first_update_ = true;
  }

  ErrorCode setParam(const Param& param)
  {
    param_ = param;
    is_param_valid_ = isParamValid(param_);
    return is_param_valid_ ? ErrorCode::OK : ErrorCode::ARG_ERR;
  }

  ErrorCode setParam(float b, float omega)
  {
    param_.b = b;
    param_.omega = omega;
    if constexpr (N == 2)
    {
      param_.beta = {2.0f * omega, omega * omega};
    }
    else
    {
      param_.beta = {3.0f * omega, 3.0f * omega * omega, omega * omega * omega};
    }
    is_param_valid_ = isParamValid(param_);
    return is_param_valid_ ? ErrorCode::OK : ErrorCode::ARG_ERR;
  }

  ErrorCode setParam(const std::array<float, N>& beta, float b)
  {
    param_.beta = beta;
    param_.b = b;
    param_.omega = 0.0f;
    is_param_valid_ = isParamValid(param_);
    return is_param_valid_ ? ErrorCode::OK : ErrorCode::ARG_ERR;
  }

 private:
  static bool isParamValid(const Param& param)
  {
    if (!std::isfinite(param.b) || param.b == 0.0f || !std::isfinite(param.omega) ||
        param.omega < 0.0f)
    {
      return false;
    }

    for (const auto& gain : param.beta)
    {
      if (!std::isfinite(gain) || gain <= 0.0f)
      {
        return false;
      }
    }
    return true;
  }

  Param param_{};
  State state_{};
  bool is_param_valid_ = false;
  bool is_first_update_ = true;
};

}  // namespace LESO
