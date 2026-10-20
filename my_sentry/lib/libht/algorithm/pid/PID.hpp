#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <utility>

#include "error_code.hpp"

namespace PID
{
using ErrorCode = Algorithm::ErrorCode;

struct Param
{
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  float error_band = 0.0f;            // 在误差小于error_band时，误差被认为是0
  float integral_active_band = 0.0f;  // 大于0时，仅在误差小于该值时累积积分
  float output_limit = 0.0f;          // 0表示不限幅
  float integral_limit = 0.0f;        // 0表示关闭积分
  bool enable_derivative_first = false;
};

struct State
{
  float error = 0.0f;
  float integral = 0.0f;
  float derivative = 0.0f;
  float last_error = 0.0f;
  float last_feedback = 0.0f;
  float error_sum = 0.0f;
  float output = 0.0f;
};

class Controller
{
 public:
  explicit Controller(const Param& param)
      : param_(param), is_param_valid_(isParamValid(param))
  {
  }

  void reset()
  {
    state_ = {};
    is_first_update_ = true;
  }

  ErrorCode update(float target, float feedback, float dt)
  {
    if (!std::isfinite(target) || !std::isfinite(feedback) || !std::isfinite(dt) ||
        dt <= 0.0f || !is_param_valid_)
    {
      return ErrorCode::ARG_ERR;
    }

    const bool is_first_update = is_first_update_;

    float error = target - feedback;
    if (!std::isfinite(error))
    {
      return ErrorCode::OUT_OF_RANGE;
    }

    if (std::fabs(error) < param_.error_band)
    {
      error = 0.0f;
    }

    float error_sum = state_.error_sum;
    if (param_.ki == 0.0f || param_.integral_limit == 0.0f)
    {
      error_sum = 0.0f;
    }
    else if (!is_first_update && (param_.integral_active_band == 0.0f ||
                                  std::fabs(error) < param_.integral_active_band))
    {
      error_sum += error * dt;
      error_sum = std::clamp(error_sum, -param_.integral_limit, param_.integral_limit);
    }

    float derivative = 0.0f;
    if (!is_first_update)
    {
      if (param_.enable_derivative_first)
      {
        derivative = -(feedback - state_.last_feedback) / dt;
      }
      else
      {
        derivative = (error - state_.last_error) / dt;
      }
    }

    const float kp_term = param_.kp * error;
    const float ki_term = param_.ki * error_sum;
    const float kd_term = param_.kd * derivative;
    float output = kp_term + ki_term + kd_term;

    if (!std::isfinite(error_sum) || !std::isfinite(derivative) || !std::isfinite(output))
    {
      return ErrorCode::OUT_OF_RANGE;
    }

    if (param_.output_limit > 0.0f)
    {
      output = std::clamp(output, -param_.output_limit, param_.output_limit);
    }

    state_.error = error;
    state_.integral = ki_term;
    state_.derivative = derivative;
    state_.last_error = error;
    state_.last_feedback = feedback;
    state_.error_sum = error_sum;
    state_.output = output;
    is_first_update_ = false;

    return ErrorCode::OK;
  }

  ErrorCode setParam(const Param& param)
  {
    this->param_ = param;
    is_param_valid_ = isParamValid(param_);

    return is_param_valid_ ? ErrorCode::OK : ErrorCode::ARG_ERR;
  }

  float getOutput() const { return state_.output; }

 private:
  static bool isParamValid(const Param& param)
  {
    return std::isfinite(param.kp) && std::isfinite(param.ki) &&
           std::isfinite(param.kd) && std::isfinite(param.error_band) &&
           std::isfinite(param.integral_active_band) &&
           std::isfinite(param.output_limit) && std::isfinite(param.integral_limit) &&
           param.error_band >= 0.0f && param.integral_active_band >= 0.0f &&
           param.output_limit >= 0.0f && param.integral_limit >= 0.0f;
  }

  Param param_;
  State state_;
  bool is_first_update_ = true;
  bool is_param_valid_ = false;
};

template <std::size_t N>
  requires(N > 0)
class CascadePID
{
 public:
  explicit CascadePID(const std::array<Param, N>& params)
      : controllers_(makeControllers(params, std::make_index_sequence<N>{}))
  {
  }

  ErrorCode setParam(std::size_t index, const Param& param)
  {
    if (index >= N)
    {
      return ErrorCode::ARG_ERR;
    }
    return controllers_[index].setParam(param);
  }

  ErrorCode update(float target, const std::array<float, N>& feedbacks, float dt)
  {
    auto next_controllers = controllers_;
    float stage_target = target;

    for (std::size_t i = 0; i < N; ++i)
    {
      auto& controller = next_controllers[i];
      const auto err = controller.update(stage_target, feedbacks[i], dt);
      if (err != ErrorCode::OK)
      {
        return err;
      }
      stage_target = controller.getOutput();
    }

    controllers_ = next_controllers;
    return ErrorCode::OK;
  }

  float getOutput() const { return controllers_.back().getOutput(); }

  void reset()
  {
    for (auto& controller : controllers_)
    {
      controller.reset();
    }
  }

 private:
  template <std::size_t... I>
  static std::array<Controller, N> makeControllers(const std::array<Param, N>& params,
                                                   std::index_sequence<I...>)
  {
    return {Controller(params[I])...};
  }

  std::array<Controller, N> controllers_;
};

}  // namespace PID
