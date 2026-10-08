#include "pid.hpp"
#include <algorithm>

namespace algorithm::control::pid {
PID::PID(float kp, float ki, float kd, float deadband,
    float integral_limit, float output_limit, PIDType type)
    : param_ {
        kp, ki, kd, deadband, integral_limit, output_limit, type
    }
{
}

void PID::set_param(float kp, float ki, float kd, float deadband,
    float integral_limit, float output_limit, PIDType type)
{
    param_.kp_ = kp;
    param_.ki_ = ki;
    param_.kd_ = kd;
    param_.deadband_ = deadband;
    param_.integral_limit_ = integral_limit;
    param_.output_limit_ = output_limit;
    param_.type_ = type;
}

void PID::reset()
{
    state_.error_ = 0.0f;
    state_.last_error_ = 0.0f;
    state_.last_l_error_ = 0.0f;
    state_.integral_ = 0.0f;
    state_.output_ = 0.0f;
    state_.last_output_ = 0.0f;
}

float PID::out() const
{
    return state_.output_;
}

float PID::error() const
{
    return state_.error_;
}

float PID::compute(float ref, float feedback)
{
    this->state_.ref_ = ref;
    state_.error_ = ref - feedback;
    if (std::abs(state_.error_) < param_.deadband_) {
        state_.error_ = 0.0;
    }

    if (param_.type_ == PIDType::POSITION) {
        state_.integral_ += state_.error_;
        state_.integral_ = std::clamp(state_.integral_, -param_.integral_limit_, param_.integral_limit_);

        float p_out = param_.kp_ * state_.error_;
        float i_out = param_.ki_ * state_.integral_;
        float d_out = param_.kd_ * (state_.error_ - state_.last_error_);

        state_.output_ = p_out + i_out + d_out;

        state_.output_ = std::clamp(state_.output_, -param_.output_limit_, param_.output_limit_);
    } else {
        float p_out = param_.kp_ * (state_.error_ - state_.last_error_);
        float i_out = param_.ki_ * state_.error_;
        float d_out = param_.kd_ * (state_.error_ - 2.0f * state_.last_error_ + state_.last_l_error_);

        state_.output_ = state_.last_output_ + p_out + i_out + d_out;

        state_.output_ = std::clamp(state_.output_, -param_.output_limit_, param_.output_limit_);
    }

    state_.last_l_error_ = state_.last_error_;
    state_.last_error_ = state_.error_;
    state_.last_output_ = state_.output_;
    return state_.output_;
}
}
