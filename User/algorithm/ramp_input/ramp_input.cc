#include "./ramp_input.hpp"

namespace algorithm::ramp {
RampInput::RampInput(float ramp_rate)
    : param_ {
        ramp_rate
    }
{
}

void RampInput::set_ramp(float ramp)
{
    param_.ramp_rate_ = ramp;
}

void RampInput::set_target(float target)
{
    state_.target_ = target;
}

float RampInput::update_target()
{
    if (state_.ramp_target_ < state_.target_) {
        state_.ramp_target_ += param_.ramp_rate_;
        if (state_.ramp_target_ >= state_.target_) {
            state_.ramp_target_ = state_.target_;
        }
    } else if (state_.ramp_target_ > state_.target_) {
        state_.ramp_target_ -= param_.ramp_rate_;
        if (state_.ramp_target_ <= state_.target_) {
            state_.ramp_target_ = state_.target_;
        }
    } else {
        state_.ramp_target_ = state_.target_;
    }
    return state_.ramp_target_;
}

float RampInput::get_ramp_target() const
{
    return state_.ramp_target_;
}
}