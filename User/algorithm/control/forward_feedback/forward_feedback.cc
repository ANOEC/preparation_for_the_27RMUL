#include "forward_feedback.hpp"

namespace algorithm::control::forward_feedback {
float ForwardFeedback::compute(float ref, float dt)
{
    state_.ref_ = ref;

    if (!state_.initialized_ || dt <= 0.0f) {
        state_.output_ = 0.0f;
        state_.last_ref_ = state_.ref_;
        state_.initialized_ = true;
        return state_.output_;
    }

    state_.output_ =
        param_.gain_ * (state_.ref_ - state_.last_ref_) / dt;
    state_.last_ref_ = state_.ref_;
    return state_.output_;
}

void ForwardFeedback::set_gain(float gain)
{
    param_.gain_ = gain;
}

float ForwardFeedback::out() const
{
    return state_.output_;
}

void ForwardFeedback::reset()
{
    state_.output_ = 0.0f;
    state_.last_ref_ = 0.0f;
    state_.ref_ = 0.0f;
    state_.initialized_ = false;
}
}
