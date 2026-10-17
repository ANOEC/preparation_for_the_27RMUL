#pragma once

#include "ladrc.hpp"
#include <algorithm>

namespace algorithm::control::ladrc {
template <std::size_t N>
void LADRC<N>::auto_set_param()
{
    if (std::abs(param_.dt_) < 1e-6f) {
        param_.dt_ = 0.002f;
    }

    if (std::abs(param_.ts_) < 1e-6f)
        return;

    param_.wc_ = std::abs(param_.wc_) < 1e-6f ? 3.0f / param_.ts_ : param_.wc_;
    param_.w0_ = std::abs(param_.w0_) < 1e-6f ? 4.0f * param_.wc_ : param_.w0_;
    if constexpr (N == 1) {
        param_.beta_[0] = 2.0f * param_.w0_;
        param_.beta_[1] = param_.w0_ * param_.w0_;

        param_.gain_[0] = param_.wc_;
    } else {
        param_.beta_[0] = 3.0f * param_.w0_;
        param_.beta_[1] = 3.0f * param_.w0_ * param_.w0_;
        param_.beta_[2] = param_.w0_ * param_.w0_ * param_.w0_;

        param_.gain_[0] = param_.wc_ * param_.wc_;
        param_.gain_[1] = 2.f * param_.wc_;
    }
}

template <std::size_t N>
void LADRC<N>::set_param(const Param& param)
{
    param_ = param;
    auto_set_param();
}

template <std::size_t N>
void LADRC<N>::set_param(float b0, float ts, float output_limit, float r)
{
    param_.b0_ = b0;
    param_.ts_ = ts;
    param_.output_limit_ = output_limit;
    param_.r_ = r;
    param_.w0_ = 0.0f;
    param_.wc_ = 0.0f;
    auto_set_param();
}

template <std::size_t N>
void LADRC<N>::set_param(float b0, float wc, float w0, float output_limit, float r)
{
    param_.b0_ = b0;
    param_.w0_ = w0;
    param_.wc_ = wc;
    param_.output_limit_ = output_limit;
    param_.r_ = r;
    param_.ts_ = 3.f / param_.wc_;
    auto_set_param();
}

template <std::size_t N>
void LADRC<N>::set_observer_input(float input)
{
    state_.observer_input_ = input;
    use_external_observer_input_ = true;
}

template <std::size_t N>
void LADRC<N>::set_disturbance_compensation_limit(float input_limit)
{
    disturbance_compensation_limit_ = std::abs(input_limit);
}

template <std::size_t N>
float LADRC<N>::compute(float ref, float feedback, float dt)
{
    if (std::abs(dt) < 1e-6f)
        dt = param_.dt_;

    param_.dt_ = dt;

    if (std::abs(param_.b0_) < 1e-6f) {
        reset();
        return 0.0f;
    }

    state_.ref_ = ref;
    td_update(dt);

    if (first_run) {
        state_.Z_[0] = feedback;
        state_.Z_[1] = 0.f;
        if constexpr (N == 2) {
            state_.Z_[2] = 0.f;
        }

        first_run = false;
        return 0.0f;
    }

    state_.e1_ = state_.Z_[0] - feedback;
    const float observer_input = use_external_observer_input_
        ? state_.observer_input_
        : state_.output_;

    if constexpr (N == 1) {
        state_.Z_dot_[1] = -param_.beta_[1] * state_.e1_;
        state_.Z_dot_[0] = state_.Z_[1] + param_.b0_ * observer_input - param_.beta_[0] * state_.e1_;
    } else {

        state_.Z_dot_[2] = -param_.beta_[2] * state_.e1_;
        state_.Z_dot_[1] = state_.Z_[2] + param_.b0_ * observer_input - param_.beta_[1] * state_.e1_;

        state_.Z_dot_[0] = state_.Z_[1] - param_.beta_[0] * state_.e1_;
    }

    for (std::size_t i = 0; i <= N; i++) {
        state_.Z_[i] += state_.Z_dot_[i] * dt;
    }
    if constexpr (N == 2) {
        if (disturbance_compensation_limit_ > 0.0f) {
            const float disturbance_state_limit =
                std::abs(param_.b0_) * disturbance_compensation_limit_;
            state_.Z_[2] = std::clamp(state_.Z_[2],
                -disturbance_state_limit, disturbance_state_limit);
        }
    }

    state_.error_ = state_.v1_ - state_.Z_[0];
    if constexpr (N == 1) {
        state_.output_ = (param_.gain_.at(0) * state_.error_ - state_.Z_[1] + state_.v2_) / param_.b0_;
    } else {
        state_.output_ = (param_.gain_.at(0) * state_.error_ - param_.gain_.at(1) * (state_.Z_[1] - state_.v2_) - state_.Z_[N]) / param_.b0_;
    }

    state_.output_ = std::clamp(state_.output_, -param_.output_limit_, param_.output_limit_);
    return state_.output_;
}

template <std::size_t N>
void LADRC<N>::reset()
{
    state_.e1_ = 0.0f;
    state_.error_ = 0.0f;
    state_.output_ = 0.0f;
    state_.ref_ = 0.0f;
    state_.v1_ = 0.0f;
    state_.v2_ = 0.0f;
    state_.observer_input_ = 0.0f;
    state_.Z_.fill(0.0f);
    state_.Z_dot_.fill(0.0f);
    use_external_observer_input_ = false;
    first_run = true;
}

template <std::size_t N>
void LADRC<N>::td_update(float dt)
{
    if (param_.r_ > 0.0f) {
        float d = param_.r_ * dt * dt;
        float a0 = dt * state_.v2_;
        float y = state_.v1_ - state_.ref_ + a0;

        float a1 = std::sqrt(d * (d + 8.0f * std::abs(y)));
        float a2 = a0 + sign(y) * (a1 - d) / 2.0f;

        float sy = (sign(y + d) - sign(y - d)) / 2.f;
        float a = (a0 + y - a2) * sy + a2;
        float fhan_out = -param_.r_ * (a / d) - sign(a) * (1.f - sy) * param_.r_;

        state_.v1_ += state_.v2_ * dt;
        state_.v2_ += fhan_out * dt;
    } else {
        state_.v1_ = state_.ref_;
        state_.v2_ = 0.f;
    }
}
}
