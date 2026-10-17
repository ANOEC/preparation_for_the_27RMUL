#pragma once

#include "cf.hpp"

namespace algorithm::filter::cf {
template <std::size_t N>
CF<N>::CF(const Param& param)
    : param_(param)
{
    assert_always(N >= 1);
    assert_always(check_param());
}

template <std::size_t N>
void CF<N>::set_param(const Param& param)
{
    param_ = param;

    assert_always(check_param());
}

template <std::size_t N>
bool CF<N>::check_param() const
{
    float total_temp = 0.0f;
    for (std::size_t i = 0; i < N; ++i) {
        total_temp += param_.a_[i];
    }

    return (std::abs(total_temp - 1.0f) < 1e-6f);
}

template <std::size_t N>
float CF<N>::update(const std::array<float, N>& inputs)
{
    state_.input_ = inputs;
    float output_temp = 0.0f;
    for (std::size_t i = 0; i < N; ++i) {
        output_temp += param_.a_[i] * state_.input_[i];
    }
    state_.output_ = output_temp;
    return state_.output_;
}
}