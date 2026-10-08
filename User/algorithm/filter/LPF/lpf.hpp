#pragma once

#include "utility/user_assert.hpp"
#include <array>
#include <cmath>
#include <numbers>

namespace algorithm::filter::lpf {
class LPF {
public:
    struct Param {
        float alpha_ = 0.0f;
    };

    LPF() = default;

    explicit LPF(const Param& param)
        : param_(param)
    {
        if (check_param()) {
            assert_always(false && "LPF alpha_ must be in [0, 1]");
        }
    }

    void set_param(float alpha)
    {
        param_.alpha_ = alpha;
        if (check_param()) {
            assert_always(false && "LPF alpha_ must be in [0, 1]");
        }
    }

    void set_param(const Param& param)
    {
        param_ = param;
        if (check_param()) {
            assert_always(false && "LPF alpha_ must be in [0, 1]");
        }
    }

    bool check_param() const
    {
        return (param_.alpha_ < 0.0f || param_.alpha_ > 1.0f);
    }

    float update(float input)
    {
        state_.input_ = input;
        if (is_init_) {
            state_.output_[0] = state_.output_[1];
            state_.output_[1] = param_.alpha_ * state_.input_ + (1.0f - param_.alpha_) * state_.output_[1];
        } else {
            is_init_ = true;
            state_.output_ = { input, input };
        }

        return state_.output_[1];
    }

    float out() const
    {
        return state_.output_[1];
    }

private:
    Param param_;

    struct {
        float input_ = 0.0f;
        alignas(4) std::array<float, 2> output_ = { 0.0f, 0.0f };
    } state_;

    bool is_init_ = false;
};

class SimpleLPF2p {
public:
    struct Param {
        std::array<float, 3> alpha_ = { 0.0f, 0.0f, 0.0f };
    };

    SimpleLPF2p() = default;

    explicit SimpleLPF2p(const Param& param)
        : param_(param)
    {
    }

    void set_param(const Param& param)
    {
        param_ = param;
        state_.input_ = 0.0f;
        state_.output_ = { 0.0f, 0.0f, 0.0f };
    }

    float update(float input)
    {
        state_.input_ = input;
        if (is_init_) {
            state_.output_[0] = state_.output_[1];
            state_.output_[1] = state_.output_[2];
            state_.output_[2] = param_.alpha_[0] * state_.output_[1] + param_.alpha_[1] * state_.output_[0] + param_.alpha_[2] * state_.input_;
        } else {
            is_init_ = true;
            state_.output_ = { input, input, input };
        }

        return state_.output_[2];
    }

    float out() const
    {
        return state_.output_[2];
    }

private:
    Param param_;

    struct {
        float input_ = 0.0f;
        alignas(4) std::array<float, 3> output_ = { 0.0f, 0.0f, 0.0f };
    } state_;

    bool is_init_ = false;
};

class Biquad {
public:
    Biquad() = default;

    Biquad(float sample_rate, float cutoff_freq, float q_factor)
    {
        compute_coefficients(sample_rate, cutoff_freq, q_factor);
    }

    // 核心计算 (Direct Form I)
    // 这种形式对浮点数精度较好，且不易溢出
    [[nodiscard]] float update(float input)
    {
        float output = 0.0f;
        if (is_init_) {
            // 差分方程: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
            output = b0 * input + b1 * z_x[0] + b2 * z_x[1]
                - a1 * z_y[0] - a2 * z_y[1];

            z_x[1] = z_x[0];
            z_x[0] = input;

            z_y[1] = z_y[0];
            z_y[0] = output;
        } else {
            z_x.fill(input);
            z_y.fill(input);
            output = input;

            is_init_ = true;
        }

        return output;
    }

    // 根据 Q 值和截止频率计算系数
    void compute_coefficients(float sample_rate, float cutoff_freq, float q_factor)
    {
        float omega = 2.0f * std::numbers::pi_v<float> * cutoff_freq / sample_rate;
        float sn = std::sin(omega);
        float cs = std::cos(omega);
        float alpha = sn / (2.0f * q_factor);

        float b0_tmp = (1.0f - cs) / 2.0f;
        float b1_tmp = 1.0f - cs;
        float b2_tmp = (1.0f - cs) / 2.0f;
        float a0_tmp = 1.0f + alpha;
        float a1_tmp = -2.0f * cs;
        float a2_tmp = 1.0f - alpha;

        b0 = b0_tmp / a0_tmp;
        b1 = b1_tmp / a0_tmp;
        b2 = b2_tmp / a0_tmp;
        a1 = a1_tmp / a0_tmp;
        a2 = a2_tmp / a0_tmp;
    }

    void reset()
    {
        z_x.fill(0);
        z_y.fill(0);
        is_init_ = false;
    }

private:
    float b0 { 0 }, b1 { 0 }, b2 { 0 };
    float a1 { 0 }, a2 { 0 };

    std::array<float, 2> z_x { 0, 0 };
    std::array<float, 2> z_y { 0, 0 };

    bool is_init_ = false;
};

template <std::size_t Order>
    requires(Order % 2 == 0)
class LowPassFilterN {
public:
    static constexpr std::size_t NumStages = Order / 2;

    LowPassFilterN() = default;

    LowPassFilterN(float sample_rate, float cutoff_freq)
    {

        for (std::size_t i = 0; i < NumStages; ++i) {
            float k = static_cast<float>(i + 1);
            float n_val = static_cast<float>(Order);

            float theta = (std::numbers::pi_v<float> * (2.0f * k - 1.0f)) / (2.0f * n_val);
            float q_factor = 1.0f / (2.0f * std::cos(theta)); // 修正的 Q 值计算

            stages[i].compute_coefficients(sample_rate, cutoff_freq, q_factor);
            stages[i].reset();
        }
    }

    // 更新滤波器
    [[nodiscard]] float update(float input)
    {
        float signal = input;

        for (auto& stage : stages) {
            signal = stage.update(signal);
        }
        return signal;
    }

    void reset()
    {
        for (auto& stage : stages) {
            stage.reset();
        }
    }

private:
    std::array<Biquad, NumStages> stages;
};
}
