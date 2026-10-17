#pragma once

#include <array>
#include <cmath>
#include <cstddef>

namespace algorithm::control::ladrc {
template <std::size_t N>
class LADRC {
public:
    struct Param {
        float r_ = 0.0;
        float b0_ = 0.0;
        float ts_ = 0.0;
        float output_limit_ = 0.0;
        float dt_ = 0.0;
        float wc_ = 0.0;
        float w0_ = 0.0;
        std::array<float, N> gain_ = {};
        std::array<float, N + 1> beta_ = {};
    };

    LADRC()
    {
        static_assert(N == 1 || N == 2);
    }

    LADRC(const Param& param)
    {
        static_assert(N == 1 || N == 2);
        set_param(param);
    }

    void auto_set_param();

    void set_param(const Param& param);
    void set_param(float b0, float ts, float output_limit, float r);
    void set_param(float b0, float wc, float w0, float output_limit, float r);
    // Supply the plant input that was actually applied during the preceding
    // sample. If unused, the observer falls back to the controller output.
    void set_observer_input(float input);
    // Limit the extended-state disturbance in equivalent input units. A
    // non-positive value disables the limit.
    void set_disturbance_compensation_limit(float input_limit);

    float compute(float ref, float feedback, float dt = 0.001f);

    void reset();

    inline float out() const
    {
        return state_.output_;
    }

    [[nodiscard]] inline float z1() const { return state_.Z_[0]; }
    [[nodiscard]] inline float z2() const { return state_.Z_[1]; }
    [[nodiscard]] inline float z3() const
    {
        if constexpr (N == 2) {
            return state_.Z_[2];
        }
        return 0.0f;
    }

protected:
    struct {
        float ref_ = 0.0;

        float error_ = 0.0;
        std::array<float, N + 1> Z_ = {};
        std::array<float, N + 1> Z_dot_ = {};
        float e1_ = {};
        float output_ = 0.0;

        float v1_;
        float v2_;
        float observer_input_ = 0.0f;
    } state_;

    Param param_;
    float disturbance_compensation_limit_ = 0.0f;
    bool use_external_observer_input_ = false;

    inline int sign(float val)
    {
        return (val > 0.0f) - (val < 0.0f);
    }

    void td_update(float dt = 0.0);

    bool first_run = true;
};
}

#include "ladrc.tpp"
