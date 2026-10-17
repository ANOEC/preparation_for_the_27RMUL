#pragma once

#include "utility/user_assert.hpp"
#include <cstring>
namespace algorithm::observer::leso {
template <std::size_t N>
class LESO {
public:
    LESO() = default;
    LESO(float b0, float omega)
    {
        static_assert(N == 1 || N == 2, "Only N=1 or N=2 is supported");

        assert_always(b0 > 0);

        param_.b0_ = b0;
        param_.omega_ = omega;

        if constexpr (N == 1) {
            param_.beta_[0] = 2.0f * omega;
            param_.beta_[1] = omega * omega;
        } else {
            param_.beta_[0] = 3.0f * omega;
            param_.beta_[1] = 3.0f * omega * omega;
            param_.beta_[2] = omega * omega * omega;
        }
    }

    LESO(float beta[N + 1], float b0)
    {
        static_assert(N == 1 || N == 2, "Only N=1 or N=2 is supported");
        assert_always(b0 > 0);

        std::memcpy(param_.beta_, beta, (N + 1) * sizeof(float));
        param_.b0_ = b0;
    }

    void set_param(float b0, float omega)
    {
        assert_always(b0 > 0);
        param_.b0_ = b0;
        param_.omega_ = omega;

        if constexpr (N == 1) {
            param_.beta_[0] = 2.0f * omega;
            param_.beta_[1] = omega * omega;
        } else if constexpr (N == 2) {
            param_.beta_[0] = 3.0f * omega;
            param_.beta_[1] = 3.0f * omega * omega;
            param_.beta_[2] = omega * omega * omega;
        }
    }

    void set_param(float beta[N + 1], float b0)
    {
        assert_always(b0 > 0);

        std::memcpy(param_.beta_, beta, (N + 1) * sizeof(float));
        param_.b0_ = b0;
    }

    void update(float y, float u, float dt = 0.002f)
    {
        if (!init_flag) {
            init_flag = true;

            status_.z_[0] = y;

            return;
        }

        float error = status_.z_[0] - y;

        if constexpr (N == 1) {
            status_.z_dot_[0] = status_.z_[1] - param_.beta_[0] * error + param_.b0_ * u;
            status_.z_dot_[1] = -param_.beta_[1] * error;
        } else if constexpr (N == 2) {
            status_.z_dot_[0] = status_.z_[1] - param_.beta_[0] * error;
            status_.z_dot_[1] = status_.z_[2] - param_.beta_[1] * error + param_.b0_ * u;
            status_.z_dot_[2] = -param_.beta_[2] * error;
        }

        for (std::size_t i = 0; i <= N; i++) {
            status_.z_[i] += status_.z_dot_[i] * dt;
        }
    }

    inline void get_state(float state[N + 1]) const
    {
        std::memcpy(state, status_.z_, (N + 1) * sizeof(float));
    }

    inline float get_disturbance() const
    {
        return status_.z_[N];
    }

    inline float get_disturbance_torque() const
    {
        return status_.z_[N] / param_.b0_;
    }

    void reset()
    {
        std::memset(status_.z_, 0, sizeof(status_.z_));
        std::memset(status_.z_dot_, 0, sizeof(status_.z_dot_));

        init_flag = false;
    }

private:
    struct {
        float beta_[N + 1] = {};
        float b0_ = 1.0f;
        float omega_ = 1.0f;
    } param_;

    struct {
        float z_[N + 1] = {};
        float z_dot_[N + 1] = {};
    } status_;

    bool init_flag = false;
};
}
