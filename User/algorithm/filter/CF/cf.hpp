#pragma once

#include "utility/user_assert.hpp"
#include <array>
#include <cstddef>
#include <cstdlib>

namespace algorithm::filter::cf {
template <std::size_t N>
class CF {
public:
    struct Param {
        std::array<float, N> a_ = { 1.0 };
    };

    CF()
    {
        assert_always(N >= 1);
    }

    explicit CF(const Param& param);

    void set_param(const Param& param);

    inline bool check_param() const;

    float update(const std::array<float, N>& inputs);

    float out() const
    {
        return state_.output_;
    }

private:
    Param param_;
    struct {
        std::array<float, N> input_ {};
        float output_ = 0.0;
    } state_;
};
}

#include "cf.tpp"