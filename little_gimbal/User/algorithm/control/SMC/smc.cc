#include "smc.hpp"
#include "utility/user_assert.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace control_algorithm::smc {
SMC::SMC(float deadband, float output_limit, float sat_limit, float J,
    ApproachRaw approach_raw, ApproachParamVariant approach_param)
    : param_ {
        deadband,
        output_limit,
        sat_limit,
        J,
        approach_raw,
        approach_param
    }
{
    if (approach_raw == ApproachRaw::EXPONENT) {
        assert_always(std::holds_alternative<LineExponentParam>(approach_param));
    } else if (approach_raw == ApproachRaw::POWER) {
        assert_always(std::holds_alternative<LinePowerParam>(approach_param));
    } else if (approach_raw == ApproachRaw::SUPER_TWISTING) {
        assert_always(std::holds_alternative<LineSuperTwistingParam>(approach_param));
    }
}

float SMC::compute(float ref, float feedback_position, float feedback_velocity)
{
    state_.ref_ = ref;
    state_.error_ = feedback_position - ref;
    if (std::abs(state_.error_) < param_.deadband_) {
        state_.error_ = 0.0;
    }

    state_.ref_ddt_ = (state_.ref_ - state_.last_ref_) - state_.ref_dt_;
    state_.ref_dt_ = state_.ref_ - state_.last_ref_;
    float error_dt_ = feedback_velocity - state_.ref_dt_;

    switch (param_.approach_raw_) {
    case ApproachRaw::EXPONENT: {
        auto& p = std::get<LineExponentParam>(param_.approach_param_);
        state_.s_ = p.c_ * state_.error_ + p.K_ * error_dt_;
        state_.output_ = param_.J_ * (state_.ref_ddt_ - p.c_ * error_dt_ - p.epsilon_ * sat(state_.s_) - p.K_ * state_.s_);
    } break;
    case ApproachRaw::POWER: {
        auto& p = std::get<LinePowerParam>(param_.approach_param_);
        state_.s_ = p.c_ * state_.error_ + p.K_ * error_dt_;
        state_.output_ = param_.J_ * (state_.ref_ddt_ - p.c_ * error_dt_ - p.K_ * state_.s_ - p.K_ * (std::pow(std::abs(state_.s_), p.alpha_) * sat(state_.s_)));
    } break;
    case ApproachRaw::SUPER_TWISTING: {
        auto& p = std::get<LineSuperTwistingParam>(param_.approach_param_);
        state_.s_ = p.c_ * state_.error_ + error_dt_;
        float v_dt_ = -p.K2_ * sign(state_.s_);
        p.v_ += v_dt_;
        state_.output_ = param_.J_ * (p.K1_ * std::sqrt(std::abs(state_.s_)) * sign(state_.s_) + p.v_);
    } break;
    }

    state_.output_ = std::clamp(state_.output_, -param_.output_limit_, param_.output_limit_);

    return state_.output_;
}

void SMC::reset()
{
    if (param_.approach_raw_ == ApproachRaw::SUPER_TWISTING) {
        auto& p = std::get<LineSuperTwistingParam>(param_.approach_param_);
        p.v_ = 0.0;
    }
    state_.ref_ = 0.0;
    state_.last_ref_ = 0.0;
    state_.ref_dt_ = 0.0;
    state_.ref_ddt_ = 0.0;
    state_.error_ = 0.0;
    state_.output_ = 0.0;
    state_.s_ = 0.0;
}
}