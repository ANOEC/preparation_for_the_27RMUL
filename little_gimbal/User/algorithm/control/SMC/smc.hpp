#pragma once

#include <cmath>
#include <cstdlib>
#include <variant>
namespace control_algorithm::smc {
enum class ApproachRaw {
    EXPONENT,
    POWER,
    SUPER_TWISTING
};

class SMC {
private:
    inline float sign(float value)
    {
        if (value > 1e-6f) {
            return 1.0;
        } else if (value < -1e-6f) {
            return -1.0;
        } else {
            return 0.0;
        }
    }

    inline float sat(float value)
    {
        if (std::abs(value) <= param_.sat_limit_) {
            return value;
        } else {
            return sign(value);
        }
    }

    struct LineExponentParam {
        float c_ = 0.0f;
        float K_ = 0.0f;
        float epsilon_ = 0.0f;
    };

    struct LinePowerParam {
        float c_ = 0.0f;
        float K_ = 0.0f;
        float alpha_ = 0.0f;
    };

    struct LineSuperTwistingParam {
        float c_ = 0.0f;
        float K1_ = 0.0f;
        float K2_ = 0.0f;
        float v_ = 0.0f;
    };

public:
    using ApproachParamVariant = std::variant<LineExponentParam, LinePowerParam, LineSuperTwistingParam>;

    SMC() = default;
    SMC(float deadband, float output_limit, float sat_limit, float J, ApproachRaw approach_raw, ApproachParamVariant approach_param);

    float compute(float ref, float feedback_position, float feedback_velocity);
    void reset();

private:
    struct {
        float deadband_ = 0.0f;
        float output_limit_ = 0.0f;
        float sat_limit_ = 1.0f;
        float J_ = 0.0f;
        ApproachRaw approach_raw_ = ApproachRaw::EXPONENT;
        ApproachParamVariant approach_param_ = LineExponentParam {};
    } param_;

    struct {
        float ref_ = 0.0;
        float last_ref_ = 0.0;
        float ref_dt_ = 0.0;
        float ref_ddt_ = 0.0;
        float error_ = 0.0;
        float output_ = 0.0;
        float s_ = 0.0;
    } state_;
};
}