#pragma once

namespace algorithm::ramp {
class RampInput {
public:
    RampInput() = default;
    RampInput(float ramp_rate);
    void set_target(float target);
    float update_target();
    float get_ramp_target() const;

    void clear_ramp()
    {
        state_.ramp_target_ = 0.0;
    }

    float get_actual_target() const
    {
        return state_.target_;
    }
    void set_ramp(float ramp);

    struct {
        float ramp_rate_ = 0.0;
    } param_;

private:
    struct {
        float target_ = 0.0;
        float ramp_target_ = 0.0;
    } state_;
};
}
