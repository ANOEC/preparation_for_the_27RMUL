#pragma once

#include "utility/user_assert.hpp"

namespace algorithm::math::wave {
class SquareWaveGenerator {
public:
    SquareWaveGenerator() = delete;

    SquareWaveGenerator(float peak, float offset, float frequency = 50.0f, float duty_cycle = 0.5f)
        : param_ { peak, offset, duty_cycle, frequency, 1.0f / frequency }
        , output_(0.0f)
        , time_(0.0f)
        , initialized_(false)
    {
        assert_always(frequency >= 0.0f);
        assert_always(peak >= 0.0f);
        assert_always(duty_cycle >= 0.0f && duty_cycle <= 1.0f);
    }

    float update(float current_time);
    void reset();

    void set_frequency(float frequency);
    void set_duty_cycle(float duty_cycle);
    void set_peak(float peak);
    void set_offset(float offset);

    inline float get_output() const
    {
        return output_;
    }

private:
    struct {
        float peak;
        float offset;
        float duty_cycle;
        float frequency;
        float period;
    } param_;

    float output_;
    float time_;
    bool initialized_ = false;
};
}