#pragma once

namespace algorithm::math::wave {
class StepWaveGenerator {
public:
    StepWaveGenerator() = delete;

    StepWaveGenerator(float peak, float offset, float init_time, float duration = 0.0f, float delay_time = 0.0f)
        : param_ { peak, offset, duration, delay_time }
        , output_(0.0f)
        , time_(init_time)
    {
    }

    float update(float current_time);
    void reset();

    inline float get_output() const
    {
        return output_;
    }

private:
    struct {
        float peak;
        float offset;
        float duration;
        float delay_time;
    } param_;

    float output_;
    float time_;
};
}