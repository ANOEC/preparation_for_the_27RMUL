#include "./step.hpp"

namespace algorithm::math::wave {
float StepWaveGenerator::update(float current_time)
{
    auto elapsed_time = current_time - time_;
    if (elapsed_time < param_.delay_time) {
        output_ = param_.offset;
    } else if (elapsed_time < param_.delay_time + param_.duration) {
        output_ = param_.peak + param_.offset;
    } else {
        if (param_.duration <= 0.0f) {
            output_ = param_.peak + param_.offset;
        } else {
            output_ = param_.offset;
        }
    }
    return output_;
}

void StepWaveGenerator::reset()
{
    time_ = 0.0f;
    output_ = 0.0f;
}
}