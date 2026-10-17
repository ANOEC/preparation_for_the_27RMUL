#include "./squre.hpp"
#include <cmath>

namespace algorithm::math::wave {
float SquareWaveGenerator::update(float current_time)
{
    if (!initialized_) {
        time_ = current_time;
        initialized_ = true;
    }

    auto elapsed_time = current_time - time_;

    auto time_in_period = std::fmod(elapsed_time, param_.period);
    if (time_in_period < param_.duty_cycle * param_.period) {
        output_ = param_.peak + param_.offset;
    } else {
        output_ = param_.offset;
    }

    return output_;
}

void SquareWaveGenerator::set_frequency(float frequency)
{
    param_.frequency = frequency;
    param_.period = 1.0f / frequency;
}

void SquareWaveGenerator::set_duty_cycle(float duty_cycle)
{
    param_.duty_cycle = duty_cycle;
}

void SquareWaveGenerator::set_peak(float peak)
{
    param_.peak = peak;
}

void SquareWaveGenerator::set_offset(float offset)
{
    param_.offset = offset;
}

void SquareWaveGenerator::reset()
{
    time_ = 0.0f;
    output_ = 0.0f;
    initialized_ = false;
}
}