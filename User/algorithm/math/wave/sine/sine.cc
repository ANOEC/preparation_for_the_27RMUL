#include "./sine.hpp"

namespace algorithm::math::wave {
float SineWaveGenerator::update(float current_time)
{
    if (!initialized_) {
        initialized_ = true;
        time_ = current_time;
        return param_.offset;
    }

    auto time_period = current_time - time_;
    output_ += param_.angularFrequency * time_period;
    time_ = current_time;
    return param_.amplitude * std::sin(output_) + param_.offset;
}

void SineWaveGenerator::set_frequency(float frequency)
{
    param_.angularFrequency = 2.0f * std::numbers::pi_v<float> * frequency;
}

void SineWaveGenerator::set_amplitude(float amp)
{
    param_.amplitude = amp;
}

void SineWaveGenerator::set_offset(float off)
{
    param_.offset = off;
}
}