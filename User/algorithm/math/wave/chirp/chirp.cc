#include "./chirp.hpp"
#include <cmath>
#include <numbers>

namespace algorithm::math::wave::chirp {

Chirp::Chirp(float start_freq, float end_freq, float period, float amplitude, float offset)
    : param_ { start_freq, end_freq, period, amplitude, offset }
{
}

void Chirp::set_param(float start_freq, float end_freq, float period, float amplitude, float offset)
{
    param_ = { start_freq, end_freq, period, amplitude, offset };
    reset();
}

void Chirp::compute(float dt)
{
    if (param_.period_ <= 0.0f) {
        status_.output_ = param_.offset_;
        return;
    }

    if (status_.finished_) {
        status_.output_ = param_.offset_;
        return;
    }

    if (status_.current_time_ < param_.period_) {
        float k = (param_.end_freq_ - param_.start_freq_) / param_.period_;
        float inst_freq = param_.start_freq_ + k * status_.current_time_;

        status_.current_phase_ += 2.0f * std::numbers::pi_v<float> * inst_freq * dt;

        while (status_.current_phase_ > 2.0f * std::numbers::pi_v<float>) {
            status_.current_phase_ -= 2.0f * std::numbers::pi_v<float>;
        }

        status_.current_time_ += dt;
        status_.output_ = param_.amplitude_ * std::sin(status_.current_phase_) + param_.offset_;
    } else {
        status_.output_ = param_.offset_;
        status_.finished_ = true;
    }
}

float Chirp::get_output() const
{
    return status_.output_;
}

bool Chirp::is_finished() const
{
    return status_.finished_;
}   

void Chirp::reset()
{
    status_.current_phase_ = 0.0f;
    status_.current_time_ = 0.0f;
    status_.finished_ = false;
    status_.output_ = param_.offset_;
}
}
