#include "./prbs.hpp"

namespace algorithm::math::wave::prbs {

Prbs::Prbs(float amplitude, float offset, float period, float update_interval)
    : param_ { amplitude, offset, period, update_interval }
{
    reset();
}

void Prbs::set_param(float amplitude, float offset, float period, float update_interval)
{
    param_ = { amplitude, offset, period, update_interval };
    reset();
}

void Prbs::compute(float dt)
{
    if (param_.period_ <= 0.0f) {
        status_.output_ = param_.offset_;
        return;
    }

    if (status_.current_time_ <= param_.period_) {
        status_.time_since_last_update_ += dt;

        if (status_.time_since_last_update_ >= param_.update_interval_) {

            uint16_t bit = ((status_.lfsr_state_ >> 10) ^ (status_.lfsr_state_ >> 8)) & 1;

            status_.lfsr_state_ = ((status_.lfsr_state_ << 1) & 0x07FF) | bit;

            if (status_.lfsr_state_ & 1) {
                status_.output_ = param_.offset_ + param_.amplitude_;
            } else {
                status_.output_ = param_.offset_ - param_.amplitude_;
            }

            status_.time_since_last_update_ -= param_.update_interval_;
        }

        status_.current_time_ += dt;
    } else {
        status_.output_ = param_.offset_;
        status_.finished_ = true;
    }
}

float Prbs::get_output() const
{
    return status_.output_;
}

bool Prbs::is_finished() const
{
    return status_.finished_;
}

void Prbs::reset()
{
    status_.current_time_ = 0.0f;
    status_.time_since_last_update_ = param_.update_interval_;
    status_.lfsr_state_ = 0x07FF; 
    status_.finished_ = false;
    status_.output_ = param_.offset_;
}
}