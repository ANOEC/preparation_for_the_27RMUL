#include "./triangle.hpp"

namespace algorithm::math::wave::triangle {
Triangle::Triangle(float period, float amplitude, float offset)
    : param_ { period, amplitude, offset }
{
}

void Triangle::set_param(float period, float amplitude, float offset)
{
    param_.period_ = period;
    param_.amplitude_ = amplitude;
    param_.offset_ = offset;
    
}

void Triangle::compute(float dt)
{
    if (param_.period_ > 0.0f) {
        status_.current_phase_ += dt / param_.period_;

        if (status_.current_phase_ >= 1.0f) {
            status_.current_phase_ -= 1.0f;
        }

        float p = status_.current_phase_;
        if (p < 0.25f) {
            status_.output_ = 4.0f * param_.amplitude_ * p + param_.offset_;
        } else if (p < 0.75f) {
            status_.output_ = -4.0f * param_.amplitude_ * (p - 0.5f) + param_.offset_;
        } else {
            status_.output_ = 4.0f * param_.amplitude_ * (p - 1.0f) + param_.offset_;
        }
    } else {
        status_.output_ = param_.offset_;
    }
}

float Triangle::get_output() const 
{
    return status_.output_;
}
}