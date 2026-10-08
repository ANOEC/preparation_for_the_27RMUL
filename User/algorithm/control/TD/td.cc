#include "./td.hpp"
#include <cmath>

namespace algorithm::control::td {
TrackingDifferentiator::TrackingDifferentiator(float r_val, float h_val, float h0_val)
{
    set_param(r_val, h_val, h0_val);
}

void TrackingDifferentiator::set_param(float r_val, float h_val, float h0_val)
{
    r = r_val;
    h = h_val;
    h0 = h0_val;
}

void TrackingDifferentiator::update(float target)
{
    float d = r * h0;
    float d0 = h0 * d;
    float y = v1 - target + h0 * v2;
    float a0 = std::sqrt(d * d + 8.0f * r * std::abs(y));
    float a = 0.0f;

    if (std::abs(y) > d0) {
        a = v2 + (a0 - d) / 2.0f * sign(y);
    } else {
        a = v2 + y / h0;
    }

    float fst = 0.0f;
    if (std::abs(a) > d) {
        fst = -r * sign(a);
    } else {
        fst = -r * a / d;
    }
    a_ = fst;

    // 积分更新
    v1 += h * v2;
    v2 += h * fst;
}

void TrackingDifferentiator::reset(float position, float velocity)
{
    v1 = position;
    v2 = velocity;
    a_ = 0.0f;
}
}
