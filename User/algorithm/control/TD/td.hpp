#pragma once

namespace algorithm::control::td {
class TrackingDifferentiator {
private:
    float v1 = 0.0f;
    float v2 = 0.0f;
    float a_ = 0.0f;
    float r = 50.0f;
    float h = 0.001f;
    float h0 = 0.001f;

    inline int sign(float val)
    {
        return (val > 0) - (val < 0);
    }

public:
    TrackingDifferentiator() = default;
    TrackingDifferentiator(float r_val, float h_val, float h0_val);

    void set_param(float r_val, float h_val, float h0_val);

    void update(float target);
    void reset(float position = 0.0f, float velocity = 0.0f);

    inline float get_position() const { return v1; }
    inline float get_velocity() const { return v2; }
    inline float get_acceleration() const { return a_; }
};
}
