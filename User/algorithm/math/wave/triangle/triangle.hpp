#pragma once

#include <cmath>
namespace algorithm::math::wave::triangle {
class Triangle {
public:
    Triangle() = default;
    Triangle(float period, float amplitude, float offset);

    void set_param(float period, float amplitude, float offset);

    void compute(float dt);

    float get_output() const;

    void reset_phase()
    {
        status_.current_phase_ = 0.0f;
    }

private:
    struct {
        float period_;
        float amplitude_;
        float offset_;
    } param_;

    struct {
        float output_;
        float current_phase_ = 0.0f;
    } status_;
};
}