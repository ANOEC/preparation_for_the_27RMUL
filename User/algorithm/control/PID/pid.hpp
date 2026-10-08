#pragma once

namespace algorithm::control::pid {
enum class PIDType {
    POSITION,
    INCREMENTAL
};

class PID {
public:
    struct Param {
        float kp_ = 0.0f;
        float ki_ = 0.0f;
        float kd_ = 0.0f;
        float deadband_ = 0.0f;
        float integral_limit_ = 0.0f;
        float output_limit_ = 0.0f;
        PIDType type_ = PIDType::POSITION;
    };

    PID() = default;
    PID(float kp, float ki, float kd, float deadband,
        float integral_limit, float output_limit, PIDType type = PIDType::POSITION);

    void set_param(float kp, float ki, float kd, float deadband,
        float integral_limit, float output_limit, PIDType type = PIDType::POSITION);
    void set_param(const Param& param)
    {
        param_ = param;
    }

    float compute(float ref, float feedback);
    void reset();
    float out() const;
    float error() const;
    
    inline const Param& get_param() const
    {
        return param_;
    }

private:
    Param param_;

    struct {
        float ref_ = 0.0f;
        float error_ = 0.0f;
        float last_error_ = 0.0f;
        float last_l_error_ = 0.0f;
        float integral_ = 0.0f;
        float output_ = 0.0f;
        float last_output_ = 0.0f;
    } state_;
};
}
