#pragma once

namespace app::chassis {

class WheelController {
public:
    struct Config {
        float kp = 0.1f;
        float ki = 0.0f;
        float kd = 0.0f;
        float integral_limit = 0.0f;
        float output_limit = 6.0f;
        float error_deadband = 0.0f;
        float acceleration_limit = 0.0f; // rad/s^2; zero disables ramping.
        float deceleration_limit = 0.0f;
        float velocity_feedforward = 0.0f;
        float acceleration_feedforward = 0.0f;
        float static_feedforward = 0.0f;
        float soft_start_seconds = 0.20f;
        float polarity = -1.0f;
        float gear_ratio = 1.0f;
        float brake_gain = 0.1f;
    };

    struct Feedback {
        float motor_speed = 0.0f;
        float motor_torque = 0.0f;
        bool online = false;
    };

    struct Result {
        float target_motor_speed = 0.0f;
        float measured_motor_speed = 0.0f;
        float torque = 0.0f;
        bool saturated = false;
    };

    WheelController() = default;
    explicit WheelController(const Config& config);

    void set_config(const Config& config);
    const Config& config() const { return config_; }
    void reset();

    // target_wheel_speed is at the wheel. Feedback and result torque follow the
    // physical chassis polarity, hiding motor wiring direction from callers.
    Result update(float target_wheel_speed, const Feedback& feedback, float dt,
                  float torque_feedforward = 0.0f, bool brake = false);

private:
    Config config_ {};
    float ramped_target_ = 0.0f;
    float integral_ = 0.0f;
    float previous_error_ = 0.0f;
    float online_time_ = 0.0f;
    bool was_online_ = false;
};

} // namespace app::chassis
