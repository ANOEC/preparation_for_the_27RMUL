#include "wheel_controller.hpp"

#include <algorithm>
#include <cmath>

namespace app::chassis {
namespace {
float clamp_symmetric(float value, float limit)
{
    return limit > 0.0f ? std::clamp(value, -limit, limit) : value;
}
}

WheelController::WheelController(const Config& config) { set_config(config); }

void WheelController::set_config(const Config& config)
{
    config_ = config;
    config_.gear_ratio = std::max(std::abs(config_.gear_ratio), 1.0e-6f);
    config_.polarity = config_.polarity < 0.0f ? -1.0f : 1.0f;
    config_.output_limit = std::abs(config_.output_limit);
    config_.integral_limit = std::abs(config_.integral_limit);
}

void WheelController::reset()
{
    ramped_target_ = 0.0f;
    integral_ = 0.0f;
    previous_error_ = 0.0f;
    online_time_ = 0.0f;
    was_online_ = false;
}

WheelController::Result WheelController::update(float target_wheel_speed,
                                                const Feedback& feedback,
                                                float dt,
                                                float torque_feedforward,
                                                bool brake)
{
    Result result {};
    if (!feedback.online || dt <= 0.0f || !std::isfinite(dt)) {
        reset();
        return result;
    }

    if (!was_online_) {
        ramped_target_ = 0.0f;
        integral_ = 0.0f;
        previous_error_ = 0.0f;
    }
    was_online_ = true;
    online_time_ += dt;

    const float desired_motor = target_wheel_speed * config_.gear_ratio;
    const float difference = desired_motor - ramped_target_;
    float ramp_limit = difference * ramped_target_ < 0.0f
                         ? config_.deceleration_limit
                         : config_.acceleration_limit;
    if (ramp_limit <= 0.0f) {
        ramped_target_ = desired_motor;
    } else {
        ramped_target_ += std::clamp(difference, -ramp_limit * dt, ramp_limit * dt);
    }

    const float measured = feedback.motor_speed * config_.polarity;
    result.target_motor_speed = ramped_target_;
    result.measured_motor_speed = measured;

    if (brake) {
        ramped_target_ = 0.0f;
        integral_ = 0.0f;
        result.torque = clamp_symmetric(-config_.brake_gain * measured,
                                        config_.output_limit);
    } else {
        float error = ramped_target_ - measured;
        if (std::abs(error) < config_.error_deadband) {
            error = 0.0f;
        }
        const float derivative = (error - previous_error_) / dt;
        const float candidate_integral = clamp_symmetric(
            integral_ + error * dt, config_.integral_limit);
        const float acceleration = difference / dt;
        const float static_ff = ramped_target_ > 0.0f ? config_.static_feedforward
                              : ramped_target_ < 0.0f ? -config_.static_feedforward
                                                      : 0.0f;
        const float without_i = config_.kp * error + config_.kd * derivative
                              + config_.velocity_feedforward * ramped_target_
                              + config_.acceleration_feedforward * acceleration
                              + static_ff + torque_feedforward;
        const float raw = without_i + config_.ki * candidate_integral;
        const float limited = clamp_symmetric(raw, config_.output_limit);

        // Conditional integration: do not deepen saturation.
        if (raw == limited || error * raw <= 0.0f) {
            integral_ = candidate_integral;
        }
        const float soft_scale = config_.soft_start_seconds > 0.0f
            ? std::clamp(online_time_ / config_.soft_start_seconds, 0.0f, 1.0f)
            : 1.0f;
        result.torque = limited * soft_scale;
        result.saturated = raw != limited;
        previous_error_ = error;
    }

    result.torque *= config_.polarity;
    if (!std::isfinite(result.torque)) {
        reset();
        return {};
    }
    return result;
}

} // namespace app::chassis
