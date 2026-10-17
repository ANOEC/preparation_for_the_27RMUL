#include "body_velocity_controller.hpp"

#include <algorithm>
#include <cmath>

namespace app::chassis {
namespace {
float ClampSymmetric(float value, float limit)
{
    return limit > 0.0f ? std::clamp(value, -limit, limit) : value;
}

float SanitizedNonnegative(float value)
{
    return std::isfinite(value) ? std::max(value, 0.0f) : 0.0f;
}
} // namespace

BodyVelocityController::BodyVelocityController(const Config& config)
{
    set_config(config);
}

void BodyVelocityController::set_config(const Config& config)
{
    config_ = config;
    config_.translation_acceleration_limit =
        SanitizedNonnegative(config_.translation_acceleration_limit);
    config_.sprint_acceleration_limit =
        SanitizedNonnegative(config_.sprint_acceleration_limit);
    config_.sprint_activation_speed =
        SanitizedNonnegative(config_.sprint_activation_speed);
    config_.yaw_acceleration_limit =
        SanitizedNonnegative(config_.yaw_acceleration_limit);
    config_.quick_brake_acceleration_limit =
        SanitizedNonnegative(config_.quick_brake_acceleration_limit);
    config_.quick_brake_command_delta =
        SanitizedNonnegative(config_.quick_brake_command_delta);
    config_.quick_brake_speed_threshold_x =
        SanitizedNonnegative(config_.quick_brake_speed_threshold_x);
    config_.quick_brake_speed_threshold_y =
        SanitizedNonnegative(config_.quick_brake_speed_threshold_y);
    config_.quick_brake_release_speed =
        SanitizedNonnegative(config_.quick_brake_release_speed);
    reset();
}

void BodyVelocityController::reset()
{
    ramped_target_ = {};
    previous_request_ = {};
    x_state_ = {};
    y_state_ = {};
    yaw_state_ = {};
    quick_braking_ = false;
    request_initialized_ = false;
}

float BodyVelocityController::RampScalar(float current, float target,
                                         float maximum_rate, float dt)
{
    if (maximum_rate <= 0.0f) {
        return target;
    }
    const float maximum_step = maximum_rate * dt;
    return current + std::clamp(target - current, -maximum_step, maximum_step);
}

bool BodyVelocityController::RequestsBraking(float previous, float current,
                                             float delta)
{
    return (previous < 0.0f && current > previous + delta) ||
           (previous > 0.0f && current < previous - delta);
}

float BodyVelocityController::UpdateAxis(const AxisConfig& config,
                                         AxisState& state, float target,
                                         float measured, float dt)
{
    float error = target - measured;
    if (std::abs(error) < std::abs(config.error_deadband)) {
        error = 0.0f;
    }

    const float derivative = state.initialized
        ? (error - state.previous_error) / dt
        : 0.0f;
    const float integral_limit = std::abs(config.integral_limit);
    const float candidate_integral = integral_limit > 0.0f
        ? std::clamp(state.integral + error * dt,
                     -integral_limit, integral_limit)
        : 0.0f;
    const float without_integral =
        config.kp * error + config.kd * derivative;
    const float raw = without_integral + config.ki * candidate_integral;
    const float limited = ClampSymmetric(raw, std::abs(config.output_limit));

    if (raw == limited || error * raw <= 0.0f) {
        state.integral = candidate_integral;
    }
    state.previous_error = error;
    state.initialized = true;
    return std::isfinite(limited) ? limited : 0.0f;
}

BodyVelocityController::Result BodyVelocityController::update(
    const ChassisVelocity& requested_velocity,
    const ChassisVelocity& measured_velocity, float dt)
{
    Result result {};
    if (dt <= 0.0f || !std::isfinite(dt) ||
        !std::isfinite(requested_velocity.vx) ||
        !std::isfinite(requested_velocity.vy) ||
        !std::isfinite(requested_velocity.wz) ||
        !std::isfinite(measured_velocity.vx) ||
        !std::isfinite(measured_velocity.vy) ||
        !std::isfinite(measured_velocity.wz)) {
        reset();
        return result;
    }

    const bool brake_requested = request_initialized_ &&
        (RequestsBraking(previous_request_.vx, requested_velocity.vx,
                         config_.quick_brake_command_delta) ||
         RequestsBraking(previous_request_.vy, requested_velocity.vy,
                         config_.quick_brake_command_delta));
    const bool moving_fast =
        std::abs(measured_velocity.vx) > config_.quick_brake_speed_threshold_x ||
        std::abs(measured_velocity.vy) > config_.quick_brake_speed_threshold_y;
    if (!quick_braking_ && brake_requested && moving_fast) {
        quick_braking_ = true;
        // Begin the braking ramp from the measured motion rather than from a
        // possibly faster stale command. This guarantees that the first
        // quick-brake target already asks for deceleration.
        ramped_target_.vx = measured_velocity.vx;
        ramped_target_.vy = measured_velocity.vy;
        x_state_ = {};
        y_state_ = {};
    }
    if (quick_braking_ &&
        std::hypot(measured_velocity.vx, measured_velocity.vy) <=
            config_.quick_brake_release_speed) {
        quick_braking_ = false;
        x_state_ = {};
        y_state_ = {};
    }

    previous_request_ = requested_velocity;
    request_initialized_ = true;

    ChassisVelocity effective_request = requested_velocity;
    if (quick_braking_) {
        effective_request.vx = 0.0f;
        effective_request.vy = 0.0f;
    }

    const float requested_translation_speed =
        std::hypot(requested_velocity.vx, requested_velocity.vy);
    const bool sprinting = requested_translation_speed >
                           config_.sprint_activation_speed;
    const float normal_acceleration = sprinting
        ? config_.sprint_acceleration_limit
        : config_.translation_acceleration_limit;
    const float acceleration = quick_braking_
        ? config_.quick_brake_acceleration_limit
        : normal_acceleration;

    const float delta_x = effective_request.vx - ramped_target_.vx;
    const float delta_y = effective_request.vy - ramped_target_.vy;
    const float delta_norm = std::hypot(delta_x, delta_y);
    const float maximum_step = acceleration * dt;
    if (acceleration <= 0.0f || delta_norm <= maximum_step || delta_norm == 0.0f) {
        ramped_target_.vx = effective_request.vx;
        ramped_target_.vy = effective_request.vy;
    } else {
        const float scale = maximum_step / delta_norm;
        ramped_target_.vx += delta_x * scale;
        ramped_target_.vy += delta_y * scale;
    }
    ramped_target_.wz = RampScalar(ramped_target_.wz, effective_request.wz,
                                   config_.yaw_acceleration_limit, dt);

    result.target_velocity = ramped_target_;
    result.force_command.vx =
        UpdateAxis(config_.x, x_state_, ramped_target_.vx,
                   measured_velocity.vx, dt);
    result.force_command.vy =
        UpdateAxis(config_.y, y_state_, ramped_target_.vy,
                   measured_velocity.vy, dt);
    result.force_command.wz =
        UpdateAxis(config_.yaw, yaw_state_, ramped_target_.wz,
                   measured_velocity.wz, dt);
    result.quick_braking = quick_braking_;
    result.sprinting = sprinting;
    return result;
}

} // namespace app::chassis
