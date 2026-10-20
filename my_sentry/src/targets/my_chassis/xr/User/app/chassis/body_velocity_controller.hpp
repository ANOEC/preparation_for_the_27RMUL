#pragma once

#include "chassis_kinematics.hpp"

namespace app::chassis {

/** Whole-body velocity loop placed between the operator command and the wheel
 * loops. It applies a vector acceleration ramp, latching quick braking and
 * independent X/Y/yaw PID feedback. */
class BodyVelocityController {
public:
    struct AxisConfig {
        float kp = 0.0f;
        float ki = 0.0f;
        float kd = 0.0f;
        float error_deadband = 0.0f;
        float integral_limit = 0.0f;
        float output_limit = 0.0f;
    };

    struct Config {
        AxisConfig x {};
        AxisConfig y {};
        AxisConfig yaw {};

        // Source-project-equivalent acceleration limits. Translational limits
        // are vector magnitudes, so diagonal motion is not accelerated faster.
        float translation_acceleration_limit = 3.0f;
        float sprint_acceleration_limit = 6.0f;
        float sprint_activation_speed = 1.5f;
        float yaw_acceleration_limit = 200.0f;

        float quick_brake_acceleration_limit = 30.0f;
        float quick_brake_command_delta = 0.1f;
        float quick_brake_speed_threshold_x = 2.0f;
        float quick_brake_speed_threshold_y = 2.0f;
        float quick_brake_release_speed = 0.1f;
    };

    struct Result {
        ChassisVelocity target_velocity {};
        ChassisVelocity force_command {};
        bool quick_braking = false;
        bool sprinting = false;
    };

    BodyVelocityController() = default;
    explicit BodyVelocityController(const Config& config);

    void set_config(const Config& config);
    const Config& config() const { return config_; }
    void reset();

    Result update(const ChassisVelocity& requested_velocity,
                  const ChassisVelocity& measured_velocity, float dt);

private:
    struct AxisState {
        float integral = 0.0f;
        float previous_error = 0.0f;
        bool initialized = false;
    };

    static float UpdateAxis(const AxisConfig& config, AxisState& state,
                            float target, float measured, float dt);
    static float RampScalar(float current, float target, float maximum_rate,
                            float dt);
    static bool RequestsBraking(float previous, float current, float delta);

    Config config_ {};
    ChassisVelocity ramped_target_ {};
    ChassisVelocity previous_request_ {};
    AxisState x_state_ {};
    AxisState y_state_ {};
    AxisState yaw_state_ {};
    bool quick_braking_ = false;
    bool request_initialized_ = false;
};

} // namespace app::chassis
