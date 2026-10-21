#pragma once

#include "app/chassis/chassis.hpp"
#include "config/chassis/generated/chassis_params.hpp"

namespace app::chassis {

/**
 * Build the chassis controller configuration from the generated TOML values.
 *
 * Keep unit conversion and hardware-interface conventions here instead of in
 * the generated header.  In particular, TOML stores the IMU yaw shift in
 * degrees, while ChassisSolver expects radians.  DJIMotor feedback is already
 * expressed at the gearbox output shaft, so WheelController uses a ratio of 1.
 */
inline Chassis::Config MakeChassisConfig()
{
    constexpr float kDegreesToRadians =
        3.14159265358979323846f / 180.0f;

    Chassis::Config config {};

    config.kinematics.wheel_radius = param::chassis::param.omni_wheel_r;
    config.kinematics.wheelbase = param::chassis::param.car_length;
    config.kinematics.track_width = param::chassis::param.car_wide;
    config.kinematics.rotation_radius = param::chassis::param.car_r;

    config.solver.kinematics = config.kinematics;
    config.solver.imu_offset_x = param::chassis::imu.offset_x;
    config.solver.imu_offset_y = param::chassis::imu.offset_y;
    config.solver.imu_yaw =
        param::chassis::imu.horizontal_shift * kDegreesToRadians;
    config.solver.gyro_observer_frequency = param::chassis::imu.w03;
    config.solver.vehicle_mass = param::chassis::param.car_weight;
    config.solver.slope_activation_degrees =
        param::chassis::param.slope_angle;

    const auto apply_body_pid = [](BodyVelocityController::AxisConfig& axis,
                                   const auto& pid) {
        axis.kp = pid.kp;
        axis.ki = pid.ki;
        axis.kd = pid.kd;
        axis.error_deadband = pid.error_threshold;
        axis.integral_limit = pid.integral_limit;
        axis.output_limit = pid.max_output;
    };
    apply_body_pid(config.body_controller.x, param::chassis::pid::x);
    apply_body_pid(config.body_controller.y, param::chassis::pid::y);
    apply_body_pid(config.body_controller.yaw, param::chassis::pid::z);
    config.body_controller.quick_brake_speed_threshold_x =
        param::chassis::param.brake_x;
    config.body_controller.quick_brake_speed_threshold_y =
        param::chassis::param.brake_y;
    config.body_controller.translation_acceleration_limit =
        param::chassis::param.translation_acceleration;
    config.body_controller.sprint_acceleration_limit =
        param::chassis::param.sprint_acceleration;
    config.body_controller.sprint_activation_speed =
        param::chassis::param.sprint_activation_speed;
    config.body_controller.yaw_acceleration_limit =
        param::chassis::param.yaw_acceleration;
    config.body_controller.quick_brake_acceleration_limit =
        param::chassis::param.quick_brake_acceleration;
    config.body_controller.quick_brake_command_delta =
        param::chassis::param.quick_brake_command_delta;
    config.body_controller.quick_brake_release_speed =
        param::chassis::param.quick_brake_release_speed;

    const auto apply_motor_pid = [](WheelController::Config& wheel,
                                    const auto& pid) {
        wheel.kp = pid.kp;
        wheel.ki = pid.ki;
        wheel.kd = pid.kd;
        wheel.error_deadband = pid.error_threshold;
        wheel.integral_limit = pid.integral_limit;
        wheel.output_limit = pid.max_output;
        wheel.polarity = -1.0f;
        wheel.gear_ratio = 1.0f;
    };

    apply_motor_pid(config.wheels[0], param::chassis::pid::motor1);
    apply_motor_pid(config.wheels[1], param::chassis::pid::motor2);
    apply_motor_pid(config.wheels[2], param::chassis::pid::motor3);
    apply_motor_pid(config.wheels[3], param::chassis::pid::motor4);

    config.follow_kp = param::chassis::pid::follow.kp;
    config.follow_kd = param::chassis::pid::follow.kd;
    config.follow_output_limit = param::chassis::pid::follow.max_output;
    config.maximum_tilt_for_spin = param::chassis::param.stop_spin_angle;

    return config;
}

} // namespace app::chassis
