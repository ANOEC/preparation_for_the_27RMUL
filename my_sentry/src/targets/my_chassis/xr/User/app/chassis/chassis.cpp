#include "chassis.hpp"

#include <algorithm>
#include <cmath>

namespace app::chassis {
namespace {
constexpr float kPi = 3.14159265358979323846f;
}

Chassis::Chassis() { set_config(Config {}); }
Chassis::Chassis(const Config& config) { set_config(config); }

void Chassis::set_config(const Config& config)
{
    config_ = config;
    config_.solver.kinematics = config_.kinematics;
    kinematics_.set_config(config_.kinematics);
    solver_.set_config(config_.solver);
    body_controller_.set_config(config_.body_controller);
    for (std::size_t i = 0; i < kWheelCount; ++i) {
        // Aggregate initialization leaves an array of default Config objects,
        // whose source-project polarity is -1.
        wheel_controllers_[i].set_config(config_.wheels[i]);
    }
    power_control_.set_config(config_.power);
    reset();
}

void Chassis::reset()
{
    for (auto& controller : wheel_controllers_) controller.reset();
    solver_.reset();
    body_controller_.reset();
    power_control_.reset();
    previous_follow_error_ = 0.0f;
    output_ = {};
}

bool Chassis::devices_healthy() const
{
    if (config_.require_command_link && !feedback_.command_link_online) return false;
    if (config_.require_imu && !feedback_.imu.online) return false;
    for (const auto& wheel : feedback_.wheels) if (!wheel.online) return false;
    return true;
}

ChassisMode Chassis::choose_mode() const
{
    if (!devices_healthy() || command_.mode == ChassisMode::Protect) return ChassisMode::Protect;
    const float tilt = std::acos(std::clamp(std::cos(feedback_.roll) *
                                            std::cos(feedback_.pitch), 0.0f, 1.0f));
    if (command_.mode == ChassisMode::Spin &&
        tilt > config_.maximum_tilt_for_spin * kPi / 180.0f) return ChassisMode::Follow;
    return command_.mode;
}

const Chassis::Output& Chassis::update(float dt)
{
    if (dt <= 0.0f || !std::isfinite(dt)) {
        output_.motor_torque.fill(0.0f);
        output_.safe_output = true;
        output_.active_mode = ChassisMode::Protect;
        return output_;
    }

    ChassisSolver::Feedback solver_feedback {};
    solver_feedback.imu = feedback_.imu;
    solver_feedback.yaw_to_world = feedback_.chassis_yaw;
    solver_feedback.roll = feedback_.roll;
    solver_feedback.pitch = feedback_.pitch;
    for (std::size_t i = 0; i < kWheelCount; ++i) {
        const auto& wheel_config = wheel_controllers_[i].config();
        solver_feedback.wheel_online[i] = feedback_.wheels[i].online;
        solver_feedback.wheel_speed[i] = feedback_.wheels[i].motor_speed *
            wheel_config.polarity / wheel_config.gear_ratio;
        solver_feedback.wheel_torque[i] = feedback_.wheels[i].motor_torque *
            wheel_config.polarity * wheel_config.gear_ratio;
    }
    output_.estimate = solver_.update(solver_feedback, dt);
    output_.active_mode = choose_mode();
    output_.safe_output = output_.active_mode == ChassisMode::Protect;

    if (output_.safe_output) {
        output_.motor_torque.fill(0.0f);
        output_.target_wheel_speed.fill(0.0f);
        output_.target_body_velocity = {};
        output_.body_force_command = {};
        output_.quick_braking = false;
        output_.sprinting = false;
        body_controller_.reset();
        previous_follow_error_ = 0.0f;
        for (auto& controller : wheel_controllers_) controller.reset();
        return output_;
    }

    ChassisVelocity target {command_.vx, command_.vy, command_.wz};
    if (command_.world_frame) target = ChassisKinematics::world_to_body(target, feedback_.chassis_yaw);
    if (output_.active_mode == ChassisMode::Follow) {
        // The follow gains come directly from the reference project's discrete PID.
        // Its D term acts on the per-cycle error delta, not error delta per second.
        // Dividing by the 1 ms control period amplifies encoder quantisation by about
        // 1000 times and makes the requested yaw rate chatter between its limits.
        const float error_delta = std::remainder(
            command_.follow_yaw_error - previous_follow_error_, 2.0f * kPi);
        target.wz = std::clamp(config_.follow_kp * command_.follow_yaw_error +
                               config_.follow_kd * error_delta +
                               config_.follow_velocity_feedforward *
                                   command_.follow_yaw_velocity,
                               -config_.follow_output_limit, config_.follow_output_limit);
        previous_follow_error_ = command_.follow_yaw_error;
    } else if (output_.active_mode == ChassisMode::Spin) {
        target.wz = command_.wz == 0.0f ? config_.spin_speed : command_.wz;
    } else if (output_.active_mode == ChassisMode::Brake) {
        target = {};
    }
    BodyVelocityController::Result body_control {};
    if (output_.active_mode == ChassisMode::Brake) {
        body_controller_.reset();
    } else {
        body_control = body_controller_.update(
            target, output_.estimate.body_velocity, dt);
    }
    output_.target_body_velocity = body_control.target_velocity;
    output_.body_force_command = body_control.force_command;
    output_.quick_braking = body_control.quick_braking;
    output_.sprinting = body_control.sprinting;
    output_.target_wheel_speed =
        kinematics_.inverse(output_.target_body_velocity);
    solver_.update_lock_state(output_.target_body_velocity, dt);

    ChassisVelocity additional_force = command_.force_feedforward;
    if (command_.world_frame) {
        additional_force = ChassisKinematics::world_to_body(
            additional_force, feedback_.chassis_yaw);
    }
    ChassisVelocity force {
        body_control.force_command.vx + additional_force.vx,
        body_control.force_command.vy + additional_force.vy,
        body_control.force_command.wz + additional_force.wz,
    };
    output_.body_force_command = force;
    const auto wheel_feedforward = solver_.allocate_torque(force);

    PowerControl::Input power_input {};
    for (std::size_t i = 0; i < kWheelCount; ++i) {
        const auto result = wheel_controllers_[i].update(
            output_.target_wheel_speed[i], feedback_.wheels[i], dt,
            wheel_feedforward[i], output_.active_mode == ChassisMode::Brake);
        power_input.requested_torque[i] = result.torque;
        power_input.wheel_speed[i] = feedback_.wheels[i].motor_speed;
        power_input.measured_torque[i] = feedback_.wheels[i].motor_torque;
        power_input.maximum_torque[i] = wheel_controllers_[i].config().output_limit;
        power_input.target_speed[i] = result.target_motor_speed;
    }
    power_input.referee_online = feedback_.referee_online;
    power_input.referee_power_limit = feedback_.referee_power_limit;
    power_input.referee_buffer_energy = feedback_.referee_buffer_energy;
    power_input.capacitor_online = feedback_.capacitor_online;
    power_input.capacitor_power = feedback_.capacitor_power;
    power_input.capacitor_energy = feedback_.capacitor_energy;
    power_input.auxiliary_power = feedback_.auxiliary_power;
    power_input.request_boost = command_.boost_power;

    output_.motor_torque = power_control_.update(power_input);
    output_.power = power_control_.status();
    return output_;
}

} // namespace app::chassis
