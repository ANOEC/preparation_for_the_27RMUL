#pragma once

#include <array>
#include <cstdint>

#include "body_velocity_controller.hpp"
#include "chassis_kinematics.hpp"
#include "chassis_solver.hpp"
#include "power_control.hpp"
#include "wheel_controller.hpp"

namespace app::chassis {

enum class ChassisMode : std::uint8_t {
    Protect,
    Free,
    Follow,
    Spin,
    Brake,
};

/** Application-level coordinator. Hardware adapters push one Feedback snapshot
 * and consume one Output snapshot; this class never sends CAN by itself. */
class Chassis {
public:
    static constexpr std::size_t kWheelCount = 4;

    struct Config {
        ChassisKinematics::Config kinematics {};
        ChassisSolver::Config solver {};
        BodyVelocityController::Config body_controller {};
        std::array<WheelController::Config, kWheelCount> wheels {};
        PowerControl::Config power {};
        float follow_kp = -6.0f;
        float follow_kd = -4.0f;
        float follow_output_limit = 8.0f;
        float spin_speed = 6.0f;
        float maximum_tilt_for_spin = 12.0f; // degrees
        bool require_imu = true;
        bool require_command_link = true;
    };

    struct Command {
        float vx = 0.0f;
        float vy = 0.0f;
        float wz = 0.0f;
        float follow_yaw_error = 0.0f;
        ChassisVelocity force_feedforward {};
        ChassisMode mode = ChassisMode::Protect;
        bool world_frame = true;
        bool boost_power = false;
    };

    struct Feedback {
        std::array<WheelController::Feedback, kWheelCount> wheels {};
        ChassisSolver::ImuFeedback imu {};
        float chassis_yaw = 0.0f;
        float roll = 0.0f;
        float pitch = 0.0f;
        bool command_link_online = false;
        bool referee_online = false;
        float referee_power_limit = 0.0f;
        float referee_buffer_energy = 0.0f;
        bool capacitor_online = false;
        float capacitor_power = 0.0f;
        float capacitor_energy = 0.0f;
        float auxiliary_power = 0.0f;
    };

    struct Output {
        std::array<float, kWheelCount> motor_torque {};
        ChassisKinematics::WheelVector target_wheel_speed {};
        ChassisVelocity target_body_velocity {};
        ChassisVelocity body_force_command {};
        ChassisSolver::Estimate estimate {};
        PowerControl::Status power {};
        ChassisMode active_mode = ChassisMode::Protect;
        bool safe_output = true;
        bool quick_braking = false;
        bool sprinting = false;
    };

    Chassis();
    explicit Chassis(const Config& config);

    void set_config(const Config& config);
    void set_command(const Command& command) { command_ = command; }
    void set_feedback(const Feedback& feedback) { feedback_ = feedback; }
    const Output& update(float dt);
    void reset();

    const Output& output() const { return output_; }
    const ChassisKinematics& kinematics() const { return kinematics_; }
    const ChassisSolver& solver() const { return solver_; }
    const PowerControl& power_control() const { return power_control_; }

private:
    bool devices_healthy() const;
    ChassisMode choose_mode() const;

    Config config_ {};
    Command command_ {};
    Feedback feedback_ {};
    Output output_ {};
    ChassisKinematics kinematics_ {};
    ChassisSolver solver_ {};
    BodyVelocityController body_controller_ {};
    std::array<WheelController, kWheelCount> wheel_controllers_ {};
    PowerControl power_control_ {};
    float previous_follow_error_ = 0.0f;
};

} // namespace app::chassis
