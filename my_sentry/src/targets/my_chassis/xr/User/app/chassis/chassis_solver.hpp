#pragma once

#include <array>

#include "chassis_kinematics.hpp"

namespace app::chassis {

/** Feedback-dependent estimation and force allocation.  It deliberately owns
 * no transport, motor or RTOS object. */
class ChassisSolver {
public:
    using WheelVector = ChassisKinematics::WheelVector;

    struct Config {
        ChassisKinematics::Config kinematics {};
        float imu_offset_x = 0.0f;
        float imu_offset_y = 0.1855f;
        float imu_yaw = 0.0f; // radians, IMU frame to chassis frame.
        float gyro_observer_frequency = 50.0f;
        float velocity_process_noise = 0.05f;
        float wheel_measurement_noise = 0.15f;
        float minimum_wheel_confidence = 0.05f;
        float residual_reference_speed = 5.0f;
        float residual_deadband = 0.05f;
        float residual_threshold = 0.25f;
        float slip_sensitivity = 1.5f;
        float lock_speed_threshold = 0.01f;
        float lock_target_threshold = 0.01f;
        float lock_delay_seconds = 1.0f;
        float vehicle_mass = 23.0f;
        float slope_activation_degrees = 8.0f;
        bool automatic_gravity_compensation = true;
    };

    struct ImuFeedback {
        std::array<float, 3> acceleration {};
        std::array<float, 3> gyro {};
        bool online = false;
    };

    struct Feedback {
        WheelVector wheel_speed {};
        WheelVector wheel_torque {};
        std::array<bool, 4> wheel_online {false, false, false, false};
        ImuFeedback imu {};
        float yaw_to_world = 0.0f;
        float roll = 0.0f;
        float pitch = 0.0f;
    };

    struct Estimate {
        ChassisVelocity body_velocity {};
        ChassisVelocity world_velocity {};
        ChassisVelocity body_acceleration {};
        std::array<float, 4> wheel_confidence {1.0f, 1.0f, 1.0f, 1.0f};
        bool slipping = false;
        bool locked = false;
    };

    ChassisSolver() = default;
    explicit ChassisSolver(const Config& config);

    void set_config(const Config& config);
    const Config& config() const { return config_; }
    void reset();
    const Estimate& update(const Feedback& feedback, float dt);
    const Estimate& estimate() const { return estimate_; }

    // target is Fx, Fy, Mz in the chassis frame. Return values are wheel
    // torque feedforward in LF, LR, RR, RF order.
    WheelVector allocate_torque(const ChassisVelocity& target,
                                const std::array<float, 4>& gravity_weight =
                                    {1.0f, 1.0f, 1.0f, 1.0f}) const;

    void set_gravity_compensation(float force_x, float force_y);
    void update_lock_state(const ChassisVelocity& target, float dt);

private:
    Config config_ {};
    ChassisKinematics kinematics_ {};
    Estimate estimate_ {};
    float smooth_omega_ = 0.0f;
    float smooth_omega_acceleration_ = 0.0f;
    float lock_timer_ = 0.0f;
    float gravity_force_x_ = 0.0f;
    float gravity_force_y_ = 0.0f;
};

} // namespace app::chassis
