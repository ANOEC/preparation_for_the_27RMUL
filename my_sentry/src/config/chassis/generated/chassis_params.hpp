#pragma once

/// @file chassis_params.hpp
/// @brief Auto-generated parameters from chassis.toml.
/// @warning DO NOT EDIT. Modify chassis.toml and regenerate instead.

namespace param::chassis {

    struct imu_t {
        static constexpr float constant_temperature = 50.0f;
        static constexpr float horizontal_shift = 0.0f;
        static constexpr float offset_x = 0.0f;
        static constexpr float offset_y = 0.1855f;
        static constexpr float w01 = 50.0f;
        static constexpr float w02 = 50.0f;
        static constexpr float w03 = 50.0f;
    };
    inline constexpr imu_t imu {};

    struct param_t {
        static constexpr float brake_x = 2.0f;
        static constexpr float brake_y = 2.0f;
        static constexpr float car_height = 0.0656f;
        static constexpr float car_length = 0.37335238f;
        static constexpr float car_r = 0.264f;
        static constexpr float car_weight = 23.0f;
        static constexpr float car_wide = 0.37335238f;
        static constexpr float chassis_sin_period1 = 0.5f;
        static constexpr float chassis_sin_period2 = 0.3f;
        static constexpr float chassis_sin_period3 = 0.13f;
        static constexpr float length_bias = 0.0f;
        static constexpr float omni_wheel_r = 0.07697f;
        static constexpr float quick_brake_acceleration = 105.0f;
        static constexpr float quick_brake_command_delta = 0.1f;
        static constexpr float quick_brake_release_speed = 0.1f;
        static constexpr float slope_angle = 8.0f;
        static constexpr float sprint_acceleration = 21.0f;
        static constexpr float sprint_activation_speed = 1.5f;
        static constexpr float steering_k1 = 0.321948f;
        static constexpr float steering_k2 = 10.657181f;
        static constexpr float steering_k3 = 1.732831f;
        static constexpr float steering_wheel_r = 0.061f;
        static constexpr float stop_spin_angle = 12.0f;
        static constexpr float translation_acceleration = 10.5f;
        static constexpr float turn_over_angle = 45.0f;
        static constexpr float turn_over_free_angle = 20.0f;
        static constexpr float width_bias = 0.0f;
        static constexpr float yaw_acceleration = 700.0f;
    };
    inline constexpr param_t param {};

    namespace pid {

        struct a_t {
            float error_threshold = 0.0f;
            float integral_limit = 0.0f;
            float kd = 0.0f;
            float ki = 0.0f;
            float kp = 2.5f;
            float max_output = 10.0f;
        };
        inline a_t a {};

        struct follow_t {
            float error_threshold = 0.0f;
            float integral_limit = 0.0f;
            float kd = -4.0f;
            float ki = 0.0f;
            float kp = -10.0f;
            float max_output = 13.0f;
            float velocity_feedforward = 1.2f;
        };
        inline follow_t follow {};

        struct motor1_t {
            float error_threshold = 0.0f;
            float integral_limit = 0.0f;
            float kd = 0.0f;
            float ki = 0.0f;
            float kp = 0.1f;
            float max_output = 6.0f;
        };
        inline motor1_t motor1 {};

        struct motor2_t {
            float error_threshold = 0.0f;
            float integral_limit = 0.0f;
            float kd = 0.0f;
            float ki = 0.0f;
            float kp = 0.1f;
            float max_output = 6.0f;
        };
        inline motor2_t motor2 {};

        struct motor3_t {
            float error_threshold = 0.0f;
            float integral_limit = 0.0f;
            float kd = 0.0f;
            float ki = 0.0f;
            float kp = 0.1f;
            float max_output = 6.0f;
        };
        inline motor3_t motor3 {};

        struct motor4_t {
            float error_threshold = 0.0f;
            float integral_limit = 0.0f;
            float kd = 0.0f;
            float ki = 0.0f;
            float kp = 0.1f;
            float max_output = 6.0f;
        };
        inline motor4_t motor4 {};

        struct x_t {
            float error_threshold = 0.0f;
            float integral_limit = 0.0f;
            float kd = 0.0f;
            float ki = 0.0f;
            float kp = 2.5f;
            float max_output = 10.0f;
        };
        inline x_t x {};

        struct y_t {
            float error_threshold = 0.0f;
            float integral_limit = 0.0f;
            float kd = 0.0f;
            float ki = 0.0f;
            float kp = 2.5f;
            float max_output = 10.0f;
        };
        inline y_t y {};

        struct z_t {
            float error_threshold = 0.0f;
            float integral_limit = 0.0f;
            float kd = 0.0f;
            float ki = 0.0f;
            float kp = 2.5f;
            float max_output = 10.0f;
        };
        inline z_t z {};

    } // namespace pid

} // namespace param::chassis
