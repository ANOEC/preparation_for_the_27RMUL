#pragma once

/// @file gimbal_params.hpp
/// @brief Auto-generated parameters from gimbal.toml.
/// @warning DO NOT EDIT. Modify gimbal.toml and regenerate instead.

namespace param::gimbal {

    namespace dual_pid {

        struct big_yaw_t {
            float P_error_threshold = 0.0f;
            float P_ff = 0.0f;
            float P_integral_limit = 0.0f;
            float P_kd = 0.0f;
            float P_ki = 0.0f;
            float P_kp = 10.0f;
            float P_max_output = 16384.0f;
            float V_error_threshold = 0.0f;
            float V_integral_limit = 0.0f;
            float V_kd = 0.0f;
            float V_ki = 0.0f;
            float V_kp = -2.5f;
            float V_max_output = 9.68625f;
        };
        inline big_yaw_t big_yaw {};

        struct pitch_t {
            float P_error_threshold = 0.0f;
            float P_ff = 0.0f;
            float P_integral_limit = 0.0f;
            float P_kd = 0.0f;
            float P_ki = 0.0f;
            float P_kp = 0.9f;
            float P_max_output = 16384.0f;
            float V_error_threshold = 0.0f;
            float V_integral_limit = 20.0f;
            float V_kd = 0.0f;
            float V_ki = 0.008f;
            float V_kp = 1.5f;
            float V_max_output = 9.68625f;
        };
        inline pitch_t pitch {};

        struct yaw_t {
            float P_error_threshold = 0.0f;
            float P_ff = 0.0f;
            float P_integral_limit = 0.0f;
            float P_kd = 0.0f;
            float P_ki = 0.0f;
            float P_kp = 16.0f;
            float P_max_output = 16384.0f;
            float V_error_threshold = 0.0f;
            float V_integral_limit = 0.0f;
            float V_kd = 0.0f;
            float V_ki = 0.0f;
            float V_kp = 1.08544921875f;
            float V_max_output = 2.223f;
        };
        inline yaw_t yaw {};

    } // namespace dual_pid

    struct gravity_test_t {
        bool enabled = true;
        int telemetry_period_ms = 10;
    };
    inline gravity_test_t gravity_test {};

    struct imu_t {
        static constexpr float constant_temperature = 50.0f;
        static constexpr bool fix_by_encoder_flag = true;
        static constexpr float fix_by_encoder_time = 1.0f;
        static constexpr float horizontal_shift = 90.0f;
    };
    inline constexpr imu_t imu {};

    namespace leso {

        struct big_yaw_t {
            float b0 = 40.0f;
            float omega = 60.0f;
        };
        inline big_yaw_t big_yaw {};

        struct pitch_t {
            float b0 = 70.0f;
            float omega = 50.0f;
        };
        inline pitch_t pitch {};

        struct yaw_t {
            float b0 = 49.65f;
            float omega = 100.0f;
        };
        inline yaw_t yaw {};

    } // namespace leso

    struct param_t {
        float big_yaw_J_min = 0.078f;
        float big_yaw_J_w1 = -0.0643486f;
        float big_yaw_J_w2 = 0.0366733f;
        float big_yaw_J_w3 = 0.1073112f;
        float big_yaw_avg_J = 0.03f;
        float big_yaw_chassis_compensation = 0.22f;
        float big_yaw_feedforward = 0.0f;
        float chassis_lbg_bandwidth = 23.0f;
        float forward_compensation_big_yaw = 0.0f;
        float forward_compensation_yaw = 0.0f;
        float gimbal_lbg_bandwidth = 30.0f;
        float gravity_w1 = 0.6243f;
        float gravity_w2 = 0.0f;
        float gravity_w3 = -0.6212f;
        float pitch_J = 0.0084f;
        float pitch_limit_lpf_alpha = 0.2f;
        float pitch_max_vel = 4.2f;
        float yaw_J_min = 0.018f;
        float yaw_J_w1 = 0.6460679f;
        float yaw_J_w2 = -1.1799474f;
        float yaw_J_w3 = 0.5565527f;
        float yaw_avg_J = 0.012f;
        float yaw_feedforward = 0.0f;
        float yaw_max_vel = 13.0f;
        float yaw_ramp_rate = 0.025f;
    };
    inline param_t param {};

    namespace patrol {

        struct big_yaw_t {
            float add_angle = 0.0055f;
        };
        inline big_yaw_t big_yaw {};

        struct pitch_t {
            float amplitude = 15.0f;
            float offset = 10.0f;
            float period = 1.0f;
        };
        inline pitch_t pitch {};

        struct upper_pitch_t {
            float amplitude = 7.5f;
            float offset = -22.5f;
            float period = 1.0f;
        };
        inline upper_pitch_t upper_pitch {};

        struct yaw_t {
            float amplitude = 45.0f;
            float offset = 0.0f;
            float period = 2.0f;
        };
        inline yaw_t yaw {};

    } // namespace patrol

    struct torque_test_t {
        bool enabled = false;
        int input_timeout_ms = 0;
        float limit_margin_deg = 2.0f;
        float max_abs_torque_nm = 9.0f;
        int telemetry_period_ms = 10;
        float torque_slew_rate_nm_per_s = 3.0f;
    };
    inline torque_test_t torque_test {};

    struct velocity_test_t {
        float amplitude_rad_per_sec = 1.5f;
        int axis = 2;
        bool enabled = false;
        float frequency_hz = 0.5f;
        float offset_rad_per_sec = 0.0f;
        int telemetry_period_ms = 10;
    };
    inline velocity_test_t velocity_test {};

} // namespace param::gimbal
