#pragma once

/// @file gimbal_params.hpp
/// @brief Auto-generated parameters from gimbal.toml.
/// @warning DO NOT EDIT. Modify gimbal.toml and regenerate instead.

namespace param::shoot {

    struct basic_param_t {
        int amount_pellet_per_round = 9;
        float fri_basic_speed = 630.0f;
        float fri_count_decline_vel = 605.0f;
        float fri_count_rise_vel = 615.0f;
        float fri_left_J = 2.44e-05f;
        float fri_ramp_rate = 3.0f;
        float fri_right_J = 2.45e-05f;
        float pluck_basic_speed = 14.0f;
        float pluck_overload_check_time = 0.35f;
        float pluck_overload_effort = 0.38f;
        float pluck_reverse_angle = 15.0f;
        float pluck_reverse_speed = -0.35f;
        float pluck_reverse_time = 1.0f;
        int shoot_heat_per = 10;
    };
    inline basic_param_t basic_param {};

    namespace dual_pid {

        struct pluck_t {
            float P_error_threshold = 0.0f;
            float P_integral_limit = 0.0f;
            float P_kd = 0.0f;
            float P_ki = 0.0f;
            float P_kp = 14.0f;
            float P_max_output = 20.0f;
            float V_error_threshold = 0.0f;
            float V_integral_limit = 10.0f;
            float V_kd = 0.0f;
            float V_ki = 0.01f;
            float V_ki_buff = 0.0f;
            float V_kp = 0.062f;
            float V_max_output = 1.8f;
        };
        inline pluck_t pluck {};

    } // namespace dual_pid

    namespace ladrc {

        struct pluck_t {
            float b0 = 98.0f;
            float huge_resistance_threshold = 1000000.0f;
            float learn_rate = 0.2f;
            float output_limit = 1.8f;
            float r = 0.0f;
            float ts = 0.01f;
            float w0 = 120.0f;
            float wc = 3.0f;
        };
        inline pluck_t pluck {};

    } // namespace ladrc

    namespace leso {

        struct pluck_t {
            float b0 = 98.0f;
            float omega = 120.0f;
        };
        inline pluck_t pluck {};

    } // namespace leso

    namespace pid {

        struct fri_left_t {
            float error_threshold = 0.0f;
            float integral_limit = 0.0f;
            float kd = 0.015f;
            float ki = 0.0f;
            float kp = 0.001f;
            float max_output = 0.3157894736842f;
        };
        inline fri_left_t fri_left {};

        struct fri_right_t {
            float error_threshold = 0.0f;
            float integral_limit = 0.0f;
            float kd = 0.015f;
            float ki = 0.0f;
            float kp = 0.001f;
            float max_output = 0.3157894736842f;
        };
        inline fri_right_t fri_right {};

    } // namespace pid

    namespace shoot_smc {

        struct fri_left_t {
            float J_ = 2.44e-05f;
            float c_ = 42.0f;
            float k1_ = 10.0f;
            float k2_ = 0.0f;
            float output_limit_ = 0.3157894736842f;
            float r_ = 150.0f;
            bool use_leso_ff = false;
            bool use_ref_ff = true;
            float w0 = 200.0f;
        };
        inline fri_left_t fri_left {};

        struct fri_right_t {
            float J_ = 2.45e-05f;
            float c_ = 42.0f;
            float k1_ = 10.0f;
            float k2_ = 0.0f;
            float output_limit_ = 0.3157894736842f;
            float r_ = 150.0f;
            bool use_leso_ff = false;
            bool use_ref_ff = true;
            float w0 = 200.0f;
        };
        inline fri_right_t fri_right {};

    } // namespace shoot_smc

} // namespace param::shoot
