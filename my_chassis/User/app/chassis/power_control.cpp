#include "power_control.hpp"

#include <algorithm>
#include <cmath>

namespace app::chassis {
namespace {
constexpr float kEpsilon = 1.0e-6f;
}

PowerControl::PowerControl(const Config& config) { set_config(config); reset(); }

void PowerControl::set_config(const Config& config)
{
    config_ = config;
    config_.k1 = std::max(config_.k1, config_.minimum_model_coefficient);
    config_.k2 = std::max(config_.k2, config_.minimum_model_coefficient);
    config_.k3 = std::max(config_.k3, 0.0f);
    config_.rls_forgetting_factor = std::clamp(config_.rls_forgetting_factor, 0.9f, 1.0f);
}

void PowerControl::reset()
{
    status_ = {};
    status_.k1 = config_.k1;
    status_.k2 = config_.k2;
    covariance_[0][0] = covariance_[1][1] = config_.rls_initial_covariance;
    covariance_[0][1] = covariance_[1][0] = 0.0f;
}

float PowerControl::predict_power(const WheelVector& torque,
                                  const WheelVector& speed) const
{
    float power = config_.k3;
    for (std::size_t i = 0; i < kWheelCount; ++i) {
        power += torque[i] * speed[i] + config_.k1 * std::abs(speed[i])
               + config_.k2 * torque[i] * torque[i];
    }
    return power;
}

float PowerControl::compute_budget(const Input& input) const
{
    const float referee_limit = input.referee_online
        ? std::max(input.referee_power_limit, 0.0f)
        : config_.referee_offline_power;
    const float buffer = input.capacitor_online ? input.capacitor_energy
                                                : input.referee_buffer_energy;
    const float low = input.capacitor_online ? config_.cap_buffer_low
                                             : config_.referee_buffer_low;
    const float high = input.capacitor_online ? config_.cap_buffer_high
                                              : config_.referee_buffer_high;
    const float minimum = referee_limit * config_.minimum_budget_ratio;
    const float base = std::max(referee_limit -
        std::max(low - buffer, 0.0f) * config_.low_buffer_gain, minimum);
    if (!input.capacitor_online || !input.request_boost) return base;
    const float energy_fraction = std::clamp((buffer - low) /
                                             std::max(high - low, kEpsilon), 0.0f, 1.0f);
    return base + config_.cap_extra_power * energy_fraction;
}

void PowerControl::update_rls(const Input& input, float measured_power)
{
    if (!config_.rls_enabled || measured_power <= 5.0f) return;
    float mechanical = 0.0f;
    float sample[2] {};
    for (std::size_t i = 0; i < kWheelCount; ++i) {
        mechanical += input.measured_torque[i] * input.wheel_speed[i];
        sample[0] += std::abs(input.wheel_speed[i]);
        sample[1] += input.measured_torque[i] * input.measured_torque[i];
    }
    const float target = measured_power - mechanical - config_.k3;
    if (target <= 0.0f) return;

    float p_h[2] {
        covariance_[0][0] * sample[0] + covariance_[0][1] * sample[1],
        covariance_[1][0] * sample[0] + covariance_[1][1] * sample[1],
    };
    const float denominator = config_.rls_forgetting_factor
        + sample[0] * p_h[0] + sample[1] * p_h[1];
    if (denominator <= kEpsilon) return;
    const float gain[2] {p_h[0] / denominator, p_h[1] / denominator};
    const float predicted = config_.k1 * sample[0] + config_.k2 * sample[1];
    const float error = target - predicted;
    config_.k1 = std::max(config_.k1 + gain[0] * error,
                          config_.minimum_model_coefficient);
    config_.k2 = std::max(config_.k2 + gain[1] * error,
                          config_.minimum_model_coefficient);

    float new_p[2][2] {};
    for (int row = 0; row < 2; ++row) {
        for (int col = 0; col < 2; ++col) {
            new_p[row][col] = (covariance_[row][col] -
                gain[row] * (sample[0] * covariance_[0][col] +
                             sample[1] * covariance_[1][col])) /
                config_.rls_forgetting_factor;
        }
    }
    for (int row = 0; row < 2; ++row)
        for (int col = 0; col < 2; ++col) covariance_[row][col] = new_p[row][col];
}

PowerControl::WheelVector PowerControl::update(const Input& input)
{
    status_.degraded = !input.referee_online || !input.capacitor_online;
    status_.budget = std::max(compute_budget(input) - std::max(input.auxiliary_power, 0.0f), 0.0f);
    status_.requested_power = predict_power(input.requested_torque, input.wheel_speed);
    status_.mechanical_power = 0.0f;
    for (std::size_t i = 0; i < kWheelCount; ++i)
        status_.mechanical_power += input.measured_torque[i] * input.wheel_speed[i];
    status_.measured_power = input.capacitor_online
        ? std::max(input.capacitor_power - input.auxiliary_power, 0.0f)
        : predict_power(input.measured_torque, input.wheel_speed);
    status_.estimated_loss = status_.measured_power - status_.mechanical_power;
    status_.efficiency = status_.measured_power > kEpsilon
        ? std::clamp(status_.mechanical_power / status_.measured_power, 0.0f, 1.0f)
        : 0.0f;
    status_.estimated_capacitor_energy_joules = input.capacitor_online
        ? std::clamp(input.capacitor_energy / std::max(config_.cap_energy_scale, kEpsilon), 0.0f, 1.0f)
            * config_.cap_energy_joules
        : 0.0f;

    update_rls(input, status_.measured_power);
    status_.k1 = config_.k1;
    status_.k2 = config_.k2;

    WheelVector result = input.requested_torque;
    for (std::size_t i = 0; i < kWheelCount; ++i) {
        const float limit = std::abs(input.maximum_torque[i]);
        if (limit > 0.0f) result[i] = std::clamp(result[i], -limit, limit);
    }

    status_.limiting = predict_power(result, input.wheel_speed) > status_.budget;
    if (status_.limiting) {
        // Monotonic bisection keeps regenerative commands and distributes the
        // remaining budget proportionally without an unstable quadratic root.
        float low = 0.0f;
        float high = 1.0f;
        for (int iteration = 0; iteration < 24; ++iteration) {
            const float scale = (low + high) * 0.5f;
            WheelVector candidate = result;
            for (std::size_t i = 0; i < kWheelCount; ++i) {
                if (candidate[i] * input.wheel_speed[i] >= 0.0f) candidate[i] *= scale;
            }
            if (predict_power(candidate, input.wheel_speed) <= status_.budget) low = scale;
            else high = scale;
        }
        for (std::size_t i = 0; i < kWheelCount; ++i) {
            if (result[i] * input.wheel_speed[i] >= 0.0f) result[i] *= low;
        }
    }
    status_.limited_power = predict_power(result, input.wheel_speed);
    for (float& torque : result) if (!std::isfinite(torque)) torque = 0.0f;
    return result;
}

} // namespace app::chassis
