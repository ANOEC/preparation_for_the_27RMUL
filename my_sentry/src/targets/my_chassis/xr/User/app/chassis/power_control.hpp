#pragma once

#include <array>
#include <cstdint>

namespace app::chassis {

class PowerControl {
public:
    static constexpr std::size_t kWheelCount = 4;
    using WheelVector = std::array<float, kWheelCount>;

    struct Config {
        float k1 = 0.22f;
        float k2 = 1.20f;
        float k3 = 3.40f;
        float minimum_model_coefficient = 1.0e-5f;
        float rls_forgetting_factor = 0.999f;
        float rls_initial_covariance = 100.0f;
        bool rls_enabled = true;

        float referee_offline_power = 35.0f;
        float cap_extra_power = 300.0f;
        float referee_buffer_low = 50.0f;
        float referee_buffer_high = 60.0f;
        float cap_buffer_low = 30.0f;
        float cap_buffer_high = 230.0f;
        float minimum_budget_ratio = 0.85f;
        float low_buffer_gain = 2.0f; // W reduction per buffer unit below low.
        float cap_energy_scale = 250.0f;
        float cap_energy_joules = 2100.0f;
    };

    struct Input {
        WheelVector wheel_speed {};
        WheelVector requested_torque {};
        WheelVector measured_torque {};
        WheelVector maximum_torque {6.0f, 6.0f, 6.0f, 6.0f};
        WheelVector target_speed {};
        bool referee_online = false;
        float referee_power_limit = 0.0f;
        float referee_buffer_energy = 0.0f;
        bool capacitor_online = false;
        float capacitor_power = 0.0f;
        float capacitor_energy = 0.0f; // source protocol's 0..250 scale.
        float auxiliary_power = 0.0f;
        bool request_boost = false;
    };

    struct Status {
        float budget = 0.0f;
        float requested_power = 0.0f;
        float limited_power = 0.0f;
        float measured_power = 0.0f;
        float mechanical_power = 0.0f;
        float estimated_loss = 0.0f;
        float efficiency = 0.0f;
        float estimated_capacitor_energy_joules = 0.0f;
        float k1 = 0.0f;
        float k2 = 0.0f;
        bool limiting = false;
        bool degraded = false;
    };

    PowerControl() = default;
    explicit PowerControl(const Config& config);

    void set_config(const Config& config);
    const Config& config() const { return config_; }
    void reset();
    WheelVector update(const Input& input);
    const Status& status() const { return status_; }
    float predict_power(const WheelVector& torque, const WheelVector& speed) const;

private:
    float compute_budget(const Input& input) const;
    void update_rls(const Input& input, float measured_power);

    Config config_ {};
    Status status_ {};
    float covariance_[2][2] {{100.0f, 0.0f}, {0.0f, 100.0f}};
};

} // namespace app::chassis
