#pragma once

#include <array>

namespace app::chassis {

struct ChassisVelocity {
    float vx = 0.0f;
    float vy = 0.0f;
    float wz = 0.0f;
};

/** Pure geometric model of the four 45-degree omni wheels used by the source
 * sentry.  Wheel order is LF, LR, RR, RF, matching the source project's H
 * matrix.  Linear units are metres and seconds; angles are radians. */
class ChassisKinematics {
public:
    static constexpr std::size_t kWheelCount = 4;
    using WheelVector = std::array<float, kWheelCount>;

    struct Config {
        float wheel_radius = 0.07697f;
        float wheelbase = 0.37335238f;
        float track_width = 0.37335238f;
        // Effective distance from chassis centre to a wheel contact direction.
        // A value <= 0 selects hypot(wheelbase/2, track_width/2).
        float rotation_radius = 0.264f;
        float max_wheel_speed = 0.0f; // rad/s; zero disables normalization.
    };

    ChassisKinematics() = default;
    explicit ChassisKinematics(const Config& config);

    void set_config(const Config& config);
    const Config& config() const { return config_; }

    WheelVector inverse(const ChassisVelocity& body_velocity) const;
    ChassisVelocity forward(const WheelVector& wheel_speed) const;

    static ChassisVelocity world_to_body(const ChassisVelocity& world, float yaw);
    static ChassisVelocity body_to_world(const ChassisVelocity& body, float yaw);

    // Preserves the wheel-speed ratios and returns the applied scale.
    static float normalize(WheelVector& wheel_speed, float max_abs_speed);

private:
    Config config_ {};
};

} // namespace app::chassis
