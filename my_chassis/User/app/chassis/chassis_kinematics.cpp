#include "chassis_kinematics.hpp"

#include <algorithm>
#include <cmath>

namespace app::chassis {
namespace {
constexpr float kInvSqrt2 = 0.7071067811865475f;
constexpr float kEpsilon = 1.0e-6f;
}

ChassisKinematics::ChassisKinematics(const Config& config) { set_config(config); }

void ChassisKinematics::set_config(const Config& config)
{
    config_ = config;
    config_.wheel_radius = std::max(std::abs(config_.wheel_radius), kEpsilon);
    if (config_.rotation_radius <= kEpsilon) {
        config_.rotation_radius = std::hypot(config_.wheelbase * 0.5f,
                                             config_.track_width * 0.5f);
    }
}

ChassisKinematics::WheelVector ChassisKinematics::inverse(
    const ChassisVelocity& velocity) const
{
    const float translation = kInvSqrt2 / config_.wheel_radius;
    const float rotation = config_.rotation_radius / config_.wheel_radius;
    WheelVector result {
        translation * (-velocity.vx + velocity.vy) + rotation * velocity.wz,
        translation * (-velocity.vx - velocity.vy) + rotation * velocity.wz,
        translation * ( velocity.vx - velocity.vy) + rotation * velocity.wz,
        translation * ( velocity.vx + velocity.vy) + rotation * velocity.wz,
    };
    normalize(result, config_.max_wheel_speed);
    return result;
}

ChassisVelocity ChassisKinematics::forward(const WheelVector& wheel) const
{
    const float linear = config_.wheel_radius * kInvSqrt2 * 0.5f;
    const float angular = config_.wheel_radius /
                          (4.0f * std::max(config_.rotation_radius, kEpsilon));
    return {
        linear * (-wheel[0] - wheel[1] + wheel[2] + wheel[3]),
        linear * ( wheel[0] - wheel[1] - wheel[2] + wheel[3]),
        angular * (wheel[0] + wheel[1] + wheel[2] + wheel[3]),
    };
}

ChassisVelocity ChassisKinematics::world_to_body(const ChassisVelocity& world,
                                                  float yaw)
{
    const float c = std::cos(yaw);
    const float s = std::sin(yaw);
    return {c * world.vx + s * world.vy,
            -s * world.vx + c * world.vy,
            world.wz};
}

ChassisVelocity ChassisKinematics::body_to_world(const ChassisVelocity& body,
                                                  float yaw)
{
    const float c = std::cos(yaw);
    const float s = std::sin(yaw);
    return {c * body.vx - s * body.vy,
            s * body.vx + c * body.vy,
            body.wz};
}

float ChassisKinematics::normalize(WheelVector& speed, float limit)
{
    if (limit <= 0.0f) {
        return 1.0f;
    }
    float maximum = 0.0f;
    for (const float value : speed) {
        maximum = std::max(maximum, std::abs(value));
    }
    if (maximum <= limit || maximum <= kEpsilon) {
        return 1.0f;
    }
    const float scale = limit / maximum;
    for (float& value : speed) {
        value *= scale;
    }
    return scale;
}

} // namespace app::chassis
