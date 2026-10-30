#pragma once

#include <cmath>
#include <numbers>

#include "chassis_kinematics.hpp"

namespace app::chassis
{
inline float ReferenceFollowYawError(float big_yaw_angle_rad)
{
  return std::remainder(-big_yaw_angle_rad,
                        2.0f * std::numbers::pi_v<float>);
}

inline float ReferenceTranslationDeviation(float big_yaw_angle_rad,
                                           float small_yaw_angle_rad,
                                           bool game_mode)
{
  const float big_yaw_offset = -big_yaw_angle_rad;
  return game_mode ? big_yaw_offset
                   : big_yaw_offset + small_yaw_angle_rad;
}

inline ChassisVelocity ReferenceTranslationToBody(float vx, float vy,
                                                  float deviation_rad)
{
  const float cosine = std::cos(deviation_rad);
  const float sine = std::sin(deviation_rad);
  return {
      cosine * vx + sine * vy,
      -sine * vx + cosine * vy,
      0.0f,
  };
}
}
