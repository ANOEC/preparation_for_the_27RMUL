#pragma once

#include <cstdint>

namespace UserApp::Command
{

enum class ControlState : std::uint8_t
{
  OFFLINE = 0,
  PROTECT = 1,
  HAND = 2,
  GAME = 3,
};

enum class ChassisMode : std::uint8_t
{
  PATROL = 0,
  FOLLOW = 1,
  SPIN = 2,
};

struct ChassisCommand
{
  float vx_mps = 0.0F;
  float vy_mps = 0.0F;
  float wz_rad_per_sec = 0.0F;
  ControlState state = ControlState::OFFLINE;
  ChassisMode mode = ChassisMode::PATROL;
  bool enabled = false;
  bool big_yaw_alive = false;
  bool yaw_alive = false;
  float big_yaw_angle_rad = 0.0F;
  float big_yaw_velocity_rad_per_sec = 0.0F;
  float yaw_angle_rad = 0.0F;
};

struct ChassisFeedback
{
  bool imu_alive = false;
  float imu_acceleration_mps2[3]{};
  float imu_gyro_rad_per_sec[3]{};
  float imu_euler_rad[3]{};
};

inline ChassisCommand MakeSafeChassisCommand(ControlState state) noexcept
{
  ChassisCommand command{};
  command.state = state;
  return command;
}

}  // namespace UserApp::Command
