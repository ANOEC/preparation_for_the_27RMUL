#pragma once

#include <array>
#include <cstdint>

namespace WireBenchmark
{
enum class ChassisMode : uint8_t
{
  FREE = 0,
  FOLLOW,
  SPIN
};

enum class AppState : uint8_t
{
  OFFLINE = 0,
  PROTECT,
  HAND,
  GAME
};

struct GimbalToChassis
{
  bool referee_alive = false;
  bool big_yaw_alive = false;
  bool yaw_alive = false;

  std::array<float, 3> target_vel;

  ChassisMode mode = ChassisMode::FREE;
  AppState state = AppState::OFFLINE;

  uint8_t game_progress;
  uint16_t stage_remain_time;
  uint16_t current_hp;
  uint16_t chassis_power_limit;
  uint16_t buffer_energy;

  float big_yaw_angle;
  float yaw_angle;

};

struct ChassisToGimbal
{
  bool imu_alive = false;

  std::array<float, 3> imu_acc;
  std::array<float, 3> imu_gyro;
  std::array<float, 3> imu_euler;

};
}  // namespace WireBenchmark

// Shared measurement types keep the noinline entry point ABI identical across TUs.
struct Classic { std::uint32_t sequence{}, flags{}; };
struct Vqf { std::array<float, 10> fields{}; };
