#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace UserApp::Host
{

// These payload layouts intentionally match steering_sentry's raw C++ ABI.
// Do not pack these structs: the companion computer protocol includes the
// natural ARM/GCC padding in CRC and packet length.
struct ChassisCommand
{
  float linear_x = 0.0f;
  float linear_y = 0.0f;
  float angular_z = 0.0f;
};

struct BigBuffCommand
{
  std::uint8_t shoot_count = 0;
};

struct GimbalCommand
{
  std::uint8_t mode = 0;  // 0: no control, 1: auto-aim, 2: patrol
  std::uint8_t fire = 0;  // 0: no fire, 1: normal auto-aim, 2: buff
  float yaw = 0.0f;
  float yaw_vel = 0.0f;
  float yaw_acc = 0.0f;
  float pitch = 0.0f;
  float pitch_vel = 0.0f;
  float pitch_acc = 0.0f;
};

struct BigBuffState
{
  std::uint32_t already_shoot_count = 0;
};

struct GimbalState
{
  std::uint8_t enemy_color = 'B';
  float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // w, x, y, z
  float yaw = 0.0f;
  float yaw_vel = 0.0f;
  float pitch = 0.0f;
  float pitch_vel = 0.0f;
  float big_yaw_relative_to_yaw_angle = 0.0f;
  float bullet_speed = 0.0f;
  std::uint16_t bullet_count = 0;
};

inline constexpr std::uint8_t kSendHeader = 0xAA;
inline constexpr std::uint8_t kRecvHeader = 0x55;

struct VisionInfo
{
  std::uint8_t header = kSendHeader;
  BigBuffState buff_state{};
  GimbalState gimbal_state{};
  std::uint16_t legacy_crc_member = 0;
};

struct NavCommandPacket
{
  std::uint8_t header = kRecvHeader;
  ChassisCommand chassis{};
  std::uint64_t nav_mode = 0;  // bit0 pitch-up patrol, bit1 buff, bit2-3 posture
  std::uint16_t legacy_crc_member = 0;
};

struct VisionCommandPacket
{
  std::uint8_t header = kRecvHeader;
  BigBuffCommand buff{};
  GimbalCommand gimbal{};
  std::uint16_t legacy_crc_member = 0;
};

static_assert(std::is_trivially_copyable_v<ChassisCommand>);
static_assert(std::is_trivially_copyable_v<BigBuffCommand>);
static_assert(std::is_trivially_copyable_v<GimbalCommand>);
static_assert(std::is_trivially_copyable_v<BigBuffState>);
static_assert(std::is_trivially_copyable_v<GimbalState>);
static_assert(sizeof(ChassisCommand) == 12);
static_assert(sizeof(BigBuffCommand) == 1);
static_assert(sizeof(GimbalCommand) == 28);
static_assert(sizeof(BigBuffState) == 4);
static_assert(sizeof(GimbalState) == 48);
static_assert(sizeof(VisionInfo) == 60);
static_assert(sizeof(NavCommandPacket) == 32);
static_assert(sizeof(VisionCommandPacket) == 36);

}  // namespace UserApp::Host
