#pragma once

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>

#include "app/communication/b2b/chassis_command.hpp"

namespace UserApp::BoardProtocol
{

constexpr std::uint32_t kGimbalToChassisId = 0x099U;
constexpr std::uint32_t kChassisToGimbalId = 0x09AU;
constexpr std::size_t kFrameSize = 64U;
constexpr std::size_t kLegacyHostToSlavePayloadSize = 44U;
constexpr std::size_t kLegacySlaveToHostPayloadSize = 40U;

using Frame = std::array<std::byte, kFrameSize>;

namespace detail
{
inline void WriteU32(Frame& frame, std::size_t offset, std::uint32_t value) noexcept
{
  frame[offset] = static_cast<std::byte>(value & 0xFFU);
  frame[offset + 1U] = static_cast<std::byte>((value >> 8U) & 0xFFU);
  frame[offset + 2U] = static_cast<std::byte>((value >> 16U) & 0xFFU);
  frame[offset + 3U] = static_cast<std::byte>((value >> 24U) & 0xFFU);
}

inline std::uint32_t ReadU32(std::span<const std::byte> frame,
                             std::size_t offset) noexcept
{
  return std::to_integer<std::uint32_t>(frame[offset]) |
         (std::to_integer<std::uint32_t>(frame[offset + 1U]) << 8U) |
         (std::to_integer<std::uint32_t>(frame[offset + 2U]) << 16U) |
         (std::to_integer<std::uint32_t>(frame[offset + 3U]) << 24U);
}

inline void WriteFloat(Frame& frame, std::size_t offset, float value) noexcept
{
  WriteU32(frame, offset, std::bit_cast<std::uint32_t>(value));
}

inline float ReadFloat(std::span<const std::byte> frame, std::size_t offset) noexcept
{
  return std::bit_cast<float>(ReadU32(frame, offset));
}
}  // namespace detail

/** Encode the reference-project HostToSlave ABI explicitly, without relying
 * on compiler struct padding. Referee fields that are not implemented stay zero. */
inline Frame EncodeChassisCommand(const Command::ChassisCommand& command) noexcept
{
  static_assert(sizeof(float) == sizeof(std::uint32_t));
  static_assert(std::numeric_limits<float>::is_iec559);

  Frame frame{};
  frame[1U] = static_cast<std::byte>(command.big_yaw_alive);
  frame[2U] = static_cast<std::byte>(command.yaw_alive);
  detail::WriteFloat(frame, 4U, command.vx_mps);
  detail::WriteFloat(frame, 8U, command.vy_mps);
  detail::WriteFloat(frame, 12U, command.wz_rad_per_sec);
  detail::WriteU32(frame, 16U, static_cast<std::uint32_t>(command.mode));
  detail::WriteU32(frame, 20U, static_cast<std::uint32_t>(command.state));
  detail::WriteFloat(frame, 24U, command.big_yaw_velocity_rad_per_sec);
  detail::WriteFloat(frame, 36U, command.big_yaw_angle_rad);
  detail::WriteFloat(frame, 40U, command.yaw_angle_rad);
  return frame;
}

inline bool DecodeChassisCommand(std::span<const std::byte> frame,
                                 Command::ChassisCommand& command) noexcept
{
  if (frame.size() < kLegacyHostToSlavePayloadSize)
  {
    return false;
  }

  const auto mode = detail::ReadU32(frame, 16U);
  const auto state = detail::ReadU32(frame, 20U);
  const auto big_yaw_alive = std::to_integer<std::uint8_t>(frame[1U]);
  const auto yaw_alive = std::to_integer<std::uint8_t>(frame[2U]);
  if (mode > static_cast<std::uint32_t>(Command::ChassisMode::SPIN) ||
      state > static_cast<std::uint32_t>(Command::ControlState::GAME) ||
      big_yaw_alive > 1U || yaw_alive > 1U)
  {
    return false;
  }

  command.vx_mps = detail::ReadFloat(frame, 4U);
  command.vy_mps = detail::ReadFloat(frame, 8U);
  command.wz_rad_per_sec = detail::ReadFloat(frame, 12U);
  command.mode = static_cast<Command::ChassisMode>(mode);
  command.state = static_cast<Command::ControlState>(state);
  command.big_yaw_velocity_rad_per_sec = detail::ReadFloat(frame, 24U);
  command.enabled = command.state == Command::ControlState::HAND ||
                    command.state == Command::ControlState::GAME;
  command.big_yaw_alive = big_yaw_alive != 0U;
  command.yaw_alive = yaw_alive != 0U;
  command.big_yaw_angle_rad = detail::ReadFloat(frame, 36U);
  command.yaw_angle_rad = detail::ReadFloat(frame, 40U);
  return true;
}

inline Frame EncodeChassisFeedback(
    const Command::ChassisFeedback& feedback) noexcept
{
  Frame frame{};
  frame[0U] = static_cast<std::byte>(feedback.imu_alive);
  for (std::size_t index = 0; index < 3U; ++index)
  {
    detail::WriteFloat(frame, 4U + index * 4U,
                       feedback.imu_acceleration_mps2[index]);
    detail::WriteFloat(frame, 16U + index * 4U,
                       feedback.imu_gyro_rad_per_sec[index]);
    detail::WriteFloat(frame, 28U + index * 4U,
                       feedback.imu_euler_rad[index]);
  }
  return frame;
}

inline bool DecodeChassisFeedback(
    std::span<const std::byte> frame,
    Command::ChassisFeedback& feedback) noexcept
{
  if (frame.size() < kLegacySlaveToHostPayloadSize)
  {
    return false;
  }

  const auto imu_alive = std::to_integer<std::uint8_t>(frame[0U]);
  if (imu_alive > 1U)
  {
    return false;
  }

  feedback.imu_alive = imu_alive != 0U;
  for (std::size_t index = 0; index < 3U; ++index)
  {
    feedback.imu_acceleration_mps2[index] =
        detail::ReadFloat(frame, 4U + index * 4U);
    feedback.imu_gyro_rad_per_sec[index] =
        detail::ReadFloat(frame, 16U + index * 4U);
    feedback.imu_euler_rad[index] =
        detail::ReadFloat(frame, 28U + index * 4U);
  }
  return true;
}

}  // namespace UserApp::BoardProtocol
