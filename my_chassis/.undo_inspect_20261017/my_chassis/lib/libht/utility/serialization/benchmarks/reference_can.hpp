#pragma once

#include <bit>
#include <cstddef>
#include <cstdint>

#include <cstring>
#include <concepts>

namespace WireReference
{
enum class DataEndian { LittleEndian, BigEndian };
namespace detail
{
template <std::integral T>
inline std::size_t SerializeField(std::byte* buffer, T value, DataEndian endian)
{
  if constexpr (sizeof(T) == 1) *buffer = static_cast<std::byte>(value);
  else
  {
    if ((std::endian::native == std::endian::little) != (endian == DataEndian::LittleEndian))
      value = std::byteswap(value);
    std::memcpy(buffer, &value, sizeof(T));
  }
  return sizeof(T);
}
template <std::integral T>
inline T DeserializeField(const std::byte* buffer, DataEndian endian)
{
  if constexpr (sizeof(T) == 1) return static_cast<T>(*buffer);
  else
  {
    T value;
    std::memcpy(&value, buffer, sizeof(T));
    if ((std::endian::native == std::endian::little) != (endian == DataEndian::LittleEndian))
      value = std::byteswap(value);
    return value;
  }
}
}
namespace detail
{
inline std::size_t SerializeFloat(std::byte* buffer, float value,
                                  DataEndian endian)
{
  return detail::SerializeField<uint32_t>(
      buffer, std::bit_cast<uint32_t>(value), endian);
}

inline float DeserializeFloat(const std::byte* buffer,
                              DataEndian endian)
{
  return std::bit_cast<float>(
      detail::DeserializeField<uint32_t>(buffer, endian));
}
}  // namespace detail

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

  float target_vel[3];

  ChassisMode mode = ChassisMode::FREE;
  AppState state = AppState::OFFLINE;

  uint8_t game_progress;
  uint16_t stage_remain_time;
  uint16_t current_hp;
  uint16_t chassis_power_limit;
  uint16_t buffer_energy;

  float big_yaw_angle;
  float yaw_angle;

  // 按声明顺序编码，无填充；bool 和枚举各占 1 字节，float 占 4 字节。
  // Declaration order, no padding; bool/enums use 1 byte, floats use 4 bytes.
  static constexpr std::size_t GetSerializedSize() { return 34u; }

  std::size_t Serialize(std::byte* buffer, DataEndian endian) const
  {
    using detail::SerializeField;
    SerializeField<uint8_t>(buffer, referee_alive, endian);
    SerializeField<uint8_t>(buffer + 1u, big_yaw_alive, endian);
    SerializeField<uint8_t>(buffer + 2u, yaw_alive, endian);
    for (std::size_t i = 0u; i < 3u; ++i)
    {
      detail::SerializeFloat(buffer + 3u + i * 4u, target_vel[i], endian);
    }
    SerializeField<uint8_t>(buffer + 15u, static_cast<uint8_t>(mode), endian);
    SerializeField<uint8_t>(buffer + 16u, static_cast<uint8_t>(state), endian);
    SerializeField<uint8_t>(buffer + 17u, game_progress, endian);
    SerializeField<uint16_t>(buffer + 18u, stage_remain_time, endian);
    SerializeField<uint16_t>(buffer + 20u, current_hp, endian);
    SerializeField<uint16_t>(buffer + 22u, chassis_power_limit, endian);
    SerializeField<uint16_t>(buffer + 24u, buffer_energy, endian);
    detail::SerializeFloat(buffer + 26u, big_yaw_angle, endian);
    detail::SerializeFloat(buffer + 30u, yaw_angle, endian);
    return GetSerializedSize();
  }

  static GimbalToChassis Deserialize(const std::byte* buffer,
                                     DataEndian endian)
  {
    using detail::DeserializeField;
    GimbalToChassis result;
    result.referee_alive = DeserializeField<uint8_t>(buffer, endian) != 0u;
    result.big_yaw_alive = DeserializeField<uint8_t>(buffer + 1u, endian) != 0u;
    result.yaw_alive = DeserializeField<uint8_t>(buffer + 2u, endian) != 0u;
    for (std::size_t i = 0u; i < 3u; ++i)
    {
      result.target_vel[i] = detail::DeserializeFloat(buffer + 3u + i * 4u, endian);
    }
    result.mode =
        static_cast<ChassisMode>(DeserializeField<uint8_t>(buffer + 15u, endian));
    result.state = static_cast<AppState>(DeserializeField<uint8_t>(buffer + 16u, endian));
    result.game_progress = DeserializeField<uint8_t>(buffer + 17u, endian);
    result.stage_remain_time = DeserializeField<uint16_t>(buffer + 18u, endian);
    result.current_hp = DeserializeField<uint16_t>(buffer + 20u, endian);
    result.chassis_power_limit = DeserializeField<uint16_t>(buffer + 22u, endian);
    result.buffer_energy = DeserializeField<uint16_t>(buffer + 24u, endian);
    result.big_yaw_angle = detail::DeserializeFloat(buffer + 26u, endian);
    result.yaw_angle = detail::DeserializeFloat(buffer + 30u, endian);
    return result;
  }
};

struct ChassisToGimbal
{
  bool imu_alive = false;

  float imu_acc[3];
  float imu_gyro[3];
  float imu_euler[3];

  // 按声明顺序编码，无填充；bool 占 1 字节，float 占 4 字节。
  // Declaration order, no padding; bool uses 1 byte, floats use 4 bytes.
  static constexpr std::size_t GetSerializedSize() { return 37u; }

  std::size_t Serialize(std::byte* buffer, DataEndian endian) const
  {
    detail::SerializeField<uint8_t>(buffer, imu_alive, endian);
    for (std::size_t i = 0u; i < 3u; ++i)
    {
      detail::SerializeFloat(buffer + 1u + i * 4u, imu_acc[i], endian);
      detail::SerializeFloat(buffer + 13u + i * 4u, imu_gyro[i], endian);
      detail::SerializeFloat(buffer + 25u + i * 4u, imu_euler[i], endian);
    }
    return GetSerializedSize();
  }

  static ChassisToGimbal Deserialize(const std::byte* buffer,
                                     DataEndian endian)
  {
    ChassisToGimbal result;
    result.imu_alive =
        detail::DeserializeField<uint8_t>(buffer, endian) != 0u;
    for (std::size_t i = 0u; i < 3u; ++i)
    {
      result.imu_acc[i] = detail::DeserializeFloat(buffer + 1u + i * 4u, endian);
      result.imu_gyro[i] = detail::DeserializeFloat(buffer + 13u + i * 4u, endian);
      result.imu_euler[i] = detail::DeserializeFloat(buffer + 25u + i * 4u, endian);
    }
    return result;
  }
};
}  // namespace WireReference
