#include "app/remote/remote.hpp"

#include <algorithm>
#include <numbers>

namespace UserApp::Remote
{

Remote::Remote(const char* remote_topic_name, const char* gimbal_topic_name,
               const char* chassis_topic_name, const char* shoot_topic_name,
               uint32_t offline_timeout_ms)
    : offline_timeout_ms_(offline_timeout_ms),
      remote_subscriber_(
          LibXR::Topic::CreateTopic<RemoteData>(remote_topic_name)),
      gimbal_topic_(LibXR::Topic::CreateTopic<GimbalCommand>(gimbal_topic_name)),
      chassis_topic_(LibXR::Topic::CreateTopic<Command::ChassisCommand>(
          chassis_topic_name)),
      shoot_topic_(LibXR::Topic::CreateTopic<ShootCommand>(shoot_topic_name))
{
  REQUIRE(remote_topic_name != nullptr);
  REQUIRE(gimbal_topic_name != nullptr);
  REQUIRE(chassis_topic_name != nullptr);
  REQUIRE(shoot_topic_name != nullptr);
  REQUIRE(offline_timeout_ms_ > 0U);
  remote_subscriber_.StartWaiting();
}

void Remote::Run(uint32_t now_ms)
{
  if (remote_subscriber_.Available())
  {
    const RemoteData remote_data = remote_subscriber_.GetData();
    remote_subscriber_.StartWaiting();

    if (IsRemoteDataValid(remote_data))
    {
      has_valid_data_ = true;
      last_valid_ms_ = now_ms;
      command_ = BuildCommand(remote_data);
      chassis_command_ = BuildChassisCommand(remote_data);
      shoot_command_ = BuildShootCommand(remote_data, command_.state);
    }
  }

  if (!has_valid_data_ || IsTimedOut(now_ms, last_valid_ms_))
  {
    command_ = BuildSafeCommand(ControlState::OFFLINE);
    chassis_command_ =
        Command::MakeSafeChassisCommand(Command::ControlState::OFFLINE);
    shoot_command_ = BuildSafeShootCommand(ControlState::OFFLINE);
    ResetShootLatch();
    has_valid_data_ = false;
  }

  gimbal_topic_.Publish(command_);
  chassis_topic_.Publish(chassis_command_);
  shoot_topic_.Publish(shoot_command_);
}

bool Remote::IsRemoteDataValid(const RemoteData& data)
{
  for (std::size_t i = 0; i < 5; ++i)
  {
    if (data.rc.ch[i] < -static_cast<int16_t>(CHANNEL_FULL_SCALE) ||
        data.rc.ch[i] > static_cast<int16_t>(CHANNEL_FULL_SCALE))
    {
      return false;
    }
  }

  const auto switch_valid = [](Module::Remote::DT7::SW_STATE state)
  {
    return state == Module::Remote::DT7::UP || state == Module::Remote::DT7::MID ||
           state == Module::Remote::DT7::DOWN;
  };

  return switch_valid(data.rc.s[0]) && switch_valid(data.rc.s[1]) &&
         data.mouse.press_l <= 1 && data.mouse.press_r <= 1;
}

float Remote::NormalizeChannel(int16_t channel)
{
  const float limited = std::clamp(static_cast<float>(channel), -CHANNEL_FULL_SCALE,
                                   CHANNEL_FULL_SCALE);
  return limited / CHANNEL_FULL_SCALE;
}

GimbalCommand Remote::BuildCommand(const RemoteData& data)
{
  constexpr float DEG_TO_RAD = std::numbers::pi_v<float> / 180.0f;

  GimbalCommand command{};
  command.yaw_offset_rad =
      -NormalizeChannel(data.rc.ch[0]) * YAW_STEP_DEG * DEG_TO_RAD;
  command.pitch_offset_rad =
      -NormalizeChannel(data.rc.ch[1]) * PITCH_STEP_DEG * DEG_TO_RAD;

  command.patrol = data.rc.s[0] == Module::Remote::DT7::DOWN;

  switch (data.rc.s[1])
  {
    case Module::Remote::DT7::UP:
      command.state = ControlState::GAME;
      command.enabled = true;
      break;
    case Module::Remote::DT7::MID:
      command.state = ControlState::HAND;
      command.enabled = true;
      break;
    case Module::Remote::DT7::DOWN:
    default:
      command = BuildSafeCommand(ControlState::PROTECT);
      break;
  }

  return command;
}

Command::ChassisCommand Remote::BuildChassisCommand(const RemoteData& data)
{
  Command::ChassisCommand command{};
  command.vx_mps = NormalizeChannel(data.rc.ch[3]) * 3.0F;
  command.vy_mps = -NormalizeChannel(data.rc.ch[2]) * 3.0F;
  command.wz_rad_per_sec = 0.0F;

  switch (data.rc.s[0])
  {
    case Module::Remote::DT7::UP:
      command.mode = Command::ChassisMode::SPIN;
      break;
    case Module::Remote::DT7::MID:
      command.mode = Command::ChassisMode::FOLLOW;
      break;
    case Module::Remote::DT7::DOWN:
    default:
      command.mode = Command::ChassisMode::PATROL;
      break;
  }

  switch (data.rc.s[1])
  {
    case Module::Remote::DT7::UP:
      command.state = Command::ControlState::GAME;
      command.enabled = true;
      break;
    case Module::Remote::DT7::MID:
      command.state = Command::ControlState::HAND;
      command.enabled = true;
      break;
    case Module::Remote::DT7::DOWN:
    default:
      command =
          Command::MakeSafeChassisCommand(Command::ControlState::PROTECT);
      break;
  }

  return command;
}

ShootCommand Remote::BuildShootCommand(const RemoteData& data, ControlState state)
{
  if (state == ControlState::PROTECT || state == ControlState::OFFLINE)
  {
    ResetShootLatch();
    return BuildSafeShootCommand(state);
  }

  const int16_t wheel = data.rc.ch[4];
  bool feeder_on = false;

  if (wheel <= FIRE_TOGGLE_PRESS_THRESHOLD)
  {
    if (friction_toggle_armed_)
    {
      friction_latched_ = !friction_latched_;
      friction_toggle_armed_ = false;
    }

    // Never feed while the wheel is pulled backwards to toggle friction.
    feeder_on = false;
  }
  else if (wheel > 0)
  {
    feeder_on = friction_latched_;
  }
  else
  {
    feeder_on = false;
  }

  if (wheel >= FIRE_TOGGLE_RELEASE_THRESHOLD)
  {
    friction_toggle_armed_ = true;
  }

  ShootCommand command{};
  command.state = state;
  command.friction_on = friction_latched_;
  command.feeder_on = feeder_on;
  return command;
}

GimbalCommand Remote::BuildSafeCommand(ControlState state)
{
  GimbalCommand command{};
  command.state = state;
  command.enabled = false;
  return command;
}

ShootCommand Remote::BuildSafeShootCommand(ControlState state)
{
  ShootCommand command{};
  command.state = state;
  command.friction_on = false;
  command.feeder_on = false;
  return command;
}

void Remote::ResetShootLatch()
{
  friction_latched_ = false;
  friction_toggle_armed_ = true;
}

bool Remote::IsTimedOut(uint32_t now_ms, uint32_t last_valid_ms) const
{
  // Unsigned subtraction remains correct when the millisecond counter wraps.
  return static_cast<uint32_t>(now_ms - last_valid_ms) > offline_timeout_ms_;
}

}  // namespace UserApp::Remote
