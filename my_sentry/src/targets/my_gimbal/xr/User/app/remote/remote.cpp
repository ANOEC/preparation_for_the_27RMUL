#include "app/remote/remote.hpp"

#include <algorithm>
#include <numbers>

namespace UserApp::Remote
{

Remote::Remote(const char* remote_topic_name, const char* gimbal_topic_name,
               uint32_t offline_timeout_ms)
    : offline_timeout_ms_(offline_timeout_ms),
      remote_subscriber_(
          LibXR::Topic::CreateTopic<RemoteData>(remote_topic_name)),
      gimbal_topic_(LibXR::Topic::CreateTopic<GimbalCommand>(gimbal_topic_name))
{
  REQUIRE(remote_topic_name != nullptr);
  REQUIRE(gimbal_topic_name != nullptr);
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
    }
  }

  if (!has_valid_data_ || IsTimedOut(now_ms, last_valid_ms_))
  {
    command_ = BuildSafeCommand(ControlState::OFFLINE);
    has_valid_data_ = false;
  }

  gimbal_topic_.Publish(command_);
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

  switch (data.rc.s[1])
  {
    case Module::Remote::DT7::UP:
      command.state = ControlState::GAME;
      command.enabled = true;
      break;
    case Module::Remote::DT7::MID:
      command.state = ControlState::MANUAL;
      command.enabled = true;
      break;
    case Module::Remote::DT7::DOWN:
    default:
      command = BuildSafeCommand(ControlState::PROTECT);
      break;
  }

  return command;
}

GimbalCommand Remote::BuildSafeCommand(ControlState state)
{
  GimbalCommand command{};
  command.state = state;
  command.enabled = false;
  return command;
}

bool Remote::IsTimedOut(uint32_t now_ms, uint32_t last_valid_ms) const
{
  // Unsigned subtraction remains correct when the millisecond counter wraps.
  return static_cast<uint32_t>(now_ms - last_valid_ms) > offline_timeout_ms_;
}

}  // namespace UserApp::Remote
