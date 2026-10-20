#include "app/remote/remote_app.hpp"

#include <algorithm>
#include <cmath>

#include "timebase.hpp"

namespace UserApp::Remote
{

RemoteApp::RemoteApp(const char* remote_topic_name, const char* chassis_topic_name,
                     uint32_t offline_timeout_ms, std::size_t thread_stack,
                     LibXR::Thread::Priority priority)
    : remote_topic_name_(remote_topic_name),
      offline_timeout_ms_(offline_timeout_ms),
      chassis_topic_(LibXR::Topic::CreateTopic<ChassisCommand>(chassis_topic_name))
{
  // 话题名来自静态配置，在线期间必须始终有效；超时时间必须大于发布周期。
  REQUIRE(remote_topic_name_ != nullptr);
  REQUIRE(chassis_topic_name != nullptr);
  REQUIRE(offline_timeout_ms_ > PUBLISH_PERIOD_MS);

  thread_.Create(this, ThreadEntry, "remote_app", thread_stack, priority);
}

void RemoteApp::ThreadEntry(RemoteApp* self)
{
  REQUIRE(self != nullptr);
  self->Run();
}

void RemoteApp::Run()
{
  RemoteData remote_data{};

  // 订阅者在工作线程内等待 /remote 创建，避免阻塞 XRobot 后续模块的构造。
  LibXR::Topic::SyncSubscriber<RemoteData> remote_subscriber(remote_topic_name_,
                                                              remote_data);

  bool has_valid_data = false;
  uint32_t last_valid_ms = 0;
  ChassisCommand command = BuildSafeCommand(ControlState::OFFLINE);

  while (true)
  {
    const LibXR::ErrorCode result = remote_subscriber.Wait(PUBLISH_PERIOD_MS);
    const uint32_t now_ms = static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());

    if (result == LibXR::ErrorCode::OK && IsRemoteDataValid(remote_data))
    {
      has_valid_data = true;
      last_valid_ms = now_ms;
      command = BuildCommand(remote_data);
    }

    if (!has_valid_data || IsTimedOut(now_ms, last_valid_ms))
    {
      command = BuildSafeCommand(ControlState::OFFLINE);
      has_valid_data = false;
    }

    // 即使遥控器数值没有变化也周期发布，便于下游控制器实施自己的命令超时保护。
    chassis_topic_.Publish(command);
  }
}

bool RemoteApp::IsRemoteDataValid(const RemoteData& data)
{
  for (std::size_t i = 0; i < 4; ++i)
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

float RemoteApp::NormalizeChannel(int16_t channel)
{
  const float limited = std::clamp(static_cast<float>(channel), -CHANNEL_FULL_SCALE,
                                   CHANNEL_FULL_SCALE);
  return limited / CHANNEL_FULL_SCALE;
}

ChassisCommand RemoteApp::BuildCommand(const RemoteData& data)
{
  ChassisCommand command{};

  // 保持旧工程方向定义：ch3 控制前后，ch2 取反后控制左右。
  command.vx_mps =
      NormalizeChannel(data.rc.ch[3]) * MAX_TRANSLATION_SPEED_MPS;
  command.vy_mps =
      -NormalizeChannel(data.rc.ch[2]) * MAX_TRANSLATION_SPEED_MPS;
  command.wz_rad_per_sec = 0.0f;

  switch (data.rc.s[0])
  {
    case Module::Remote::DT7::UP:
      command.mode = ChassisMode::SPIN;
      break;
    case Module::Remote::DT7::MID:
      command.mode = ChassisMode::FOLLOW;
      break;
    case Module::Remote::DT7::DOWN:
    default:
      command.mode = ChassisMode::FREE;
      break;
  }

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

ChassisCommand RemoteApp::BuildSafeCommand(ControlState state)
{
  ChassisCommand command{};
  command.vx_mps = 0.0f;
  command.vy_mps = 0.0f;
  command.wz_rad_per_sec = 0.0f;
  command.state = state;
  command.mode = ChassisMode::FREE;
  command.enabled = false;
  return command;
}

bool RemoteApp::IsTimedOut(uint32_t now_ms, uint32_t last_valid_ms) const
{
  return static_cast<uint32_t>(now_ms - last_valid_ms) > offline_timeout_ms_;
}

}  // namespace UserApp::Remote
