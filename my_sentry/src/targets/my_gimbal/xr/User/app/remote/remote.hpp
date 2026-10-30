#pragma once

#include <cstddef>
#include <cstdint>

#include "Details/DT7/DT7.hpp"
#include "RefereeProtocol/RefereeProtocolGenerated.hpp"
#include "app/communication/b2b/chassis_command.hpp"
#include "app/host/protocol.hpp"
#include "topic.hpp"

namespace UserApp::Remote
{

/** The operating state selected by the DT7 switches. */
enum class ControlState : uint8_t
{
  OFFLINE,
  PROTECT,
  HAND,
  GAME,
};

/** Remote-control request consumed by the gimbal control task. */
struct GimbalCommand
{
  float yaw_offset_rad = 0.0f;
  float pitch_offset_rad = 0.0f;
  ControlState state = ControlState::OFFLINE;
  bool enabled = false;
  bool patrol = false;
};

/** Remote-control request consumed by the shooter task. */
struct ShootCommand
{
  ControlState state = ControlState::OFFLINE;
  bool friction_on = false;
  bool feeder_on = false;
};

/** DT7-to-application adapter. */
class Remote
{
 public:
  explicit Remote(
      const char* remote_topic_name = "/remote",
      const char* gimbal_topic_name = "/cmd/gimbal",
      const char* chassis_topic_name = "/cmd/chassis",
      const char* shoot_topic_name = "/cmd/shoot",
      uint32_t offline_timeout_ms = 500);

  /** Execute one non-blocking receive, safety-check and publish cycle. */
  void Run(uint32_t now_ms);

 private:
  using RemoteData = Module::Remote::DT7::sRemoteInfo;

  static constexpr float CHANNEL_FULL_SCALE = 660.0f;
  static constexpr float YAW_STEP_DEG = 0.6f;
  static constexpr float PITCH_STEP_DEG = 1.0f;
  // Keep hysteresis between press and release so DT7 wheel jitter cannot
  // toggle the friction wheels repeatedly.  Some transmitters only reach
  // about -550 after calibration, so -580 is too strict in practice.
  static constexpr int16_t FIRE_TOGGLE_PRESS_THRESHOLD = -500;
  static constexpr int16_t FIRE_TOGGLE_RELEASE_THRESHOLD = -300;

  static bool IsRemoteDataValid(const RemoteData& data);
  static float NormalizeChannel(int16_t channel);
  static GimbalCommand BuildCommand(const RemoteData& data);
  static Command::ChassisCommand BuildChassisCommand(const RemoteData& data);
  ShootCommand BuildShootCommand(const RemoteData& data, ControlState state);
  static GimbalCommand BuildSafeCommand(ControlState state);
  static ShootCommand BuildSafeShootCommand(ControlState state);
  bool IsTimedOut(uint32_t now_ms, uint32_t last_valid_ms) const;
  void ResetShootLatch();
  void ReceiveHostAndReferee(uint32_t now_ms);
  void ApplyGameOverrides(uint32_t now_ms);

  uint32_t offline_timeout_ms_;
  LibXR::Topic::Domain referee_domain_{"referee"};
  LibXR::Topic::ASyncSubscriber<RemoteData> remote_subscriber_;
  LibXR::Topic::ASyncSubscriber<Host::ChassisCommand> host_chassis_subscriber_;
  LibXR::Topic::ASyncSubscriber<std::uint64_t> nav_mode_subscriber_;
  LibXR::Topic::ASyncSubscriber<RefereeProtocol::GameStatus> game_status_subscriber_;
  LibXR::Topic gimbal_topic_;
  LibXR::Topic chassis_topic_;
  LibXR::Topic shoot_topic_;
  bool has_valid_data_ = false;
  uint32_t last_valid_ms_ = 0;
  Host::ChassisCommand host_chassis_command_{};
  std::uint64_t nav_mode_ = 0;
  RefereeProtocol::GameStatus game_status_{};
  uint32_t host_command_ms_ = 0;
  uint32_t nav_mode_ms_ = 0;
  uint32_t game_status_ms_ = 0;
  bool host_command_received_ = false;
  bool nav_mode_received_ = false;
  bool game_status_received_ = false;
  static constexpr uint32_t HOST_TIMEOUT_MS = 2000U;
  static constexpr uint32_t REFEREE_TIMEOUT_MS = 2000U;
  GimbalCommand command_ = BuildSafeCommand(ControlState::OFFLINE);
  Command::ChassisCommand chassis_command_ =
      Command::MakeSafeChassisCommand(Command::ControlState::OFFLINE);
  ShootCommand shoot_command_ = BuildSafeShootCommand(ControlState::OFFLINE);

  // Pull the DT7 wheel backwards once to toggle friction wheels; return it
  // near centre to re-arm the next toggle.  Pushing forward feeds only while
  // the friction-wheel latch is on.
  bool friction_latched_ = false;
  bool friction_toggle_armed_ = true;
};

}  // namespace UserApp::Remote
