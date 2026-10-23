#pragma once

#include <cstddef>
#include <cstdint>

#include "Details/DT7/DT7.hpp"
#include "topic.hpp"

namespace UserApp::Remote
{

/** The operating state selected by the right-hand DT7 switch. */
enum class ControlState : uint8_t
{
  OFFLINE,
  PROTECT,
  MANUAL,
  GAME,
};

/**
 * Remote-control request consumed by the gimbal control task.
 *
 * Both angles are per-message increments in radians.  Keeping them as
 * increments preserves the behavior of the reference project: the gimbal
 * task owns the absolute target and applies these values every control cycle.
 */
struct GimbalCommand
{
  float yaw_offset_rad = 0.0f;
  float pitch_offset_rad = 0.0f;
  ControlState state = ControlState::OFFLINE;
  bool enabled = false;
};

/**
 * DT7-to-gimbal adapter.
 *
 * The generated task owns scheduling. This class only owns remote-control
 * state and executes one non-blocking control step when Run() is called.
 */
class Remote
{
 public:
  explicit Remote(
      const char* remote_topic_name = "/remote",
      const char* gimbal_topic_name = "/cmd/gimbal",
      uint32_t offline_timeout_ms = 500);

  /** Execute one non-blocking receive, safety-check and publish cycle. */
  void Run(uint32_t now_ms);

 private:
  using RemoteData = Module::Remote::DT7::sRemoteInfo;

  static constexpr float CHANNEL_FULL_SCALE = 660.0f;
  static constexpr float YAW_STEP_DEG = 0.6f;
  static constexpr float PITCH_STEP_DEG = 1.0f;

  static bool IsRemoteDataValid(const RemoteData& data);
  static float NormalizeChannel(int16_t channel);
  static GimbalCommand BuildCommand(const RemoteData& data);
  static GimbalCommand BuildSafeCommand(ControlState state);
  bool IsTimedOut(uint32_t now_ms, uint32_t last_valid_ms) const;

  uint32_t offline_timeout_ms_;
  LibXR::Topic::ASyncSubscriber<RemoteData> remote_subscriber_;
  LibXR::Topic gimbal_topic_;
  bool has_valid_data_ = false;
  uint32_t last_valid_ms_ = 0;
  GimbalCommand command_ = BuildSafeCommand(ControlState::OFFLINE);
};

}  // namespace UserApp::Remote
