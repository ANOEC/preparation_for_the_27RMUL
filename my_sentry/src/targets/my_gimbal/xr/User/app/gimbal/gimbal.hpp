#pragma once

#include <cstdint>

#include "DJIMotor.hpp"
#include "DMMotor.hpp"
#include "VQFModule.hpp"
#include "RefereeProtocol/RefereeProtocolGenerated.hpp"
#include "app/gimbal/control.hpp"
#include "app/host/protocol.hpp"
#include "app/remote/remote.hpp"
#include "topic.hpp"
#include "wave/sine.hpp"

namespace UserApp::Gimbal
{

class Gimbal
{
 public:
  explicit Gimbal(uint32_t feedback_timeout_ms = 100);
  void Run(float dt_s, uint32_t now_ms);

 private:
  struct AxisState
  {
    float angle = 0.0f;
    float velocity = 0.0f;
    float target = 0.0f;
    float torque = 0.0f;
    float measured_torque = 0.0f;
  };

  void ReceiveFeedback(uint32_t now_ms);
  void ReceiveCommand(uint32_t now_ms);
  void ReceiveHostAndReferee(uint32_t now_ms);
  void PublishVisionState();
  void UpdateAttitude(const VqfOutput<float>& imu);
  static float NearestEquivalentAngle(float wrapped_target, float reference);
  float PitchGravityTorque() const;
  void EnterSafeState();
  void ReceiveTorqueTestCommand(uint32_t now_ms, bool accept_command);
  void RunTorqueTest(float dt_s, uint32_t now_ms);
  void SendTorqueTestTelemetry(uint32_t now_ms, float target_torque);
  void SendGravityTestTelemetry(uint32_t now_ms, float gravity_torque);
  void SendPitchPositionTelemetry(uint32_t now_ms);
  void RunVelocityTest(float dt_s, uint32_t now_ms);
  void SendVelocityTelemetry(uint32_t now_ms, float target_velocity,
                             float actual_velocity, float commanded_torque,
                             float measured_torque);
  void PublishTorque(float yaw, float big_yaw, float pitch);
  bool FeedbackHealthy(uint32_t now_ms) const;
  static bool TimedOut(uint32_t now_ms, uint32_t timestamp_ms,
                       uint32_t timeout_ms);

  uint32_t feedback_timeout_ms_;
  uint32_t yaw_feedback_ms_ = 0;
  uint32_t big_yaw_feedback_ms_ = 0;
  uint32_t pitch_feedback_ms_ = 0;
  uint32_t imu_feedback_ms_ = 0;
  uint32_t command_ms_ = 0;
  bool yaw_received_ = false;
  bool big_yaw_received_ = false;
  bool pitch_received_ = false;
  bool imu_received_ = false;
  bool command_received_ = false;
  bool targets_initialized_ = false;
  bool yaw_initialized_ = false;
  float previous_wrapped_yaw_ = 0.0f;
  float continuous_yaw_ = 0.0f;
  float roll_angle_ = 0.0f;

  AxisState yaw_{};
  AxisState big_yaw_{};
  AxisState pitch_{};
  float yaw_motor_relative_angle_ = 0.0f;
  float yaw_motor_relative_velocity_ = 0.0f;
  float big_yaw_target_offset_ = 0.0f;
  float pitch_encoder_angle_ = 0.0f;
  Remote::GimbalCommand command_{};
  Host::GimbalCommand vision_command_{};
  std::uint64_t nav_mode_ = 0;
  RefereeProtocol::RobotStatus robot_status_{};
  RefereeProtocol::RobotShoot robot_shoot_{};
  RefereeProtocol::ProjectileAllowance projectile_allowance_{};
  uint32_t vision_command_ms_ = 0;
  uint32_t nav_mode_ms_ = 0;
  bool vision_command_received_ = false;
  bool nav_mode_received_ = false;
  bool vision_yaw_over_ = false;
  float vision_yaw_hold_target_ = 0.0f;
  static constexpr uint32_t HOST_TIMEOUT_MS = 2000U;
  static constexpr uint32_t VISION_EXTRAPOLATION_LIMIT_MS = 50U;

  CascadeController yaw_controller_{};
  CascadeController big_yaw_controller_{};
  CascadeController pitch_controller_{};
  Wave::SineWaveGenerator patrol_yaw_wave_{};
  Wave::SineWaveGenerator patrol_pitch_wave_{};
  Wave::SineWaveGenerator patrol_upper_pitch_wave_{};
  bool patrol_running_ = false;
  Wave::SineWaveGenerator pitch_position_test_wave_{};
  bool pitch_position_test_running_ = false;
  Wave::SineWaveGenerator velocity_test_wave_{};
  bool velocity_test_running_ = false;
  uint32_t velocity_telemetry_ms_ = 0;
  float torque_test_requested_nm_ = 0.0f;
  float torque_test_applied_nm_ = 0.0f;
  uint32_t torque_test_command_ms_ = 0;
  uint32_t torque_test_telemetry_ms_ = 0;
  bool torque_test_command_received_ = false;
  uint32_t gravity_test_telemetry_ms_ = 0;
  uint32_t position_telemetry_ms_ = 0;

  LibXR::Topic::ASyncSubscriber<DJIMotor::SolvedFeedback> yaw_subscriber_;
  LibXR::Topic::ASyncSubscriber<DMMotor::SolvedFeedback> big_yaw_subscriber_;
  LibXR::Topic::ASyncSubscriber<DMMotor::SolvedFeedback> pitch_subscriber_;
  LibXR::Topic::ASyncSubscriber<VqfOutput<float>> imu_subscriber_;
  LibXR::Topic::ASyncSubscriber<Remote::GimbalCommand> command_subscriber_;
  LibXR::Topic::ASyncSubscriber<Host::GimbalCommand> vision_command_subscriber_;
  LibXR::Topic::ASyncSubscriber<std::uint64_t> nav_mode_subscriber_;
  LibXR::Topic::Domain referee_domain_{"referee"};
  LibXR::Topic::ASyncSubscriber<RefereeProtocol::RobotStatus> robot_status_subscriber_;
  LibXR::Topic::ASyncSubscriber<RefereeProtocol::RobotShoot> robot_shoot_subscriber_;
  LibXR::Topic::ASyncSubscriber<RefereeProtocol::ProjectileAllowance> projectile_allowance_subscriber_;
  LibXR::Topic vision_state_topic_;
  LibXR::Topic yaw_torque_topic_;
  LibXR::Topic big_yaw_torque_topic_;
  LibXR::Topic pitch_torque_topic_;
};

}  // namespace UserApp::Gimbal
