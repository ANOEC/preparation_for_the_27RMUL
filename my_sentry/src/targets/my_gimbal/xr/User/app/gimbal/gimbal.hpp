#pragma once

#include <cstdint>

#include "DJIMotor.hpp"
#include "DMMotor.hpp"
#include "VQFModule.hpp"
#include "app/gimbal/control.hpp"
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
  void UpdateAttitude(const VqfOutput<float>& imu);
  void EnterSafeState();
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

  AxisState yaw_{};
  AxisState big_yaw_{};
  AxisState pitch_{};
  float yaw_motor_relative_angle_ = 0.0f;
  float yaw_motor_relative_velocity_ = 0.0f;
  float big_yaw_target_offset_ = 0.0f;
  float pitch_encoder_angle_ = 0.0f;
  Remote::GimbalCommand command_{};

  CascadeController yaw_controller_{};
  CascadeController big_yaw_controller_{};
  CascadeController pitch_controller_{};
  Wave::SineWaveGenerator velocity_test_wave_{};
  bool velocity_test_running_ = false;
  uint32_t velocity_telemetry_ms_ = 0;

  LibXR::Topic::ASyncSubscriber<DJIMotor::SolvedFeedback> yaw_subscriber_;
  LibXR::Topic::ASyncSubscriber<DMMotor::SolvedFeedback> big_yaw_subscriber_;
  LibXR::Topic::ASyncSubscriber<DMMotor::SolvedFeedback> pitch_subscriber_;
  LibXR::Topic::ASyncSubscriber<VqfOutput<float>> imu_subscriber_;
  LibXR::Topic::ASyncSubscriber<Remote::GimbalCommand> command_subscriber_;
  LibXR::Topic yaw_torque_topic_;
  LibXR::Topic big_yaw_torque_topic_;
  LibXR::Topic pitch_torque_topic_;
};

}  // namespace UserApp::Gimbal
