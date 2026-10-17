#pragma once

#include <atomic>
#include <cstdint>

#include "DJIMotor.hpp"
#include "VQFModule.hpp"
#include "control/gimbal_algorithm.hpp"
#include "control/gimbal_sys_id.hpp"
#include "libxr.hpp"
#include "message.hpp"

enum class GimbalMode : uint8_t { kNormal = 0, kCruise = 1 };

struct GimbalAxisTelemetry {
  float target_angle_rad;
  float feedback_angle_rad;
  float angle_error_rad;
  float position_pid_output_rad_per_sec;
  float target_velocity_rad_per_sec;
  float feedback_velocity_rad_per_sec;
  float velocity_error_rad_per_sec;
  float velocity_pid_output_nm;
  float velocity_feedforward_rad_per_sec;
  float acceleration_feedforward_nm;
  float position_feedforward_rad_per_sec;
  float algorithm_output_nm;
  float ladrc_z1;
  float ladrc_z2;
  float ladrc_z3;
  float final_command_nm;
  float motor_encoder_angle_rad;
  float motor_reported_speed_rad_per_sec;
};

struct GimbalControlTelemetry {
  GimbalAxisTelemetry yaw;
  GimbalAxisTelemetry pitch;
  float gyro_x_rad_per_sec;
  float gyro_y_rad_per_sec;
  float gyro_z_rad_per_sec;
  float yaw_continuous_angle_rad;
  float pitch_gravity_compensation_nm;
  uint8_t yaw_motor_feedback_fresh;
  uint8_t pitch_motor_feedback_fresh;
  uint8_t attitude_sample_valid;
  uint8_t attitude_fresh;
  uint8_t controller_enabled;
  uint8_t yaw_diagnostic_active;
  uint8_t pitch_diagnostic_active;
  uint8_t yaw_stall_protected;
  uint8_t pitch_stall_protected;
  uint8_t yaw_encoder_limit_active;
  uint8_t pitch_encoder_limit_active;
  uint8_t cruise_mode_active;
};

class GimbalController {
 public:
  GimbalController();

  GimbalController(const GimbalController&) = delete;
  GimbalController& operator=(const GimbalController&) = delete;

  void Reset();
  void SetEnabled(bool enabled);
  void SetYawAngleReference(float reference_rad);
  void SetPitchAngleReference(float reference_rad);
  void SetMode(GimbalMode mode);
  void ConfigureCruise(float yaw_amplitude_rad, float yaw_frequency_hz,
                       float pitch_amplitude_rad, float pitch_frequency_hz);
  void RequestYawDiagnosticTorque(float torque_nm);
  void RequestPitchDiagnosticTorque(float torque_nm);
  bool IsHomed() const;
  GimbalMode RequestedMode() const;
  bool CruiseModeActive() const;

 private:
  using FeedbackMessage = LibXR::Topic::Message<DJIMotor::SolvedFeedback>;
  using AttitudeMessage = LibXR::Topic::Message<VqfOutput<float>>;

  static void ThreadEntry(GimbalController* controller);
  void Run();
  void ResetAlgorithms();

  app::gimbal::GimbalAlgorithm yaw_controller_;
  app::gimbal::GimbalAlgorithm pitch_controller_;
  LibXR::SPSCQueue<FeedbackMessage> yaw_feedback_queue_;
  LibXR::SPSCQueue<FeedbackMessage> pitch_feedback_queue_;
  LibXR::Topic yaw_feedback_topic_;
  LibXR::Topic pitch_feedback_topic_;
  LibXR::Topic::QueuedSubscriber yaw_feedback_subscriber_;
  LibXR::Topic::QueuedSubscriber pitch_feedback_subscriber_;
  LibXR::SPSCQueue<AttitudeMessage> attitude_queue_;
  LibXR::Topic attitude_topic_;
  LibXR::Topic::QueuedSubscriber attitude_subscriber_;
  LibXR::Topic yaw_command_topic_;
  LibXR::Topic pitch_command_topic_;
  LibXR::Topic telemetry_topic_;
  LibXR::Topic sys_id_telemetry_topic_;
  LibXR::Thread thread_;
  std::atomic<bool> reset_requested_{false};
  std::atomic<bool> enabled_requested_{false};
  std::atomic<float> requested_yaw_angle_rad_{0.0F};
  std::atomic<float> requested_pitch_angle_rad_{0.0F};
  std::atomic<bool> yaw_reference_updated_{false};
  std::atomic<bool> pitch_reference_updated_{false};
  std::atomic<uint8_t> requested_mode_{
      static_cast<uint8_t>(GimbalMode::kNormal)};
  std::atomic<bool> cruise_parameters_updated_{false};
  std::atomic<float> cruise_yaw_amplitude_rad_{0.0F};
  std::atomic<float> cruise_yaw_frequency_hz_{0.0F};
  std::atomic<float> cruise_pitch_amplitude_rad_{0.0F};
  std::atomic<float> cruise_pitch_frequency_hz_{0.0F};
  std::atomic<bool> cruise_mode_active_status_{false};
  std::atomic<float> yaw_diagnostic_torque_nm_{0.0F};
  std::atomic<float> pitch_diagnostic_torque_nm_{0.0F};
  std::atomic<bool> yaw_diagnostic_torque_requested_{false};
  std::atomic<bool> pitch_diagnostic_torque_requested_{false};
  std::atomic<bool> homing_complete_status_{false};
};

void GimbalControllerInit();
void GimbalControllerReset();
bool GimbalControllerSetEnabled(bool enabled);
bool GimbalControllerSetYawAngleReference(float reference_rad);
bool GimbalControllerSetPitchAngleReference(float reference_rad);
bool GimbalControllerSetMode(GimbalMode mode);
bool GimbalControllerConfigureCruise(float yaw_amplitude_rad,
                                     float yaw_frequency_hz,
                                     float pitch_amplitude_rad,
                                     float pitch_frequency_hz);
GimbalMode GimbalControllerGetRequestedMode();
bool GimbalControllerCruiseModeActive();
bool GimbalControllerRequestYawDiagnosticTorque(float torque_nm);
bool GimbalControllerRequestPitchDiagnosticTorque(float torque_nm);
bool GimbalControllerIsHomed();
