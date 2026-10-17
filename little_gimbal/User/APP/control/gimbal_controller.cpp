#include "gimbal_controller.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "config/app_config.hpp"
#include "control/cruise_target_generator.hpp"

namespace {
namespace Config = AppConfig::Gimbal;
using PID = app::gimbal::GimbalAlgorithm::PID;

GimbalController* controller_instance = nullptr;

float WrapToPi(float angle) {
  constexpr float kPi = std::numbers::pi_v<float>;
  constexpr float kTwoPi = 2.0F * kPi;
  while (angle > kPi) angle -= kTwoPi;
  while (angle < -kPi) angle += kTwoPi;
  return angle;
}

float WrapToTwoPi(float angle) {
  constexpr float kTwoPi = 2.0F * std::numbers::pi_v<float>;
  angle = std::fmod(angle, kTwoPi);
  return angle < 0.0F ? angle + kTwoPi : angle;
}

[[maybe_unused]] float EnforceMinimumMotionCommand(
    float command, float position_error, float velocity_target,
    float motor_command_sign, float position_deadband, float velocity_deadband,
    float minimum_torque) {
  if (std::abs(position_error) <= position_deadband ||
      std::abs(velocity_target) <= velocity_deadband) {
    return command;
  }
  const float desired_motor_direction = motor_command_sign * velocity_target;
  if (command * desired_motor_direction <= 0.0F ||
      std::abs(command) < minimum_torque) {
    return std::copysign(minimum_torque, desired_motor_direction);
  }
  return command;
}

PID::Param PositionParams(float kp, float ki, float kd, float deadband,
                          float integral_limit, float output_limit) {
  return {.kp_ = kp,
          .ki_ = ki,
          .kd_ = kd,
          .deadband_ = deadband,
          .integral_limit_ = integral_limit,
          .output_limit_ = output_limit,
          .type_ = algorithm::control::pid::PIDType::POSITION};
}

GimbalAxisTelemetry MakeAxisTelemetry(
    const app::gimbal::GimbalAlgorithm::Debug& debug, float final_command,
    const DJIMotor::SolvedFeedback& motor) {
  return {
      .target_angle_rad = debug.position_target,
      .feedback_angle_rad = debug.position_feedback,
      .angle_error_rad = debug.position_error,
      .position_pid_output_rad_per_sec = debug.position_pid_output,
      .target_velocity_rad_per_sec = debug.velocity_target,
      .feedback_velocity_rad_per_sec = debug.velocity_feedback,
      .velocity_error_rad_per_sec = debug.velocity_error,
      .velocity_pid_output_nm = debug.velocity_pid_output,
      .velocity_feedforward_rad_per_sec = debug.velocity_feedforward,
      .acceleration_feedforward_nm = debug.acceleration_feedforward,
      .position_feedforward_rad_per_sec = debug.position_feedforward,
      .algorithm_output_nm = debug.final_output,
      .ladrc_z1 = debug.ladrc_z1,
      .ladrc_z2 = debug.ladrc_z2,
      .ladrc_z3 = debug.ladrc_z3,
      .final_command_nm = final_command,
      .motor_encoder_angle_rad = motor.position_rad,
      .motor_reported_speed_rad_per_sec = motor.velocity_rad_per_sec,
  };
}
}  // namespace

GimbalController::GimbalController()
    : yaw_feedback_queue_(32),
      pitch_feedback_queue_(32),
      yaw_feedback_topic_(LibXR::Topic::CreateTopic<DJIMotor::SolvedFeedback>(
          "yaw_motor_feedback")),
      pitch_feedback_topic_(LibXR::Topic::CreateTopic<DJIMotor::SolvedFeedback>(
          "pitch_motor_feedback")),
      yaw_feedback_subscriber_(yaw_feedback_topic_, yaw_feedback_queue_),
      pitch_feedback_subscriber_(pitch_feedback_topic_, pitch_feedback_queue_),
      attitude_queue_(32),
      attitude_topic_(LibXR::Topic::CreateTopic<VqfOutput<float>>("vqf_output")),
      attitude_subscriber_(attitude_topic_, attitude_queue_),
      yaw_command_topic_(LibXR::Topic::CreateTopic<float>(
          "yaw_motor_torque_command")),
      pitch_command_topic_(LibXR::Topic::CreateTopic<float>(
          "pitch_motor_torque_command")),
      telemetry_topic_(LibXR::Topic::CreateTopic<GimbalControlTelemetry>(
          "gimbal_control")),
      sys_id_telemetry_topic_(
          LibXR::Topic::CreateTopic<GimbalSysIdTelemetry>("gimbal_sys_id")) {
  yaw_controller_.set_pid_params(
      PositionParams(Config::kYawAngleKp, Config::kYawAngleKi,
                     Config::kYawAngleKd, Config::kYawAngleDeadbandRad,
                     Config::kYawAngleIntegralLimit,
                     Config::kYawSpeedRefLimitRadPerSec),
      PositionParams(Config::kYawSpeedKp, Config::kYawSpeedKi,
                     Config::kYawSpeedKd, Config::kYawSpeedDeadbandRadPerSec,
                     Config::kYawSpeedIntegralLimit,
                     Config::kYawVelocityPidOutputLimitNm));
  pitch_controller_.set_pid_params(
      PositionParams(Config::kPitchAngleKp, Config::kPitchAngleKi,
                     Config::kPitchAngleKd, Config::kPitchAngleDeadbandRad,
                     Config::kPitchAngleIntegralLimit,
                     Config::kPitchSpeedRefLimitRadPerSec),
      PositionParams(Config::kPitchSpeedKp, Config::kPitchSpeedKi,
                     Config::kPitchSpeedKd,
                     Config::kPitchSpeedDeadbandRadPerSec,
                     Config::kPitchSpeedIntegralLimit,
                     Config::kPitchVelocityPidOutputLimitNm));
  const auto& yaw_ladrc1 = Config::kYawLadrc1Param;
  yaw_controller_.set_ladrc1_params(yaw_ladrc1.b0, yaw_ladrc1.ts,
                                    yaw_ladrc1.output_limit, yaw_ladrc1.r);
  const auto& yaw_ladrc2 = Config::kYawLadrc2Param;
  yaw_controller_.set_ladrc2_params(yaw_ladrc2.b0, yaw_ladrc2.ts,
                                    yaw_ladrc2.output_limit, yaw_ladrc2.r);
  yaw_controller_.set_ladrc2_disturbance_compensation_limit(
      Config::kYawLadrc2DisturbanceCompensationLimitNm);
  const auto& pitch_ladrc1 = Config::kPitchLadrc1Param;
  pitch_controller_.set_ladrc1_params(
      pitch_ladrc1.b0, pitch_ladrc1.ts, pitch_ladrc1.output_limit,
      pitch_ladrc1.r);
  const auto& pitch_ladrc2 = Config::kPitchLadrc2Param;
  pitch_controller_.set_ladrc2_params(
      pitch_ladrc2.b0, pitch_ladrc2.ts, pitch_ladrc2.output_limit,
      pitch_ladrc2.r);
  yaw_controller_.set_ff_params(Config::kYawPositionFeedforwardGain);
  pitch_controller_.set_ff_params(Config::kPitchPositionFeedforwardGain);
  ResetAlgorithms();
  enabled_requested_.store(Config::kControlEnabledAtStartup,
                           std::memory_order_relaxed);
  requested_mode_.store(
      static_cast<uint8_t>(Config::kCruiseEnabledAtStartup
                               ? GimbalMode::kCruise
                               : GimbalMode::kNormal),
      std::memory_order_relaxed);
  ConfigureCruise(Config::kCruiseYawAmplitudeRad,
                  Config::kCruiseYawFrequencyHz,
                  Config::kCruisePitchAmplitudeRad,
                  Config::kCruisePitchFrequencyHz);

  float zero = 0.0F;
  yaw_command_topic_.Publish(zero);
  pitch_command_topic_.Publish(zero);
  thread_.Create(this, ThreadEntry, "gimbal_control", 3072,
                 LibXR::Thread::Priority::HIGH);
}

void GimbalController::Reset() {
  reset_requested_.store(true, std::memory_order_release);
}

void GimbalController::SetEnabled(bool enabled) {
  enabled_requested_.store(enabled, std::memory_order_release);
}

void GimbalController::SetYawAngleReference(float reference_rad) {
  requested_yaw_angle_rad_.store(reference_rad, std::memory_order_relaxed);
  yaw_reference_updated_.store(true, std::memory_order_release);
}

void GimbalController::SetPitchAngleReference(float reference_rad) {
  requested_pitch_angle_rad_.store(reference_rad, std::memory_order_relaxed);
  pitch_reference_updated_.store(true, std::memory_order_release);
}

void GimbalController::SetMode(GimbalMode mode) {
  requested_mode_.store(static_cast<uint8_t>(mode),
                        std::memory_order_release);
}

void GimbalController::ConfigureCruise(float yaw_amplitude_rad,
                                       float yaw_frequency_hz,
                                       float pitch_amplitude_rad,
                                       float pitch_frequency_hz) {
  cruise_yaw_amplitude_rad_.store(yaw_amplitude_rad,
                                  std::memory_order_relaxed);
  cruise_yaw_frequency_hz_.store(yaw_frequency_hz,
                                 std::memory_order_relaxed);
  cruise_pitch_amplitude_rad_.store(pitch_amplitude_rad,
                                    std::memory_order_relaxed);
  cruise_pitch_frequency_hz_.store(pitch_frequency_hz,
                                   std::memory_order_relaxed);
  cruise_parameters_updated_.store(true, std::memory_order_release);
}

void GimbalController::RequestYawDiagnosticTorque(float torque_nm) {
  yaw_diagnostic_torque_nm_.store(torque_nm, std::memory_order_relaxed);
  yaw_diagnostic_torque_requested_.store(true, std::memory_order_release);
}

void GimbalController::RequestPitchDiagnosticTorque(float torque_nm) {
  pitch_diagnostic_torque_nm_.store(torque_nm, std::memory_order_relaxed);
  pitch_diagnostic_torque_requested_.store(true, std::memory_order_release);
}

bool GimbalController::IsHomed() const {
  return homing_complete_status_.load(std::memory_order_acquire);
}

GimbalMode GimbalController::RequestedMode() const {
  return static_cast<GimbalMode>(
      requested_mode_.load(std::memory_order_acquire));
}

bool GimbalController::CruiseModeActive() const {
  return cruise_mode_active_status_.load(std::memory_order_acquire);
}

void GimbalController::ThreadEntry(GimbalController* controller) {
  controller->Run();
}

void GimbalController::ResetAlgorithms() {
  yaw_controller_.reset();
  pitch_controller_.reset();
}

void GimbalController::Run() {
  auto wake_time = LibXR::Timebase::GetMilliseconds();
  uint32_t last_yaw_feedback_ms = 0;
  uint32_t last_pitch_feedback_ms = 0;
  uint32_t last_attitude_ms = 0;
  bool yaw_manual_torque_active = false;
  bool pitch_manual_torque_active = false;
  bool have_yaw_feedback = false;
  bool have_pitch_feedback = false;
  bool have_attitude = false;
  bool attitude_sample_valid = false;
  bool control_was_active = false;
  bool homing_complete = false;
  bool cruise_active = false;
  app::gimbal::CruiseTargetGenerator cruise_generator;
  GimbalSystemIdentification system_identification;
  [[maybe_unused]] bool homing_settle_candidate = false;
  [[maybe_unused]] uint32_t homing_settle_start_ms = 0;
  bool yaw_initialized = false;
  float last_yaw_wrapped = 0.0F;
  float yaw_continuous = 0.0F;
  float yaw_target = 0.0F;
  float yaw_requested_target = 0.0F;
  float yaw_command_applied = 0.0F;
  uint32_t yaw_stall_start_ms = 0;
  bool yaw_stall_candidate = false;
  bool yaw_stall_protected = false;
#if GIMBAL_YAW_CTRL_MODE == CTRL_LADRC2
  bool yaw_breakaway_active = false;
  bool yaw_breakaway_motion_candidate = false;
  uint32_t yaw_breakaway_motion_start_ms = 0;
  float yaw_breakaway_direction = 0.0F;
  float yaw_breakaway_torque_nm = Config::kYawLadrc2BreakawayTorqueNm;
  const auto reset_yaw_breakaway = [&]() {
    yaw_breakaway_active = false;
    yaw_breakaway_motion_candidate = false;
    yaw_breakaway_motion_start_ms = 0;
    yaw_breakaway_direction = 0.0F;
    yaw_breakaway_torque_nm = Config::kYawLadrc2BreakawayTorqueNm;
  };
#endif
  float pitch_target = 0.0F;
  float pitch_requested_target = 0.0F;
  float pitch_command_applied = 0.0F;
  uint32_t pitch_stall_start_ms = 0;
  bool pitch_stall_candidate = false;
  bool pitch_stall_protected = false;
  float yaw_encoder_zero_phase = Config::kYawEncoderCenterPhaseRad;
  float pitch_zero_offset = 0.0F;
  float pitch_angle = 0.0F;
  float yaw_velocity = 0.0F;
  float pitch_velocity = 0.0F;
  float gyro_x = 0.0F;
  float gyro_y = 0.0F;
  float gyro_z = 0.0F;
  DJIMotor::SolvedFeedback yaw_motor{};
  DJIMotor::SolvedFeedback pitch_motor{};

  while (true) {
    LibXR::Thread::SleepUntil(wake_time, Config::kControlPeriodMs);
    const uint32_t now =
        static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    // Feedforward is opt-in for each reference source. It defaults to zero on
    // every cycle so normal references and all inactive/protection paths
    // cannot retain a previous cruise value.
    [[maybe_unused]] float yaw_velocity_feedforward = 0.0F;

    FeedbackMessage motor_sample{};
    while (yaw_feedback_queue_.Pop(motor_sample) == LibXR::ErrorCode::OK) {
      if (std::isfinite(motor_sample.data.position_rad) &&
          std::isfinite(motor_sample.data.velocity_rad_per_sec)) {
        yaw_motor = motor_sample.data;
        last_yaw_feedback_ms = now;
        have_yaw_feedback = true;
      }
    }
    while (pitch_feedback_queue_.Pop(motor_sample) == LibXR::ErrorCode::OK) {
      if (std::isfinite(motor_sample.data.position_rad) &&
          std::isfinite(motor_sample.data.velocity_rad_per_sec)) {
        pitch_motor = motor_sample.data;
        last_pitch_feedback_ms = now;
        have_pitch_feedback = true;
      }
    }

    AttitudeMessage attitude_sample{};
    while (attitude_queue_.Pop(attitude_sample) == LibXR::ErrorCode::OK) {
      attitude_sample_valid = false;
      const auto& q = attitude_sample.data.orientation_6d;
      const auto& gyro = attitude_sample.data
                             .bias_corrected_angular_velocity_sensor_rad_per_second;
      const float norm_sq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
      if (std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) &&
          std::isfinite(q.z) && std::isfinite(norm_sq) && norm_sq > 0.25F &&
          std::isfinite(gyro.x) && std::isfinite(gyro.y) &&
          std::isfinite(gyro.z)) {
        const auto euler =
            LibXR::Quaternion<float>(q.w, q.x, q.y, q.z).ToEulerAngleZYX();
        const float yaw_wrapped = euler.z();
        if (std::isfinite(euler.y()) && std::isfinite(yaw_wrapped)) {
          if (!yaw_initialized) {
            yaw_continuous = yaw_wrapped;
            yaw_initialized = true;
          } else {
            yaw_continuous += WrapToPi(yaw_wrapped - last_yaw_wrapped);
          }
          last_yaw_wrapped = yaw_wrapped;
          pitch_angle = euler.y();
          gyro_x = gyro.x;
          gyro_y = gyro.y;
          gyro_z = gyro.z;
          yaw_velocity = Config::kYawGyroSign * gyro_z;
          pitch_velocity = Config::kPitchGyroSign * gyro_y;
          last_attitude_ms = now;
          attitude_sample_valid = true;
          have_attitude = true;
        }
      }
    }

    if (reset_requested_.exchange(false, std::memory_order_acq_rel)) {
      ResetAlgorithms();
#if GIMBAL_YAW_CTRL_MODE == CTRL_LADRC2
      reset_yaw_breakaway();
#endif
      system_identification.Reset();
      cruise_generator.Stop();
      cruise_active = false;
      cruise_mode_active_status_.store(false, std::memory_order_release);
      homing_complete = false;
      homing_complete_status_.store(false, std::memory_order_release);
      homing_settle_candidate = false;
      yaw_encoder_zero_phase = Config::kYawEncoderCenterPhaseRad;
      pitch_zero_offset = 0.0F;
      enabled_requested_.store(false, std::memory_order_release);
    }

    const bool yaw_feedback_fresh =
        have_yaw_feedback &&
        (now - last_yaw_feedback_ms <= Config::kFeedbackTimeoutMs);
    const bool pitch_feedback_fresh =
        have_pitch_feedback &&
        (now - last_pitch_feedback_ms <= Config::kFeedbackTimeoutMs);
    const bool attitude_fresh =
        have_attitude && attitude_sample_valid &&
        (now - last_attitude_ms <= Config::kAttitudeTimeoutMs);
    const bool enabled = enabled_requested_.load(std::memory_order_acquire);
    const bool control_active = enabled && yaw_feedback_fresh &&
                                pitch_feedback_fresh && attitude_fresh;
    const float yaw_encoder_phase = WrapToTwoPi(yaw_motor.position_rad);
    const float yaw_encoder_center_angle =
        Config::kYawEncoderToVqfScale *
        WrapToPi(yaw_encoder_phase - yaw_encoder_zero_phase);
    const float yaw_feedback_angle =
        yaw_encoder_center_angle;
    const float pitch_feedback_angle =
        homing_complete
            ? pitch_angle - pitch_zero_offset
            : Config::kPitchEncoderToVqfScale *
                  (pitch_motor.position_rad - Config::kPitchEncoderCenterRad);

    if (control_active && !control_was_active) {
      // Before homing, zero is the calibrated mechanical center. After homing,
      // resume bumplessly from the current center-relative attitude.
      yaw_target = homing_complete ? yaw_feedback_angle : 0.0F;
      yaw_requested_target = yaw_target;
      pitch_target = homing_complete ? pitch_feedback_angle : 0.0F;
      pitch_requested_target = pitch_target;
      yaw_command_applied = 0.0F;
      pitch_command_applied = 0.0F;
      yaw_stall_candidate = false;
      yaw_stall_protected = false;
#if GIMBAL_YAW_CTRL_MODE == CTRL_LADRC2
      reset_yaw_breakaway();
#endif
      pitch_stall_candidate = false;
      pitch_stall_protected = false;
      requested_yaw_angle_rad_.store(yaw_target,
                                      std::memory_order_relaxed);
      requested_pitch_angle_rad_.store(pitch_target,
                                        std::memory_order_relaxed);
      yaw_reference_updated_.store(false, std::memory_order_release);
      pitch_reference_updated_.store(false, std::memory_order_release);
      ResetAlgorithms();
    } else if (!control_active) {
      ResetAlgorithms();
#if GIMBAL_YAW_CTRL_MODE == CTRL_LADRC2
      reset_yaw_breakaway();
#endif
      system_identification.Reset();
      cruise_generator.Stop();
      cruise_active = false;
      cruise_mode_active_status_.store(false, std::memory_order_release);
      if (!homing_complete) homing_settle_candidate = false;
      yaw_manual_torque_active = false;
      pitch_manual_torque_active = false;
      yaw_command_applied = 0.0F;
      pitch_command_applied = 0.0F;
      yaw_stall_candidate = false;
      pitch_stall_candidate = false;
    }
    control_was_active = control_active;

    if (cruise_parameters_updated_.exchange(false,
                                             std::memory_order_acq_rel)) {
      cruise_generator.Configure(
          cruise_yaw_amplitude_rad_.load(std::memory_order_relaxed),
          cruise_yaw_frequency_hz_.load(std::memory_order_relaxed),
          cruise_pitch_amplitude_rad_.load(std::memory_order_relaxed),
          cruise_pitch_frequency_hz_.load(std::memory_order_relaxed));
      if (control_active && homing_complete && cruise_active) {
        cruise_generator.Start(Config::kCruiseYawCenterRad,
                               Config::kCruisePitchCenterRad, now);
        yaw_controller_.reset();
#if GIMBAL_YAW_CTRL_MODE == CTRL_LADRC2
        reset_yaw_breakaway();
#endif
        pitch_controller_.reset();
      }
    }

    const bool cruise_requested =
        requested_mode_.load(std::memory_order_acquire) ==
        static_cast<uint8_t>(GimbalMode::kCruise);
    if (control_active && homing_complete &&
        cruise_requested != cruise_active) {
      cruise_active = cruise_requested;
      yaw_manual_torque_active = false;
      pitch_manual_torque_active = false;
      yaw_stall_candidate = false;
      yaw_stall_protected = false;
#if GIMBAL_YAW_CTRL_MODE == CTRL_LADRC2
      reset_yaw_breakaway();
#endif
      pitch_stall_candidate = false;
      pitch_stall_protected = false;
      yaw_command_applied = 0.0F;
      pitch_command_applied = 0.0F;
      yaw_requested_target = yaw_feedback_angle;
      pitch_requested_target = pitch_feedback_angle;
      requested_yaw_angle_rad_.store(yaw_feedback_angle,
                                      std::memory_order_relaxed);
      requested_pitch_angle_rad_.store(pitch_feedback_angle,
                                        std::memory_order_relaxed);
      yaw_reference_updated_.store(false, std::memory_order_release);
      pitch_reference_updated_.store(false, std::memory_order_release);
      ResetAlgorithms();
      if (cruise_active) {
        cruise_generator.Start(Config::kCruiseYawCenterRad,
                               Config::kCruisePitchCenterRad, now);
      } else {
        cruise_generator.Stop();
      }
      cruise_mode_active_status_.store(cruise_active,
                                       std::memory_order_release);
    }

    if (control_active && homing_complete && !cruise_active &&
        yaw_reference_updated_.exchange(false, std::memory_order_acq_rel)) {
      const float requested_angle =
          requested_yaw_angle_rad_.load(std::memory_order_relaxed);
      yaw_target = yaw_feedback_angle;
      yaw_requested_target = requested_angle;
      if constexpr (Config::kEnableYawLimit) {
        yaw_requested_target = std::clamp(
            yaw_requested_target, Config::kYawMinRad, Config::kYawMaxRad);
      }
      yaw_manual_torque_active = false;
      yaw_stall_candidate = false;
      yaw_stall_protected = false;
#if GIMBAL_YAW_CTRL_MODE == CTRL_LADRC2
      reset_yaw_breakaway();
#endif
      yaw_controller_.reset();
    }
    if (control_active && homing_complete && !cruise_active &&
        pitch_reference_updated_.exchange(false, std::memory_order_acq_rel)) {
      if (pitch_manual_torque_active) pitch_target = pitch_feedback_angle;
      pitch_requested_target =
          requested_pitch_angle_rad_.load(std::memory_order_relaxed);
      pitch_manual_torque_active = false;
      pitch_stall_candidate = false;
      pitch_stall_protected = false;
      pitch_controller_.reset();
    }
    if (control_active) {
      if (cruise_active) {
        // Cruise is a reference source only; the selected feedback controller
        // below remains unchanged.
        const auto cruise_target = cruise_generator.Update(now);
        yaw_target = std::clamp(cruise_target.yaw_angle_rad,
                                Config::kYawMinRad, Config::kYawMaxRad);
        // Do not feed motion farther into a configured angle limit.
        if (yaw_target == cruise_target.yaw_angle_rad) {
          yaw_velocity_feedforward =
              cruise_target.yaw_velocity_rad_per_sec;
        }
        pitch_target = std::clamp(cruise_target.pitch_angle_rad,
                                  Config::kPitchMinRad,
                                  Config::kPitchMaxRad);
        yaw_reference_updated_.store(false, std::memory_order_release);
        pitch_reference_updated_.store(false, std::memory_order_release);
      } else {
        if constexpr (Config::kEnableYawLimit) {
          yaw_requested_target = std::clamp(
              yaw_requested_target, Config::kYawMinRad, Config::kYawMaxRad);
        }
        if constexpr (Config::kEnablePitchLimit) {
          pitch_requested_target = std::clamp(
              pitch_requested_target, Config::kPitchMinRad,
              Config::kPitchMaxRad);
        }
        yaw_target = yaw_requested_target;
        pitch_target = pitch_requested_target;
      }
    }
    if (control_active && !homing_complete) {
      yaw_reference_updated_.store(false, std::memory_order_release);
      pitch_reference_updated_.store(false, std::memory_order_release);
    }

    if (yaw_diagnostic_torque_requested_.exchange(
            false, std::memory_order_acq_rel)) {
      const float requested_torque =
          yaw_diagnostic_torque_nm_.load(std::memory_order_relaxed);
      yaw_manual_torque_active =
          control_active && homing_complete &&
          std::abs(requested_torque) > 0.0F;
      yaw_requested_target = yaw_feedback_angle;
      yaw_target = yaw_feedback_angle;
      requested_yaw_angle_rad_.store(yaw_feedback_angle,
                                      std::memory_order_relaxed);
      yaw_stall_candidate = false;
      yaw_stall_protected = false;
#if GIMBAL_YAW_CTRL_MODE == CTRL_LADRC2
      reset_yaw_breakaway();
#endif
      yaw_controller_.reset();
      yaw_command_applied = 0.0F;
    }
    if (pitch_diagnostic_torque_requested_.exchange(
            false, std::memory_order_acq_rel)) {
      const float requested_torque =
          pitch_diagnostic_torque_nm_.load(std::memory_order_relaxed);
      pitch_manual_torque_active =
          control_active && homing_complete &&
          std::abs(requested_torque) > 0.0F;
      pitch_requested_target = pitch_feedback_angle;
      pitch_target = pitch_feedback_angle;
      requested_pitch_angle_rad_.store(pitch_feedback_angle,
                                        std::memory_order_relaxed);
      pitch_stall_candidate = false;
      pitch_stall_protected = false;
      pitch_controller_.reset();
      pitch_command_applied = 0.0F;
    }
#if ENABLE_GIMBAL_SYS_ID
    // The identification waveform is the only permitted torque source.
    yaw_manual_torque_active = false;
    pitch_manual_torque_active = false;
#endif
    const bool yaw_diagnostic_active =
        control_active && yaw_manual_torque_active;
    const bool pitch_diagnostic_active =
        control_active && pitch_manual_torque_active;

    float yaw_command = 0.0F;
    float yaw_command_desired = 0.0F;
    float pitch_command_desired = 0.0F;
    float gravity_compensation = 0.0F;
    bool yaw_encoder_limit_active = false;
    bool pitch_encoder_limit_active = false;
#if ENABLE_GIMBAL_SYS_ID
    if (control_active) {
      // Open-loop identification: no homing, position/speed controller,
      // feedforward, friction compensation, gravity compensation, or
      // closed-loop holding torque is allowed on either axis. The selected
      // axis receives only the configured torque waveform; the other axis is
      // explicitly kept at zero torque.
#if GIMBAL_SYS_ID_AXIS == SYS_ID_AXIS_YAW
      yaw_command_desired = system_identification.Update(
          yaw_feedback_angle, pitch_feedback_angle, yaw_velocity);
      pitch_command_desired = 0.0F;
#else
      yaw_command_desired = 0.0F;
      pitch_command_desired = system_identification.Update(
          yaw_feedback_angle, pitch_feedback_angle, pitch_velocity);
#endif
    }
#else
    if (control_active && !homing_complete) {
      const float yaw_homing_error = WrapToPi(
          Config::kYawEncoderCenterPhaseRad - yaw_encoder_phase);
      const float pitch_homing_error =
          Config::kPitchEncoderCenterRad - pitch_motor.position_rad;
      const bool yaw_at_center =
          std::abs(yaw_homing_error) <= Config::kHomingEncoderToleranceRad;
      const bool pitch_at_center =
          std::abs(pitch_homing_error) <= Config::kHomingEncoderToleranceRad;

      constexpr float kControlPeriodSeconds =
          static_cast<float>(Config::kControlPeriodMs) / 1000.0F;
      yaw_controller_.algorithm_compute(0.0F, yaw_feedback_angle,
                                        yaw_velocity,
                                        kControlPeriodSeconds);
      yaw_command_desired = std::clamp(
          Config::kYawMotorCommandSign * yaw_controller_.out(),
          -Config::kYawMotorOutputLimitNm, Config::kYawMotorOutputLimitNm);

      gravity_compensation =
          Config::kPitchGravityW1 * std::sin(pitch_angle) +
          Config::kPitchGravityW2 * std::cos(pitch_angle) +
          Config::kPitchGravityW3;
      pitch_controller_.algorithm_compute(0.0F, pitch_feedback_angle,
                                          pitch_velocity,
                                          kControlPeriodSeconds);
      pitch_command_desired = std::clamp(
          Config::kPitchMotorCommandSign * pitch_controller_.out() +
              gravity_compensation,
          -Config::kPitchMotorNegativeOutputLimitNm,
          Config::kPitchMotorPositiveOutputLimitNm);

      const bool centers_are_stable =
          yaw_at_center && pitch_at_center &&
          std::abs(yaw_motor.velocity_rad_per_sec) <=
              Config::kHomingMotorSpeedThresholdRadPerSec &&
          std::abs(pitch_motor.velocity_rad_per_sec) <=
              Config::kHomingMotorSpeedThresholdRadPerSec;
      if (centers_are_stable) {
        if (!homing_settle_candidate) {
          homing_settle_start_ms = now;
          homing_settle_candidate = true;
        } else if (now - homing_settle_start_ms >=
                   Config::kHomingSettleTimeMs) {
          yaw_encoder_zero_phase = yaw_encoder_phase;
          pitch_zero_offset = pitch_angle;
          homing_complete = true;
          homing_complete_status_.store(true, std::memory_order_release);
          homing_settle_candidate = false;
          yaw_target = 0.0F;
          yaw_requested_target = 0.0F;
          pitch_target = 0.0F;
          pitch_requested_target = 0.0F;
          requested_yaw_angle_rad_.store(0.0F, std::memory_order_relaxed);
          requested_pitch_angle_rad_.store(0.0F, std::memory_order_relaxed);
          yaw_command_desired = 0.0F;
          pitch_command_desired = 0.0F;
          yaw_command_applied = 0.0F;
          pitch_command_applied = 0.0F;
          ResetAlgorithms();
#if GIMBAL_YAW_CTRL_MODE == CTRL_LADRC2
          reset_yaw_breakaway();
#endif
        }
      } else {
        homing_settle_candidate = false;
      }
    } else if (control_active) {
      gravity_compensation =
          Config::kPitchGravityW1 * std::sin(pitch_angle) +
          Config::kPitchGravityW2 * std::cos(pitch_angle) +
          Config::kPitchGravityW3;

      if (yaw_diagnostic_active) {
        yaw_command_desired = std::clamp(
            yaw_diagnostic_torque_nm_.load(std::memory_order_relaxed),
            -Config::kYawDiagnosticTorqueLimitNm,
            Config::kYawDiagnosticTorqueLimitNm);
      } else if (!yaw_stall_protected) {
#if GIMBAL_YAW_CTRL_MODE == CTRL_PID
        yaw_controller_.algorithm_compute(yaw_target, yaw_feedback_angle,
                                          yaw_velocity,
                                          static_cast<float>(Config::kControlPeriodMs) /
                                              1000.0F,
                                          yaw_velocity_feedforward *
                                              Config::kYawPidCruiseVelocityFeedforwardGain);
#elif GIMBAL_YAW_CTRL_MODE == CTRL_PID_LADRC1
        yaw_controller_.algorithm_compute_pid_ladrc1(
            yaw_target, yaw_feedback_angle, yaw_velocity,
            static_cast<float>(Config::kControlPeriodMs) / 1000.0F,
            yaw_velocity_feedforward *
                Config::kYawPidLadrc1CruiseVelocityFeedforwardGain);
#elif GIMBAL_YAW_CTRL_MODE == CTRL_LADRC2
        const float yaw_position_error = yaw_target - yaw_feedback_angle;

        if (yaw_breakaway_active) {
          const bool target_still_needs_same_direction =
              yaw_position_error * yaw_breakaway_direction >
              Config::kYawAngleDeadbandRad;
          if (!target_still_needs_same_direction) {
            reset_yaw_breakaway();
            yaw_controller_.reset();
          } else {
            const bool confirmed_motion_sample =
                std::abs(yaw_velocity) >=
                    Config::kYawLadrc2BreakawayReleaseGyroRadPerSec &&
                std::abs(yaw_motor.velocity_rad_per_sec) >=
                    Config::kYawLadrc2BreakawayReleaseMotorRadPerSec;
            if (confirmed_motion_sample) {
              if (!yaw_breakaway_motion_candidate) {
                yaw_breakaway_motion_candidate = true;
                yaw_breakaway_motion_start_ms = now;
              } else if (now - yaw_breakaway_motion_start_ms >=
                         Config::kYawLadrc2BreakawayMotionConfirmMs) {
                reset_yaw_breakaway();
                yaw_controller_.reset();
              }
            } else {
              yaw_breakaway_motion_candidate = false;
              constexpr float kControlPeriodSeconds =
                  static_cast<float>(Config::kControlPeriodMs) / 1000.0F;
              yaw_breakaway_torque_nm = std::min(
                  Config::kYawLadrc2BreakawayMaximumTorqueNm,
                  yaw_breakaway_torque_nm +
                      Config::kYawLadrc2BreakawayRampRateNmPerSec *
                          kControlPeriodSeconds);
            }
          }
        }

        if (!yaw_breakaway_active) {
          // The observer update covers the preceding sample, so use the motor
          // torque that was actually applied during that sample.
          yaw_controller_.set_ladrc2_observer_input(
              Config::kYawMotorCommandSign * yaw_command_applied);
          yaw_controller_.algorithm_compute_ladrc2(
              yaw_target, yaw_feedback_angle, yaw_velocity,
              static_cast<float>(Config::kControlPeriodMs) / 1000.0F);
        }
#else
#error "Invalid GIMBAL_YAW_CTRL_MODE"
#endif
        float yaw_control_output = yaw_controller_.out();
#if GIMBAL_YAW_CTRL_MODE == CTRL_LADRC2
        if (yaw_breakaway_active) {
          // Keep the observer clear while static-friction torque is forced;
          // otherwise a stationary shaft appears as false z2 velocity.
          yaw_controller_.reset();
          yaw_control_output = std::copysign(
              yaw_breakaway_torque_nm,
              yaw_breakaway_direction);
        } else if (std::abs(yaw_position_error) >
                       Config::kYawAngleDeadbandRad &&
                   std::abs(yaw_velocity) <
                       Config::kYawLadrc2BreakawaySpeedThresholdRadPerSec &&
                   std::abs(yaw_motor.velocity_rad_per_sec) <
                       Config::kYawLadrc2BreakawaySpeedThresholdRadPerSec) {
          yaw_breakaway_active = true;
          yaw_breakaway_motion_candidate = false;
          yaw_breakaway_direction = std::copysign(1.0F, yaw_position_error);
          yaw_breakaway_torque_nm = Config::kYawLadrc2BreakawayTorqueNm;
          yaw_controller_.reset();
          yaw_control_output =
              yaw_breakaway_direction * yaw_breakaway_torque_nm;
        }
        yaw_command_desired =
            Config::kYawMotorCommandSign * yaw_control_output;
#else
        const auto& yaw_debug = yaw_controller_.debug();
        if (std::abs(yaw_debug.position_error) >
                Config::kYawAngleDeadbandRad &&
            std::abs(yaw_debug.velocity_target) >
                Config::kYawSpeedDeadbandRadPerSec) {
          yaw_control_output += std::copysign(
              Config::kYawStaticFrictionCompensationNm,
              yaw_debug.velocity_target);
        }
        yaw_command_desired = EnforceMinimumMotionCommand(
            Config::kYawMotorCommandSign * yaw_control_output,
            yaw_debug.position_error, yaw_debug.velocity_target,
            Config::kYawMotorCommandSign, Config::kYawAngleDeadbandRad,
            Config::kYawSpeedDeadbandRadPerSec,
            Config::kYawStaticFrictionCompensationNm);
#endif
        yaw_command_desired = std::clamp(
            yaw_command_desired, -Config::kYawMotorOutputLimitNm,
            Config::kYawMotorOutputLimitNm);
      }
      if (pitch_diagnostic_active) {
        pitch_command_desired = std::clamp(
            pitch_diagnostic_torque_nm_.load(std::memory_order_relaxed),
            -Config::kPitchDiagnosticTorqueLimitNm,
            Config::kPitchDiagnosticTorqueLimitNm);
      } else if (!pitch_stall_protected) {
#if GIMBAL_PITCH_CTRL_MODE == CTRL_PID
        pitch_controller_.algorithm_compute(
            pitch_target, pitch_feedback_angle, pitch_velocity,
            static_cast<float>(Config::kControlPeriodMs) / 1000.0F);
#elif GIMBAL_PITCH_CTRL_MODE == CTRL_PID_LADRC1
        pitch_controller_.algorithm_compute_pid_ladrc1(
            pitch_target, pitch_feedback_angle, pitch_velocity,
            static_cast<float>(Config::kControlPeriodMs) / 1000.0F);
#elif GIMBAL_PITCH_CTRL_MODE == CTRL_LADRC2
        pitch_controller_.algorithm_compute_ladrc2(
            pitch_target, pitch_feedback_angle, pitch_velocity,
            static_cast<float>(Config::kControlPeriodMs) / 1000.0F);
#else
#error "Invalid GIMBAL_PITCH_CTRL_MODE"
#endif
        float pitch_control_output = pitch_controller_.out();
        const auto& pitch_debug = pitch_controller_.debug();
        if (std::abs(pitch_debug.position_error) >
                Config::kPitchAngleDeadbandRad &&
            std::abs(pitch_debug.velocity_target) >
                Config::kPitchSpeedDeadbandRadPerSec) {
          pitch_control_output += std::copysign(
              Config::kPitchStaticFrictionCompensationNm,
              pitch_debug.velocity_target);
        }
        pitch_command_desired = EnforceMinimumMotionCommand(
            Config::kPitchMotorCommandSign * pitch_control_output +
                gravity_compensation,
            pitch_debug.position_error, pitch_debug.velocity_target,
            Config::kPitchMotorCommandSign, Config::kPitchAngleDeadbandRad,
            Config::kPitchSpeedDeadbandRadPerSec,
            Config::kPitchStaticFrictionCompensationNm);
        pitch_command_desired = std::clamp(
            pitch_command_desired, -Config::kPitchMotorNegativeOutputLimitNm,
            Config::kPitchMotorPositiveOutputLimitNm);
      }
    }
#endif

    if constexpr (Config::kEnableYawEncoderLimit) {
      const float yaw_encoder_phase =
          WrapToTwoPi(yaw_motor.position_rad);
      // The calibrated valid arc crosses the encoder's 2*pi wrap. Phases in
      // the physically unreachable gap are divided at its midpoint so a
      // startup at either hard stop is classified correctly.
      const bool at_positive_stop =
          yaw_encoder_phase > Config::kYawEncoderPositiveSafePhaseRad &&
          yaw_encoder_phase < Config::kYawEncoderForbiddenSectorSplitRad;
      const bool at_negative_stop =
          yaw_encoder_phase < Config::kYawEncoderNegativeSafePhaseRad &&
          yaw_encoder_phase >= Config::kYawEncoderForbiddenSectorSplitRad;
      const bool pushes_positive_stop =
          at_positive_stop && yaw_command_desired > 0.0F;
      const bool pushes_negative_stop =
          at_negative_stop && yaw_command_desired < 0.0F;
      yaw_encoder_limit_active =
          pushes_positive_stop || pushes_negative_stop;
      if (yaw_encoder_limit_active) {
        yaw_command_desired = 0.0F;
        yaw_command_applied = 0.0F;
        yaw_controller_.reset();
#if GIMBAL_YAW_CTRL_MODE == CTRL_LADRC2
        reset_yaw_breakaway();
#endif
#if ENABLE_GIMBAL_SYS_ID && GIMBAL_SYS_ID_AXIS == SYS_ID_AXIS_YAW
        system_identification.AbortForSafety();
#endif
      }
    }

    if constexpr (Config::kEnablePitchEncoderLimit) {
      // Block only the motor direction that drives the encoder farther through
      // either calibrated safe boundary.
      const float pitch_encoder_drive =
          Config::kPitchEncoderCommandSign * pitch_command_desired;
      const bool pushes_below_encoder_min =
          pitch_motor.position_rad <= Config::kPitchEncoderMinRad &&
          pitch_encoder_drive < 0.0F;
      const bool pushes_above_encoder_max =
          pitch_motor.position_rad >= Config::kPitchEncoderMaxRad &&
          pitch_encoder_drive > 0.0F;
      pitch_encoder_limit_active =
          pushes_below_encoder_min || pushes_above_encoder_max;
      if (pitch_encoder_limit_active) {
        pitch_command_desired = 0.0F;
        pitch_command_applied = 0.0F;
        pitch_controller_.reset();
#if ENABLE_GIMBAL_SYS_ID && GIMBAL_SYS_ID_AXIS == SYS_ID_AXIS_PITCH
        system_identification.AbortForSafety();
#endif
      }
    }

    float pitch_command = 0.0F;
    if (control_active) {
      yaw_command_applied = yaw_command_desired;
      pitch_command_applied = pitch_command_desired;
      yaw_command_applied = std::clamp(
          yaw_command_applied, -Config::kYawMotorOutputLimitNm,
          Config::kYawMotorOutputLimitNm);
      pitch_command_applied = std::clamp(
          pitch_command_applied, -Config::kPitchMotorNegativeOutputLimitNm,
          Config::kPitchMotorPositiveOutputLimitNm);
      yaw_command = yaw_command_applied;
      pitch_command = pitch_command_applied;
    } else {
      yaw_command_applied = 0.0F;
      pitch_command_applied = 0.0F;
    }

    const bool yaw_is_stalled =
        control_active && homing_complete && !yaw_stall_protected &&
#if GIMBAL_YAW_CTRL_MODE == CTRL_LADRC2
        !yaw_breakaway_active &&
#endif
        std::abs(yaw_command) >= Config::kYawStallCommandThresholdNm &&
        std::abs(yaw_velocity) <= Config::kYawStallGyroThresholdRadPerSec &&
        std::abs(yaw_motor.velocity_rad_per_sec) <=
            Config::kYawStallMotorSpeedThresholdRadPerSec;
    if (yaw_is_stalled) {
      if (!yaw_stall_candidate) {
        yaw_stall_start_ms = now;
        yaw_stall_candidate = true;
      } else if (now - yaw_stall_start_ms >= Config::kYawStallTimeMs) {
        yaw_stall_protected = true;
        yaw_manual_torque_active = false;
        yaw_stall_candidate = false;
        yaw_requested_target = yaw_feedback_angle;
        yaw_target = yaw_feedback_angle;
        requested_yaw_angle_rad_.store(yaw_feedback_angle,
                                        std::memory_order_relaxed);
        yaw_controller_.reset();
#if GIMBAL_YAW_CTRL_MODE == CTRL_LADRC2
        reset_yaw_breakaway();
#endif
        yaw_command_applied = 0.0F;
        yaw_command = 0.0F;
#if ENABLE_GIMBAL_SYS_ID && GIMBAL_SYS_ID_AXIS == SYS_ID_AXIS_YAW
        system_identification.AbortForSafety();
#endif
      }
    } else {
      yaw_stall_candidate = false;
    }

    const bool pitch_is_stalled =
        control_active && homing_complete && !pitch_stall_protected &&
        std::abs(pitch_command) >= Config::kPitchStallCommandThresholdNm &&
        std::abs(pitch_velocity) <=
            Config::kPitchStallGyroThresholdRadPerSec &&
        std::abs(pitch_motor.velocity_rad_per_sec) <=
            Config::kPitchStallMotorSpeedThresholdRadPerSec;
    if (pitch_is_stalled) {
      if (!pitch_stall_candidate) {
        pitch_stall_start_ms = now;
        pitch_stall_candidate = true;
      } else if (now - pitch_stall_start_ms >= Config::kPitchStallTimeMs) {
        // Release a mechanical stop or other sustained stall immediately.
        // The next explicit target command clears the latch and can command
        // motion away from the stop.
        pitch_stall_protected = true;
        pitch_manual_torque_active = false;
        pitch_stall_candidate = false;
        pitch_requested_target = pitch_feedback_angle;
        pitch_target = pitch_feedback_angle;
        requested_pitch_angle_rad_.store(pitch_feedback_angle,
                                          std::memory_order_relaxed);
        pitch_controller_.reset();
        pitch_command_applied = 0.0F;
        pitch_command = 0.0F;
#if ENABLE_GIMBAL_SYS_ID && GIMBAL_SYS_ID_AXIS == SYS_ID_AXIS_PITCH
        system_identification.AbortForSafety();
#endif
      }
    } else {
      pitch_stall_candidate = false;
    }

#if ENABLE_GIMBAL_SYS_ID
    if (system_identification.Active()) {
      const float selected_input =
          GIMBAL_SYS_ID_AXIS == SYS_ID_AXIS_YAW ? yaw_command : pitch_command;
      const float selected_angle = GIMBAL_SYS_ID_AXIS == SYS_ID_AXIS_YAW
                                       ? yaw_feedback_angle
                                       : pitch_feedback_angle;
      const float selected_velocity = GIMBAL_SYS_ID_AXIS == SYS_ID_AXIS_YAW
                                          ? yaw_velocity
                                          : pitch_velocity;
      auto sys_id_telemetry = system_identification.Telemetry(
          selected_input, selected_angle, selected_velocity);
      sys_id_telemetry_topic_.Publish(sys_id_telemetry);
    }
#endif

    // Refresh both slots every tick. Zero commands keep the shared CAN frame
    // alive while disabled and prevent stale torque after an outage.
    yaw_command_topic_.Publish(yaw_command);
    pitch_command_topic_.Publish(pitch_command);

    auto yaw_telemetry =
        MakeAxisTelemetry(yaw_controller_.debug(), yaw_command, yaw_motor);
    auto pitch_telemetry = MakeAxisTelemetry(
        pitch_controller_.debug(), pitch_command, pitch_motor);
    yaw_telemetry.target_angle_rad = homing_complete ? yaw_target : 0.0F;
    yaw_telemetry.feedback_angle_rad =
        homing_complete
            ? yaw_encoder_center_angle
            : Config::kYawEncoderToVqfScale *
                  WrapToPi(yaw_encoder_phase -
                           Config::kYawEncoderCenterPhaseRad);
    pitch_telemetry.target_angle_rad = homing_complete ? pitch_target : 0.0F;
    pitch_telemetry.feedback_angle_rad =
        homing_complete
            ? pitch_angle - pitch_zero_offset
            : Config::kPitchEncoderToVqfScale *
                  (pitch_motor.position_rad - Config::kPitchEncoderCenterRad);

    GimbalControlTelemetry telemetry{
        .yaw = yaw_telemetry,
        .pitch = pitch_telemetry,
        .gyro_x_rad_per_sec = gyro_x,
        .gyro_y_rad_per_sec = gyro_y,
        .gyro_z_rad_per_sec = gyro_z,
        .yaw_continuous_angle_rad = yaw_continuous,
        .pitch_gravity_compensation_nm = gravity_compensation,
        .yaw_motor_feedback_fresh = static_cast<uint8_t>(yaw_feedback_fresh),
        .pitch_motor_feedback_fresh =
            static_cast<uint8_t>(pitch_feedback_fresh),
        .attitude_sample_valid = static_cast<uint8_t>(attitude_sample_valid),
        .attitude_fresh = static_cast<uint8_t>(attitude_fresh),
        .controller_enabled = static_cast<uint8_t>(enabled),
        .yaw_diagnostic_active =
            static_cast<uint8_t>(yaw_diagnostic_active),
        .pitch_diagnostic_active =
            static_cast<uint8_t>(pitch_diagnostic_active),
        .yaw_stall_protected = static_cast<uint8_t>(yaw_stall_protected),
        .pitch_stall_protected =
            static_cast<uint8_t>(pitch_stall_protected),
        .yaw_encoder_limit_active =
            static_cast<uint8_t>(yaw_encoder_limit_active),
        .pitch_encoder_limit_active =
            static_cast<uint8_t>(pitch_encoder_limit_active),
        .cruise_mode_active = static_cast<uint8_t>(cruise_active),
    };
    telemetry_topic_.Publish(telemetry);
  }
}

void GimbalControllerInit() {
  static GimbalController controller;
  controller_instance = &controller;
}

void GimbalControllerReset() {
  if (controller_instance != nullptr) controller_instance->Reset();
}

bool GimbalControllerSetEnabled(bool enabled) {
  if (controller_instance == nullptr) return false;
  controller_instance->SetEnabled(enabled);
  return true;
}

bool GimbalControllerSetYawAngleReference(float reference_rad) {
  if (controller_instance == nullptr || !controller_instance->IsHomed() ||
      !std::isfinite(reference_rad) ||
      std::abs(reference_rad) > Config::kYawAngleCommandLimitRad) {
    return false;
  }
  controller_instance->SetYawAngleReference(reference_rad);
  return true;
}

bool GimbalControllerSetPitchAngleReference(float reference_rad) {
  if (controller_instance == nullptr || !controller_instance->IsHomed() ||
      !std::isfinite(reference_rad) ||
      std::abs(reference_rad) > Config::kPitchAngleCommandLimitRad) {
    return false;
  }
  controller_instance->SetPitchAngleReference(reference_rad);
  return true;
}

bool GimbalControllerSetMode(GimbalMode mode) {
  if (controller_instance == nullptr ||
      (mode != GimbalMode::kNormal && mode != GimbalMode::kCruise)) {
    return false;
  }
  controller_instance->SetMode(mode);
  return true;
}

bool GimbalControllerConfigureCruise(float yaw_amplitude_rad,
                                     float yaw_frequency_hz,
                                     float pitch_amplitude_rad,
                                     float pitch_frequency_hz) {
  const bool values_are_valid =
      std::isfinite(yaw_amplitude_rad) &&
      std::isfinite(yaw_frequency_hz) &&
      std::isfinite(pitch_amplitude_rad) &&
      std::isfinite(pitch_frequency_hz) && yaw_amplitude_rad >= 0.0F &&
      yaw_amplitude_rad <= Config::kYawAngleCommandLimitRad &&
      pitch_amplitude_rad >= 0.0F &&
      pitch_amplitude_rad <= Config::kPitchAngleCommandLimitRad &&
      yaw_frequency_hz >= 0.0F &&
      yaw_frequency_hz <= Config::kCruiseMaximumFrequencyHz &&
      pitch_frequency_hz >= 0.0F &&
      pitch_frequency_hz <= Config::kCruiseMaximumFrequencyHz;
  if (controller_instance == nullptr || !values_are_valid) return false;

  controller_instance->ConfigureCruise(
      yaw_amplitude_rad, yaw_frequency_hz, pitch_amplitude_rad,
      pitch_frequency_hz);
  return true;
}

GimbalMode GimbalControllerGetRequestedMode() {
  return controller_instance == nullptr ? GimbalMode::kNormal
                                        : controller_instance->RequestedMode();
}

bool GimbalControllerCruiseModeActive() {
  return controller_instance != nullptr &&
         controller_instance->CruiseModeActive();
}

bool GimbalControllerRequestYawDiagnosticTorque(float torque_nm) {
  if (controller_instance == nullptr || !std::isfinite(torque_nm) ||
      std::abs(torque_nm) > Config::kYawDiagnosticTorqueLimitNm) {
    return false;
  }
  controller_instance->RequestYawDiagnosticTorque(torque_nm);
  return true;
}

bool GimbalControllerRequestPitchDiagnosticTorque(float torque_nm) {
  if (controller_instance == nullptr || !std::isfinite(torque_nm) ||
      std::abs(torque_nm) > Config::kPitchDiagnosticTorqueLimitNm) {
    return false;
  }
  controller_instance->RequestPitchDiagnosticTorque(torque_nm);
  return true;
}

bool GimbalControllerIsHomed() {
  return controller_instance != nullptr && controller_instance->IsHomed();
}
