#include "vofa_telemetry.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <numbers>

#include "DJIMotor.hpp"
#include "VQFModule.hpp"
#include "config/app_config.hpp"
#include "control/gimbal_controller.hpp"
#include "libxr.hpp"
#include "message.hpp"

namespace {
using ImuData = Eigen::Matrix<float, 3, 1>;
using ImuMessage = LibXR::Topic::Message<ImuData>;
using AttitudeMessage = LibXR::Topic::Message<VqfOutput<float>>;
using MotorMessage = LibXR::Topic::Message<DJIMotor::SolvedFeedback>;
using ControlMessage = LibXR::Topic::Message<GimbalControlTelemetry>;
using SysIdMessage = LibXR::Topic::Message<GimbalSysIdTelemetry>;

constexpr float kRadiansToDegrees = 180.0F / std::numbers::pi_v<float>;
LibXR::Thread vofa_thread;

template <typename Message>
bool DrainLatest(LibXR::SPSCQueue<Message>& queue, Message& latest) {
  bool updated = false;
  Message sample{};
  while (queue.Pop(sample) == LibXR::ErrorCode::OK) {
    latest = sample;
    updated = true;
  }
  return updated;
}

void TelemetryTask(void*) {
  LibXR::SPSCQueue<ImuMessage> gyro_queue(64);
  LibXR::SPSCQueue<ImuMessage> accel_queue(64);
  LibXR::SPSCQueue<AttitudeMessage> attitude_queue(64);
  LibXR::SPSCQueue<MotorMessage> pitch_motor_queue(8);
  LibXR::SPSCQueue<MotorMessage> yaw_motor_queue(8);
  LibXR::SPSCQueue<ControlMessage> control_queue(32);
  LibXR::SPSCQueue<SysIdMessage> sys_id_queue(32);

  LibXR::Topic gyro_topic =
      LibXR::Topic::CreateTopic<ImuData>("bmi088_gyro");
  LibXR::Topic accel_topic =
      LibXR::Topic::CreateTopic<ImuData>("bmi088_accl");
  LibXR::Topic attitude_topic =
      LibXR::Topic::CreateTopic<VqfOutput<float>>("vqf_output");
  LibXR::Topic pitch_motor_topic =
      LibXR::Topic::CreateTopic<DJIMotor::SolvedFeedback>(
          "pitch_motor_feedback");
  LibXR::Topic yaw_motor_topic =
      LibXR::Topic::CreateTopic<DJIMotor::SolvedFeedback>(
          "yaw_motor_feedback");
  LibXR::Topic control_topic =
      LibXR::Topic::CreateTopic<GimbalControlTelemetry>("gimbal_control");
  LibXR::Topic sys_id_topic =
      LibXR::Topic::CreateTopic<GimbalSysIdTelemetry>("gimbal_sys_id");

  LibXR::Topic::QueuedSubscriber gyro(gyro_topic, gyro_queue);
  LibXR::Topic::QueuedSubscriber accel(accel_topic, accel_queue);
  LibXR::Topic::QueuedSubscriber attitude(attitude_topic, attitude_queue);
  LibXR::Topic::QueuedSubscriber pitch_motor(pitch_motor_topic,
                                              pitch_motor_queue);
  LibXR::Topic::QueuedSubscriber yaw_motor(yaw_motor_topic, yaw_motor_queue);
  LibXR::Topic::QueuedSubscriber control(control_topic, control_queue);
  LibXR::Topic::QueuedSubscriber sys_id(sys_id_topic, sys_id_queue);

  ImuMessage gyro_latest{};
  ImuMessage accel_latest{};
  AttitudeMessage attitude_latest{};
  MotorMessage pitch_motor_latest{};
  MotorMessage yaw_motor_latest{};
  ControlMessage control_latest{};
  SysIdMessage sys_id_latest{};
  attitude_latest.data.orientation_6d.w = 1.0F;
  auto wake_time = LibXR::Timebase::GetMilliseconds();

  while (true) {
    constexpr uint32_t kTelemetryPeriodMs =
#if ENABLE_GIMBAL_SYS_ID
        AppConfig::Telemetry::kSysIdPeriodMs;
#else
        AppConfig::Telemetry::kPeriodMs;
#endif
    LibXR::Thread::SleepUntil(wake_time, kTelemetryPeriodMs);
    DrainLatest(gyro_queue, gyro_latest);
    DrainLatest(accel_queue, accel_latest);
    DrainLatest(attitude_queue, attitude_latest);
    DrainLatest(pitch_motor_queue, pitch_motor_latest);
    DrainLatest(yaw_motor_queue, yaw_motor_latest);
    DrainLatest(control_queue, control_latest);
    [[maybe_unused]] const bool sys_id_updated =
        DrainLatest(sys_id_queue, sys_id_latest);

    if constexpr (!AppConfig::Telemetry::kEnable) continue;

#if ENABLE_GIMBAL_SYS_ID
    // Exactly four VOFA channels for the axis selected by GIMBAL_SYS_ID_AXIS:
    // timestamp_s, command_Nm, velocity_rad_s, angle_rad.
    if (sys_id_updated) {
      LibXR::STDIO::Printf<"gimbal_sys_id:%.6f,%.6f,%.6f,%.6f\r\n">(
          sys_id_latest.data.time_s, sys_id_latest.data.input_nm,
          sys_id_latest.data.angular_velocity_rad_per_sec,
          sys_id_latest.data.angle_rad);
    }
    continue;
#endif

    if constexpr (AppConfig::Telemetry::kMode ==
                  AppConfig::Telemetry::Mode::kImu) {
      LibXR::STDIO::Printf<"imu:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n">(
          gyro_latest.data.x(), gyro_latest.data.y(), gyro_latest.data.z(),
          accel_latest.data.x(), accel_latest.data.y(), accel_latest.data.z());
    } else if constexpr (AppConfig::Telemetry::kMode ==
                         AppConfig::Telemetry::Mode::kAttitude) {
      const auto& q = attitude_latest.data.orientation_6d;
      const auto euler =
          LibXR::Quaternion<float>(q.w, q.x, q.y, q.z).ToEulerAngleZYX();
      LibXR::STDIO::Printf<"attitude:%.3f,%.3f,%.3f\r\n">(
          euler.x() * kRadiansToDegrees, euler.y() * kRadiansToDegrees,
          euler.z() * kRadiansToDegrees);
    } else if constexpr (AppConfig::Telemetry::kMode ==
                         AppConfig::Telemetry::Mode::kMotor) {
      LibXR::STDIO::Printf<"motor:%.3f,%.3f,%.3f,%.3f\r\n">(
          pitch_motor_latest.data.position_rad,
          pitch_motor_latest.data.velocity_rad_per_sec,
          yaw_motor_latest.data.position_rad,
          yaw_motor_latest.data.velocity_rad_per_sec);
    } else if constexpr (AppConfig::Telemetry::kMode ==
                         AppConfig::Telemetry::Mode::kAll) {
      const auto& q = attitude_latest.data.orientation_6d;
      const auto euler =
          LibXR::Quaternion<float>(q.w, q.x, q.y, q.z).ToEulerAngleZYX();
      LibXR::STDIO::Printf<
          "data:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n">(
          gyro_latest.data.x(), gyro_latest.data.y(), gyro_latest.data.z(),
          accel_latest.data.x(), accel_latest.data.y(), accel_latest.data.z(),
          euler.x(), euler.y(), euler.z(),
          pitch_motor_latest.data.position_rad,
          pitch_motor_latest.data.velocity_rad_per_sec,
          yaw_motor_latest.data.position_rad,
          yaw_motor_latest.data.velocity_rad_per_sec);
    } else if constexpr (AppConfig::Telemetry::kMode ==
                         AppConfig::Telemetry::Mode::kYawCalibration) {
      const auto& yaw_axis = control_latest.data.yaw;
      LibXR::STDIO::Printf<
          "yaw_cal:%.5f,%.5f,%.5f,%.5f,%.5f,%u,%u\r\n">(
          control_latest.data.yaw_continuous_angle_rad,
          yaw_axis.motor_encoder_angle_rad,
          control_latest.data.gyro_z_rad_per_sec,
          yaw_axis.motor_reported_speed_rad_per_sec,
          yaw_axis.final_command_nm,
           static_cast<unsigned>(control_latest.data.yaw_motor_feedback_fresh),
           static_cast<unsigned>(control_latest.data.attitude_fresh));
    } else if constexpr (AppConfig::Telemetry::kMode ==
                         AppConfig::Telemetry::Mode::kPitchCalibration) {
      const auto& q = attitude_latest.data.orientation_6d;
      const auto euler =
          LibXR::Quaternion<float>(q.w, q.x, q.y, q.z).ToEulerAngleZYX();
      const auto& pitch_axis = control_latest.data.pitch;
      // Columns: VQF pitch, motor encoder, gyro Y, motor speed, command,
      // motor feedback fresh, attitude fresh. Angles are in rad.
      LibXR::STDIO::Printf<
          "pitch_cal:%.6f,%.6f,%.6f,%.6f,%.6f,%u,%u\r\n">(
          euler.y(), pitch_axis.motor_encoder_angle_rad,
          control_latest.data.gyro_y_rad_per_sec,
          pitch_axis.motor_reported_speed_rad_per_sec,
          pitch_axis.final_command_nm,
          static_cast<unsigned>(
              control_latest.data.pitch_motor_feedback_fresh),
          static_cast<unsigned>(control_latest.data.attitude_fresh));
    } else if constexpr (AppConfig::Telemetry::kMode ==
                         AppConfig::Telemetry::Mode::kControl) {
      const auto& yaw_axis = control_latest.data.yaw;
      const auto& pitch_axis = control_latest.data.pitch;
      // Keep the original 14 channels stable. The final two diagnostic
      // channels are the measured Yaw IMU and motor speeds (rad/s), used to
      // verify the LADRC<2> breakaway state independently of observer z2.
      LibXR::STDIO::Printf<
          "gimbal_ctrl:%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\r\n">(
          pitch_axis.target_angle_rad * kRadiansToDegrees,
          pitch_axis.feedback_angle_rad * kRadiansToDegrees,
          yaw_axis.target_angle_rad * kRadiansToDegrees,
          yaw_axis.feedback_angle_rad * kRadiansToDegrees,
          yaw_axis.algorithm_output_nm * 1000.0F, yaw_axis.final_command_nm * 1000.0F,
          yaw_axis.ladrc_z1, yaw_axis.ladrc_z2, yaw_axis.ladrc_z3,
          pitch_axis.algorithm_output_nm, pitch_axis.final_command_nm,
          pitch_axis.ladrc_z1, pitch_axis.ladrc_z2, pitch_axis.ladrc_z3,
          AppConfig::Gimbal::kYawGyroSign *
              control_latest.data.gyro_z_rad_per_sec,
          yaw_axis.motor_reported_speed_rad_per_sec);
    }
  }
}
}  // namespace

void VofaTelemetryInit() {
  vofa_thread.Create(static_cast<void*>(nullptr), TelemetryTask,
                     "vofa_telemetry", 3072, LibXR::Thread::Priority::LOW);
}
