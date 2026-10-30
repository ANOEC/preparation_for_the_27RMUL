#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>

#include "DJIMotor.hpp"
#include "VQFModule.hpp"
#include "app/chassis/chassis.hpp"
#include "app/chassis/chassis_config.hpp"
#include "app/chassis/reference_coordinates.hpp"
#include "app/communication/b2b/chassis_command.hpp"
#include "task_generated.hpp"
#include "timebase.hpp"
#include "topic.hpp"

namespace
{
using ChassisCommand = UserApp::Command::ChassisCommand;
using MotorFeedback = DJIMotor::SolvedFeedback;
using ImuOutput = VqfOutput<float>;

constexpr std::uint32_t kCommandTimeoutMs = 100U;
constexpr std::uint32_t kMotorFeedbackTimeoutMs = 100U;
constexpr std::uint32_t kImuTimeoutMs = 100U;
constexpr float kMinimumQuaternionNormSquared = 1.0e-6f;

template <typename Data>
struct InputSample
{
  Data value{};
  std::uint32_t last_receive_ms = 0U;
  bool received = false;
};

class ChassisInputAdapter
{
 public:
  ChassisInputAdapter()
      : command_sub_(
            LibXR::Topic::CreateTopic<ChassisCommand>("/cmd/chassis")),
        motor_1_sub_(LibXR::Topic::CreateTopic<MotorFeedback>(
            "/fb/motor/chassis_motor1")),
        motor_2_sub_(LibXR::Topic::CreateTopic<MotorFeedback>(
            "/fb/motor/chassis_motor2")),
        motor_3_sub_(LibXR::Topic::CreateTopic<MotorFeedback>(
            "/fb/motor/chassis_motor3")),
        motor_4_sub_(LibXR::Topic::CreateTopic<MotorFeedback>(
            "/fb/motor/chassis_motor4")),
        imu_sub_(LibXR::Topic::CreateTopic<ImuOutput>("vqf_output"))
  {
    command_sub_.StartWaiting();
    motor_1_sub_.StartWaiting();
    motor_2_sub_.StartWaiting();
    motor_3_sub_.StartWaiting();
    motor_4_sub_.StartWaiting();
    imu_sub_.StartWaiting();
  }

  void Poll(std::uint32_t now_ms)
  {
    PollOne(command_sub_, command_, now_ms);
    PollOne(motor_1_sub_, motors_[0], now_ms);
    PollOne(motor_2_sub_, motors_[1], now_ms);
    PollOne(motor_3_sub_, motors_[2], now_ms);
    PollOne(motor_4_sub_, motors_[3], now_ms);
    PollOne(imu_sub_, imu_, now_ms);
  }

  [[nodiscard]] app::chassis::Chassis::Command MakeCommand(
      std::uint32_t now_ms) const
  {
    app::chassis::Chassis::Command output{};
    const auto& input = command_.value;
    if (!CommandOnline(now_ms) || !input.enabled ||
        !input.big_yaw_alive || !input.yaw_alive ||
        (input.state != UserApp::Command::ControlState::HAND &&
         input.state != UserApp::Command::ControlState::GAME))
    {
      return output;
    }

    const bool game_mode =
        input.state == UserApp::Command::ControlState::GAME;
    const float translation_deviation =
        app::chassis::ReferenceTranslationDeviation(
            input.big_yaw_angle_rad, input.yaw_angle_rad, game_mode);
    const auto body_translation = app::chassis::ReferenceTranslationToBody(
        input.vx_mps, input.vy_mps, translation_deviation);
    output.vx = body_translation.vx;
    output.vy = body_translation.vy;
    output.wz = input.wz_rad_per_sec;
    output.mode = ConvertMode(input.mode);
    output.world_frame = false;
    // FOLLOW uses the negative relative angle; measured big-yaw velocity is
    // added as a same-direction feed-forward term by the chassis controller.
    output.follow_yaw_error =
        app::chassis::ReferenceFollowYawError(input.big_yaw_angle_rad);
    output.follow_yaw_velocity = input.big_yaw_velocity_rad_per_sec;
    return output;
  }

  [[nodiscard]] app::chassis::Chassis::Feedback MakeFeedback(
      std::uint32_t now_ms) const
  {
    app::chassis::Chassis::Feedback output{};
    output.command_link_online = CommandOnline(now_ms);

    for (std::size_t index = 0; index < motors_.size(); ++index)
    {
      const auto& input = motors_[index];
      const bool online = IsFresh(input, now_ms, kMotorFeedbackTimeoutMs) &&
                          std::isfinite(input.value.velocity_rad_per_sec) &&
                          std::isfinite(input.value.torque_nm);
      output.wheels[index].online = online;
      if (online)
      {
        output.wheels[index].motor_speed = input.value.velocity_rad_per_sec;
        output.wheels[index].motor_torque = input.value.torque_nm;
      }
    }

    FillImuFeedback(output, now_ms);
    return output;
  }

 private:
  template <typename Data>
  static bool IsFresh(const InputSample<Data>& sample, std::uint32_t now_ms,
                      std::uint32_t timeout_ms)
  {
    return sample.received &&
           static_cast<std::uint32_t>(now_ms - sample.last_receive_ms) <=
               timeout_ms;
  }

  [[nodiscard]] bool CommandOnline(std::uint32_t now_ms) const
  {
    const auto& input = command_.value;
    return IsFresh(command_, now_ms, kCommandTimeoutMs) &&
           input.state != UserApp::Command::ControlState::OFFLINE &&
           std::isfinite(input.vx_mps) && std::isfinite(input.vy_mps) &&
           std::isfinite(input.wz_rad_per_sec) &&
           std::isfinite(input.big_yaw_angle_rad) &&
           std::isfinite(input.big_yaw_velocity_rad_per_sec) &&
           std::isfinite(input.yaw_angle_rad);
  }

  static app::chassis::ChassisMode ConvertMode(
      UserApp::Command::ChassisMode mode)
  {
    switch (mode)
    {
      case UserApp::Command::ChassisMode::PATROL:
        // The left-switch DOWN position starts gimbal patrol, but the
        // reference chassis remains FREE instead of chasing that scan motion.
        return app::chassis::ChassisMode::Free;
      case UserApp::Command::ChassisMode::FOLLOW:
        return app::chassis::ChassisMode::Follow;
      case UserApp::Command::ChassisMode::SPIN:
        return app::chassis::ChassisMode::Spin;
      default:
        return app::chassis::ChassisMode::Protect;
    }
  }

  void FillImuFeedback(app::chassis::Chassis::Feedback& output,
                       std::uint32_t now_ms) const
  {
    if (!IsFresh(imu_, now_ms, kImuTimeoutMs))
    {
      return;
    }

    const auto& input = imu_.value;
    const auto& acceleration =
        input.acceleration_imu_meters_per_second_squared;
    const auto& gyro =
        input.bias_corrected_angular_velocity_sensor_rad_per_second;
    const auto& orientation = input.orientation_6d;
    if (!std::isfinite(acceleration.x) || !std::isfinite(acceleration.y) ||
        !std::isfinite(acceleration.z) || !std::isfinite(gyro.x) ||
        !std::isfinite(gyro.y) || !std::isfinite(gyro.z) ||
        !std::isfinite(orientation.w) || !std::isfinite(orientation.x) ||
        !std::isfinite(orientation.y) || !std::isfinite(orientation.z))
    {
      return;
    }

    const float norm_squared = orientation.w * orientation.w +
                               orientation.x * orientation.x +
                               orientation.y * orientation.y +
                               orientation.z * orientation.z;
    if (norm_squared < kMinimumQuaternionNormSquared)
    {
      return;
    }

    const float inverse_norm = 1.0f / std::sqrt(norm_squared);
    const float w = orientation.w * inverse_norm;
    const float x = orientation.x * inverse_norm;
    const float y = orientation.y * inverse_norm;
    const float z = orientation.z * inverse_norm;

    output.imu.acceleration = {acceleration.x, acceleration.y, acceleration.z};
    output.imu.gyro = {gyro.x, gyro.y, gyro.z};
    output.imu.online = true;

    output.roll = std::atan2(2.0f * (w * x + y * z),
                             1.0f - 2.0f * (x * x + y * y));
    output.pitch = std::asin(
        std::clamp(2.0f * (w * y - z * x), -1.0f, 1.0f));
    output.chassis_yaw = std::atan2(2.0f * (w * z + x * y),
                                    1.0f - 2.0f * (y * y + z * z));
  }

  template <typename Data>
  static void PollOne(LibXR::Topic::ASyncSubscriber<Data>& subscriber,
                      InputSample<Data>& sample, std::uint32_t now_ms)
  {
    if (!subscriber.Available())
    {
      return;
    }

    sample.value = subscriber.GetData();
    sample.last_receive_ms = now_ms;
    sample.received = true;
    subscriber.StartWaiting();
  }

  LibXR::Topic::ASyncSubscriber<ChassisCommand> command_sub_;
  LibXR::Topic::ASyncSubscriber<MotorFeedback> motor_1_sub_;
  LibXR::Topic::ASyncSubscriber<MotorFeedback> motor_2_sub_;
  LibXR::Topic::ASyncSubscriber<MotorFeedback> motor_3_sub_;
  LibXR::Topic::ASyncSubscriber<MotorFeedback> motor_4_sub_;
  LibXR::Topic::ASyncSubscriber<ImuOutput> imu_sub_;

  InputSample<ChassisCommand> command_{};
  std::array<InputSample<MotorFeedback>, 4> motors_{};
  InputSample<ImuOutput> imu_{};
};

class ChassisTorquePublisher
{
 public:
  ChassisTorquePublisher()
      : motor_1_topic_(
            LibXR::Topic::CreateTopic<float>("/torque/chassis_motor1")),
        motor_2_topic_(
            LibXR::Topic::CreateTopic<float>("/torque/chassis_motor2")),
        motor_3_topic_(
            LibXR::Topic::CreateTopic<float>("/torque/chassis_motor3")),
        motor_4_topic_(
            LibXR::Topic::CreateTopic<float>("/torque/chassis_motor4"))
  {
  }

  void Publish(const app::chassis::Chassis::Output& output)
  {
    auto torque = output.motor_torque;
    for (auto& value : torque)
    {
      if (!std::isfinite(value))
      {
        value = 0.0f;
      }
    }

    motor_1_topic_.Publish(torque[0]);
    motor_2_topic_.Publish(torque[1]);
    motor_3_topic_.Publish(torque[2]);
    motor_4_topic_.Publish(torque[3]);
  }

 private:
  LibXR::Topic motor_1_topic_;
  LibXR::Topic motor_2_topic_;
  LibXR::Topic motor_3_topic_;
  LibXR::Topic motor_4_topic_;
};

app::chassis::Chassis chassis_app{app::chassis::MakeChassisConfig()};
}

void generated::task::TaskHooks<generated::task::chassis_task_tag>::loop(
    const generated::task::LoopContext& context)
{
  static ChassisInputAdapter inputs;
  static ChassisTorquePublisher torque_publisher;

  const auto now_ms =
      static_cast<std::uint32_t>(LibXR::Timebase::GetMilliseconds());
  inputs.Poll(now_ms);

  chassis_app.set_command(inputs.MakeCommand(now_ms));
  chassis_app.set_feedback(inputs.MakeFeedback(now_ms));

  const auto& output = chassis_app.update(context.dt_s);
  torque_publisher.Publish(output);

  if (context.daemon_service != nullptr)
  {
    context.daemon_service->feed();
  }
}
