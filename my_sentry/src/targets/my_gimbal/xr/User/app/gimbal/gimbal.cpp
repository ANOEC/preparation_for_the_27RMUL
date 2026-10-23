#include "app/gimbal/gimbal.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "config/gimbal/generated/gimbal_params.hpp"

namespace UserApp::Gimbal
{
namespace
{
constexpr float kTwoPi = 2.0f * std::numbers::pi_v<float>;
constexpr float kPitchUpLimit = 29.5f * std::numbers::pi_v<float> / 180.0f;
constexpr float kPitchDownLimit = -35.5f * std::numbers::pi_v<float> / 180.0f;
constexpr float kBigYawAlignmentRate = 1.0f;  // rad/s

template <typename Parameters>
PidParameters PositionParameters(const Parameters& p)
{
  return {.kp = p.P_kp,
          .ki = p.P_ki,
          .kd = p.P_kd,
          .deadband = p.P_error_threshold,
          .integral_limit = p.P_integral_limit,
          .output_limit = p.P_max_output};
}

template <typename Parameters>
PidParameters VelocityParameters(const Parameters& p)
{
  return {.kp = p.V_kp,
          .ki = p.V_ki,
          .kd = p.V_kd,
          .deadband = p.V_error_threshold,
          .integral_limit = p.V_integral_limit,
          .output_limit = p.V_max_output};
}
}  // namespace

Gimbal::Gimbal(uint32_t feedback_timeout_ms)
    : feedback_timeout_ms_(feedback_timeout_ms),
      yaw_subscriber_(LibXR::Topic::CreateTopic<DJIMotor::SolvedFeedback>(
          "/fb/motor/yaw")),
      big_yaw_subscriber_(LibXR::Topic::CreateTopic<DMMotor::SolvedFeedback>(
          "/fb/motor/big_yaw")),
      pitch_subscriber_(LibXR::Topic::CreateTopic<DMMotor::SolvedFeedback>(
          "/fb/motor/pitch")),
      imu_subscriber_(LibXR::Topic::CreateTopic<VqfOutput<float>>("vqf_output")),
      command_subscriber_(LibXR::Topic::CreateTopic<Remote::GimbalCommand>(
          "/cmd/gimbal")),
      yaw_torque_topic_(LibXR::Topic::CreateTopic<float>("/torque/yaw")),
      big_yaw_torque_topic_(LibXR::Topic::CreateTopic<DMMotor::ControlCommand>(
          "/torque/big_yaw")),
      pitch_torque_topic_(LibXR::Topic::CreateTopic<DMMotor::ControlCommand>(
          "/torque/pitch"))
{
  REQUIRE(feedback_timeout_ms_ > 0U);
  yaw_controller_.Configure(PositionParameters(param::gimbal::dual_pid::yaw),
                            VelocityParameters(param::gimbal::dual_pid::yaw),
                            param::gimbal::dual_pid::yaw.P_ff);
  big_yaw_controller_.Configure(
      PositionParameters(param::gimbal::dual_pid::big_yaw),
      VelocityParameters(param::gimbal::dual_pid::big_yaw),
      param::gimbal::dual_pid::big_yaw.P_ff);
  pitch_controller_.Configure(PositionParameters(param::gimbal::dual_pid::pitch),
                              VelocityParameters(param::gimbal::dual_pid::pitch),
                              param::gimbal::dual_pid::pitch.P_ff);

  yaw_subscriber_.StartWaiting();
  big_yaw_subscriber_.StartWaiting();
  pitch_subscriber_.StartWaiting();
  imu_subscriber_.StartWaiting();
  command_subscriber_.StartWaiting();
}

void Gimbal::Run(float dt_s, uint32_t now_ms)
{
  ReceiveFeedback(now_ms);
  ReceiveCommand(now_ms);

  const bool enabled = command_received_ && command_.enabled &&
                       (command_.state == Remote::ControlState::MANUAL ||
                        command_.state == Remote::ControlState::GAME);
  if (!FeedbackHealthy(now_ms) || !enabled ||
      TimedOut(now_ms, command_ms_, feedback_timeout_ms_))
  {
    EnterSafeState();
    PublishTorque(0.0f, 0.0f, 0.0f);
    return;
  }

  if (!targets_initialized_)
  {
    yaw_.target = yaw_.angle;
    big_yaw_.target = big_yaw_.angle;
    pitch_.target = pitch_.angle;
    big_yaw_target_offset_ = big_yaw_.angle - yaw_.angle;
    targets_initialized_ = true;
  }

  yaw_.target += command_.yaw_offset_rad;
  // Start from big yaw's measured angle, then smoothly merge it into the
  // reference project's shared yaw target. This is bumpless even when the two
  // encoders use different coordinate origins at the enable transition.
  const float max_alignment_step =
      kBigYawAlignmentRate * std::max(dt_s, 0.0f);
  big_yaw_target_offset_ -=
      std::clamp(big_yaw_target_offset_, -max_alignment_step,
                 max_alignment_step);
  big_yaw_.target = yaw_.target + big_yaw_target_offset_;
  pitch_.target += command_.pitch_offset_rad;

  // Preserve the reference project's encoder-assisted adaptive pitch stop.
  const float pitch_offset = -pitch_encoder_angle_ - pitch_.angle;
  pitch_.target = std::clamp(pitch_.target, kPitchDownLimit - pitch_offset,
                             kPitchUpLimit - pitch_offset);

  yaw_.torque = yaw_controller_.Compute(yaw_.target, yaw_.angle, yaw_.velocity,
                                        dt_s);
  big_yaw_.torque = big_yaw_controller_.Compute(
      big_yaw_.target, big_yaw_.angle, big_yaw_.velocity, dt_s);
  pitch_.torque = pitch_controller_.Compute(
      pitch_.target, pitch_.angle, pitch_.velocity, dt_s);
  pitch_.torque += param::gimbal::param.gravity_w1 * std::sin(pitch_.angle) +
                   param::gimbal::param.gravity_w2 * std::cos(pitch_.angle) +
                   param::gimbal::param.gravity_w3;
  pitch_.torque = std::clamp(pitch_.torque,
                             -param::gimbal::dual_pid::pitch.V_max_output,
                             param::gimbal::dual_pid::pitch.V_max_output);

  PublishTorque(yaw_.torque, big_yaw_.torque, pitch_.torque);
}

void Gimbal::ReceiveFeedback(uint32_t now_ms)
{
  if (yaw_subscriber_.Available())
  {
    const auto feedback = yaw_subscriber_.GetData();
    yaw_subscriber_.StartWaiting();
    yaw_motor_relative_angle_ = feedback.position_rad;
    yaw_motor_relative_velocity_ = feedback.velocity_rad_per_sec;
    yaw_feedback_ms_ = now_ms;
    yaw_received_ = true;
  }
  if (big_yaw_subscriber_.Available())
  {
    (void)big_yaw_subscriber_.GetData();
    big_yaw_subscriber_.StartWaiting();
    big_yaw_feedback_ms_ = now_ms;
    big_yaw_received_ = true;
  }
  if (pitch_subscriber_.Available())
  {
    const auto feedback = pitch_subscriber_.GetData();
    pitch_subscriber_.StartWaiting();
    pitch_encoder_angle_ = feedback.position_rad;
    pitch_feedback_ms_ = now_ms;
    pitch_received_ = true;
  }
  if (imu_subscriber_.Available())
  {
    const auto feedback = imu_subscriber_.GetData();
    imu_subscriber_.StartWaiting();
    UpdateAttitude(feedback);
    imu_feedback_ms_ = now_ms;
    imu_received_ = true;
  }

  big_yaw_.angle = yaw_.angle - yaw_motor_relative_angle_;
  big_yaw_.velocity = yaw_.velocity - yaw_motor_relative_velocity_;
}

void Gimbal::ReceiveCommand(uint32_t now_ms)
{
  if (!command_subscriber_.Available())
  {
    return;
  }
  command_ = command_subscriber_.GetData();
  command_subscriber_.StartWaiting();
  command_ms_ = now_ms;
  command_received_ = true;
}

void Gimbal::UpdateAttitude(const VqfOutput<float>& imu)
{
  const auto& q = imu.orientation_6d;
  const float w = q.w;
  const float x = q.x;
  const float y = q.y;
  const float z = q.z;
  const float sin_pitch = std::clamp(2.0f * (w * y - z * x), -1.0f, 1.0f);
  const float wrapped_yaw =
      std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
  const float raw_roll =
      std::atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
  const float raw_pitch = std::asin(sin_pitch);

  if (!yaw_initialized_)
  {
    previous_wrapped_yaw_ = wrapped_yaw;
    continuous_yaw_ = wrapped_yaw;
    yaw_initialized_ = true;
  }
  else
  {
    continuous_yaw_ += std::remainder(wrapped_yaw - previous_wrapped_yaw_, kTwoPi);
    previous_wrapped_yaw_ = wrapped_yaw;
  }

  const auto& gyro = imu.bias_corrected_angular_velocity_sensor_rad_per_second;
  const float shift = param::gimbal::imu.horizontal_shift *
                      std::numbers::pi_v<float> / 180.0f;
  // Match the reference project's horizontal IMU mounting correction.
  const float pitch = raw_roll * std::sin(shift) + raw_pitch * std::cos(shift);
  const float gyro_roll = gyro.x * std::cos(shift) - gyro.y * std::sin(shift);
  const float gyro_pitch = gyro.x * std::sin(shift) + gyro.y * std::cos(shift);
  yaw_.angle = continuous_yaw_;
  yaw_.velocity = gyro.z * std::cos(pitch) - gyro_roll * std::sin(pitch);
  pitch_.angle = pitch;
  pitch_.velocity = gyro_pitch;
}

void Gimbal::EnterSafeState()
{
  targets_initialized_ = false;
  yaw_controller_.Reset();
  big_yaw_controller_.Reset();
  pitch_controller_.Reset();
}

void Gimbal::PublishTorque(float yaw, float big_yaw, float pitch)
{
  yaw_torque_topic_.Publish(yaw);
  DMMotor::ControlCommand big_yaw_command{
      .kind = DMMotor::CommandKind::CONTROL,
      .torque_nm = big_yaw,
  };
  DMMotor::ControlCommand pitch_command{
      .kind = DMMotor::CommandKind::CONTROL,
      .torque_nm = pitch,
  };
  big_yaw_torque_topic_.Publish(big_yaw_command);
  pitch_torque_topic_.Publish(pitch_command);
}

bool Gimbal::FeedbackHealthy(uint32_t now_ms) const
{
  return yaw_received_ && big_yaw_received_ && pitch_received_ && imu_received_ &&
         !TimedOut(now_ms, yaw_feedback_ms_, feedback_timeout_ms_) &&
         !TimedOut(now_ms, big_yaw_feedback_ms_, feedback_timeout_ms_) &&
         !TimedOut(now_ms, pitch_feedback_ms_, feedback_timeout_ms_) &&
         !TimedOut(now_ms, imu_feedback_ms_, feedback_timeout_ms_);
}

bool Gimbal::TimedOut(uint32_t now_ms, uint32_t timestamp_ms, uint32_t timeout_ms)
{
  return static_cast<uint32_t>(now_ms - timestamp_ms) > timeout_ms;
}

}  // namespace UserApp::Gimbal
