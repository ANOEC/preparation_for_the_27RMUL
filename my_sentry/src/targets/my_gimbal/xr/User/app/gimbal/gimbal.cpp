#include "app/gimbal/gimbal.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "app/debug/vofa_plus_libxr.hpp"
#include "config/gimbal/generated/gimbal_params.hpp"

namespace UserApp::Gimbal
{
namespace
{
constexpr float kTwoPi = 2.0f * std::numbers::pi_v<float>;
constexpr float kPitchUpLimit = 29.5f * std::numbers::pi_v<float> / 180.0f;
constexpr float kPitchDownLimit = -8.5f * std::numbers::pi_v<float> / 180.0f;
constexpr float kBigYawAlignmentRate = 1.0f;  // rad/s
constexpr float kDegreesToRadians = std::numbers::pi_v<float> / 180.0f;

template <typename Parameters>
PidParameters PositionParameters(const Parameters& p)
{
  return {.kp = p.P_kp,
          .ki = p.P_ki,
          .kd = p.P_kd,
          .error_band = p.P_error_threshold,
          .output_limit = p.P_max_output,
          .integral_limit = p.P_integral_limit};
}

template <typename Parameters>
PidParameters VelocityParameters(const Parameters& p)
{
  return {.kp = p.V_kp,
          .ki = p.V_ki,
          .kd = p.V_kd,
          .error_band = p.V_error_threshold,
          .output_limit = p.V_max_output,
          .integral_limit = p.V_integral_limit};
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
  REQUIRE(param::gimbal::velocity_test.axis >= 0 &&
          param::gimbal::velocity_test.axis <= 2);
  REQUIRE(param::gimbal::velocity_test.telemetry_period_ms > 0);
  REQUIRE(!(param::gimbal::torque_test.enabled &&
            param::gimbal::velocity_test.enabled));
  REQUIRE(!(param::gimbal::gravity_test.enabled &&
            param::gimbal::torque_test.enabled));
  REQUIRE(!(param::gimbal::gravity_test.enabled &&
            param::gimbal::velocity_test.enabled));
  REQUIRE(param::gimbal::torque_test.max_abs_torque_nm > 0.0f);
  REQUIRE(param::gimbal::torque_test.torque_slew_rate_nm_per_s > 0.0f);
  REQUIRE(param::gimbal::torque_test.input_timeout_ms >= 0);
  REQUIRE(param::gimbal::torque_test.telemetry_period_ms > 0);
  REQUIRE(param::gimbal::torque_test.limit_margin_deg >= 0.0f);
  REQUIRE(param::gimbal::gravity_test.telemetry_period_ms > 0);
  REQUIRE(param::gimbal::position_telemetry.period_ms > 0);
  REQUIRE(velocity_test_wave_.setParam(
              param::gimbal::velocity_test.frequency_hz,
              param::gimbal::velocity_test.amplitude_rad_per_sec,
              param::gimbal::velocity_test.offset_rad_per_sec) ==
          Wave::ErrorCode::OK);

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
  const bool feedback_healthy = FeedbackHealthy(now_ms);
  if (param::gimbal::torque_test.enabled)
  {
    ReceiveTorqueTestCommand(now_ms, feedback_healthy && enabled);
  }

  if (!feedback_healthy || !enabled ||
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

  if (param::gimbal::torque_test.enabled)
  {
    RunTorqueTest(dt_s, now_ms);
    return;
  }

  if (param::gimbal::velocity_test.enabled)
  {
    RunVelocityTest(dt_s, now_ms);
    return;
  }
  velocity_test_running_ = false;

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
  const float gravity_torque = PitchGravityTorque();
  if (param::gimbal::gravity_test.enabled)
  {
    // Keep the position loop bypassed. The zero-speed loop only supplies
    // damping; gravity feedforward carries the static load.
    pitch_.torque = pitch_controller_.ComputeVelocity(
        0.0f, pitch_.velocity, dt_s);
    pitch_.torque += gravity_torque;
  }
  else
  {
    pitch_.torque = pitch_controller_.Compute(
        pitch_.target, pitch_.angle, pitch_.velocity, dt_s);
    pitch_.torque += gravity_torque;
  }
  pitch_.torque = std::clamp(pitch_.torque,
                             -param::gimbal::dual_pid::pitch.V_max_output,
                             param::gimbal::dual_pid::pitch.V_max_output);

  PublishTorque(yaw_.torque, big_yaw_.torque, pitch_.torque);
  if (param::gimbal::gravity_test.enabled)
  {
    SendGravityTestTelemetry(now_ms, pitch_.torque);
  }
  else if (param::gimbal::position_telemetry.enabled)
  {
    SendPitchPositionTelemetry(now_ms);
  }
}

void Gimbal::ReceiveTorqueTestCommand(uint32_t now_ms, bool accept_command)
{
  auto* vofa = UserApp::Debug::GetVofaPlus();
  if (vofa == nullptr)
  {
    return;
  }

  float requested_torque = 0.0f;
  const auto result = vofa->Receive(&requested_torque, 1);
  if (result != debug::vofa_plus::VofaPlus::Result::OK)
  {
    return;
  }

  if (!accept_command || !std::isfinite(requested_torque))
  {
    torque_test_requested_nm_ = 0.0f;
    torque_test_command_received_ = false;
    return;
  }

  torque_test_requested_nm_ =
      std::clamp(requested_torque,
                 -param::gimbal::torque_test.max_abs_torque_nm,
                 param::gimbal::torque_test.max_abs_torque_nm);
  torque_test_command_ms_ = now_ms;
  torque_test_command_received_ = true;
}

void Gimbal::RunTorqueTest(float dt_s, uint32_t now_ms)
{
  const bool watchdog_disabled =
      param::gimbal::torque_test.input_timeout_ms == 0;
  const bool input_fresh = torque_test_command_received_ &&
                           (watchdog_disabled ||
                            !TimedOut(now_ms, torque_test_command_ms_,
                                      static_cast<uint32_t>(
                                          param::gimbal::torque_test
                                              .input_timeout_ms)));

  // Normal pitch target limiting is encoder based. Apply the same mechanical
  // convention here, with an extra margin because raw torque has no position
  // controller to stop it at the boundary.
  const float margin =
      param::gimbal::torque_test.limit_margin_deg * kDegreesToRadians;
  const float encoder_lower_limit = -kPitchUpLimit + margin;
  const float encoder_upper_limit = -kPitchDownLimit - margin;
  const bool inside_limits = pitch_encoder_angle_ > encoder_lower_limit &&
                             pitch_encoder_angle_ < encoder_upper_limit;

  if (!input_fresh || !inside_limits)
  {
    torque_test_requested_nm_ = 0.0f;
    torque_test_applied_nm_ = 0.0f;
    torque_test_command_received_ = false;
  }
  else
  {
    const float max_step =
        param::gimbal::torque_test.torque_slew_rate_nm_per_s *
        std::max(dt_s, 0.0f);
    torque_test_applied_nm_ +=
        std::clamp(torque_test_requested_nm_ - torque_test_applied_nm_,
                   -max_step, max_step);
  }

  yaw_.torque = 0.0f;
  big_yaw_.torque = 0.0f;
  pitch_.torque = torque_test_applied_nm_;
  PublishTorque(0.0f, 0.0f, pitch_.torque);
  SendTorqueTestTelemetry(now_ms, pitch_.torque);
}

void Gimbal::SendTorqueTestTelemetry(uint32_t now_ms, float target_torque)
{
  if (static_cast<uint32_t>(now_ms - torque_test_telemetry_ms_) <
      static_cast<uint32_t>(
          param::gimbal::torque_test.telemetry_period_ms))
  {
    return;
  }
  torque_test_telemetry_ms_ = now_ms;

  auto* vofa = UserApp::Debug::GetVofaPlus();
  if (vofa == nullptr)
  {
    return;
  }

  const float channels[] = {
      target_torque,
      pitch_.measured_torque,
      pitch_.angle,
  };
  (void)vofa->Send(channels, 3, "pitch_torque_test");
}

void Gimbal::SendGravityTestTelemetry(uint32_t now_ms, float gravity_torque)
{
  if (static_cast<uint32_t>(now_ms - gravity_test_telemetry_ms_) <
      static_cast<uint32_t>(
          param::gimbal::gravity_test.telemetry_period_ms))
  {
    return;
  }
  gravity_test_telemetry_ms_ = now_ms;

  auto* vofa = UserApp::Debug::GetVofaPlus();
  if (vofa == nullptr)
  {
    return;
  }

  const float channels[] = {
      gravity_torque,
      pitch_.measured_torque,
      pitch_.angle,
  };
  (void)vofa->Send(channels, 3, "pitch_gravity_test");
}

void Gimbal::SendPitchPositionTelemetry(uint32_t now_ms)
{
  if (static_cast<uint32_t>(now_ms - position_telemetry_ms_) <
      static_cast<uint32_t>(param::gimbal::position_telemetry.period_ms))
  {
    return;
  }
  position_telemetry_ms_ = now_ms;

  auto* vofa = UserApp::Debug::GetVofaPlus();
  if (vofa == nullptr)
  {
    return;
  }

  const float channels[] = {
      pitch_.target,
      pitch_.angle,
  };
  (void)vofa->Send(channels, 2, "pitch_position");
}

void Gimbal::RunVelocityTest(float dt_s, uint32_t now_ms)
{
  if (!velocity_test_running_)
  {
    velocity_test_wave_.reset();
    yaw_controller_.Reset();
    big_yaw_controller_.Reset();
    pitch_controller_.Reset();
    velocity_test_running_ = true;
  }

  if (velocity_test_wave_.update(dt_s) != Wave::ErrorCode::OK)
  {
    PublishTorque(0.0f, 0.0f, 0.0f);
    return;
  }

  const float target_velocity = velocity_test_wave_.getOutput();
  float actual_velocity = 0.0f;
  float commanded_torque = 0.0f;
  float measured_torque = 0.0f;
  float yaw_torque = 0.0f;
  float big_yaw_torque = 0.0f;
  float pitch_torque = 0.0f;

  switch (param::gimbal::velocity_test.axis)
  {
    case 0:
      actual_velocity = yaw_.velocity;
      commanded_torque = yaw_controller_.ComputeVelocity(
          target_velocity, actual_velocity, dt_s);
      measured_torque = yaw_.measured_torque;
      yaw_torque = commanded_torque;
      break;
    case 1:
      actual_velocity = big_yaw_.velocity;
      commanded_torque = big_yaw_controller_.ComputeVelocity(
          target_velocity, actual_velocity, dt_s);
      measured_torque = big_yaw_.measured_torque;
      big_yaw_torque = commanded_torque;
      break;
    case 2:
      actual_velocity = pitch_.velocity;
      commanded_torque = pitch_controller_.ComputeVelocity(
          target_velocity, actual_velocity, dt_s);
      commanded_torque += PitchGravityTorque();
      commanded_torque = std::clamp(
          commanded_torque,
          -param::gimbal::dual_pid::pitch.V_max_output,
          param::gimbal::dual_pid::pitch.V_max_output);
      measured_torque = pitch_.measured_torque;
      pitch_torque = commanded_torque;
      break;
    default:
      PublishTorque(0.0f, 0.0f, 0.0f);
      return;
  }

  yaw_.torque = yaw_torque;
  big_yaw_.torque = big_yaw_torque;
  pitch_.torque = pitch_torque;
  PublishTorque(yaw_torque, big_yaw_torque, pitch_torque);
  SendVelocityTelemetry(now_ms, target_velocity, actual_velocity,
                        commanded_torque, measured_torque);
}

void Gimbal::SendVelocityTelemetry(uint32_t now_ms, float target_velocity,
                                   float actual_velocity,
                                   float commanded_torque,
                                   float measured_torque)
{
  if (static_cast<uint32_t>(now_ms - velocity_telemetry_ms_) <
      static_cast<uint32_t>(param::gimbal::velocity_test.telemetry_period_ms))
  {
    return;
  }
  velocity_telemetry_ms_ = now_ms;

  auto* vofa = UserApp::Debug::GetVofaPlus();
  if (vofa == nullptr)
  {
    return;
  }

  const float channels[] = {target_velocity, actual_velocity,
                            commanded_torque, measured_torque};
  (void)vofa->Send(channels, 4, "speed_loop");
}

void Gimbal::ReceiveFeedback(uint32_t now_ms)
{
  if (yaw_subscriber_.Available())
  {
    const auto feedback = yaw_subscriber_.GetData();
    yaw_subscriber_.StartWaiting();
    yaw_motor_relative_angle_ = feedback.position_rad;
    yaw_motor_relative_velocity_ = feedback.velocity_rad_per_sec;
    yaw_.measured_torque = feedback.torque_nm;
    yaw_feedback_ms_ = now_ms;
    yaw_received_ = true;
  }


  
  if (big_yaw_subscriber_.Available())
  {
    const auto feedback = big_yaw_subscriber_.GetData();
    big_yaw_subscriber_.StartWaiting();
    big_yaw_.measured_torque = feedback.torque_nm;
    big_yaw_feedback_ms_ = now_ms;
    big_yaw_received_ = true;
  }
  if (pitch_subscriber_.Available())
  {
    const auto feedback = pitch_subscriber_.GetData();
    pitch_subscriber_.StartWaiting();
    pitch_encoder_angle_ = feedback.position_rad;
    pitch_.measured_torque = feedback.torque_nm;
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

float Gimbal::PitchGravityTorque() const
{
  return param::gimbal::param.gravity_w1 * std::sin(pitch_.angle) +
         param::gimbal::param.gravity_w2 * std::cos(pitch_.angle) +
         param::gimbal::param.gravity_w3;
}

void Gimbal::EnterSafeState()
{
  targets_initialized_ = false;
  velocity_test_running_ = false;
  velocity_test_wave_.reset();
  yaw_controller_.Reset();
  big_yaw_controller_.Reset();
  pitch_controller_.Reset();
  torque_test_requested_nm_ = 0.0f;
  torque_test_applied_nm_ = 0.0f;
  torque_test_command_received_ = false;
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
