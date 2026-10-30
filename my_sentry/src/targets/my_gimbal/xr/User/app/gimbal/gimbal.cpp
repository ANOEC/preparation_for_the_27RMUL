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
constexpr float kPitchDownLimit = -25.0f * std::numbers::pi_v<float> / 180.0f;
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
      vision_command_subscriber_(
          LibXR::Topic::CreateTopic<Host::GimbalCommand>("/vision_cmd")),
      nav_mode_subscriber_(
          LibXR::Topic::CreateTopic<std::uint64_t>("/vision_nav_mode")),
      robot_status_subscriber_(
          LibXR::Topic::CreateTopic<RefereeProtocol::RobotStatus>(
              "regular/cmd/0201/rx", &referee_domain_)),
      robot_shoot_subscriber_(
          LibXR::Topic::CreateTopic<RefereeProtocol::RobotShoot>(
              "regular/cmd/0207/rx", &referee_domain_)),
      projectile_allowance_subscriber_(
          LibXR::Topic::CreateTopic<RefereeProtocol::ProjectileAllowance>(
              "regular/cmd/0208/rx", &referee_domain_)),
      vision_state_topic_(
          LibXR::Topic::CreateTopic<Host::GimbalState>("/vision_state")),
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
  REQUIRE(param::gimbal::param.pitch_max_vel > 0.0f);
  REQUIRE(param::gimbal::param.pitch_velocity_ff_gain >= 0.0f);
  REQUIRE(param::gimbal::patrol::yaw.period > 0.0f);
  REQUIRE(param::gimbal::patrol::pitch.period > 0.0f);
  REQUIRE(param::gimbal::patrol::upper_pitch.period > 0.0f);
  REQUIRE(!(param::gimbal::patrol::pitch.enabled &&
            param::gimbal::torque_test.enabled));
  REQUIRE(!(param::gimbal::patrol::pitch.enabled &&
            param::gimbal::velocity_test.enabled));
  REQUIRE(!(param::gimbal::patrol::pitch.enabled &&
            param::gimbal::gravity_test.enabled));
  REQUIRE(patrol_yaw_wave_.setParam(
              1.0f / param::gimbal::patrol::yaw.period,
              param::gimbal::patrol::yaw.amplitude * kDegreesToRadians,
              param::gimbal::patrol::yaw.offset * kDegreesToRadians) ==
          Wave::ErrorCode::OK);
  REQUIRE(patrol_pitch_wave_.setParam(
              1.0f / param::gimbal::patrol::pitch.period,
              param::gimbal::patrol::pitch.amplitude * kDegreesToRadians,
              param::gimbal::patrol::pitch.offset * kDegreesToRadians) ==
          Wave::ErrorCode::OK);
  REQUIRE(patrol_upper_pitch_wave_.setParam(
              1.0f / param::gimbal::patrol::upper_pitch.period,
              param::gimbal::patrol::upper_pitch.amplitude * kDegreesToRadians,
              param::gimbal::patrol::upper_pitch.offset * kDegreesToRadians) ==
          Wave::ErrorCode::OK);
  REQUIRE(pitch_position_test_wave_.setParam(
              1.0f / param::gimbal::patrol::pitch.period,
              param::gimbal::patrol::pitch.amplitude * kDegreesToRadians,
              param::gimbal::patrol::pitch.offset * kDegreesToRadians) ==
          Wave::ErrorCode::OK);
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
  vision_command_subscriber_.StartWaiting();
  nav_mode_subscriber_.StartWaiting();
  robot_status_subscriber_.StartWaiting();
  robot_shoot_subscriber_.StartWaiting();
  projectile_allowance_subscriber_.StartWaiting();
}

void Gimbal::Run(float dt_s, uint32_t now_ms)
{
  ReceiveFeedback(now_ms);
  ReceiveCommand(now_ms);
  ReceiveHostAndReferee(now_ms);
  PublishVisionState();

  const bool enabled = command_received_ && command_.enabled &&
                       (command_.state == Remote::ControlState::HAND ||
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

  const float previous_pitch_target = pitch_.target;
  float yaw_velocity_feedforward = 0.0f;
  float big_yaw_velocity_feedforward = 0.0f;
  float pitch_velocity_feedforward = 0.0f;
  float yaw_acceleration_feedforward = 0.0f;
  float pitch_acceleration_feedforward = 0.0f;

  const bool game_mode = command_.state == Remote::ControlState::GAME;
  const bool vision_fresh = vision_command_received_ &&
                            !TimedOut(now_ms, vision_command_ms_, HOST_TIMEOUT_MS);
  const bool nav_fresh = nav_mode_received_ &&
                         !TimedOut(now_ms, nav_mode_ms_, HOST_TIMEOUT_MS);
  const bool vision_patrol = game_mode && vision_fresh &&
                             vision_command_.mode == 2U;
  const bool manual_patrol = !game_mode && command_.patrol;
  const bool patrol_requested = vision_patrol || manual_patrol;

  if (patrol_requested)
  {
    if (!patrol_running_)
    {
      patrol_yaw_wave_.reset();
      patrol_pitch_wave_.reset();
      patrol_upper_pitch_wave_.reset();
      yaw_controller_.Reset();
      big_yaw_controller_.Reset();
      pitch_controller_.Reset();
      patrol_running_ = true;
    }

    auto& pitch_wave = (vision_patrol && nav_fresh && ((nav_mode_ & 0x01ULL) != 0ULL))
                           ? patrol_upper_pitch_wave_
                           : patrol_pitch_wave_;
    if (patrol_yaw_wave_.setOffset(
            big_yaw_.angle +
            param::gimbal::patrol::yaw.offset * kDegreesToRadians) !=
            Wave::ErrorCode::OK ||
        patrol_yaw_wave_.update(dt_s) != Wave::ErrorCode::OK ||
        pitch_wave.update(dt_s) != Wave::ErrorCode::OK)
    {
      EnterSafeState();
      PublishTorque(0.0f, 0.0f, 0.0f);
      return;
    }

    yaw_.target = patrol_yaw_wave_.getOutput();
    big_yaw_.target += param::gimbal::patrol::big_yaw.add_angle;
    pitch_.target = pitch_wave.getOutput();
    pitch_position_test_running_ = false;
  }
  else if (game_mode)
  {
    patrol_running_ = false;
    pitch_position_test_running_ = false;

    if (vision_fresh && vision_command_.mode == 1U)
    {
      // Match the senior project's active auto-aim path: extrapolate a fresh
      // host reference for at most 50 ms and forward velocity/acceleration.
      const auto age_ms = static_cast<std::uint32_t>(now_ms - vision_command_ms_);
      const float extrapolation_s =
          static_cast<float>(std::min(age_ms, VISION_EXTRAPOLATION_LIMIT_MS)) *
          0.001f;
      const float predicted_yaw =
          vision_command_.yaw + vision_command_.yaw_vel * extrapolation_s;
      const float predicted_pitch =
          vision_command_.pitch + vision_command_.pitch_vel * extrapolation_s;

      // Prevent the small-yaw stage from continuing to wind up when its
      // relative angle becomes too large. The reference project enters this
      // protection at 45 degrees and releases below 10 degrees.
      const float relative_yaw = std::abs(yaw_motor_relative_angle_);
      if (!vision_yaw_over_ && relative_yaw > 45.0f * kDegreesToRadians)
      {
        vision_yaw_over_ = true;
        vision_yaw_hold_target_ = yaw_.angle;
      }
      else if (vision_yaw_over_ && relative_yaw < 10.0f * kDegreesToRadians)
      {
        vision_yaw_over_ = false;
      }

      const float desired_yaw =
          NearestEquivalentAngle(predicted_yaw, yaw_.angle);
      yaw_.target = vision_yaw_over_ ? vision_yaw_hold_target_ : desired_yaw;
      big_yaw_target_offset_ = 0.0f;
      big_yaw_.target = yaw_.target;
      pitch_.target = predicted_pitch;
      yaw_velocity_feedforward =
          vision_yaw_over_ ? 0.0f : std::clamp(vision_command_.yaw_vel,
              -param::gimbal::param.yaw_max_vel,
               param::gimbal::param.yaw_max_vel);
      big_yaw_velocity_feedforward = 0.0f;
      pitch_velocity_feedforward = std::clamp(vision_command_.pitch_vel,
          -param::gimbal::param.pitch_max_vel,
           param::gimbal::param.pitch_max_vel);
      yaw_acceleration_feedforward =
          vision_yaw_over_ ? 0.0f : vision_command_.yaw_acc;
      pitch_acceleration_feedforward = vision_command_.pitch_acc;
    }
    else
    {
      vision_yaw_over_ = false;
      // No valid visual target: hold the current measured pose instead of reusing
      // an expired command. This is the safe equivalent of the reference timeout.
      yaw_.target = yaw_.angle;
      big_yaw_.target = big_yaw_.angle;
      pitch_.target = pitch_.angle;
      big_yaw_target_offset_ = big_yaw_.angle - yaw_.angle;
    }
  }
  else
  {
    patrol_running_ = false;
    yaw_.target += command_.yaw_offset_rad;
    const float max_alignment_step =
        kBigYawAlignmentRate * std::max(dt_s, 0.0f);
    big_yaw_target_offset_ -=
        std::clamp(big_yaw_target_offset_, -max_alignment_step,
                   max_alignment_step);
    big_yaw_.target = yaw_.target + big_yaw_target_offset_;

    if (param::gimbal::patrol::pitch.enabled)
    {
      if (!pitch_position_test_running_)
      {
        pitch_position_test_wave_.reset();
        pitch_controller_.Reset();
        pitch_position_test_running_ = true;
      }
      else if (pitch_position_test_wave_.update(dt_s) != Wave::ErrorCode::OK)
      {
        EnterSafeState();
        PublishTorque(0.0f, 0.0f, 0.0f);
        return;
      }
      const float sine_target = pitch_position_test_wave_.getOutput();
      const float max_target_step =
          param::gimbal::param.pitch_max_vel * std::max(dt_s, 0.0f);
      pitch_.target += std::clamp(sine_target - pitch_.target,
                                  -max_target_step, max_target_step);
    }
    else
    {
      pitch_position_test_running_ = false;
      pitch_.target += command_.pitch_offset_rad;
    }
  }

  // Preserve the reference project's encoder-assisted adaptive pitch stop.
  const float pitch_offset = -pitch_encoder_angle_ - pitch_.angle;
  pitch_.target = std::clamp(pitch_.target, kPitchDownLimit - pitch_offset,
                             kPitchUpLimit - pitch_offset);
  if (!(game_mode && vision_fresh && vision_command_.mode == 1U) && dt_s > 0.0f)
  {
    const float target_velocity = std::clamp(
        (pitch_.target - previous_pitch_target) / dt_s,
        -param::gimbal::param.pitch_max_vel,
        param::gimbal::param.pitch_max_vel);
    pitch_velocity_feedforward =
        param::gimbal::param.pitch_velocity_ff_gain * target_velocity;
  }

  yaw_.torque = yaw_controller_.Compute(yaw_.target, yaw_.angle, yaw_.velocity,
                                        dt_s, yaw_velocity_feedforward,
                                        yaw_acceleration_feedforward);
  big_yaw_.torque = big_yaw_controller_.Compute(
      big_yaw_.target, big_yaw_.angle, big_yaw_.velocity, dt_s,
      big_yaw_velocity_feedforward);
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
        pitch_.target, pitch_.angle, pitch_.velocity, dt_s,
        pitch_velocity_feedforward, pitch_acceleration_feedforward);
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

void Gimbal::ReceiveHostAndReferee(uint32_t now_ms)
{
  if (vision_command_subscriber_.Available())
  {
    vision_command_ = vision_command_subscriber_.GetData();
    vision_command_subscriber_.StartWaiting();
    vision_command_ms_ = now_ms;
    vision_command_received_ = true;
  }
  if (nav_mode_subscriber_.Available())
  {
    nav_mode_ = nav_mode_subscriber_.GetData();
    nav_mode_subscriber_.StartWaiting();
    nav_mode_ms_ = now_ms;
    nav_mode_received_ = true;
  }
  if (robot_status_subscriber_.Available())
  {
    robot_status_ = robot_status_subscriber_.GetData();
    robot_status_subscriber_.StartWaiting();
  }
  if (robot_shoot_subscriber_.Available())
  {
    robot_shoot_ = robot_shoot_subscriber_.GetData();
    robot_shoot_subscriber_.StartWaiting();
  }
  if (projectile_allowance_subscriber_.Available())
  {
    projectile_allowance_ = projectile_allowance_subscriber_.GetData();
    projectile_allowance_subscriber_.StartWaiting();
  }
}

void Gimbal::PublishVisionState()
{
  Host::GimbalState state{};
  state.yaw = yaw_.angle;
  state.yaw_vel = yaw_.velocity;
  state.pitch = pitch_.angle;
  state.pitch_vel = pitch_.velocity;
  state.enemy_color = robot_status_.robot_id > 99U ? 'R' : 'B';
  state.big_yaw_relative_to_yaw_angle = yaw_motor_relative_angle_;
  state.bullet_speed = robot_shoot_.initial_speed;
  state.bullet_count = projectile_allowance_.projectile_allowance_17mm;

  const float half_roll = roll_angle_ * 0.5f;
  const float half_pitch = pitch_.angle * 0.5f;
  const float half_yaw = std::remainder(yaw_.angle, kTwoPi) * 0.5f;
  const float cr = std::cos(half_roll);
  const float sr = std::sin(half_roll);
  const float cp = std::cos(half_pitch);
  const float sp = std::sin(half_pitch);
  const float cy = std::cos(half_yaw);
  const float sy = std::sin(half_yaw);
  state.q[0] = cr * cp * cy + sr * sp * sy;
  state.q[1] = sr * cp * cy - cr * sp * sy;
  state.q[2] = cr * sp * cy + sr * cp * sy;
  state.q[3] = cr * cp * sy - sr * sp * cy;
  vision_state_topic_.Publish(state);
}

float Gimbal::NearestEquivalentAngle(float wrapped_target, float reference)
{
  return reference + std::remainder(wrapped_target - reference, kTwoPi);
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
  roll_angle_ = raw_roll * std::cos(shift) - raw_pitch * std::sin(shift);
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
  vision_yaw_over_ = false;
  patrol_running_ = false;
  patrol_yaw_wave_.reset();
  patrol_pitch_wave_.reset();
  patrol_upper_pitch_wave_.reset();
  pitch_position_test_running_ = false;
  pitch_position_test_wave_.reset();
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
