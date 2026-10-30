#include "app/shoot/shoot.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "shoot_params.hpp"

namespace UserApp::Shoot
{
namespace
{
LegacyPid::Param MakeLegacyPid(float kp, float ki, float kd, float deadband,
                               float integral_limit, float output_limit)
{
  return LegacyPid::Param{kp, ki, kd, deadband, integral_limit, output_limit};
}
}  // namespace

Shoot::Shoot()
    : command_subscriber_("/cmd/shoot"),
      vision_command_topic_(
          LibXR::Topic::CreateTopic<VisionCommand>("/vision_cmd")),
      big_buff_command_topic_(
          LibXR::Topic::CreateTopic<BigBuffCommand>("/buff_cmd")),
      vision_command_subscriber_(vision_command_topic_),
      big_buff_command_subscriber_(big_buff_command_topic_),
      big_buff_state_topic_(
          LibXR::Topic::CreateTopic<BigBuffState>("/big_buff_state")),
      fri_left_subscriber_("/fb/motor/fri_left"),
      fri_right_subscriber_("/fb/motor/fri_right"),
      pluck_subscriber_("/fb/motor/pluck"),
      fri_left_torque_topic_(
          LibXR::Topic::CreateTopic<float>("/torque/fri_left")),
      fri_right_torque_topic_(
          LibXR::Topic::CreateTopic<float>("/torque/fri_right")),
      pluck_torque_topic_(LibXR::Topic::CreateTopic<float>("/torque/pluck")),
      robot_status_subscriber_("regular/cmd/0201/rx", &referee_domain_),
      power_heat_subscriber_("regular/cmd/0202/rx", &referee_domain_),
      game_status_subscriber_("regular/cmd/0001/rx", &referee_domain_)
{
  fri_left_pid_.SetParam(MakeLegacyPid(
      param::shoot::pid::fri_left.kp, param::shoot::pid::fri_left.ki,
      param::shoot::pid::fri_left.kd,
      param::shoot::pid::fri_left.error_threshold,
      param::shoot::pid::fri_left.integral_limit,
      param::shoot::pid::fri_left.max_output));

  fri_right_pid_.SetParam(MakeLegacyPid(
      param::shoot::pid::fri_right.kp, param::shoot::pid::fri_right.ki,
      param::shoot::pid::fri_right.kd,
      param::shoot::pid::fri_right.error_threshold,
      param::shoot::pid::fri_right.integral_limit,
      param::shoot::pid::fri_right.max_output));

  const LegacyPid::Param position_param = MakeLegacyPid(
      param::shoot::dual_pid::pluck.P_kp,
      param::shoot::dual_pid::pluck.P_ki,
      param::shoot::dual_pid::pluck.P_kd,
      param::shoot::dual_pid::pluck.P_error_threshold,
      param::shoot::dual_pid::pluck.P_integral_limit,
      param::shoot::dual_pid::pluck.P_max_output);

  const LegacyPid::Param velocity_param = MakeLegacyPid(
      param::shoot::dual_pid::pluck.V_kp,
      param::shoot::dual_pid::pluck.V_ki,
      param::shoot::dual_pid::pluck.V_kd,
      param::shoot::dual_pid::pluck.V_error_threshold,
      param::shoot::dual_pid::pluck.V_integral_limit,
      param::shoot::dual_pid::pluck.V_max_output);

  feeder_controller_.SetParams(velocity_param, position_param);
  feeder_controller_.SetMode(FeederControlMode::VELOCITY);
  static_cast<void>(feeder_leso_.setParam(param::shoot::leso::pluck.b0,
                                          param::shoot::leso::pluck.omega));

  command_subscriber_.StartWaiting();
  vision_command_subscriber_.StartWaiting();
  big_buff_command_subscriber_.StartWaiting();
  fri_left_subscriber_.StartWaiting();
  fri_right_subscriber_.StartWaiting();
  pluck_subscriber_.StartWaiting();
  robot_status_subscriber_.StartWaiting();
  power_heat_subscriber_.StartWaiting();
  game_status_subscriber_.StartWaiting();
}

void Shoot::Run(float dt_s, uint32_t now_ms)
{
  if (!std::isfinite(dt_s) || dt_s <= 0.0f)
  {
    dt_s = 0.001f;
  }
  // Avoid one delayed iteration producing a very large ramp/observer step.
  dt_s = std::clamp(dt_s, 0.0001f, 0.01f);

  ReceiveInputs(now_ms);
  UpdateMotorProtection(now_ms);
  UpdateReferences(dt_s, now_ms);
  ComputeControl(dt_s);
  PublishControl();
  PublishState();
}

void Shoot::ReceiveInputs(uint32_t now_ms)
{
  if (command_subscriber_.Available())
  {
    command_ = command_subscriber_.GetData();
    command_subscriber_.StartWaiting();
    command_last_ms_ = now_ms;
    command_alive_ = true;
  }
  else if (command_alive_ && TimedOut(now_ms, command_last_ms_, COMMAND_TIMEOUT_MS))
  {
    command_alive_ = false;
    command_ = {};
  }

  ReceiveMotorFeedback(now_ms);
  ReceiveHostCommands(now_ms);
  ReceiveReferee(now_ms);
}

void Shoot::ReceiveHostCommands(uint32_t now_ms)
{
  if (vision_command_subscriber_.Available())
  {
    vision_command_ = vision_command_subscriber_.GetData();
    vision_command_subscriber_.StartWaiting();
    vision_command_last_ms_ = now_ms;
    vision_command_alive_ = true;
  }
  else if (vision_command_alive_ &&
           TimedOut(now_ms, vision_command_last_ms_, HOST_COMMAND_TIMEOUT_MS))
  {
    vision_command_alive_ = false;
    vision_command_ = {};
  }

  if (big_buff_command_subscriber_.Available())
  {
    big_buff_command_ = big_buff_command_subscriber_.GetData();
    big_buff_command_subscriber_.StartWaiting();
    big_buff_command_last_ms_ = now_ms;
    big_buff_command_alive_ = true;
  }
  else if (big_buff_command_alive_ &&
           TimedOut(now_ms, big_buff_command_last_ms_, HOST_COMMAND_TIMEOUT_MS))
  {
    big_buff_command_alive_ = false;
    big_buff_command_ = {};
  }
}

void Shoot::ReceiveMotorFeedback(uint32_t now_ms)
{
  const auto receive = [now_ms](auto& subscriber, MotorState& state)
  {
    if (subscriber.Available())
    {
      state.feedback = subscriber.GetData();
      subscriber.StartWaiting();
      state.last_update_ms = now_ms;
      state.alive = true;
    }
    else if (state.alive && TimedOut(now_ms, state.last_update_ms, MOTOR_TIMEOUT_MS))
    {
      state.alive = false;
    }
  };

  receive(fri_left_subscriber_, fri_left_);
  receive(fri_right_subscriber_, fri_right_);
  receive(pluck_subscriber_, pluck_);
}

void Shoot::ReceiveReferee(uint32_t now_ms)
{
  if (robot_status_subscriber_.Available())
  {
    robot_status_ = robot_status_subscriber_.GetData();
    robot_status_subscriber_.StartWaiting();
    robot_status_last_ms_ = now_ms;
    robot_status_alive_ = true;
  }
  else if (robot_status_alive_ &&
           TimedOut(now_ms, robot_status_last_ms_, REFEREE_TIMEOUT_MS))
  {
    robot_status_alive_ = false;
  }

  if (power_heat_subscriber_.Available())
  {
    power_heat_ = power_heat_subscriber_.GetData();
    power_heat_subscriber_.StartWaiting();
    power_heat_last_ms_ = now_ms;
    power_heat_alive_ = true;
  }
  else if (power_heat_alive_ &&
           TimedOut(now_ms, power_heat_last_ms_, REFEREE_TIMEOUT_MS))
  {
    power_heat_alive_ = false;
  }

  if (game_status_subscriber_.Available())
  {
    game_status_ = game_status_subscriber_.GetData();
    game_status_subscriber_.StartWaiting();
    game_status_last_ms_ = now_ms;
    game_status_alive_ = true;
  }
  else if (game_status_alive_ &&
           TimedOut(now_ms, game_status_last_ms_, REFEREE_TIMEOUT_MS))
  {
    game_status_alive_ = false;
  }
}

void Shoot::UpdateMotorProtection(uint32_t now_ms)
{
  const bool all_alive = fri_left_.alive && fri_right_.alive && pluck_.alive;

  if (!all_alive)
  {
    motors_ready_ = false;
    motor_protection_active_ = true;
    motor_recovery_start_ms_ = now_ms;
    return;
  }

  if (!motors_ready_)
  {
    motors_ready_ = true;
    motor_recovery_start_ms_ = now_ms;
    motor_protection_active_ = true;
  }

  if (static_cast<uint32_t>(now_ms - motor_recovery_start_ms_) >=
      MOTOR_RECOVERY_DELAY_MS)
  {
    motor_protection_active_ = false;
  }
}

void Shoot::UpdateReferences(float dt_s, uint32_t now_ms)
{
  bool safe_state = command_alive_ &&
                    (command_.state == Remote::ControlState::HAND ||
                     command_.state == Remote::ControlState::GAME);
  bool friction_requested = false;
  float feeder_reference = 0.0f;
  FeederControlMode requested_mode = FeederControlMode::VELOCITY;

  if (safe_state && command_.state == Remote::ControlState::HAND)
  {
    friction_requested = command_.friction_on;
    feeder_reference = command_.feeder_on
                           ? param::shoot::basic_param.pluck_basic_speed
                           : 0.0f;
    big_buff_shooting_ = false;
    big_buff_first_entry_ = true;
  }
  else if (safe_state && command_.state == Remote::ControlState::GAME)
  {
    const bool manual_fire = command_.feeder_on;
    const bool vision_fire =
        vision_command_alive_ &&
        (vision_command_.fire == 1U || vision_command_.fire == 2U);

    // GAME accepts both sources.  A live vision fire request may spin up the
    // friction wheels by itself; the DT7 latch remains an independent manual
    // request.  Manual feeding has priority and always uses velocity mode.
    friction_requested = command_.friction_on || vision_fire;

    if (manual_fire)
    {
      feeder_reference = param::shoot::basic_param.pluck_basic_speed;
      big_buff_shooting_ = false;
      big_buff_first_entry_ = true;
    }
    else if (!vision_command_alive_ || vision_command_.fire == 0U)
    {
      big_buff_shooting_ = false;
      big_buff_first_entry_ = true;
    }
    else if (vision_command_.fire == 1U)
    {
      feeder_reference = param::shoot::basic_param.pluck_basic_speed;
      big_buff_shooting_ = false;
      big_buff_first_entry_ = true;
      last_big_buff_count_ = big_buff_command_.shoot_count;
      big_buff_shoot_count_ = last_big_buff_count_;
    }
    else if (vision_command_.fire == 2U)
    {
      if (big_buff_command_alive_)
      {
        if (last_big_buff_count_ != big_buff_command_.shoot_count)
        {
          big_buff_shoot_amount_ = static_cast<uint8_t>(
              static_cast<uint16_t>(big_buff_command_.shoot_count) -
              static_cast<uint16_t>(last_big_buff_count_));
          last_big_buff_count_ = big_buff_command_.shoot_count;
          big_buff_shooting_ = true;
          big_buff_last_target_ms_ = now_ms;
        }

        if (big_buff_shooting_)
        {
          const float pellet_step =
              2.0f * std::numbers::pi_v<float> /
              static_cast<float>(
                  param::shoot::basic_param.amount_pellet_per_round);
          if (big_buff_first_entry_)
          {
            big_buff_first_entry_ = false;
            big_buff_target_position_rad_ =
                pluck_.feedback.position_rad +
                pellet_step * static_cast<float>(big_buff_shoot_amount_);
          }
          if (static_cast<uint32_t>(now_ms - big_buff_last_target_ms_) >=
              BIG_BUFF_RETRY_MS)
          {
            big_buff_target_position_rad_ =
                pluck_.feedback.position_rad +
                pellet_step * static_cast<float>(big_buff_shoot_amount_);
            big_buff_last_target_ms_ = now_ms;
          }
          requested_mode = FeederControlMode::POSITION;
          feeder_reference = big_buff_target_position_rad_;
        }
        else
        {
          big_buff_shoot_count_ = last_big_buff_count_;
          big_buff_first_entry_ = true;
        }
      }
      else
      {
        // Without a counted-shot command there is no safe position target.
        // Keep the friction wheels ready but do not feed.
        big_buff_shooting_ = false;
        big_buff_first_entry_ = true;
      }
    }
  }

  if (!safe_state || motor_protection_active_)
  {
    friction_requested = false;
    feeder_reference = 0.0f;
    requested_mode = FeederControlMode::VELOCITY;
  }

  SetFeederMode(requested_mode);
  friction_requested = friction_requested && !motor_protection_active_;
  const float ramp_target =
      friction_requested ? param::shoot::basic_param.fri_basic_speed : 0.0f;

  // Reference RampInput changed by 3 rad/s every 1-ms iteration. Preserve the
  // same physical slope while tolerating small task jitter.
  const float ramp_step =
      param::shoot::basic_param.fri_ramp_rate * (dt_s / 0.001f);
  friction_ramp_rad_per_sec_ =
      MoveTowards(friction_ramp_rad_per_sec_, ramp_target, ramp_step);

  fri_left_reference_ = -friction_ramp_rad_per_sec_;
  fri_right_reference_ = friction_ramp_rad_per_sec_;

  UpdateShotEstimator();
  UpdateHeatProtection(dt_s);
  UpdateJamProtection(now_ms, feeder_reference);

  const bool left_follows =
      std::abs(friction_ramp_rad_per_sec_ -
               std::abs(fri_left_.feedback.velocity_rad_per_sec)) <= 100.0f;
  const bool right_follows =
      std::abs(friction_ramp_rad_per_sec_ -
               std::abs(fri_right_.feedback.velocity_rad_per_sec)) <= 100.0f;
  const bool friction_running =
      std::abs(fri_left_reference_) >= 100.0f &&
      std::abs(fri_right_reference_) >= 100.0f;

  // Same interlock as steering_sentry: feeder may not run until both friction
  // wheels have caught the ramp reference.
  if (!(left_follows && right_follows && friction_running))
  {
    if (!feeder_interlock_active_)
    {
      feeder_interlock_active_ = true;
      feeder_interlock_position_rad_ = pluck_.feedback.position_rad;
    }
    feeder_reference = feeder_mode_ == FeederControlMode::POSITION
                           ? feeder_interlock_position_rad_
                           : 0.0f;
  }
  else
  {
    feeder_interlock_active_ = false;
  }

  // Heat limiting only applies once actual referee game data says the game is
  // active. Without referee data, manual bench testing remains possible.
  if (heat_stop_ && game_status_alive_ && game_status_.game_progress != 0U)
  {
    feeder_reference = feeder_mode_ == FeederControlMode::POSITION
                           ? pluck_.feedback.position_rad
                           : 0.0f;
  }

  if (!safe_state || motor_protection_active_)
  {
    feeder_reference = 0.0f;
  }

  feeder_reference_ = feeder_reference;
}

void Shoot::UpdateJamProtection(uint32_t now_ms, float& feeder_reference)
{
  if (jam_reversing_)
  {
    if (feeder_mode_ == FeederControlMode::POSITION)
    {
      feeder_reference = jam_reverse_position_rad_;
      if (std::abs(pluck_.feedback.position_rad - jam_reverse_position_rad_) <
          0.05f)
      {
        jam_reversing_ = false;
        jam_detecting_ = false;
      }
    }
    else
    {
      feeder_reference = param::shoot::basic_param.pluck_reverse_speed;
      const uint32_t reverse_ms = static_cast<uint32_t>(
          param::shoot::basic_param.pluck_reverse_time * 1000.0f);
      if (static_cast<uint32_t>(now_ms - jam_reverse_start_ms_) >= reverse_ms)
      {
        jam_reversing_ = false;
        jam_detecting_ = false;
      }
    }
    return;
  }

  // Preserve the original sign-sensitive overload check. On this vehicle the
  // normal feeding direction produces positive measured feeder torque.
  const bool overloaded =
      pluck_.feedback.torque_nm > param::shoot::basic_param.pluck_overload_effort;
  constexpr float POSITION_STUCK_SPEED_RAD_PER_SEC = 0.001f;
  const bool position_stuck =
      feeder_mode_ == FeederControlMode::POSITION && big_buff_shooting_ &&
      std::abs(pluck_.feedback.velocity_rad_per_sec) <
          POSITION_STUCK_SPEED_RAD_PER_SEC;

  if ((!overloaded && !position_stuck) ||
      (feeder_mode_ == FeederControlMode::VELOCITY && feeder_reference <= 0.0f))
  {
    jam_detecting_ = false;
    return;
  }

  if (!jam_detecting_)
  {
    jam_detecting_ = true;
    jam_detect_start_ms_ = now_ms;
    return;
  }

  const uint32_t check_ms = static_cast<uint32_t>(
      param::shoot::basic_param.pluck_overload_check_time * 1000.0f);
  if (static_cast<uint32_t>(now_ms - jam_detect_start_ms_) >= check_ms)
  {
    jam_reversing_ = true;
    jam_reverse_start_ms_ = now_ms;
    if (feeder_mode_ == FeederControlMode::POSITION)
    {
      const float reverse_angle_rad =
          param::shoot::basic_param.pluck_reverse_angle /
          180.0f * std::numbers::pi_v<float>;
      jam_reverse_position_rad_ =
          pluck_.feedback.position_rad - reverse_angle_rad;
      feeder_reference = jam_reverse_position_rad_;
    }
    else
    {
      feeder_reference = param::shoot::basic_param.pluck_reverse_speed;
    }
  }
}

void Shoot::UpdateShotEstimator()
{
  const float average_friction_speed =
      0.5f * (std::abs(fri_left_.feedback.velocity_rad_per_sec) +
              std::abs(fri_right_.feedback.velocity_rad_per_sec));

  if (average_friction_speed < 5.0f)
  {
    friction_speed_drop_ = false;
    friction_ready_ = false;
  }
  else if (average_friction_speed >= param::shoot::basic_param.fri_count_rise_vel)
  {
    friction_ready_ = true;
  }

  if (!friction_ready_)
  {
    pluck_check_position_rad_ = pluck_.feedback.position_rad;
    return;
  }

  const float pellet_step =
      2.0f * std::numbers::pi_v<float> /
      static_cast<float>(param::shoot::basic_param.amount_pellet_per_round);

  while ((pluck_.feedback.position_rad - pluck_check_position_rad_) >=
         pellet_step * 0.99f)
  {
    estimated_heat_ += static_cast<float>(param::shoot::basic_param.shoot_heat_per);
    pluck_check_position_rad_ += pellet_step;
  }

  if (pluck_check_position_rad_ > pluck_.feedback.position_rad)
  {
    // A jam-reversal moved the feeder backwards. Re-anchor the estimator.
    pluck_check_position_rad_ = pluck_.feedback.position_rad;
  }

  if (average_friction_speed < param::shoot::basic_param.fri_count_decline_vel)
  {
    friction_speed_drop_ = true;
  }
  else if (friction_speed_drop_ &&
           average_friction_speed >= param::shoot::basic_param.fri_count_rise_vel)
  {
    friction_speed_drop_ = false;
    if (big_buff_shooting_)
    {
      ++big_buff_shoot_count_;
      if (big_buff_shoot_count_ >= last_big_buff_count_)
      {
        big_buff_shooting_ = false;
      }
    }
  }
}

void Shoot::UpdateHeatProtection(float dt_s)
{
  if (!robot_status_alive_)
  {
    // No trustworthy heat limit => do not manufacture a threshold. The motor
    // interlocks and manual controls still work for bench testing.
    return;
  }

  const float heat_limit =
      static_cast<float>(robot_status_.shooter_barrel_heat_limit);
  const float cooling_per_second =
      static_cast<float>(robot_status_.shooter_barrel_cooling_value);

  if (power_heat_alive_)
  {
    const float referee_heat =
        static_cast<float>(power_heat_.shooter_17mm_barrel_heat);
    // Never let the local estimate be less conservative than a fresh referee value.
    estimated_heat_ = std::max(estimated_heat_, referee_heat);
  }

  estimated_heat_ = std::max(0.0f, estimated_heat_ - cooling_per_second * dt_s);

  const float stop_threshold =
      heat_limit - 2.0f * static_cast<float>(param::shoot::basic_param.shoot_heat_per);
  if (heat_limit > 0.0f && estimated_heat_ >= stop_threshold)
  {
    heat_stop_ = true;
  }
  if (heat_limit > 0.0f && estimated_heat_ <= heat_limit * 0.5f)
  {
    heat_stop_ = false;
  }
}

void Shoot::ComputeControl(float dt_s)
{
  if (motor_protection_active_)
  {
    ResetControllers();
    return;
  }

  if (std::abs(friction_ramp_rad_per_sec_) < 1.0e-6f)
  {
    fri_left_pid_.Reset();
    fri_right_pid_.Reset();
    fri_left_output_nm_ = 0.0f;
    fri_right_output_nm_ = 0.0f;
  }
  else
  {
    fri_left_output_nm_ = fri_left_pid_.Update(
        fri_left_reference_, fri_left_.feedback.velocity_rad_per_sec);
    fri_right_output_nm_ = fri_right_pid_.Update(
        fri_right_reference_, fri_right_.feedback.velocity_rad_per_sec);
  }

  // libht LESO API is update(input, feedback, dt); the old observer used the
  // opposite argument order. Observer<3> is the equivalent three-state LESO.
  static_cast<void>(feeder_leso_.update(feeder_output_nm_,
                                        pluck_.feedback.position_rad, dt_s));
  const auto observer_state = feeder_leso_.getStatus();
  const float disturbance_torque = feeder_leso_.getDisturbingTorque();

  feeder_output_nm_ = feeder_controller_.Update(
      feeder_reference_, observer_state[0], observer_state[1]);

  if (!jam_reversing_)
  {
    feeder_output_nm_ -= disturbance_torque;
  }

  feeder_output_nm_ = std::clamp(
      feeder_output_nm_, -param::shoot::dual_pid::pluck.V_max_output,
      param::shoot::dual_pid::pluck.V_max_output);
}

void Shoot::PublishControl()
{
  // Explicitly publish zero while protected. DJIMotor has its own 100-ms command
  // timeout as a second independent fail-safe.
  if (motor_protection_active_)
  {
    float zero = 0.0f;
    fri_left_torque_topic_.Publish(zero);
    fri_right_torque_topic_.Publish(zero);
    pluck_torque_topic_.Publish(zero);
    return;
  }

  fri_left_torque_topic_.Publish(fri_left_output_nm_);
  fri_right_torque_topic_.Publish(fri_right_output_nm_);
  pluck_torque_topic_.Publish(feeder_output_nm_);
}

void Shoot::PublishState()
{
  big_buff_state_.already_shoot_count = big_buff_shoot_count_;
  big_buff_state_topic_.Publish(big_buff_state_);
}

void Shoot::SetFeederMode(FeederControlMode mode)
{
  if (mode == feeder_mode_)
  {
    return;
  }

  feeder_mode_ = mode;
  feeder_controller_.SetMode(mode);
  feeder_controller_.SetVelocityKi(
      mode == FeederControlMode::POSITION
          ? param::shoot::dual_pid::pluck.V_ki_buff
          : param::shoot::dual_pid::pluck.V_ki);
}

void Shoot::ResetControllers()
{
  fri_left_pid_.Reset();
  fri_right_pid_.Reset();
  feeder_controller_.Reset();
  feeder_leso_.reset();
  fri_left_output_nm_ = 0.0f;
  fri_right_output_nm_ = 0.0f;
  feeder_output_nm_ = 0.0f;
  feeder_reference_ = 0.0f;
}

float Shoot::MoveTowards(float current, float target, float max_step)
{
  if (max_step <= 0.0f)
  {
    return current;
  }
  if (current < target)
  {
    return std::min(current + max_step, target);
  }
  if (current > target)
  {
    return std::max(current - max_step, target);
  }
  return current;
}

}  // namespace UserApp::Shoot
