#include "app/communication/b2b/chassis_command_link.hpp"

#include <algorithm>
#include <cmath>
#include <span>

namespace UserApp::Communication
{

ChassisCommandLink::ChassisCommandLink(
    Role role, const char* command_topic_name,
    const char* feedback_topic_name, const char* rx_frame_topic_name,
    const char* tx_frame_topic_name, std::uint32_t receive_timeout_ms)
    : role_(role),
      receive_timeout_ms_(receive_timeout_ms),
      frame_subscriber_(LibXR::Topic::CreateTopic<BoardProtocol::Frame>(
          rx_frame_topic_name)),
      command_subscriber_(
          LibXR::Topic::CreateTopic<Command::ChassisCommand>(command_topic_name)),
      yaw_subscriber_(LibXR::Topic::CreateTopic<DJIMotor::SolvedFeedback>(
          "/fb/motor/yaw")),
      big_yaw_subscriber_(LibXR::Topic::CreateTopic<DMMotor::SolvedFeedback>(
          "/fb/motor/big_yaw")),
      game_status_subscriber_(
          LibXR::Topic::CreateTopic<RefereeProtocol::GameStatus>(
              "regular/cmd/0001/rx", &referee_domain_)),
      robot_status_subscriber_(
          LibXR::Topic::CreateTopic<RefereeProtocol::RobotStatus>(
              "regular/cmd/0201/rx", &referee_domain_)),
      power_heat_subscriber_(
          LibXR::Topic::CreateTopic<RefereeProtocol::PowerHeat>(
              "regular/cmd/0202/rx", &referee_domain_)),
      imu_subscriber_(
          LibXR::Topic::CreateTopic<VqfOutput<float>>("vqf_output")),
      command_topic_(LibXR::Topic::CreateTopic<Command::ChassisCommand>(
          command_topic_name)),
      feedback_topic_(LibXR::Topic::CreateTopic<Command::ChassisFeedback>(
          feedback_topic_name)),
      frame_tx_topic_(LibXR::Topic::CreateTopic<BoardProtocol::Frame>(
          tx_frame_topic_name))
{
  REQUIRE(command_topic_name != nullptr);
  REQUIRE(feedback_topic_name != nullptr);
  REQUIRE(rx_frame_topic_name != nullptr);
  REQUIRE(tx_frame_topic_name != nullptr);
  REQUIRE(receive_timeout_ms_ > 0U);

  frame_subscriber_.StartWaiting();

  if (role_ == Role::GIMBAL)
  {
    command_subscriber_.StartWaiting();
    yaw_subscriber_.StartWaiting();
    big_yaw_subscriber_.StartWaiting();
    game_status_subscriber_.StartWaiting();
    robot_status_subscriber_.StartWaiting();
    power_heat_subscriber_.StartWaiting();
  }
  else
  {
    imu_subscriber_.StartWaiting();
  }

}

void ChassisCommandLink::Run(std::uint32_t now_ms)
{
  if (role_ == Role::GIMBAL)
  {
    RunGimbal(now_ms);
  }
  else
  {
    RunChassis(now_ms);
  }
}

void ChassisCommandLink::RunGimbal(std::uint32_t now_ms)
{
  if (frame_subscriber_.Available())
  {
    const auto frame = frame_subscriber_.GetData();
    frame_subscriber_.StartWaiting();
    Command::ChassisFeedback feedback{};
    const auto bytes = std::as_bytes(std::span(frame));
    if (BoardProtocol::DecodeChassisFeedback(bytes, feedback))
    {
      bool valid = true;
      for (std::size_t index = 0; index < 3U; ++index)
      {
        valid = valid &&
                std::isfinite(feedback.imu_acceleration_mps2[index]) &&
                std::isfinite(feedback.imu_gyro_rad_per_sec[index]) &&
                std::isfinite(feedback.imu_euler_rad[index]);
      }
      if (valid)
      {
        receive_feedback_ = feedback;
        last_feedback_ms_ = now_ms;
        has_feedback_ = true;
      }
    }
  }
  if (!has_feedback_ ||
      static_cast<std::uint32_t>(now_ms - last_feedback_ms_) >
          receive_timeout_ms_)
  {
    receive_feedback_ = {};
    has_feedback_ = false;
  }
  feedback_topic_.Publish(receive_feedback_);

  if (command_subscriber_.Available())
  {
    transmit_command_ = command_subscriber_.GetData();
    command_subscriber_.StartWaiting();
  }
  PollGimbalFeedback(now_ms);
  PollReferee(now_ms);

  transmit_command_.yaw_alive =
      has_yaw_feedback_ &&
      static_cast<std::uint32_t>(now_ms - yaw_feedback_ms_) <=
          receive_timeout_ms_ &&
      std::isfinite(transmit_command_.yaw_angle_rad);
  transmit_command_.big_yaw_alive =
      has_big_yaw_feedback_ &&
      static_cast<std::uint32_t>(now_ms - big_yaw_feedback_ms_) <=
          receive_timeout_ms_ &&
      std::isfinite(transmit_command_.big_yaw_angle_rad);

  Send(BoardProtocol::kGimbalToChassisId,
       BoardProtocol::EncodeChassisCommand(transmit_command_));
}

void ChassisCommandLink::PollReferee(std::uint32_t now_ms)
{
  if (game_status_subscriber_.Available())
  {
    game_status_ = game_status_subscriber_.GetData();
    game_status_subscriber_.StartWaiting();
    game_status_ms_ = now_ms;
    has_game_status_ = true;
  }
  if (robot_status_subscriber_.Available())
  {
    robot_status_ = robot_status_subscriber_.GetData();
    robot_status_subscriber_.StartWaiting();
    robot_status_ms_ = now_ms;
    has_robot_status_ = true;
  }
  if (power_heat_subscriber_.Available())
  {
    power_heat_ = power_heat_subscriber_.GetData();
    power_heat_subscriber_.StartWaiting();
    power_heat_ms_ = now_ms;
    has_power_heat_ = true;
  }

  constexpr std::uint32_t kGameStatusTimeoutMs = 2000U;
  constexpr std::uint32_t kFastStatusTimeoutMs = 500U;
  const bool game_fresh = has_game_status_ &&
      static_cast<std::uint32_t>(now_ms - game_status_ms_) <=
          kGameStatusTimeoutMs;
  const bool robot_fresh = has_robot_status_ &&
      static_cast<std::uint32_t>(now_ms - robot_status_ms_) <=
          kFastStatusTimeoutMs;
  const bool power_fresh = has_power_heat_ &&
      static_cast<std::uint32_t>(now_ms - power_heat_ms_) <=
          kFastStatusTimeoutMs;
  transmit_command_.referee_alive = game_fresh && robot_fresh && power_fresh;
  transmit_command_.game_progress = game_status_.game_progress;
  transmit_command_.stage_remain_time_s = game_status_.stage_remain_time;
  transmit_command_.current_hp = robot_status_.current_hp;
  transmit_command_.chassis_power_limit_w = robot_status_.chassis_power_limit;
  transmit_command_.buffer_energy_j = power_heat_.buffer_energy;
  if (!transmit_command_.referee_alive)
  {
    transmit_command_.game_progress = 0U;
    transmit_command_.stage_remain_time_s = 0U;
    transmit_command_.current_hp = 0U;
    transmit_command_.chassis_power_limit_w = 0U;
    transmit_command_.buffer_energy_j = 0U;
  }
}

void ChassisCommandLink::RunChassis(std::uint32_t now_ms)
{
  if (frame_subscriber_.Available())
  {
    const auto frame = frame_subscriber_.GetData();
    frame_subscriber_.StartWaiting();
    Command::ChassisCommand command{};
    const auto bytes = std::as_bytes(std::span(frame));
    if (BoardProtocol::DecodeChassisCommand(bytes, command) &&
        std::isfinite(command.vx_mps) && std::isfinite(command.vy_mps) &&
        std::isfinite(command.wz_rad_per_sec) &&
        std::isfinite(command.big_yaw_angle_rad) &&
        std::isfinite(command.big_yaw_velocity_rad_per_sec) &&
        std::isfinite(command.yaw_angle_rad))
    {
      receive_command_ = command;
      last_command_ms_ = now_ms;
      has_command_ = true;
    }
  }

  if (!has_command_ ||
      static_cast<std::uint32_t>(now_ms - last_command_ms_) >
          receive_timeout_ms_)
  {
    receive_command_ =
        Command::MakeSafeChassisCommand(Command::ControlState::OFFLINE);
    has_command_ = false;
  }
  command_topic_.Publish(receive_command_);

  PollChassisImu(now_ms);
  Send(BoardProtocol::kChassisToGimbalId,
       BoardProtocol::EncodeChassisFeedback(transmit_feedback_));
}

void ChassisCommandLink::PollGimbalFeedback(std::uint32_t now_ms)
{
  if (yaw_subscriber_.Available())
  {
    const auto feedback = yaw_subscriber_.GetData();
    yaw_subscriber_.StartWaiting();
    yaw_angle_rad_ = feedback.position_rad;
    yaw_feedback_ms_ = now_ms;
    has_yaw_feedback_ = true;
  }

  if (big_yaw_subscriber_.Available())
  {
    const auto feedback = big_yaw_subscriber_.GetData();
    big_yaw_subscriber_.StartWaiting();
    big_yaw_angle_rad_ = feedback.position_rad;
    big_yaw_velocity_rad_per_sec_ = feedback.velocity_rad_per_sec;
    big_yaw_feedback_ms_ = now_ms;
    has_big_yaw_feedback_ = true;
  }
  transmit_command_.yaw_angle_rad = yaw_angle_rad_;
  transmit_command_.big_yaw_angle_rad = big_yaw_angle_rad_;
  transmit_command_.big_yaw_velocity_rad_per_sec =
      big_yaw_velocity_rad_per_sec_;
}

void ChassisCommandLink::PollChassisImu(std::uint32_t now_ms)
{
  if (imu_subscriber_.Available())
  {
    const auto imu = imu_subscriber_.GetData();
    imu_subscriber_.StartWaiting();

    const auto& acceleration =
        imu.acceleration_imu_meters_per_second_squared;
    const auto& gyro =
        imu.bias_corrected_angular_velocity_sensor_rad_per_second;
    const auto& orientation = imu.orientation_6d;
    const float norm_squared = orientation.w * orientation.w +
                               orientation.x * orientation.x +
                               orientation.y * orientation.y +
                               orientation.z * orientation.z;
    if (std::isfinite(acceleration.x) && std::isfinite(acceleration.y) &&
        std::isfinite(acceleration.z) && std::isfinite(gyro.x) &&
        std::isfinite(gyro.y) && std::isfinite(gyro.z) &&
        std::isfinite(orientation.w) && std::isfinite(orientation.x) &&
        std::isfinite(orientation.y) && std::isfinite(orientation.z) &&
        norm_squared > 1.0e-6F)
    {
      const float inverse_norm = 1.0F / std::sqrt(norm_squared);
      const float w = orientation.w * inverse_norm;
      const float x = orientation.x * inverse_norm;
      const float y = orientation.y * inverse_norm;
      const float z = orientation.z * inverse_norm;

      transmit_feedback_.imu_acceleration_mps2[0] = acceleration.x;
      transmit_feedback_.imu_acceleration_mps2[1] = acceleration.y;
      transmit_feedback_.imu_acceleration_mps2[2] = acceleration.z;
      transmit_feedback_.imu_gyro_rad_per_sec[0] = gyro.x;
      transmit_feedback_.imu_gyro_rad_per_sec[1] = gyro.y;
      transmit_feedback_.imu_gyro_rad_per_sec[2] = gyro.z;
      transmit_feedback_.imu_euler_rad[0] =
          std::atan2(2.0F * (w * x + y * z),
                     1.0F - 2.0F * (x * x + y * y));
      transmit_feedback_.imu_euler_rad[1] = std::asin(
          std::clamp(2.0F * (w * y - z * x), -1.0F, 1.0F));
      transmit_feedback_.imu_euler_rad[2] =
          std::atan2(2.0F * (w * z + x * y),
                     1.0F - 2.0F * (y * y + z * z));
      imu_feedback_ms_ = now_ms;
      has_imu_feedback_ = true;
    }
  }

  transmit_feedback_.imu_alive =
      has_imu_feedback_ &&
      static_cast<std::uint32_t>(now_ms - imu_feedback_ms_) <=
          receive_timeout_ms_;
  if (!transmit_feedback_.imu_alive)
  {
    transmit_feedback_ = {};
    has_imu_feedback_ = false;
  }
}

void ChassisCommandLink::Send(std::uint32_t id,
                              const BoardProtocol::Frame& payload)
{
  const auto expected_id = role_ == Role::GIMBAL
                               ? BoardProtocol::kGimbalToChassisId
                               : BoardProtocol::kChassisToGimbalId;
  REQUIRE(id == expected_id);
  auto frame = payload;
  frame_tx_topic_.Publish(frame);
}

}  // namespace UserApp::Communication

