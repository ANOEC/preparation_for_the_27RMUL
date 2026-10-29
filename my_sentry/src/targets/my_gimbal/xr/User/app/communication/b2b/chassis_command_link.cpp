#include "app/communication/b2b/chassis_command_link.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <span>

namespace UserApp::Communication
{

ChassisCommandLink::ChassisCommandLink(
    LibXR::FDCAN& can, Role role, const char* command_topic_name,
    const char* feedback_topic_name, std::uint32_t receive_timeout_ms)
    : can_(can),
      role_(role),
      receive_timeout_ms_(receive_timeout_ms),
      command_subscriber_(
          LibXR::Topic::CreateTopic<Command::ChassisCommand>(command_topic_name)),
      yaw_subscriber_(LibXR::Topic::CreateTopic<DJIMotor::SolvedFeedback>(
          "/fb/motor/yaw")),
      big_yaw_subscriber_(LibXR::Topic::CreateTopic<DMMotor::SolvedFeedback>(
          "/fb/motor/big_yaw")),
      imu_subscriber_(
          LibXR::Topic::CreateTopic<VqfOutput<float>>("vqf_output")),
      command_topic_(LibXR::Topic::CreateTopic<Command::ChassisCommand>(
          command_topic_name)),
      feedback_topic_(LibXR::Topic::CreateTopic<Command::ChassisFeedback>(
          feedback_topic_name))
{
  REQUIRE(command_topic_name != nullptr);
  REQUIRE(feedback_topic_name != nullptr);
  REQUIRE(receive_timeout_ms_ > 0U);

  std::uint32_t receive_id = BoardProtocol::kGimbalToChassisId;
  if (role_ == Role::GIMBAL)
  {
    receive_id = BoardProtocol::kChassisToGimbalId;
    command_subscriber_.StartWaiting();
    yaw_subscriber_.StartWaiting();
    big_yaw_subscriber_.StartWaiting();
  }
  else
  {
    imu_subscriber_.StartWaiting();
  }

  const auto callback = LibXR::FDCAN::CallbackFD::Create(OnFrame, this);
  can_.Register(callback, LibXR::CAN::Type::STANDARD,
                LibXR::CAN::FilterMode::ID_RANGE, receive_id, receive_id);
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

void ChassisCommandLink::OnFrame(bool, ChassisCommandLink* self,
                                 const LibXR::FDCAN::FDPack& frame)
{
  if (self == nullptr || frame.type != LibXR::CAN::Type::STANDARD)
  {
    return;
  }

  const auto bytes = std::as_bytes(std::span(frame.data, frame.len));
  if (self->role_ == Role::CHASSIS &&
      frame.id == BoardProtocol::kGimbalToChassisId)
  {
    Command::ChassisCommand command{};
    if (!BoardProtocol::DecodeChassisCommand(bytes, command) ||
        !std::isfinite(command.vx_mps) || !std::isfinite(command.vy_mps) ||
        !std::isfinite(command.wz_rad_per_sec) ||
        !std::isfinite(command.big_yaw_angle_rad) ||
        !std::isfinite(command.yaw_angle_rad))
    {
      return;
    }
    static_cast<void>(self->command_queue_.Push(command));
    return;
  }

  if (self->role_ == Role::GIMBAL &&
      frame.id == BoardProtocol::kChassisToGimbalId)
  {
    Command::ChassisFeedback feedback{};
    if (!BoardProtocol::DecodeChassisFeedback(bytes, feedback))
    {
      return;
    }
    for (std::size_t index = 0; index < 3U; ++index)
    {
      if (!std::isfinite(feedback.imu_acceleration_mps2[index]) ||
          !std::isfinite(feedback.imu_gyro_rad_per_sec[index]) ||
          !std::isfinite(feedback.imu_euler_rad[index]))
      {
        return;
      }
    }
    static_cast<void>(self->feedback_queue_.Push(feedback));
  }
}

void ChassisCommandLink::RunGimbal(std::uint32_t now_ms)
{
  PollGimbalFeedback(now_ms);

  Command::ChassisFeedback feedback{};
  while (feedback_queue_.Pop(feedback) == LibXR::ErrorCode::OK)
  {
    receive_feedback_ = feedback;
    last_feedback_ms_ = now_ms;
    has_feedback_ = true;
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

void ChassisCommandLink::RunChassis(std::uint32_t now_ms)
{
  Command::ChassisCommand next{};
  while (command_queue_.Pop(next) == LibXR::ErrorCode::OK)
  {
    receive_command_ = next;
    last_command_ms_ = now_ms;
    has_command_ = true;
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
    transmit_command_.yaw_angle_rad = feedback.position_rad;
    yaw_feedback_ms_ = now_ms;
    has_yaw_feedback_ = true;
  }

  if (big_yaw_subscriber_.Available())
  {
    const auto feedback = big_yaw_subscriber_.GetData();
    big_yaw_subscriber_.StartWaiting();
    transmit_command_.big_yaw_angle_rad = feedback.position_rad;
    big_yaw_feedback_ms_ = now_ms;
    has_big_yaw_feedback_ = true;
  }
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
  LibXR::FDCAN::FDPack frame{};
  frame.id = id;
  frame.type = LibXR::CAN::Type::STANDARD;
  frame.len = static_cast<std::uint8_t>(payload.size());
  std::memcpy(frame.data, payload.data(), payload.size());
  static_cast<void>(can_.AddMessage(frame));
}

}  // namespace UserApp::Communication

