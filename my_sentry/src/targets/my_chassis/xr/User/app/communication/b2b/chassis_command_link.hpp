#pragma once

#include <cstdint>

#include "DJIMotor.hpp"
#include "DMMotor.hpp"
#include "RefereeProtocol/RefereeProtocolGenerated.hpp"
#include "VQFModule.hpp"
#include "app/communication/b2b/board_protocol.hpp"
#include "topic.hpp"

namespace UserApp::Communication
{

class ChassisCommandLink
{
 public:
  enum class Role : std::uint8_t
  {
    GIMBAL,
    CHASSIS,
  };

  explicit ChassisCommandLink(
      Role role,
      const char* command_topic_name = "/cmd/chassis",
      const char* feedback_topic_name = "/fb/board/chassis",
      const char* rx_frame_topic_name = "/b2b/rx",
      const char* tx_frame_topic_name = "/b2b/tx",
      std::uint32_t receive_timeout_ms = 1000U);

  void Run(std::uint32_t now_ms);

 private:
  void RunGimbal(std::uint32_t now_ms);
  void RunChassis(std::uint32_t now_ms);
  void PollGimbalFeedback(std::uint32_t now_ms);
  void PollReferee(std::uint32_t now_ms);
  void PollChassisImu(std::uint32_t now_ms);
  void Send(std::uint32_t id, const BoardProtocol::Frame& payload);

  Role role_;
  std::uint32_t receive_timeout_ms_;
  LibXR::Topic::ASyncSubscriber<BoardProtocol::Frame> frame_subscriber_;
  LibXR::Topic::ASyncSubscriber<Command::ChassisCommand> command_subscriber_;
  LibXR::Topic::ASyncSubscriber<DJIMotor::SolvedFeedback> yaw_subscriber_;
  LibXR::Topic::ASyncSubscriber<DMMotor::SolvedFeedback> big_yaw_subscriber_;
  LibXR::Topic::Domain referee_domain_{"referee"};
  LibXR::Topic::ASyncSubscriber<RefereeProtocol::GameStatus>
      game_status_subscriber_;
  LibXR::Topic::ASyncSubscriber<RefereeProtocol::RobotStatus>
      robot_status_subscriber_;
  LibXR::Topic::ASyncSubscriber<RefereeProtocol::PowerHeat>
      power_heat_subscriber_;
  LibXR::Topic::ASyncSubscriber<VqfOutput<float>> imu_subscriber_;
  LibXR::Topic command_topic_;
  LibXR::Topic feedback_topic_;
  LibXR::Topic frame_tx_topic_;
  Command::ChassisCommand transmit_command_{};
  Command::ChassisCommand receive_command_{};
  Command::ChassisFeedback transmit_feedback_{};
  Command::ChassisFeedback receive_feedback_{};
  std::uint32_t last_command_ms_ = 0U;
  std::uint32_t last_feedback_ms_ = 0U;
  std::uint32_t yaw_feedback_ms_ = 0U;
  std::uint32_t big_yaw_feedback_ms_ = 0U;
  std::uint32_t game_status_ms_ = 0U;
  std::uint32_t robot_status_ms_ = 0U;
  std::uint32_t power_heat_ms_ = 0U;
  std::uint32_t imu_feedback_ms_ = 0U;
  bool has_command_ = false;
  bool has_feedback_ = false;
  bool has_yaw_feedback_ = false;
  bool has_big_yaw_feedback_ = false;
  bool has_game_status_ = false;
  bool has_robot_status_ = false;
  bool has_power_heat_ = false;
  bool has_imu_feedback_ = false;
  RefereeProtocol::GameStatus game_status_{};
  RefereeProtocol::RobotStatus robot_status_{};
  RefereeProtocol::PowerHeat power_heat_{};
  float yaw_angle_rad_ = 0.0F;
  float big_yaw_angle_rad_ = 0.0F;
  float big_yaw_velocity_rad_per_sec_ = 0.0F;
};

}  // namespace UserApp::Communication
