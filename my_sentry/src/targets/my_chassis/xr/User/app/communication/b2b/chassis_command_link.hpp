#pragma once

#include <cstdint>

#include "DJIMotor.hpp"
#include "DMMotor.hpp"
#include "VQFModule.hpp"
#include "app/communication/b2b/board_protocol.hpp"
#include "can.hpp"
#include "spsc_queue.hpp"
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
      LibXR::FDCAN& can, Role role,
      const char* command_topic_name = "/cmd/chassis",
      const char* feedback_topic_name = "/fb/board/chassis",
      std::uint32_t receive_timeout_ms = 100U);

  void Run(std::uint32_t now_ms);

 private:
  static void OnFrame(bool in_isr, ChassisCommandLink* self,
                      const LibXR::FDCAN::FDPack& frame);
  void RunGimbal(std::uint32_t now_ms);
  void RunChassis(std::uint32_t now_ms);
  void PollGimbalFeedback(std::uint32_t now_ms);
  void PollChassisImu(std::uint32_t now_ms);
  void Send(std::uint32_t id, const BoardProtocol::Frame& payload);

  LibXR::FDCAN& can_;
  Role role_;
  std::uint32_t receive_timeout_ms_;
  LibXR::Topic::ASyncSubscriber<Command::ChassisCommand> command_subscriber_;
  LibXR::Topic::ASyncSubscriber<DJIMotor::SolvedFeedback> yaw_subscriber_;
  LibXR::Topic::ASyncSubscriber<DMMotor::SolvedFeedback> big_yaw_subscriber_;
  LibXR::Topic::ASyncSubscriber<VqfOutput<float>> imu_subscriber_;
  LibXR::Topic command_topic_;
  LibXR::Topic feedback_topic_;
  LibXR::SPSCQueue<Command::ChassisCommand> command_queue_{4U};
  LibXR::SPSCQueue<Command::ChassisFeedback> feedback_queue_{4U};
  Command::ChassisCommand transmit_command_{};
  Command::ChassisCommand receive_command_{};
  Command::ChassisFeedback transmit_feedback_{};
  Command::ChassisFeedback receive_feedback_{};
  std::uint32_t last_command_ms_ = 0U;
  std::uint32_t last_feedback_ms_ = 0U;
  std::uint32_t yaw_feedback_ms_ = 0U;
  std::uint32_t big_yaw_feedback_ms_ = 0U;
  std::uint32_t imu_feedback_ms_ = 0U;
  bool has_command_ = false;
  bool has_feedback_ = false;
  bool has_yaw_feedback_ = false;
  bool has_big_yaw_feedback_ = false;
  bool has_imu_feedback_ = false;
};

}  // namespace UserApp::Communication
