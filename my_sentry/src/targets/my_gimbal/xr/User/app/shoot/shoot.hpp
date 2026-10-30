#pragma once

#include <array>
#include <cstdint>

#include "DJIMotor.hpp"
#include "RefereeProtocol/RefereeProtocolGenerated.hpp"
#include "app/host/protocol.hpp"
#include "app/remote/remote.hpp"
#include "app/shoot/control.hpp"
#include "leso/leso.hpp"
#include "topic.hpp"

namespace UserApp::Shoot
{

class Shoot
{
 public:
  Shoot();

  /** One 1-kHz shooter control iteration. */
  void Run(float dt_s, uint32_t now_ms);

 private:
  static constexpr uint32_t MOTOR_TIMEOUT_MS = 100U;
  static constexpr uint32_t COMMAND_TIMEOUT_MS = 100U;
  static constexpr uint32_t REFEREE_TIMEOUT_MS = 2000U;
  static constexpr uint32_t MOTOR_RECOVERY_DELAY_MS = 500U;
  static constexpr uint32_t HOST_TIMEOUT_MS = 2000U;

  struct MotorState
  {
    DJIMotor::SolvedFeedback feedback{};
    uint32_t last_update_ms = 0U;
    bool alive = false;
  };

  void ReceiveInputs(uint32_t now_ms);
  void ReceiveMotorFeedback(uint32_t now_ms);
  void ReceiveHost(uint32_t now_ms);
  void ReceiveReferee(uint32_t now_ms);
  void UpdateMotorProtection(uint32_t now_ms);
  void UpdateReferences(float dt_s, uint32_t now_ms);
  void UpdateJamProtection(uint32_t now_ms, float& feeder_reference);
  void UpdateHeatProtection(float dt_s);
  void UpdateShotEstimator();
  void ComputeControl(float dt_s);
  void PublishControl();
  void ResetControllers();

  static bool TimedOut(uint32_t now_ms, uint32_t last_ms, uint32_t timeout_ms)
  {
    return static_cast<uint32_t>(now_ms - last_ms) > timeout_ms;
  }

  static float MoveTowards(float current, float target, float max_step);

  // Application command from DT7 adapter.
  LibXR::Topic::ASyncSubscriber<Remote::ShootCommand> command_subscriber_;
  LibXR::Topic::ASyncSubscriber<Host::GimbalCommand> vision_command_subscriber_;
  LibXR::Topic::ASyncSubscriber<Host::BigBuffCommand> buff_command_subscriber_;
  LibXR::Topic buff_state_topic_;
  Remote::ShootCommand command_{};
  Host::GimbalCommand vision_command_{};
  Host::BigBuffCommand buff_command_{};
  uint32_t vision_command_last_ms_ = 0U;
  uint32_t buff_command_last_ms_ = 0U;
  bool vision_command_alive_ = false;
  bool buff_command_alive_ = false;
  uint32_t command_last_ms_ = 0U;
  bool command_alive_ = false;

  // Motor feedback.
  LibXR::Topic::ASyncSubscriber<DJIMotor::SolvedFeedback> fri_left_subscriber_;
  LibXR::Topic::ASyncSubscriber<DJIMotor::SolvedFeedback> fri_right_subscriber_;
  LibXR::Topic::ASyncSubscriber<DJIMotor::SolvedFeedback> pluck_subscriber_;
  MotorState fri_left_{};
  MotorState fri_right_{};
  MotorState pluck_{};

  // Torque command topics consumed by DJIMotor.
  LibXR::Topic fri_left_torque_topic_;
  LibXR::Topic fri_right_torque_topic_;
  LibXR::Topic pluck_torque_topic_;

  // RefereeSystem publishes these under domain "referee".
  LibXR::Topic::Domain referee_domain_{"referee"};
  LibXR::Topic::ASyncSubscriber<RefereeProtocol::RobotStatus>
      robot_status_subscriber_;
  LibXR::Topic::ASyncSubscriber<RefereeProtocol::PowerHeat>
      power_heat_subscriber_;
  LibXR::Topic::ASyncSubscriber<RefereeProtocol::GameStatus>
      game_status_subscriber_;
  RefereeProtocol::RobotStatus robot_status_{};
  RefereeProtocol::PowerHeat power_heat_{};
  RefereeProtocol::GameStatus game_status_{};
  uint32_t robot_status_last_ms_ = 0U;
  uint32_t power_heat_last_ms_ = 0U;
  uint32_t game_status_last_ms_ = 0U;
  bool robot_status_alive_ = false;
  bool power_heat_alive_ = false;
  bool game_status_alive_ = false;

  LegacyPid fri_left_pid_{};
  LegacyPid fri_right_pid_{};
  FeederController feeder_controller_{};
  LESO::Observer<3> feeder_leso_{1.0f, 1.0f};

  float friction_ramp_rad_per_sec_ = 0.0f;
  float fri_left_reference_ = 0.0f;
  float fri_right_reference_ = 0.0f;
  float feeder_reference_ = 0.0f;

  float fri_left_output_nm_ = 0.0f;
  float fri_right_output_nm_ = 0.0f;
  float feeder_output_nm_ = 0.0f;

  bool motors_ready_ = false;
  bool motor_protection_active_ = true;
  uint32_t motor_recovery_start_ms_ = 0U;

  bool jam_detecting_ = false;
  uint32_t jam_detect_start_ms_ = 0U;
  bool jam_reversing_ = false;
  uint32_t jam_reverse_start_ms_ = 0U;
  float jam_reverse_position_rad_ = 0.0f;

  bool buff_shoot_active_ = false;
  bool buff_first_entry_ = true;
  uint8_t last_buff_command_count_ = 0U;
  uint8_t buff_shoot_amount_ = 0U;
  uint32_t buff_completed_count_ = 0U;
  float buff_target_position_rad_ = 0.0f;
  uint32_t buff_last_target_ms_ = 0U;

  bool friction_ready_ = false;
  bool friction_speed_drop_ = false;
  float pluck_check_position_rad_ = 0.0f;
  float estimated_heat_ = 0.0f;
  bool heat_stop_ = false;
};

}  // namespace UserApp::Shoot
