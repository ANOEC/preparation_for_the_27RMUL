#include "app_framework.hpp"
#include "libxr.hpp"

// Module headers
#include "BlinkLED.hpp"
#include "DJIMotor.hpp"
#include "CanManager.hpp"
#include "BMI088.hpp"
#include "VQFModule.hpp"

static void XRobotMain(LibXR::HardwareContainer &hw) {
  using namespace LibXR;
  ApplicationManager appmgr;

  // Auto-generated module instantiations
  static BlinkLED BlinkLED_0(hw, appmgr, 250);
  static DJIMotor pitch_motor(
      hw,
      appmgr,
      1,
      DJIMotor::MotorType::GM6020,
      1.0,
      0.0,
      "pitch_motor_feedback",
      "pitch_motor_torque_command",
      100
  );
  static DJIMotor yaw_motor(
      hw,
      appmgr,
      2,
      DJIMotor::MotorType::GM6020,
      1.0,
      0.0,
      "yaw_motor_feedback",
      "yaw_motor_torque_command",
      100
  );
  static CanManager can_manager(
      hw,
      appmgr,
      {{"can2", Module::CanManager::FrameKind::CLASSIC, {&pitch_motor, &yaw_motor}}},
      1,
      2048,
      LibXR::Thread::Priority::HIGH,
      true
  );
  static VQFModule<float> VQFModule_0(
      hw,
      appmgr,
      "bmi088_accl",
      "bmi088_gyro",
      "vqf_output",
      0.0005,
      {3.0, 9.0, true, true, true, 0.5, 100.0, 2.0, 0.1, 0.0001, 0.03, 1.5, 0.5, 2.0, 0.5, 0.05, 20.0, 0.1, 10.0, 20.0, 5.0, 20.0, 0.5, 60.0, 2.0},
      16,
      4096,
      LibXR::Thread::Priority::HIGH
  );
  static BMI088 BMI088_0(
      hw,
      appmgr,
      BMI088::GyroFreq::GYRO_2000HZ_BW532HZ,
      BMI088::AcclFreq::ACCL_1600HZ,
      BMI088::GyroRange::DEG_2000DPS,
      BMI088::AcclRange::ACCL_24G,
      {1.0, 0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, false},
      "bmi088_gyro",
      "bmi088_accl",
      45,
      2048
  );

  while (true) {
    appmgr.MonitorAll();
    Thread::Sleep(1000);
  }
}
