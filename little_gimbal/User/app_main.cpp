#include "app_main.h"

#include "cdc_uart.hpp"
#include "libxr.hpp"
#include "main.h"
#include "stm32_adc.hpp"
#include "stm32_can.hpp"
#include "stm32_canfd.hpp"
#include "stm32_dac.hpp"
#include "stm32_flash.hpp"
#include "stm32_gpio.hpp"
#include "stm32_i2c.hpp"
#include "stm32_power.hpp"
#include "stm32_pwm.hpp"
#include "stm32_spi.hpp"
#include "stm32_timebase.hpp"
#include "stm32_uart.hpp"
#include "stm32_usb_dev.hpp"
#include "stm32_watchdog.hpp"
#include "flash_map.hpp"
#include "app_framework.hpp"
#include "xrobot_main.hpp"
#include "control/gimbal_controller.hpp"
#include "telemetry/vofa_telemetry.hpp"
#include "config/app_config.hpp"

#include <cerrno>
#include <cmath>
#include <cstdlib>

using namespace LibXR;

/* User Code Begin 1 */
/* User Code End 1 */
// NOLINTBEGIN
// clang-format off
/* External HAL Declarations */
extern CAN_HandleTypeDef hcan2;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart1;

/* DMA Resources */
static uint8_t spi1_tx_buf[32];
static uint8_t spi1_rx_buf[32];
static uint8_t usart1_tx_buf[384];
static uint8_t usart1_rx_buf[384];

extern "C" void app_main(void) {
  // clang-format on
  // NOLINTEND
  /* User Code Begin 2 */
  
  /* User Code End 2 */
  // clang-format off
  // NOLINTBEGIN
  STM32TimerTimebase timebase(&htim6);
  PlatformInit(45, 1024);
  STM32PowerManager power_manager;

  /* GPIO Configuration */
  STM32GPIO ACCEL_CS(ACCEL_CS_GPIO_Port, ACCEL_CS_Pin);
  STM32GPIO GYRO_CS(GYRO_CS_GPIO_Port, GYRO_CS_Pin);
  STM32GPIO GYRO_INT(GYRO_INT_GPIO_Port, GYRO_INT_Pin, EXTI9_5_IRQn);
  STM32GPIO LED(LED_GPIO_Port, LED_Pin);


  STM32PWM pwm_tim10_ch1(&htim10, TIM_CHANNEL_1, false);

  STM32SPI spi1(&hspi1, spi1_rx_buf, spi1_tx_buf, 32);

  STM32UART usart1(&huart1,
              usart1_rx_buf, usart1_tx_buf, 5);

  STM32CAN can2(&hcan2, 5);

  /* Terminal Configuration */
  STDIO::read_ = usart1.read_port_;
  STDIO::write_ = usart1.write_port_;

  RamFS ramfs("XRobot");
  auto gimbal_enable_command = RamFS::CreateCommand(
      "gimbal_enable",
      +[](void*, int argc, char** argv) -> int {
        if (argc != 2 || (argv[1][0] != '0' && argv[1][0] != '1') ||
            argv[1][1] != '\0' ||
            !GimbalControllerSetEnabled(argv[1][0] == '1')) {
          STDIO::Print<"usage: gimbal_enable <0|1>\r\n">();
          return -1;
        }
        STDIO::Printf<"gimbal control %s\r\n">(
            argv[1][0] == '1' ? "enabled" : "disabled");
        return 0;
      },
      static_cast<void*>(nullptr));
  ramfs.Add(gimbal_enable_command);
  auto yaw_ref_command = RamFS::CreateCommand(
      "yaw_ref",
      +[](void*, int argc, char** argv) -> int {
        if (argc != 2) {
          STDIO::Print<"usage: yaw_ref <center_relative_rad>\r\n">();
          return -1;
        }
        errno = 0;
        char* end = nullptr;
        const float reference_rad = std::strtof(argv[1], &end);
        if (errno != 0 || end == argv[1] || *end != '\0') {
          STDIO::Print<"yaw_ref rejected\r\n">();
          return -1;
        }
        if (!GimbalControllerIsHomed()) {
          STDIO::Print<"yaw_ref rejected: homing not complete\r\n">();
          return -1;
        }
        if (!GimbalControllerSetYawAngleReference(reference_rad)) {
          STDIO::Print<"yaw_ref rejected: range is [-2.08, 2.08] rad\r\n">();
          return -1;
        }
        STDIO::Printf<"yaw_ref accepted: %.4f rad\r\n">(reference_rad);
        return 0;
      },
      static_cast<void*>(nullptr));
  ramfs.Add(yaw_ref_command);
  auto pitch_ref_command = RamFS::CreateCommand(
      "pitch_ref",
      +[](void*, int argc, char** argv) -> int {
        if (argc != 2) {
          STDIO::Print<"usage: pitch_ref <center_relative_rad>\r\n">();
          return -1;
        }
        errno = 0;
        char* end = nullptr;
        const float reference_rad = std::strtof(argv[1], &end);
        if (errno != 0 || end == argv[1] || *end != '\0') {
          STDIO::Print<"pitch_ref rejected\r\n">();
          return -1;
        }
        if (!GimbalControllerIsHomed()) {
          STDIO::Print<"pitch_ref rejected: homing not complete\r\n">();
          return -1;
        }
        if (!GimbalControllerSetPitchAngleReference(reference_rad)) {
          STDIO::Print<"pitch_ref rejected: range is [-0.80, 0.80] rad\r\n">();
          return -1;
        }
        STDIO::Printf<"pitch_ref accepted: %.4f rad\r\n">(reference_rad);
        return 0;
      },
      static_cast<void*>(nullptr));
  ramfs.Add(pitch_ref_command);
  auto yaw_torque_command = RamFS::CreateCommand(
      "yaw_torque",
      +[](void*, int argc, char** argv) -> int {
        if (argc != 2) {
          STDIO::Print<"usage: yaw_torque <Nm>\r\n">();
          return -1;
        }
        errno = 0;
        char* end = nullptr;
        const float torque_nm = std::strtof(argv[1], &end);
        if (errno != 0 || end == argv[1] || *end != '\0' ||
            !GimbalControllerRequestYawDiagnosticTorque(torque_nm)) {
          STDIO::Print<"yaw_torque rejected\r\n">();
          return -1;
        }
        STDIO::Printf<
            "yaw_torque accepted: %.3f Nm (0 stops, yaw_ref exits)\r\n">(
            torque_nm);
        return 0;
      },
      static_cast<void*>(nullptr));
  ramfs.Add(yaw_torque_command);
  auto pitch_torque_command = RamFS::CreateCommand(
      "pitch_torque",
      +[](void*, int argc, char** argv) -> int {
        if (argc != 2) {
          STDIO::Print<"usage: pitch_torque <Nm>\r\n">();
          return -1;
        }
        errno = 0;
        char* end = nullptr;
        const float torque_nm = std::strtof(argv[1], &end);
        if (errno != 0 || end == argv[1] || *end != '\0' ||
            !GimbalControllerRequestPitchDiagnosticTorque(torque_nm)) {
          STDIO::Print<"pitch_torque rejected\r\n">();
          return -1;
        }
        STDIO::Printf<
            "pitch_torque accepted: %.3f Nm (0 stops, pitch_ref exits)\r\n">(
            torque_nm);
        return 0;
      },
      static_cast<void*>(nullptr));
  ramfs.Add(pitch_torque_command);
  auto gimbal_mode_command = RamFS::CreateCommand(
      "gimbal_mode",
      +[](void*, int argc, char** argv) -> int {
        if (argc != 2 || (argv[1][0] != '0' && argv[1][0] != '1') ||
            argv[1][1] != '\0') {
          STDIO::Print<"usage: gimbal_mode <0 normal|1 cruise>\r\n">();
          return -1;
        }
        const auto mode = argv[1][0] == '1' ? GimbalMode::kCruise
                                            : GimbalMode::kNormal;
        if (!GimbalControllerSetMode(mode)) {
          STDIO::Print<"gimbal_mode rejected\r\n">();
          return -1;
        }
        STDIO::Printf<"gimbal mode requested: %s\r\n">(
            mode == GimbalMode::kCruise ? "cruise" : "normal");
        return 0;
      },
      static_cast<void*>(nullptr));
  ramfs.Add(gimbal_mode_command);
  auto cruise_config_command = RamFS::CreateCommand(
      "cruise_config",
      +[](void*, int argc, char** argv) -> int {
        if (argc != 5) {
          STDIO::Print<
              "usage: cruise_config <yaw_amp_rad> <yaw_hz> <pitch_amp_rad> <pitch_hz>\r\n">();
          return -1;
        }
        float values[4]{};
        for (int i = 0; i < 4; ++i) {
          errno = 0;
          char* end = nullptr;
          values[i] = std::strtof(argv[i + 1], &end);
          if (errno != 0 || end == argv[i + 1] || *end != '\0' ||
              !std::isfinite(values[i])) {
            STDIO::Print<"cruise_config rejected: invalid number\r\n">();
            return -1;
          }
        }
        if (!GimbalControllerConfigureCruise(values[0], values[1],
                                             values[2], values[3])) {
          STDIO::Print<
              "cruise_config rejected: check amplitude/frequency ranges\r\n">();
          return -1;
        }
        STDIO::Printf<
            "cruise config: yaw A=%.3f rad f=%.3f Hz, pitch A=%.3f rad f=%.3f Hz\r\n">(
            values[0], values[1], values[2], values[3]);
        return 0;
      },
      static_cast<void*>(nullptr));
  ramfs.Add(cruise_config_command);
  Terminal<32, 96, 5, 5> terminal(ramfs);
  auto terminal_task = Timer::CreateTask(terminal.TaskFun, &terminal, 10);
  Timer::Add(terminal_task);
  Timer::Start(terminal_task);


  LibXR::HardwareContainer peripherals{
    LibXR::Entry<LibXR::GPIO>({ACCEL_CS, {"ACCEL_CS", "bmi088_accl_cs"}}),
    LibXR::Entry<LibXR::GPIO>({GYRO_CS, {"GYRO_CS", "bmi088_gyro_cs"}}),
    LibXR::Entry<LibXR::GPIO>({GYRO_INT, {"GYRO_INT", "bmi088_gyro_int"}}),
    LibXR::Entry<LibXR::GPIO>({LED, {"LED"}}),
    LibXR::Entry<LibXR::PWM>({pwm_tim10_ch1, {"BMI088_HEAT", "pwm_bmi088_heat", "pwm_tim10_ch1"}}),
    LibXR::Entry<LibXR::SPI>({spi1, {"SPI1", "spi1", "spi_bmi088"}}),
    LibXR::Entry<LibXR::UART>({usart1, {"USART1", "usart1"}}),
    LibXR::Entry<LibXR::PowerManager>({power_manager, {"power_manager"}}),
    LibXR::Entry<LibXR::RamFS>({ramfs, {"ramfs"}}),
    LibXR::Entry<LibXR::Terminal<32, 96, 5, 5>>({terminal, {"terminal"}}),
    LibXR::Entry<LibXR::CAN>({can2, {"can2"}})
  };

  // clang-format on
  // NOLINTEND
  /* User Code Begin 3 */
  if constexpr (AppConfig::Gimbal::kControllerEnable) {
    GimbalControllerInit();
  }
  if constexpr (AppConfig::Telemetry::kEnable) {
    VofaTelemetryInit();
  }
  XRobotMain(peripherals);

  while(true) {
    Thread::Sleep(UINT32_MAX);
  }
  /* User Code End 3 */
}
