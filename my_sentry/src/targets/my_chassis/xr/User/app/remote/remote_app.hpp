#pragma once

#include <cstddef>
#include <cstdint>

#include "Details/DT7/DT7.hpp"
#include "thread.hpp"
#include "topic.hpp"

namespace UserApp::Remote
{

/** 遥控器决定的机器人工作状态。 */
enum class ControlState : uint8_t
{
  OFFLINE,  ///< 遥控器离线，所有运动指令必须归零。
  PROTECT,  ///< 人工保护状态，遥控器在线但禁止运动。
  MANUAL,   ///< 手动调试状态，允许底盘响应遥控器。
  GAME,     ///< 比赛状态，允许底盘响应遥控器。
};

/** 遥控器选择的底盘运行模式。 */
enum class ChassisMode : uint8_t
{
  FREE,    ///< 自由模式，不执行云台跟随。
  FOLLOW,  ///< 跟随模式，由底盘控制器计算跟随角速度。
  SPIN,    ///< 小陀螺模式，由底盘控制器生成旋转角速度。
};

/** 发布给底盘控制器的目标指令，所有速度均使用国际单位制。 */
struct ChassisCommand
{
  float vx_mps = 0.0f;            ///< 底盘前后速度，单位 m/s。
  float vy_mps = 0.0f;            ///< 底盘左右速度，单位 m/s。
  float wz_rad_per_sec = 0.0f;     ///< 主动旋转角速度，单位 rad/s。
  ControlState state = ControlState::OFFLINE;
  ChassisMode mode = ChassisMode::FREE;
  bool enabled = false;            ///< false 时下游必须输出零力矩。
};

/**
 * 遥控器上层应用：把 DT7 原始话题转换为带失联保护的底盘目标话题。
 *
 * 本类只负责“人的意图”和安全状态，不负责麦轮运动学、速度 PID 或电机控制。
 */
class RemoteApp
{
 public:
  /**
   * 创建遥控器上层任务并启动工作线程。
   * @param remote_topic_name DT7 解码模块发布的话题名。
   * @param chassis_topic_name 输出给底盘控制器的话题名。
   * @param offline_timeout_ms 多久收不到遥控数据后判定离线。
   * @param thread_stack 工作线程栈大小，单位字节。
   * @param priority 工作线程优先级。
   */
  explicit RemoteApp(
      const char* remote_topic_name = "/remote",
      const char* chassis_topic_name = "/cmd/chassis",
      uint32_t offline_timeout_ms = 500,
      std::size_t thread_stack = 2048,
      LibXR::Thread::Priority priority = LibXR::Thread::Priority::MEDIUM);

 private:
  using RemoteData = Module::Remote::DT7::sRemoteInfo;

  static constexpr uint32_t PUBLISH_PERIOD_MS = 10;
  static constexpr float CHANNEL_FULL_SCALE = 660.0f;
  static constexpr float MAX_TRANSLATION_SPEED_MPS = 3.0f;

  /** 线程入口，把静态线程回调转发到当前对象的 Run()。 */
  static void ThreadEntry(RemoteApp* self);

  /** 等待遥控话题、接收最新数据、执行超时保护并周期发布底盘指令。 */
  void Run();

  /** 检查解码后的摇杆、拨杆和鼠标按键是否仍在 DT7 合法范围内。 */
  static bool IsRemoteDataValid(const RemoteData& data);

  /** 将一个摇杆通道限幅并归一化到 [-1, 1]。 */
  static float NormalizeChannel(int16_t channel);

  /** 按旧工程的通道方向、速度上限和拨杆定义生成底盘指令。 */
  static ChassisCommand BuildCommand(const RemoteData& data);

  /** 生成离线或保护状态下的安全零速度指令。 */
  static ChassisCommand BuildSafeCommand(ControlState state);

  /** 使用无符号回绕安全减法判断距离最后一次有效数据是否超时。 */
  bool IsTimedOut(uint32_t now_ms, uint32_t last_valid_ms) const;

  const char* remote_topic_name_;
  uint32_t offline_timeout_ms_;
  LibXR::Topic chassis_topic_;
  LibXR::Thread thread_;
};

}  // namespace UserApp::Remote
