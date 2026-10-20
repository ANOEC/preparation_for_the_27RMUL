#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "BMI088.hpp"
#include "DJIMotor.hpp"
#include "Details/DT7/DT7.hpp"
#include "VQFModule.hpp"
#include "app/remote/remote_app.hpp"
#include "mutex.hpp"
#include "thread.hpp"
#include "topic.hpp"

namespace UserApp::Debug
{

/**
 * 通过 RTT 查看当前工程话题在线状态和最新数据的调试工具。
 *
 * 在 JLinkRTTClient 中输入 `topic help` 可以查看完整命令列表。
 */
class TopicDebugApp
{
 public:
  explicit TopicDebugApp(uint32_t offline_timeout_ms = 500,
                         uint32_t print_period_ms = 500,
                         std::size_t thread_stack = 3072,
                         LibXR::Thread::Priority priority =
                             LibXR::Thread::Priority::LOW);

 private:
  using RemoteData = Module::Remote::DT7::sRemoteInfo;
  using ChassisCommand = UserApp::Remote::ChassisCommand;
  using MotorFeedback = DJIMotor::SolvedFeedback;
  using ImuVector = Eigen::Matrix<float, 3, 1>;
  using VqfOutputData = VqfOutput<float>;

  class MonitorBase
  {
   public:
    explicit MonitorBase(const char* name) : name_(name) {}
    virtual ~MonitorBase() = default;

    [[nodiscard]] const char* Name() const { return name_; }
    virtual void PrintStatus(LibXR::MicrosecondTimestamp now, uint64_t timeout_us,
                             bool print_value) = 0;

   private:
    const char* name_;
  };

  template <typename Data>
  class Monitor final : public MonitorBase
  {
   public:
    using Printer = void (*)(const Data& data);

    Monitor(const char* name, Printer printer)
        : MonitorBase(name),
          topic_(LibXR::Topic::CreateTopic<Data>(name)),
          callback_(LibXR::Topic::Callback::Create(OnMessage, this)),
          printer_(printer)
    {
      topic_.RegisterCallback(callback_);
    }

    void PrintStatus(LibXR::MicrosecondTimestamp now, uint64_t timeout_us,
                     bool print_value) override;

   private:
    /** 收到新消息时保存最新值，并计算本次与上一次更新之间的时间。 */
    static void OnMessage(bool in_isr, Monitor* self,
                          LibXR::MicrosecondTimestamp timestamp,
                          const Data& data);

    LibXR::Topic topic_;
    LibXR::Topic::Callback callback_;
    Printer printer_;
    LibXR::Mutex mutex_;
    Data latest_{};
    LibXR::MicrosecondTimestamp last_update_{};
    uint64_t update_interval_us_ = 0;
    bool received_ = false;
  };

  static constexpr std::size_t COMMAND_LINE_SIZE = 96;
  static constexpr uint32_t INPUT_WAIT_MS = 50;

  static void ThreadEntry(TopicDebugApp* self);

  /** RTT 命令线程：接收命令，并按设定周期输出选中话题。 */
  void Run();

  /** 解析一整行命令，例如 `topic echo /remote`。 */
  void ExecuteCommand(char* line);

  /** 打印全部已注册话题的在线状态。 */
  void PrintTopicList();

  /** 查找话题；返回值小于 0 表示名称不存在。 */
  [[nodiscard]] int FindMonitor(const char* name) const;

  /** 输出命令帮助。 */
  static void PrintHelp();

  static void PrintRemote(const RemoteData& data);
  static void PrintChassisCommand(const ChassisCommand& data);
  static void PrintMotorFeedback(const MotorFeedback& data);
  static void PrintImuVector(const ImuVector& data);
  static void PrintVqfOutput(const VqfOutputData& data);
  static void PrintTorque(const float& data);

  // 监视器成员及注册表由 topic_monitor.toml 生成。
#define USER_APP_TOPIC_MONITOR_MEMBERS
#include "debug/generated/topic_monitor.hpp"
#undef USER_APP_TOPIC_MONITOR_MEMBERS

  std::atomic<int> selected_monitor_{-1};
  std::atomic<uint32_t> offline_timeout_ms_;
  std::atomic<uint32_t> print_period_ms_;
  LibXR::Thread thread_;
};

}  // namespace UserApp::Debug

