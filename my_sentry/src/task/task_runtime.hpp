#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "thread.hpp"

namespace generated::task
{

class DaemonService;

/** Override this weak hook to place the application in a project-specific safe state. */
extern "C" void generated_task_timeout_hook(const char* task_name)
    __attribute__((weak));

class TaskDaemon
{
 public:
  static TaskDaemon& Instance();
  void Register(DaemonService& service);
  void Start();

 private:
  static constexpr std::size_t MAX_SERVICES = 16;
  static constexpr std::uint32_t CHECK_PERIOD_MS = 10;

  static void ThreadEntry(TaskDaemon* self);
  void Run();

  std::array<DaemonService*, MAX_SERVICES> services_{};
  std::size_t service_count_ = 0;
  std::atomic<bool> started_{false};
  LibXR::Thread thread_;
};

class DaemonService
{
 public:
  DaemonService(const char* name, std::uint32_t timeout_ms);

  void Start();
  void feed();
  bool timed_out(std::uint32_t now_ms);
  const char* name() const { return name_; }

 private:
  const char* name_;
  const std::uint32_t timeout_ms_;
  std::atomic<std::uint32_t> last_feed_ms_{0};
  std::atomic<bool> active_{false};
  std::atomic<bool> reported_{false};
};

}  // namespace generated::task
