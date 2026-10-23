#include "task/task_runtime.hpp"

#include "libxr_def.hpp"
#include "timebase.hpp"

namespace generated::task
{

TaskDaemon& TaskDaemon::Instance()
{
  static TaskDaemon daemon;
  return daemon;
}

void TaskDaemon::Register(DaemonService& service)
{
  REQUIRE(!started_.load(std::memory_order_acquire));
  REQUIRE(service_count_ < services_.size());
  services_[service_count_++] = &service;
}

void TaskDaemon::Start()
{
  bool expected = false;
  if (!started_.compare_exchange_strong(expected, true, std::memory_order_acq_rel))
  {
    return;
  }
  if (service_count_ != 0)
  {
    thread_.Create(this, ThreadEntry, "task_daemon", 1024U,
                   LibXR::Thread::Priority::LOW);
  }
}

void TaskDaemon::ThreadEntry(TaskDaemon* self)
{
  REQUIRE(self != nullptr);
  self->Run();
}

void TaskDaemon::Run()
{
  for (;;)
  {
    const auto now_ms =
        static_cast<std::uint32_t>(LibXR::Timebase::GetMilliseconds());
    for (std::size_t index = 0; index < service_count_; ++index)
    {
      auto* service = services_[index];
      if (service->timed_out(now_ms) && generated_task_timeout_hook != nullptr)
      {
        generated_task_timeout_hook(service->name());
      }
    }
    LibXR::Thread::Sleep(CHECK_PERIOD_MS);
  }
}

DaemonService::DaemonService(const char* name, std::uint32_t timeout_ms)
    : name_(name), timeout_ms_(timeout_ms)
{
  REQUIRE(name != nullptr);
  REQUIRE(timeout_ms > 0U);
  TaskDaemon::Instance().Register(*this);
}

void DaemonService::Start()
{
  last_feed_ms_.store(
      static_cast<std::uint32_t>(LibXR::Timebase::GetMilliseconds()),
      std::memory_order_relaxed);
  reported_.store(false, std::memory_order_relaxed);
  active_.store(true, std::memory_order_release);
}

void DaemonService::feed()
{
  last_feed_ms_.store(
      static_cast<std::uint32_t>(LibXR::Timebase::GetMilliseconds()),
      std::memory_order_relaxed);
  reported_.store(false, std::memory_order_relaxed);
}

bool DaemonService::timed_out(std::uint32_t now_ms)
{
  if (!active_.load(std::memory_order_acquire) ||
      static_cast<std::uint32_t>(
          now_ms - last_feed_ms_.load(std::memory_order_relaxed)) <= timeout_ms_)
  {
    return false;
  }
  bool expected = false;
  return reported_.compare_exchange_strong(expected, true,
                                            std::memory_order_acq_rel);
}

}  // namespace generated::task
