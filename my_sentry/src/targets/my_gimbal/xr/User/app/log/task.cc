#include "app/log/log.hpp"

#include <cstdint>

#include "task_generated.hpp"

void generated::task::TaskHooks<generated::task::log_task_tag>::loop(
    const generated::task::LoopContext& context)
{
  UserApp::Log::Run(static_cast<std::uint32_t>(context.now_us / 1000U));
  if (context.daemon_service != nullptr) context.daemon_service->feed();
}
