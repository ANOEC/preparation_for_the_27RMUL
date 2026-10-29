#include "app/communication/b2b/task.hpp"
#include "task_generated.hpp"
#include "timebase.hpp"

void generated::task::TaskHooks<
    generated::task::communication_task_tag>::loop(
    const generated::task::LoopContext& context)
{
  UserApp::BoardLink::Run(
      static_cast<std::uint32_t>(LibXR::Timebase::GetMilliseconds()));

  if (context.daemon_service != nullptr)
  {
    context.daemon_service->feed();
  }
}
