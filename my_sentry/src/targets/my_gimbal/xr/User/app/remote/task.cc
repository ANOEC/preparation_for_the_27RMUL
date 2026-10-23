#include "app/remote/remote.hpp"

#include "task_generated.hpp"
#include "timebase.hpp"

void generated::task::TaskHooks<generated::task::remote_task_tag>::loop(
    const generated::task::LoopContext& context)
{
  // Construct after PlatformInit(), on the generated task's first iteration.
  static UserApp::Remote::Remote remote;
  remote.Run(static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds()));

  if (context.daemon_service != nullptr)
  {
    context.daemon_service->feed();
  }
}
