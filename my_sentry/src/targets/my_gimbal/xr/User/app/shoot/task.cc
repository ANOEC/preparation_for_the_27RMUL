#include "app/shoot/shoot.hpp"

#include "task_generated.hpp"
#include "timebase.hpp"

void generated::task::TaskHooks<generated::task::shoot_task_tag>::loop(
    const generated::task::LoopContext& context)
{
  // Construct after PlatformInit(), on the generated task's first iteration.
  static UserApp::Shoot::Shoot shoot;
  shoot.Run(context.dt_s,
            static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds()));

  if (context.daemon_service != nullptr)
  {
    context.daemon_service->feed();
  }
}
