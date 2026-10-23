#include "app/gimbal/gimbal.hpp"

#include "task_generated.hpp"
#include "timebase.hpp"

void generated::task::TaskHooks<generated::task::gimbal_task_tag>::loop(
    const generated::task::LoopContext& context)
{
  // Topic infrastructure is ready by the time the generated task first runs.
  static UserApp::Gimbal::Gimbal gimbal;
  gimbal.Run(context.dt_s,
             static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds()));

  if (context.daemon_service != nullptr)
  {
    context.daemon_service->feed();
  }
}
