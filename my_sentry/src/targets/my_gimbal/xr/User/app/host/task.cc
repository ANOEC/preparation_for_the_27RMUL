#include "app/host/task.hpp"

#include "app/host/host.hpp"
#include "task_generated.hpp"
#include "timebase.hpp"

namespace UserApp::Host
{
namespace
{
LibXR::UART* g_vision_cdc = nullptr;
LibXR::UART* g_referee_cdc = nullptr;
}

void Initialize(LibXR::UART& vision_cdc, LibXR::UART& referee_cdc) noexcept
{
  g_vision_cdc = &vision_cdc;
  g_referee_cdc = &referee_cdc;
}

}  // namespace UserApp::Host

void generated::task::TaskHooks<generated::task::host_task_tag>::loop(
    const generated::task::LoopContext& context)
{
  if (UserApp::Host::g_vision_cdc == nullptr ||
      UserApp::Host::g_referee_cdc == nullptr)
  {
    return;
  }

  static UserApp::Host::Host host(*UserApp::Host::g_vision_cdc,
                                  *UserApp::Host::g_referee_cdc);
  host.Run(static_cast<std::uint32_t>(LibXR::Timebase::GetMilliseconds()));

  if (context.daemon_service != nullptr)
  {
    context.daemon_service->feed();
  }
}
