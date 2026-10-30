#pragma once

#include "uart.hpp"

namespace UserApp::Host
{

void Initialize(LibXR::UART& vision_cdc, LibXR::UART& referee_cdc) noexcept;

}  // namespace UserApp::Host
