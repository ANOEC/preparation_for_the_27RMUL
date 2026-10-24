#pragma once

#include "uart.hpp"
#include "vofa_plus.hpp"

namespace UserApp::Debug
{

[[nodiscard]] debug::vofa_plus::VofaPlus::Transport MakeVofaPlusTransport(
    LibXR::UART& uart) noexcept;

debug::vofa_plus::VofaPlus& InitializeVofaPlus(LibXR::UART& uart) noexcept;

[[nodiscard]] debug::vofa_plus::VofaPlus* GetVofaPlus() noexcept;

}  // namespace UserApp::Debug
