#pragma once

#include <cstdint>

#include "can.hpp"

namespace UserApp::BoardLink
{
void Initialize(LibXR::FDCAN& can);
void Run(std::uint32_t now_ms);
}
