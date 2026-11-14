#pragma once

#include <cstdint>

#include "pwm.hpp"

namespace UserApp::Log
{

void Initialize(LibXR::PWM& buzzer_pwm);
void Run(std::uint32_t now_ms);

}  // namespace UserApp::Log
