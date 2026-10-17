#pragma once

#include <cstdint>

namespace algorithm::time {
double get_current_time_s();
double get_current_time_ms();
double get_current_time_us();

uint64_t get_current_time_s_uint();
uint64_t get_current_time_ms_uint();
uint64_t get_current_time_us_uint();

uint64_t get_current_time_ns();
}