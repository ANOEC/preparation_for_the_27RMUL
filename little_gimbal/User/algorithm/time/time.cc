#include "./time.hpp"
#include <cstdint>

#if defined(__ZEPHYR__)
#include "zephyr/kernel.h"
#elif defined(FREERTOS_CONFIG_H)
#include "FreeRTOS.h"
#include "task.h"
#elif defined(__linux__) || defined(__unix__) || (defined(__APPLE__) && defined(__MACH__)) || defined(__EMSCRIPTEN__)
#include <chrono>
#else
#error "Unsupported platform"
#endif

namespace {
constexpr double NS_TO_S_MULT = 1e-9;
constexpr double NS_TO_MS_MULT = 1e-6;
constexpr double NS_TO_US_MULT = 1e-3;

constexpr uint64_t NS_IN_S = 1'000'000'000ULL;
constexpr uint64_t NS_IN_MS = 1'000'000ULL;
constexpr uint64_t NS_IN_US = 1'000ULL;

inline double ns_to_s(uint64_t ns) { return static_cast<double>(ns) * NS_TO_S_MULT; }
inline double ns_to_ms(uint64_t ns) { return static_cast<double>(ns) * NS_TO_MS_MULT; }
inline double ns_to_us(uint64_t ns) { return static_cast<double>(ns) * NS_TO_US_MULT; }

inline uint64_t ns_to_s_uint(uint64_t ns) { return ns / NS_IN_S; }
inline uint64_t ns_to_ms_uint(uint64_t ns) { return ns / NS_IN_MS; }
inline uint64_t ns_to_us_uint(uint64_t ns) { return ns / NS_IN_US; }
}

namespace algorithm::time {

double get_current_time_s() { return ns_to_s(get_current_time_ns()); }
double get_current_time_ms() { return ns_to_ms(get_current_time_ns()); }
double get_current_time_us() { return ns_to_us(get_current_time_ns()); }

uint64_t get_current_time_s_uint() { return ns_to_s_uint(get_current_time_ns()); }
uint64_t get_current_time_ms_uint() { return ns_to_ms_uint(get_current_time_ns()); }
uint64_t get_current_time_us_uint() { return ns_to_us_uint(get_current_time_ns()); }

uint64_t get_current_time_ns()
{
#if defined(__ZEPHYR__)
    return static_cast<uint64_t>(k_ticks_to_ns_floor64(k_uptime_ticks()));

#elif defined(FREERTOS_CONFIG_H)
    uint64_t ms_time = (static_cast<uint64_t>(xTaskGetTickCount()) * 1000ULL) / configTICK_RATE_HZ;

    return ms_time * NS_IN_MS;

#elif defined(__linux__) || defined(__unix__) || (defined(__APPLE__) && defined(__MACH__)) || defined(__EMSCRIPTEN__)
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch())
            .count());
#else
#error "Unsupported platform"
#endif
}

}