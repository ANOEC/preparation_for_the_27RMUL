#pragma once

// Optional STM32 CRC-peripheral backend.
//
// This translation unit is inert unless a genuine STM32 environment is
// detected, so it can be added to any build (including host builds) without
// breaking it. Detection happens in two steps and relies only on macros that
// ST's own CMSIS device headers provide:
//
//   1. __has_include() locates a CMSIS family header without failing when the
//      target is not an STM32. The header is included only if it exists.
//   2. The CMSIS register macros decide availability. Requiring POLYSIZE,
//      REV_IN and REV_OUT excludes the older fixed-polynomial CRC units
//      (STM32F1/F2/F4), where this backend could not work at all.
//
// CRC_STM32_BACKEND_AVAILABLE is defined to 1 or 0 for consumers to test.
// Define CRC_STM32_BACKEND_DISABLE to force the backend off.

#if !defined(CRC_STM32_BACKEND_DISABLE) && defined(__has_include)
#if __has_include("stm32h7xx.h")
#include "stm32h7xx.h"
#elif __has_include("stm32h5xx.h")
#include "stm32h5xx.h"
#elif __has_include("stm32f7xx.h")
#include "stm32f7xx.h"
#elif __has_include("stm32f3xx.h")
#include "stm32f3xx.h"
#elif __has_include("stm32f0xx.h")
#include "stm32f0xx.h"
#elif __has_include("stm32g0xx.h")
#include "stm32g0xx.h"
#elif __has_include("stm32g4xx.h")
#include "stm32g4xx.h"
#elif __has_include("stm32l0xx.h")
#include "stm32l0xx.h"
#elif __has_include("stm32l4xx.h")
#include "stm32l4xx.h"
#elif __has_include("stm32l5xx.h")
#include "stm32l5xx.h"
#elif __has_include("stm32u5xx.h")
#include "stm32u5xx.h"
#elif __has_include("stm32wbxx.h")
#include "stm32wbxx.h"
#elif __has_include("stm32wlxx.h")
#include "stm32wlxx.h"
#elif __has_include("stm32c0xx.h")
#include "stm32c0xx.h"
#endif
#endif

// The peripheral must exist and expose a programmable polynomial, both input
// and output reflection, and a known CRC clock-enable bit.
#if defined(CRC_BASE) && defined(CRC_CR_RESET) && defined(CRC_CR_POLYSIZE) && \
    defined(CRC_CR_REV_IN_0) && defined(CRC_CR_REV_OUT)
#if defined(RCC_AHB4ENR_CRCEN)
#define CRC_STM32_BACKEND_AVAILABLE 1
#define CRC_STM32_CLOCK_ENABLE() (RCC->AHB4ENR |= RCC_AHB4ENR_CRCEN)
#elif defined(RCC_AHB1ENR_CRCEN)
#define CRC_STM32_BACKEND_AVAILABLE 1
#define CRC_STM32_CLOCK_ENABLE() (RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN)
#elif defined(RCC_AHBENR_CRCEN)
#define CRC_STM32_BACKEND_AVAILABLE 1
#define CRC_STM32_CLOCK_ENABLE() (RCC->AHBENR |= RCC_AHBENR_CRCEN)
#endif
#endif

#if !defined(CRC_STM32_BACKEND_AVAILABLE)
#define CRC_STM32_BACKEND_AVAILABLE 0
#endif

#if CRC_STM32_BACKEND_AVAILABLE

#include <array>
#include <cstddef>
#include <cstdint>

#include "crc/crc.hpp"

namespace crc::stm32
{

// Enables the CRC peripheral clock. Call once before InstallBackend().
void Init() noexcept;

// Backend for the STM32 CRC unit: 8/16/32-bit polynomials with any init,
// input/output reflection and xorout. Install with
// crc::InstallBackend(&crc::stm32::HardwareBackend()) during start-up.
//
// Measured slower than the software tables on STM32H723; see docs/STM32.md.
[[nodiscard]] const crc::Backend& HardwareBackend() noexcept;

// On-target comparison of the hardware backend with the software tables.
inline constexpr std::array<std::size_t, 5> kSelfTestSizes{5, 18, 127, 273, 309};

struct SelfTestResult
{
  // Hardware/software disagreements over every preset and length 0..309.
  std::uint32_t mismatches{};
  std::uint32_t cases{};
  // DWT cycles per preset (Crc8MaximReferee, Crc16Mcrf4xx, Crc16Ibm3740,
  // Crc32IsoHdlc) and per kSelfTestSizes entry.
  std::array<std::array<std::uint32_t, kSelfTestSizes.size()>, 4> software_cycles{};
  std::array<std::array<std::uint32_t, kSelfTestSizes.size()>, 4> hardware_cycles{};
};

// Drives the CRC unit directly, bypassing crc::Compute's exclusion. Run it
// after Init() and before InstallBackend(), while nothing else uses the unit.
[[nodiscard]] SelfTestResult RunSelfTest() noexcept;

}  // namespace crc::stm32

#endif  // CRC_STM32_BACKEND_AVAILABLE
