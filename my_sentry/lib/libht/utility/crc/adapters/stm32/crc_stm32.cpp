#include "crc_stm32.hpp"

#if CRC_STM32_BACKEND_AVAILABLE

#include <span>

namespace crc::stm32
{
namespace
{

bool Supports(void*, const crc::Params& params) noexcept
{
  return params.width == 8 || params.width == 16 || params.width == 32;
}

std::uint32_t PolySizeBits(std::uint8_t width) noexcept
{
  switch (width)
  {
    case 8:
      return CRC_CR_POLYSIZE_1;  // 0b10
    case 16:
      return CRC_CR_POLYSIZE_0;  // 0b01
    default:
      return 0;  // 0b00: 32 bit
  }
}

std::uint32_t Compute(void*, const crc::Params& params,
                      std::span<const std::byte> data) noexcept
{
  // Byte-wide writes: REV_IN byte mode reflects each input byte, and any
  // length is handled without a word/tail split.
  CRC->POL = params.poly;
  CRC->INIT = params.init;
  CRC->CR = PolySizeBits(params.width) | (params.refin ? CRC_CR_REV_IN_0 : 0U) |
            (params.refout ? CRC_CR_REV_OUT : 0U);
  CRC->CR |= CRC_CR_RESET;

  auto* const dr8 = reinterpret_cast<volatile std::uint8_t*>(&CRC->DR);
  for (const auto byte : data)
  {
    *dr8 = static_cast<std::uint8_t>(byte);
  }

  const std::uint32_t mask =
      params.width == 32 ? 0xFFFFFFFFU : ((1U << params.width) - 1U);
  // The unit has no xorout stage.
  return (CRC->DR ^ params.xorout) & mask;
}

constinit const crc::Backend kBackend{nullptr, &Supports, &Compute};

std::uint32_t Cycles() noexcept { return DWT->CYCCNT; }

template <typename S>
void Compare(std::size_t preset, std::span<const std::uint8_t> buffer,
             SelfTestResult& result) noexcept
{
  for (std::size_t size = 0; size <= buffer.size(); ++size)
  {
    const auto view = buffer.first(size);
    const auto software = S::Software(view);
    const auto hardware = Compute(nullptr, S::kParams, std::as_bytes(view));
    ++result.cases;
    if (hardware != software)
    {
      ++result.mismatches;
    }
  }
  for (std::size_t index = 0; index < kSelfTestSizes.size(); ++index)
  {
    const auto view = buffer.first(kSelfTestSizes[index]);
    auto start = Cycles();
    volatile auto software = S::Software(view);
    result.software_cycles[preset][index] = Cycles() - start;
    start = Cycles();
    volatile auto hardware = Compute(nullptr, S::kParams, std::as_bytes(view));
    result.hardware_cycles[preset][index] = Cycles() - start;
    (void)software;
    (void)hardware;
  }
}

}  // namespace

void Init() noexcept
{
  CRC_STM32_CLOCK_ENABLE();
  // Read back so the clock is running before the first CRC register access.
  (void)CRC->CR;
}

const crc::Backend& HardwareBackend() noexcept { return kBackend; }

SelfTestResult RunSelfTest() noexcept
{
  if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U)
  {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }

  static std::array<std::uint8_t, 309> buffer{};
  std::uint32_t lcg = 0x12345678U;
  for (auto& value : buffer)
  {
    lcg = lcg * 1664525U + 1013904223U;
    value = static_cast<std::uint8_t>(lcg >> 24U);
  }

  SelfTestResult result{};
  const auto view = std::span<const std::uint8_t>(buffer);
  Compare<crc::Crc8MaximReferee>(0, view, result);
  Compare<crc::Crc16Mcrf4xx>(1, view, result);
  Compare<crc::Crc16Ibm3740>(2, view, result);
  Compare<crc::Crc32IsoHdlc>(3, view, result);
  return result;
}

}  // namespace crc::stm32

#endif  // CRC_STM32_BACKEND_AVAILABLE
