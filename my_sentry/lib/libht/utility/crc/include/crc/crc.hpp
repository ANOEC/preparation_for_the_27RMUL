#pragma once

#include <atomic>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>

namespace crc
{

// Rocksoft-model parameters. `poly` and `init` are given in the unreflected
// (MSB-first) form, as in the CRC catalogue.
struct Params
{
  std::uint8_t width{};
  std::uint32_t poly{};
  std::uint32_t init{};
  bool refin{};
  bool refout{};
  std::uint32_t xorout{};
};

// Hardware acceleration hook. `compute` is only called while the library holds
// the backend exclusively and must return the final CRC value (xorout applied)
// for `params`. Neither function may block, allocate or throw.
struct Backend
{
  void* context{};
  bool (*supports)(void* context, const Params& params) noexcept{};
  std::uint32_t (*compute)(void* context, const Params& params,
                           std::span<const std::byte> data) noexcept{};
};

namespace detail
{

inline std::atomic<const Backend*> installed_backend{nullptr};
inline std::atomic_flag backend_busy{};

constexpr std::uint32_t Mask(std::uint8_t width) noexcept
{
  return width == 32 ? 0xFFFFFFFFU : ((1U << width) - 1U);
}

constexpr std::uint32_t Reflect(std::uint32_t value, std::uint8_t width) noexcept
{
  std::uint32_t result = 0;
  for (std::uint8_t bit = 0; bit < width; ++bit)
  {
    result = (result << 1U) | ((value >> bit) & 1U);
  }
  return result;
}

template <std::uint8_t Width>
using ValueFor = std::conditional_t<
    Width == 8, std::uint8_t,
    std::conditional_t<Width == 16, std::uint16_t, std::uint32_t>>;

template <typename Byte>
constexpr std::uint8_t ToU8(Byte value) noexcept
{
  return static_cast<std::uint8_t>(value);
}

template <std::uint8_t Width, std::uint32_t Poly, bool RefIn>
constexpr std::array<ValueFor<Width>, 256> MakeTable() noexcept
{
  constexpr auto mask = Mask(Width);
  std::array<ValueFor<Width>, 256> table{};
  for (std::uint32_t index = 0; index < table.size(); ++index)
  {
    std::uint32_t value = 0;
    if constexpr (RefIn)
    {
      constexpr auto poly = Reflect(Poly, Width);
      value = index;
      for (int bit = 0; bit < 8; ++bit)
      {
        value = (value & 1U) != 0U ? (value >> 1U) ^ poly : value >> 1U;
      }
    }
    else
    {
      constexpr auto top = 1U << (Width - 1U);
      value = index << (Width - 8U);
      for (int bit = 0; bit < 8; ++bit)
      {
        value = (value & top) != 0U ? ((value << 1U) ^ Poly) & mask : (value << 1U) & mask;
      }
    }
    table[index] = static_cast<ValueFor<Width>>(value);
  }
  return table;
}

template <std::uint8_t Width, std::uint32_t Poly, bool RefIn>
inline constexpr auto kTable = MakeTable<Width, Poly, RefIn>();

}  // namespace detail

template <std::uint8_t Width, std::uint32_t Poly, std::uint32_t Init, bool RefIn,
          bool RefOut, std::uint32_t XorOut>
struct Spec
{
  static_assert(Width == 8 || Width == 16 || Width == 32,
                "crc::Spec supports 8, 16 and 32 bit widths");
  static_assert((Poly & ~detail::Mask(Width)) == 0 && (Init & ~detail::Mask(Width)) == 0 &&
                    (XorOut & ~detail::Mask(Width)) == 0,
                "crc::Spec parameters must fit in Width bits");

  using Value = detail::ValueFor<Width>;
  static constexpr Params kParams{Width, Poly, Init, RefIn, RefOut, XorOut};

  template <typename Byte>
  static constexpr Value Software(std::span<const Byte> data) noexcept
  {
    constexpr auto mask = detail::Mask(Width);
    constexpr const auto& table = detail::kTable<Width, Poly, RefIn>;
    std::uint32_t reg = RefIn ? detail::Reflect(Init, Width) : Init;
    for (const auto byte : data)
    {
      const auto value = detail::ToU8(byte);
      if constexpr (RefIn)
      {
        // Width 8 has no bits left above the shift; avoid a full-width shift.
        const auto high = Width == 8 ? 0U : (reg >> 8U);
        reg = table[(reg ^ value) & 0xFFU] ^ high;
      }
      else
      {
        const auto low = Width == 8 ? 0U : ((reg << 8U) & mask);
        reg = low ^ table[((reg >> (Width - 8U)) ^ value) & 0xFFU];
      }
    }
    // A reflected register already holds the reflected output.
    if constexpr (RefIn != RefOut)
    {
      reg = detail::Reflect(reg, Width);
    }
    return static_cast<Value>((reg ^ XorOut) & mask);
  }
};

// Referee system Appendix 1 CRC8: CRC-8/MAXIM polynomial with initial 0xFF
// (differs from CRC-8/MAXIM-DOW, whose initial value is 0x00).
using Crc8MaximReferee = Spec<8, 0x31, 0xFF, true, true, 0x00>;
using Crc16Mcrf4xx = Spec<16, 0x1021, 0xFFFF, true, true, 0x0000>;
using Crc16Ibm3740 = Spec<16, 0x1021, 0xFFFF, false, false, 0x0000>;
using Crc32IsoHdlc = Spec<32, 0x04C11DB7, 0xFFFFFFFF, true, true, 0xFFFFFFFF>;

// Install at most one backend during start-up, before any concurrent CRC use.
// Passing nullptr restores the pure software path.
inline void InstallBackend(const Backend* backend) noexcept
{
  detail::installed_backend.store(backend, std::memory_order_release);
}

[[nodiscard]] inline const Backend* InstalledBackend() noexcept
{
  return detail::installed_backend.load(std::memory_order_acquire);
}

namespace detail
{

// Never waits: a busy, absent or unsupporting backend falls back to software.
template <typename S, typename Byte>
constexpr typename S::Value Dispatch(std::span<const Byte> data) noexcept
{
  if (!std::is_constant_evaluated())
  {
    const auto* backend = InstalledBackend();
    if (backend != nullptr && backend->supports(backend->context, S::kParams) &&
        !backend_busy.test_and_set(std::memory_order_acquire))
    {
      const auto value = backend->compute(backend->context, S::kParams, std::as_bytes(data));
      backend_busy.clear(std::memory_order_release);
      return static_cast<typename S::Value>(value);
    }
  }
  return S::Software(data);
}

}  // namespace detail

template <typename S>
[[nodiscard]] constexpr typename S::Value Compute(std::span<const std::byte> data) noexcept
{
  return detail::Dispatch<S>(data);
}

template <typename S>
[[nodiscard]] constexpr typename S::Value Compute(std::span<const std::uint8_t> data) noexcept
{
  return detail::Dispatch<S>(data);
}

}  // namespace crc
