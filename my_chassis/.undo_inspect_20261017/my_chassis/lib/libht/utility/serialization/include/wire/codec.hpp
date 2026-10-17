#pragma once

#include <array>
#include <bit>
#include <climits>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <tuple>
#include <type_traits>
#include <utility>

#if defined(WIRE_BACKEND_PFR) && defined(WIRE_BACKEND_ZPP)
#error "Select one wire backend consistently for the whole program"
#endif
#if defined(WIRE_BACKEND_PFR)
#include <boost/pfr/core.hpp>
#else
#include <zpp_bits.h>
#endif

namespace wire
{
static_assert(CHAR_BIT == 8);
static_assert(sizeof(bool) == 1);
static_assert(std::endian::native == std::endian::little ||
              std::endian::native == std::endian::big);

enum class Endian
{
  Little,
  Big
};
enum class Status
{
  Ok,
  OutputTooSmall,
  InvalidLength,
  InvalidValue
};
struct Result
{
  Status status{Status::InvalidValue};
  std::size_t bytes{};
  explicit constexpr operator bool() const noexcept { return status == Status::Ok; }
};

template <class T>
struct Codec;
template <class T>
struct Schema;

template <class T>
concept Supported =
    !std::is_const_v<T> && !std::is_volatile_v<T> && !std::is_reference_v<T> &&
    requires(const T& source, T& target, std::span<std::byte> output,
             std::span<const std::byte> input) {
      typename std::integral_constant<std::size_t, Codec<T>::MinEncodedSize>;
      typename std::integral_constant<std::size_t, Codec<T>::MaxEncodedSize>;
      requires Codec<T>::MinEncodedSize <= Codec<T>::MaxEncodedSize;
      requires Codec<T>::MaxEncodedSize < std::numeric_limits<std::size_t>::max();
      {
        Codec<T>::Encode(source, output, Endian::Little)
      } noexcept -> std::same_as<Result>;
      {
        Codec<T>::Decode(input, target, Endian::Little)
      } noexcept -> std::same_as<Result>;
    };

template <Supported T>
inline constexpr std::size_t MinEncodedSize = Codec<T>::MinEncodedSize;
template <Supported T>
inline constexpr std::size_t MaxEncodedSize = Codec<T>::MaxEncodedSize;
template <class T>
concept FixedSize = Supported<T> && (MinEncodedSize<T> == MaxEncodedSize<T>);
template <FixedSize T>
inline constexpr std::size_t EncodedSize = MaxEncodedSize<T>;

namespace detail
{
template <class... T>
struct Types
{
};
template <class T>
struct Array
{
  static constexpr bool value = false;
};
template <class T, std::size_t N>
struct Array<std::array<T, N>>
{
  static constexpr bool value = true;
  using Element = T;
  static constexpr std::size_t Count = N;
};
template <class T>
struct Member;
template <class T, class C>
struct Member<T C::*>
{
  using Type = T;
};
template <class T>
concept HasSchema = requires { Schema<T>::Fields; };

template <class T>
constexpr auto FieldTypes()
{
  if constexpr (HasSchema<T>)
  {
    return std::apply([](auto... member)
                      { return Types<typename Member<decltype(member)>::Type...>{}; },
                      Schema<T>::Fields);
  }
#if defined(WIRE_BACKEND_PFR)
  else
  {
    return []<std::size_t... I>(std::index_sequence<I...>)
    {
      return Types<boost::pfr::tuple_element_t<I, T>...>{};
    }(std::make_index_sequence<boost::pfr::tuple_size_v<T>>{});
  }
#else
  else
  {
    return zpp::bits::visit_members_types<T>([]<class... F>() { return Types<F...>{}; });
  }
#endif
}

template <class T, class F>
constexpr void Each(T& value, F&& function) noexcept
{
  using U = std::remove_cvref_t<T>;
  if constexpr (HasSchema<U>)
    std::apply([&](auto... member) { (function(value.*member), ...); },
               Schema<U>::Fields);
#if defined(WIRE_BACKEND_PFR)
  else
    boost::pfr::for_each_field(value, std::forward<F>(function));
#else
  else
    zpp::bits::visit_members(value, [&](auto&... member) { (function(member), ...); });
#endif
}

template <class T>
consteval bool Scalar()
{
  return std::same_as<T, bool> || std::same_as<T, std::int8_t> ||
         std::same_as<T, std::uint8_t> || std::same_as<T, std::int16_t> ||
         std::same_as<T, std::uint16_t> || std::same_as<T, std::int32_t> ||
         std::same_as<T, std::uint32_t> || std::same_as<T, std::int64_t> ||
         std::same_as<T, std::uint64_t> ||
         (std::same_as<T, float> && sizeof(T) == 4 &&
          std::numeric_limits<T>::is_iec559) ||
         (std::same_as<T, double> && sizeof(T) == 8 && std::numeric_limits<T>::is_iec559);
}

template <class T>
consteval bool Ordinary()
{
  if constexpr (std::is_const_v<T> || std::is_volatile_v<T> || std::is_reference_v<T>)
    return false;
  else if constexpr (Scalar<T>())
    return true;
  else if constexpr (std::is_enum_v<T>)
    return Scalar<std::underlying_type_t<T>>();
  else if constexpr (Array<T>::value)
    {
    using E = typename Array<T>::Element;
    if constexpr (!FixedSize<E>) return false;
    else return EncodedSize<E> == 0 ||
                Array<T>::Count <= (std::numeric_limits<std::size_t>::max() - 1) / EncodedSize<E>;
  }
  else if constexpr (std::is_class_v<T> && std::is_aggregate_v<T> &&
                     std::is_trivially_copyable_v<T> &&
                     std::is_default_constructible_v<T> &&
                     std::is_nothrow_copy_constructible_v<T> &&
                     std::is_nothrow_copy_assignable_v<T>)
  {
#if !defined(WIRE_BACKEND_PFR)
    if constexpr (!HasSchema<T> && zpp::bits::concepts::has_explicit_serialize<T>)
      return false;
#endif
    return []<class... F>(Types<F...>)
    {
      if constexpr (!(FixedSize<F> && ...)) return false;
      else
      {
        std::size_t total = 0;
        for (const auto size : std::array<std::size_t, sizeof...(F)>{EncodedSize<F>...})
        {
          if (size >= std::numeric_limits<std::size_t>::max() - total) return false;
          total += size;
        }
        return true;
      }
    }(FieldTypes<T>());
  }
  else
    return false;
}

template <class T>
consteval std::size_t Size()
{
  if constexpr (Scalar<T>() || std::is_enum_v<T>)
    return sizeof(T);
  else if constexpr (Array<T>::value)
    return Array<T>::Count * EncodedSize<typename Array<T>::Element>;
  else
    return []<class... F>(Types<F...>)
    { return (std::size_t{} + ... + EncodedSize<F>); }(FieldTypes<T>());
}

template <class T>
consteval bool Pure()
{
  if constexpr (!requires { typename Codec<T>::AutomaticType; })
    return false;
  else if constexpr (!std::same_as<typename Codec<T>::AutomaticType, T>)
    return false;
  else if constexpr (HasSchema<T>)
    return false;
  else if constexpr (Scalar<T>() || std::is_enum_v<T>)
    return true;
  else if constexpr (Array<T>::value)
    return Pure<typename Array<T>::Element>();
  else
    return []<class... F>(Types<F...>) { return (Pure<F>() && ...); }(FieldTypes<T>());
}

template <class T>
constexpr void Normalize(std::span<std::byte> bytes, std::size_t& cursor) noexcept
{
  if constexpr (std::same_as<T, bool>)
  {
    bytes[cursor] = bytes[cursor] == std::byte{} ? std::byte{} : std::byte{1};
    ++cursor;
  }
  else if constexpr (Scalar<T>() || std::is_enum_v<T>)
    cursor += sizeof(T);
  else if constexpr (Array<T>::value)
    for (std::size_t i = 0; i < Array<T>::Count; ++i)
      Normalize<typename Array<T>::Element>(bytes, cursor);
  else
    []<class... F>(Types<F...>, auto& data, auto& offset)
    { (Normalize<F>(data, offset), ...); }(FieldTypes<T>(), bytes, cursor);
}

template <class T, bool = Ordinary<T>()>
struct DefaultCodec
{
};
template <class T>
struct DefaultCodec<T, true>
{
  static constexpr std::size_t MinEncodedSize = Size<T>();
  static constexpr std::size_t MaxEncodedSize = MinEncodedSize;

  static constexpr Result Encode(const T& value, std::span<std::byte> output,
                                 Endian endian) noexcept
  {
    if (output.size() < MaxEncodedSize) return {Status::OutputTooSmall, 0};
#if !defined(WIRE_BACKEND_PFR)
    if constexpr (Pure<T>())
    {
      auto run = [&](auto order)
      {
        zpp::bits::out archive(output, decltype(order){});
        return zpp::bits::success(archive(value)) && archive.position() == MaxEncodedSize;
      };
      const bool ok = endian == Endian::Little ? run(zpp::bits::endian::little{})
                                               : run(zpp::bits::endian::big{});
      return ok ? Result{Status::Ok, MaxEncodedSize} : Result{};
    }
    else
#endif
        if constexpr (Scalar<T>() || std::is_enum_v<T>)
    {
      if constexpr (std::same_as<T, bool>)
        output[0] = value ? std::byte{1} : std::byte{};
      else
      {
        const auto bytes = std::bit_cast<std::array<std::byte, sizeof(T)>>(value);
        const bool reverse =
            (endian == Endian::Little) != (std::endian::native == std::endian::little);
        for (std::size_t i = 0; i < sizeof(T); ++i)
          output[i] = bytes[reverse ? sizeof(T) - 1 - i : i];
      }
    }
    else
    {
      std::size_t cursor = 0;
      Result result{Status::Ok, 0};
      auto field = [&](const auto& member)
      {
        using U = std::remove_cvref_t<decltype(member)>;
        if (!result) return;
        result = Codec<U>::Encode(member, output.subspan(cursor, ::wire::EncodedSize<U>),
                                  endian);
        if (result && result.bytes != ::wire::EncodedSize<U>) result = {};
        cursor += ::wire::EncodedSize<U>;
      };
      if constexpr (Array<T>::value)
      {
        for (const auto& member : value) field(member);
      }
      else
        Each(value, field);
      if (!result) return {result.status, 0};
    }
    return {Status::Ok, MaxEncodedSize};
  }

  static constexpr Result Decode(std::span<const std::byte> input, T& value,
                                 Endian endian) noexcept
  {
    if (input.size() != MaxEncodedSize) return {Status::InvalidLength, 0};
#if !defined(WIRE_BACKEND_PFR)
    if constexpr (Pure<T>())
    {
      std::array<std::byte, MaxEncodedSize> normalized{};
      for (std::size_t i = 0; i < MaxEncodedSize; ++i) normalized[i] = input[i];
      std::size_t cursor = 0;
      Normalize<T>(normalized, cursor);
      auto run = [&](auto order)
      {
        zpp::bits::in archive(normalized, decltype(order){});
        return zpp::bits::success(archive(value)) && archive.position() == MaxEncodedSize;
      };
      const bool ok = endian == Endian::Little ? run(zpp::bits::endian::little{})
                                               : run(zpp::bits::endian::big{});
      return ok ? Result{Status::Ok, MaxEncodedSize} : Result{};
    }
    else
#endif
        if constexpr (Scalar<T>() || std::is_enum_v<T>)
    {
      if constexpr (std::same_as<T, bool>)
        value = input[0] != std::byte{};
      else
      {
        std::array<std::byte, sizeof(T)> bytes{};
        const bool reverse =
            (endian == Endian::Little) != (std::endian::native == std::endian::little);
        for (std::size_t i = 0; i < sizeof(T); ++i)
          bytes[i] = input[reverse ? sizeof(T) - 1 - i : i];
        value = std::bit_cast<T>(bytes);
      }
    }
    else
    {
      std::size_t cursor = 0;
      Result result{Status::Ok, 0};
      auto field = [&](auto& member)
      {
        using U = std::remove_cvref_t<decltype(member)>;
        if (!result) return;
        result = Codec<U>::Decode(input.subspan(cursor, ::wire::EncodedSize<U>), member,
                                  endian);
        if (result && result.bytes != ::wire::EncodedSize<U>) result = {};
        cursor += ::wire::EncodedSize<U>;
      };
      if constexpr (Array<T>::value)
      {
        for (auto& member : value) field(member);
      }
      else
        Each(value, field);
      if (!result) return {result.status, 0};
    }
    return {Status::Ok, MaxEncodedSize};
  }
};
}  // namespace detail

template <class T>
struct Codec : detail::DefaultCodec<T>
{
  using AutomaticType = T;
};

// These checked entry points commit only a complete, successful codec result.
template <Supported T>
constexpr Result Encode(const T& value, std::span<std::byte> output,
                        Endian endian = Endian::Little) noexcept
{
  if (output.size() < MinEncodedSize<T>) return {Status::OutputTooSmall, 0};
  std::array<std::byte, MaxEncodedSize<T>> temporary{};
  const auto result = Codec<T>::Encode(value, temporary, endian);
  if (!result) return {result.status, 0};
  if (result.bytes < MinEncodedSize<T> || result.bytes > MaxEncodedSize<T>) return {};
  if (output.size() < result.bytes) return {Status::OutputTooSmall, 0};
  for (std::size_t i = 0; i < result.bytes; ++i) output[i] = temporary[i];
  return result;
}

template <Supported T>
  requires(std::is_nothrow_copy_constructible_v<T> &&
           std::is_nothrow_copy_assignable_v<T>)
constexpr Result Decode(std::span<const std::byte> input, T& value,
                        Endian endian = Endian::Little) noexcept
{
  if (input.size() < MinEncodedSize<T> || input.size() > MaxEncodedSize<T>)
    return {Status::InvalidLength, 0};
  T temporary = value;
  const auto result = Codec<T>::Decode(input, temporary, endian);
  if (!result) return {result.status, 0};
  if (result.bytes < MinEncodedSize<T> || result.bytes > MaxEncodedSize<T> ||
      result.bytes != input.size()) return {};
  value = temporary;
  return result;
}
}  // namespace wire
