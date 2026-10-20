#pragma once

#include "rmlink/types.hpp"
#include "wire/codec.hpp"

#include <array>
#include <concepts>
#include <cstddef>
#include <span>
#include <type_traits>

namespace rmlink {

template <class T>
concept StructuredPayload = std::default_initializable<T> && ::wire::Supported<T> &&
    (::wire::MaxEncodedSize<T> <= kMaxPayloadSize) &&
    std::is_nothrow_copy_constructible_v<T> && std::is_nothrow_copy_assignable_v<T>;

template <StructuredPayload T> class TypedStream {
public:
  using Callback = void (*)(void *context, const T &value,
                            const EventMetadata &metadata) noexcept;

  TypedStream(std::uint16_t stream_id, void *context,
              Callback callback) noexcept
      : registration_{stream_id, this, &TypedStream::DecodeThunk},
        context_(context), callback_(callback) {}

  [[nodiscard]] const StreamRegistration &registration() const noexcept {
    return registration_;
  }
  [[nodiscard]] std::uint64_t decode_errors() const noexcept {
    return decode_errors_;
  }

private:
  static void DecodeThunk(void *context, std::span<const std::byte> payload,
                          const EventMetadata &metadata) noexcept {
    auto &self = *static_cast<TypedStream *>(context);
    const auto result = ::wire::Decode(payload, self.value_);
    if (!result) {
      ++self.decode_errors_;
      return;
    }
    if (self.callback_ != nullptr) {
      self.callback_(self.context_, self.value_, metadata);
    }
  }

  StreamRegistration registration_{};
  void *context_{};
  Callback callback_{};
  T value_{};
  std::uint64_t decode_errors_{};
};

} // namespace rmlink
