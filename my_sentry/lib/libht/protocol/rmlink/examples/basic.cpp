#include "rmlink/rmlink.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace {

struct Sample {
  std::uint16_t value{};
};
static_assert(::wire::EncodedSize<Sample> == 2);

struct Backend {
  std::uint64_t now_us{};

  static std::uint64_t Now(void *context) noexcept {
    return static_cast<Backend *>(context)->now_us;
  }
  static rmlink::SendResult Send(void *, std::span<const std::byte>,
                                 std::uint64_t) noexcept {
    return rmlink::SendResult::Busy;
  }
  static void Abort(void *) noexcept {}
};

void OnBinary(void *, std::span<const std::byte>,
              const rmlink::EventMetadata &) noexcept {}

void OnSample(void *, const Sample &,
              const rmlink::EventMetadata &) noexcept {}

} // namespace

int main() {
  Backend backend{};
  rmlink::TypedStream<Sample> typed_stream(2, nullptr, &OnSample);
  const std::array streams{
      rmlink::StreamRegistration{1, nullptr, &OnBinary},
      typed_stream.registration(),
  };
  rmlink::Config config{};
  config.role = rmlink::Role::Host;
  config.streams = streams;
  rmlink::Core core(config,
                    {&backend, &Backend::Now, &Backend::Send, &Backend::Abort});
  if (core.initialization_status() != rmlink::Status::Ok) {
    return 1;
  }
  return core.Begin() == rmlink::Status::Ok ? 0 : 1;
}
