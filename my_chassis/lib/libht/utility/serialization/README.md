# Bounded payload codecs

`LibHT::serialization` is a C++20 header-only library shared by CAN and RMLink.
Normal wire payloads contain only their data members; no field registration or
encoding member functions are required. The default backend is the pinned
zpp::bits commit in `third_party/versions.json`. Configure `WIRE_BACKEND=pfr` in
a separate build directory to use the pinned Boost.PFR implementation.

```cmake
add_subdirectory(path/to/LibHT/utility/serialization)
target_link_libraries(consumer PUBLIC LibHT::serialization)
```

```cpp
#include <wire/codec.hpp>
struct Telemetry {
  uint16_t sequence{};
  bool alive{};
  std::array<float, 3> velocity{};
};
static_assert(wire::EncodedSize<Telemetry> == 15);
std::array<std::byte, wire::EncodedSize<Telemetry>> bytes{};
Telemetry input{}, output{};
auto sent = wire::Encode(input, bytes, wire::Endian::Little);
auto received = wire::Decode(bytes, output, wire::Endian::Little);
```

Check `Result::status` (`Ok`, `OutputTooSmall`, `InvalidLength`, `InvalidValue`)
or its explicit bool conversion. Success reports the actual byte count; failure
reports zero. `MinEncodedSize<T>` and `MaxEncodedSize<T>` are compile-time bounds;
`FixedSize<T>` means the bounds match, and only then is `EncodedSize<T>` available.
Encode needs capacity for the actual result, even if smaller than the maximum,
and preserves the unused suffix. Decode requires input within the bounds and
successful consumption of every byte; fixed input must have the exact length. Checked `wire::Encode/Decode`
stage their work and commit only on success, even when a custom codec writes
before returning failure. The underlying `Codec<T>` operations are formatting
operations; call the checked functions at transport/application boundaries.

## Wire contract

Fixed-width integers and enums use the selected byte order. Floats are IEEE-754
binary32/binary64 with bit patterns preserved. A bool occupies one byte, emits
0/1 and accepts any nonzero input as true. zpp input is normalized before any
bool object is read. Arrays have no length prefix; aggregates have no padding,
field names, tags or version bytes. Source declaration order is wire order.
Use independent golden vectors to detect accidental field reordering.

Automatic types are normally aligned aggregates without bases, references,
const/volatile fields, dynamic containers, pointers, unions or C arrays. Use
`std::array` instead of `T[N]`. Automatic reflection is not proof of packed
member alignment: packed structs and bitfields require an explicit codec or
conversion to an ordinary wire type. Backend-native serialization hooks are
not an alternative to the shared extension contract. Both backends use the
same supported types and protocol rules; keep backend definitions consistent
across every translation unit. No runtime backend dispatch is used.

## Extensions

A type-external `wire::Codec<T>` specialization supplies constexpr `std::size_t`
`MinEncodedSize` and `MaxEncodedSize` (equal for fixed formats), and
noexcept `Encode(const T&, span<byte>, Endian)` / `Decode(span<const byte>, T&,
Endian)`, returning `wire::Result`. It owns its format, validation and bool
representation. Custom code must not allocate, block, lock or throw. Correct
buffer access is its responsibility; the checked public boundary protects
caller data on failure, but cannot make an out-of-bounds custom codec safe.

For selecting or reordering normally aligned members, define one list:

```cpp
struct Selected { uint16_t first{}; uint8_t local{}; uint16_t last{}; };
template <> struct wire::Schema<Selected> {
  static constexpr auto Fields = std::tuple{&Selected::last, &Selected::first};
};
```

Decode preserves unselected members. Explicit codec > Schema > automatic
aggregate is the shared selection order. Nested extension nodes are processed
through that same contract; they are not handed to an incompatible bulk
backend operation. Automatic aggregates, arrays and Schema lists accept only
fixed-length children; a variable member, including a final member, needs an
explicit enclosing codec. Bounds must satisfy min <= max < SIZE_MAX; missing,
contradictory or unbounded metadata and length sum/product overflow are rejected.
There is no adapter for old EncodedSize-only codec metadata.

`tests/usage_example.cpp` compiles ordinary, Schema, fixed external and bounded
external examples. `tests/bounded_payload.hpp` contains a complete 6..118-byte
message codec: three header fields plus 0..112 owned content bytes. Its memory
length never crosses the wire. Twenty content bytes encode into a 26-byte output;
Decode derives the length from the input and clears the unused owned tail. The
format codec delegates fixed header fields to shared operations inside the outer
transaction. Checked entry points reject successful results outside the bounds,
partial decode consumption and any failed formatting result without committing.
No container length prefix or Measure call is added.

## Bounded bit operations

`<wire/bits.hpp>` supplies `ReadBits`, `WriteBits` and `SignExtend`. Bit offsets
are measured from the start of the span. `BitOrder::Lsb0` numbers each byte from
its low bit and places the first field bit at the low end of the value (11-bit
remote channels). `BitOrder::Msb0` numbers from the high bit and places the first
field bit at the high end of the value (DM 12-bit fields). This is independent
of `Endian` for ordinary integer fields; no host integer pointers are used.

```cpp
#include <wire/bits.hpp>
std::array<std::byte, 2> bytes{};
auto written = wire::WriteBits(bytes, 0, 11, 0x400, wire::BitOrder::Msb0);
std::uint64_t bits{};
auto read = wire::ReadBits(bytes, 0, 11, bits, wire::BitOrder::Msb0);
std::int64_t temperature{};
auto extended = wire::SignExtend(bits, 11, temperature); // -1024
```

Widths are 1..64. Every access validates the complete span range and offset
arithmetic before reading or writing; writes also reject values outside the
unsigned field range. A failure returns zero bytes and preserves the output or
target. Successful bit access reports the number of touched bytes; SignExtend
reports zero bytes because it accesses no buffer. Unselected bits are preserved;
protocol codecs must initialize reserved bits according to their own format.
No new reserved-bit validation is imposed on received protocol data. All three
operations are constexpr/noexcept, bounded to at most 64 iterations, and allocate
no memory. Full 64-bit sign conversion avoids shifts by 64 and signed overflow.
`tests/compile_probe.cpp` compiles the example's operations for both host and Arm.

## Verification

From the LibHT root, use the standalone native XMake graph:

```sh
xmake f -P . -m debug --tests=y --wire_backend=zpp --wire_standard=cxx20
xmake test -P . -j 4 -g tests
python3 tests/run_matrix.py --output build/matrix
python3 tests/test_standalone.py --output build/isolation
```

The matrix runs zpp/PFR with C++20/C++23 and preserves logs. The isolation test
copies only LibHT into a temporary workspace and checks its compile commands
for consumer-framework leakage. Both return a nonzero status on failure.
GoogleTest is discovered through pkg-config; set PKG_CONFIG_PATH as needed.
The standalone CMake workflow remains available:

```sh
cmake -S utility/serialization/tests -B build/serialization/zpp -DWIRE_BACKEND=zpp
cmake --build build/serialization/zpp --parallel 4
ctest --test-dir build/serialization/zpp --output-on-failure
```

Set `CMAKE_CXX_STANDARD=23` to run the suite in C++23. The compile-check script
also tests C++20/C++23 and rejected types, and checks Cortex-M7 hard-float
without exceptions/RTTI when `arm-none-eabi-g++` is available on PATH. The
allocation test counts allocations only during prepared codec calls.

Encode scratch uses `MaxEncodedSize<T>` bytes even for short messages; Decode
stages a copy of the current object to preserve unselected Schema fields.
Transaction buffers and decoded objects have bounded stack cost, not zero cost.
Transport capacities must be checked before instantiating buffers. Compiler
stack reports and real measurements, including failed inputs, are needed for
an ISR budget. Host or simulator timing must not be reported as board timing.
