# Codec resource and timing evidence

Run from the LibHT root. Dependency pins are in `../third_party/versions.json`.
`reference_can.hpp` preserves the pre-migration 34/37-byte field layout and
hand-written methods; only its namespace and standalone integer helper wiring
were changed. The 8-byte classic and 40-byte VQF references in
`resource_probe.cpp` reproduce the original fixed field operations. `payloads.hpp` supplies standalone sample types. Consumer application
golden-vector tests independently check their real payload types. These references
are test/measurement code, never an application codec fallback.

## Arm object sizes and stack

```sh
python3 utility/serialization/benchmarks/resources.py --output build/codec-resources
```

The script discovers GNU Arm tools from PATH, uses Cortex-M7 hard-float, C++23,
no exceptions/RTTI, and compares `-O2`/`-Os` under both backends. It saves object
hashes, exact flags, source hashes, section sizes, symbols, disassembly, `.su`
and GCC callgraph files. `resources.json` distinguishes each entry point from
its reachable owned helpers. The whole object includes both the hand reference
and the selected backend: its total is not a backend-only firmware Flash size.
Shared helper bytes must not be summed twice across entry points.

Stack sums use the longest visible owned call chain, including saved registers
as reported by GCC. External library callees (for example `memset` in a PFR
`-Os` path) are listed explicitly; their stack is not treated as measured zero.
These numbers exclude RTOS/interrupt frames and do not establish a system ISR
budget. Undefined allocation/throw symbols cause the script to fail. Source
review and the scoped allocation test complement that check.

## Host microbenchmark

```sh
cmake -S utility/serialization/benchmarks -B build/codec-benchmark/zpp -G Ninja -DWIRE_BACKEND=zpp
cmake --build build/codec-benchmark/zpp --parallel 4
build/codec-benchmark/zpp/wire_host_benchmark
cmake -S utility/serialization/benchmarks -B build/codec-benchmark/pfr -G Ninja -DWIRE_BACKEND=pfr
cmake --build build/codec-benchmark/pfr --parallel 4
build/codec-benchmark/pfr/wire_host_benchmark
```

Output is JSON: 100000 calls per direction, payload size, host steady-clock
nanoseconds per call, checksum, and noncanonical-bool case. Mutating inputs,
observable output checksums and noinline codec entry points prevent elimination
of the measured work. The 8/34/37/40-byte payloads and hand-written references
use the same capacity/transaction boundary. This is host throughput evidence;
it is not a Cortex-M7 cycle estimate. Record host, compiler, flags, source
versions and machine load with the output, and repeat before drawing conclusions.

## Cortex-M7 DWT entry point

```sh
cmake -S utility/serialization/benchmarks -B build/codec-benchmark/dwt -G Ninja -DCMAKE_SYSTEM_NAME=Generic -DCMAKE_SYSTEM_PROCESSOR=arm \
  -DCMAKE_CXX_COMPILER=arm-none-eabi-g++ -DCMAKE_TRY_COMPILE_TARGET_TYPE=STATIC_LIBRARY \
  -DCMAKE_CXX_FLAGS="-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard"
cmake --build build/codec-benchmark/dwt --parallel 4
```

This compiles object files only. It neither flashes nor runs a board and does
not alter the application entry point. A user-owned bench firmware can add
`resource_probe.cpp` and `dwt_probe.cpp`, link `LibHT::serialization`, include
the benchmark directory containing `payloads.hpp`, then call:

```cpp
if (WireBenchmarkDwtInit()) {
  auto reference = WireBenchmarkDwtRunReference(1000);
  auto canonical = WireBenchmarkDwtRun(1000);
  auto noncanonical = WireBenchmarkDwtRun(1000, true);
  // Inspect/store results after timing; do not print inside measured loops.
}
```

Compile that bench separately with `WIRE_BACKEND=zpp` and `pfr`. The helper
enables Cortex-M7 trace/CYCCNT, checks NOCYCCNT and unlocks the M7 DWT. It leaves
clocks, interrupts and caches as configured by the caller. Run only after board
initialization; record CPU frequency, memory placement, optimization, cache and
interrupt state. Results include loop/barrier/checksum overhead. Keep the run
shorter than one 32-bit counter wrap, warm up, repeat and retain distributions.
The selected codec and reference receive equal iteration counts. No CAN, motor
or USB commands are emitted by the harness.

| Field | Required record |
| --- | --- |
| Board / compiler / backend | exact device, tool version and dependency pins |
| Source / ELF | commit and working-source hashes, ELF SHA-256 |
| CPU / cache / IRQ / placement | clock, cache/IRQ state, code/data memory |
| Input / iterations | canonical and noncanonical flags, iteration count |
| Output | valid flag, checksum, total encode/decode cycles, repetitions |
| Status | not run / failed / passed; retain original failure logs |

New board timing results require explicit user-requested execution on an available
platform. Object compilation, host timing and Renode functional regression must
not be marked as board timing or complete hardware acceptance.

`WireBenchmarkDwtRunBounded(iterations, content_length)` adds the standalone
6..118-byte owned-message example, with content lengths 0..112 (use 20 and 112
for 26/118-byte results). Its maximum scratch remains 118 bytes. It is compiled
with both backends in the consumer resource check; compiling the entry is not
board timing. The consumer's real-board results are tracked separately from this compilation check.

Robot consumers additionally measure actual device codecs and Topic queue
layouts from their firmware compile commands. In xr-example, run:

```sh
python3 tests/payload_resources.py \
  --compile-commands build/h723-demo/debug/compile_commands.json \
  --output build/unify-codecs/resources
```

This consumer probe stays outside LibHT. It records Arm object sizes, section
sizes, per-entry owned call-chain stack, disassembly, unresolved callees and DWT
compile results. Queue storage includes the SPSC spare slot and excludes
allocator metadata. These are separate from code/static RAM and MCU cycles;
ISR/RTOS frames and unmeasured external calls must be budgeted separately.

## Executed STM32H723 comparison

On 2026-10-07, the xr-example user connected an STM32H723VGT6 and requested direct
verification. `tests/hardware/codec_dwt.py` builds separate zpp/PFR, O2/Os RAM ELF
images and executes them over CMSIS-DAP with a registered GDB completion breakpoint.
The code runs from AXI SRAM with data/stack in DTCM, HSI nominal 64 MHz, IRQ masked,
and I/D caches off/on. Each case warms up twice and measures thirteen batches of
1000 encode/decode calls. All 1456 captured rows passed validity/checksum checks.

The robot workspace report is `openspec/changes/add-shared-payload-codec/HARDWARE_VALIDATION.md`.
It records the source/ELF/tool hashes and distributions, including the substantial
PFR -Os slowdown. This is RAM codec timing, separate from production Flash/clock,
peripheral integration, or a complete system interrupt budget. The runner verifies RAM LOAD segments and GDB readback before measurement.
