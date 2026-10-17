# Shared payload codec validation

The report below is historical codec acceptance. Current library-only native
commands are documented in `../../README.md`; `../../tests/run_matrix.py` runs
standalone backend/standard configurations. Robot integration and hardware
acceptance belong to the consumer project. Historical paths/hashes below retain
their original meaning; they are not current reproduction commands.

UTC: 2026-10-06T19:29:40.650270+00:00

## Software evidence

Final matrix: 15 isolated configurations, 401 CTest executions passed. Counts include repeated backend/standard configurations, not distinct specification scenarios.

| Suite | zpp | PFR |
| --- | --- | --- |
| Library C++20 | 13 passed | 13 passed |
| Library C++23 | 13 passed | 13 passed |
| Application payloads | 12 passed | 12 passed |
| BtoBOnCan | 17 passed | 17 passed |
| RMLink core/wire/example | 43 passed | 43 passed |
| RMLink LibXR ports/Topic | 18 passed | 18 passed |
| CAN manager/motor integration | 78 passed | 78 passed |
| ASan/UBSan library | 13 passed | not separately run |

The library runs scoped allocation checks and C++20/C++23 native/Arm compile and invalid-type rejection probes. The inherited-external-codec regression protects explicit selection from bulk-backend bypass. Original application baseline before migration was 11 passed; a classic-packet golden-vector test was added during migration.

## Final firmware and simulation

- debug: GNU Arm build passed; Renode 9/9 scenarios passed.
  ELF SHA-256: `d0fdb2c5b23c0c831d201227b4f223f5eda7363de2aa91cd9472f01c94a5cf5a`.
  Robot result: `build/firmware/debug/renode-results/20261007-032652/robot_output.xml`.
- release: GNU Arm build passed; Renode 9/9 scenarios passed.
  ELF SHA-256: `a86babd7148ab39e27acb965a38eb7af223719c5ad7619868e9720e229fb5176`.
  Robot result: `build/firmware/release/renode-results/20261007-032659/robot_output.xml`.

Debug linked FLASH 201112 B, DTCMRAM 119920 B and RAM_D3 1952 B; Release FLASH 198684 B, DTCMRAM 119888 B and RAM_D3 1952 B. Both fit current linker regions. No previous full-firmware size delta is claimed.

Initial sandbox Renode attempts failed before scenarios due configuration locking/local socket restrictions. Logs were retained separately; the pending sandbox retry was interrupted after a loopback PermissionError. Final runs used isolated build-directory configuration and an execution environment permitting local sockets. These startup failures are not counted as scenario passes.

## Arm resource comparison

Cortex-M7 hard-float, GNU Arm 15.3.1, C++23, no exceptions/RTTI, -O2:

| 34-byte Gimbal path | Entry point bytes | Owned call-chain stack bytes |
| --- | ---: | ---: |
| hand-written encode | 100 | 32 |
| hand-written decode | 130 | 36 |
| zpp encode | 130 | 48 |
| pfr encode | 198 | 80 |
| zpp decode | 214 | 96 |
| pfr decode | 204 | 80 |

The initial implementation stages codec output/decoded objects; zpp additionally normalizes bool input. Its decode stack is larger than the hand-written reference. The canonical-bool shortcut was explored but is not enabled in production. No universal speed/size improvement is claimed.

Full -O2/-Os section and callgraph reports are in `build/codec-resources/resources.json`, with disassembly, symbols, .su and .ci artifacts. Whole-object totals include reference plus backend functions. Some PFR paths call external memset; those callee stacks are explicitly unmeasured. No allocation/throw symbols were found in the probes. These reports exclude interrupt/RTOS frames and are not a complete system ISR budget.

## Timing and remaining work

Host benchmark JSON for both backends is in `build/codec-resources/host-zpp.json` and `host-pfr.json`: 100000 iterations per direction, all four payload sizes, hand-written references, canonical/noncanonical bool inputs, checksums, source hashes and host/compiler identification. Host timing is not MCU timing.

The DWT object target compiled after the final header revision. `benchmarks/README.md` documents selected-backend and hand-reference entry points and the board result template. No board timing or flashing was performed. OpenSpec task 7.4 remains unchecked; full hardware/resource acceptance is not complete.

## Reproduction and provenance

Run `python3 lib/libht/Utility/Serialization/tests/run_matrix.py --output build/codec-validation` from the project root. Resource, host and DWT commands are in `benchmarks/README.md`. Firmware uses `cmake --preset debug|release`, the corresponding build preset and test preset. Refresh the generated XRobot header after input changes; do not disable its freshness check.

Build evidence: `build/codec-validation/matrix.json` and per-configuration logs; `build/codec-resources/acceptance.json`, `resources.json`, host benchmark JSON and separate Renode final logs. Renode robot XML paths are listed above. Reproduce evidence after moving the checkout; configuration does not depend on a particular user directory.

Git HEADs below identify source repositories, not clean/published revisions. Source hashes in acceptance.json identify working payload/codec files. No commit, push or hardware deployment was performed; pre-existing src/main.cpp changes were preserved.

- project: `8447d0a3d24bc33b753286cd9ea71d750485f5db`
- libht: `d349aae52bc4682b034de8b1d10e624f68533fc8`
- CanManager: `f862c38ac615ba143de175bb1a7fa9f9364a210c`
- BtoBOnCan: `0be24459f33072c80c28703d8cf7ae7d9d70f6b9`

## Subsequent real-board measurement, 2026-10-07

After the historical software report above, the xr-example user connected a
STM32H723VGT6 and explicitly requested direct hardware verification. The pending
OpenSpec 7.4 task is now checked based on 1456 valid DWT records across zpp/PFR,
O2/Os and cache off/on, with exact source/ELF/tool hashes retained by the consumer.
See `openspec/changes/add-shared-payload-codec/HARDWARE_VALIDATION.md` in xr-example.
The board used RAM code/data, HSI nominal 64MHz and IRQ disabled; these results do
not replace production Flash, peripheral integration or full ISR/RTOS acceptance.
The earlier not-run statements describe the prior report's date, not this later run.
