// Copyright 2026 Khalil Estell and the libhal contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

module;

#include <span>

export module hal.arm_mcu.cortex_m:stack_trace;

import hal;

namespace hal::cortex_m {
/**
 * @brief Record the calls that led to the caller
 *
 * Walks the stack from the caller outward and writes one address per call,
 * the innermost first. Each address is inside the call instruction, so a
 * symbolizer maps it to the line that made the call:
 *
 * ```
 * llvm-symbolizer --obj=app.elf --inlines 0x08001a3e 0x080020f2 ...
 * arm-none-eabi-addr2line -e app.elf -f -C -i 0x08001a3e 0x080020f2 ...
 * ```
 *
 * It is meant for terminate and fault handlers: it allocates nothing, never
 * throws, and only reads memory inside RAM. It runs in an interrupt too.
 * There, the walk continues past the interrupt into the code it interrupted.
 * That address is the interrupted instruction itself, not a call.
 *
 * The walk follows the Arm exception tables (`.ARM.exidx`). This library
 * compiles itself and the application with `-funwind-tables`, so every
 * function gets unwind information, including those that can't throw, and a
 * function that never returns still saves its return address. It stops at:
 *
 * - the end of `p_addresses`, or `_start`, where the program began
 * - a function without unwind information. That's a function compiled
 *   without `-funwind-tables` that the compiler could tell can't throw, such
 *   as C code or code in a prebuilt library. Its own address is the last one.
 * - a saved stack pointer outside RAM, or a return address that isn't Thumb
 *   code, as in a corrupted stack
 *
 * If an interrupt arrived inside a function without unwind information, the
 * last address is that function's return address from the link register,
 * which may be stale.
 *
 * An exception that escapes a `noexcept` function has already unwound the
 * frames below it by the time `std::terminate()` runs, so the trace starts at
 * the `noexcept` function, not at the `throw`.
 *
 * On a host build, it records nothing.
 *
 * ```cpp
 * std::array<hal::uptr, 16> buffer{};
 * for (auto const address : hal::cortex_m::stack_trace(buffer)) {
 *   print_hex(address);
 * }
 * ```
 *
 * @param p_addresses - where to write the addresses
 * @return the part of `p_addresses` written, innermost call first
 */
export [[nodiscard]] std::span<hal::uptr> stack_trace(
  std::span<hal::uptr> p_addresses) noexcept;
}  // namespace hal::cortex_m
