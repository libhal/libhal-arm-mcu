// Copyright 2024 - 2025 Khalil Estell and the libhal contributors
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

#pragma once

#include <array>

#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>

namespace hal::cortex_m {
/// Structure type to access the Data Watchpoint and Trace Register (DWT).
struct dwt_register_t
{
  /// Offset: 0x000 (R/W)  Control Register
  u32 volatile ctrl;
  /// Offset: 0x004 (R/W)  Cycle Count Register
  u32 volatile cyccnt;
  /// Offset: 0x008 (R/W)  CPI Count Register
  u32 volatile cpicnt;
  /// Offset: 0x00C (R/W)  Exception Overhead Count Register
  u32 volatile exccnt;
  /// Offset: 0x010 (R/W)  Sleep Count Register
  u32 volatile sleepcnt;
  /// Offset: 0x014 (R/W)  LSU Count Register
  u32 volatile lsucnt;
  /// Offset: 0x018 (R/W)  Folded-instruction Count Register
  u32 volatile foldcnt;
  /// Offset: 0x01C (R/ )  Program Counter Sample Register
  u32 const volatile pcsr;
  /// Offset: 0x020 (R/W)  Comparator Register 0
  u32 volatile comp0;
  /// Offset: 0x024 (R/W)  Mask Register 0
  u32 volatile mask0;
  /// Offset: 0x028 (R/W)  Function Register 0
  u32 volatile function0;
  /// Reserved 0
  std::array<u32, 1> reserved0;
  /// Offset: 0x030 (R/W)  Comparator Register 1
  u32 volatile comp1;
  /// Offset: 0x034 (R/W)  Mask Register 1
  u32 volatile mask1;
  /// Offset: 0x038 (R/W)  Function Register 1
  u32 volatile function1;
  /// Reserved 1
  std::array<u32, 1> reserved1;
  /// Offset: 0x040 (R/W)  Comparator Register 2
  u32 volatile comp2;
  /// Offset: 0x044 (R/W)  Mask Register 2
  u32 volatile mask2;
  /// Offset: 0x048 (R/W)  Function Register 2
  u32 volatile function2;
  /// Reserved 2
  std::array<u32, 1> reserved2;
  /// Offset: 0x050 (R/W)  Comparator Register 3
  u32 volatile comp3;
  /// Offset: 0x054 (R/W)  Mask Register 3
  u32 volatile mask3;
  /// Offset: 0x058 (R/W)  Function Register 3
  u32 volatile function3;
};

/// Structure type to access the Core Debug Register (CoreDebug)
struct core_debug_registers_t
{
  /// Offset: 0x000 (R/W)  Debug Halting Control and Status Register
  u32 volatile dhcsr;
  /// Offset: 0x004 ( /W)  Debug Core Register Selector Register
  u32 volatile dcrsr;
  /// Offset: 0x008 (R/W)  Debug Core Register Data Register
  u32 volatile dcrdr;
  /// Offset: 0x00C (R/W)  Debug Exception and Monitor Control Register
  u32 volatile demcr;
};

/**
 * @brief This bit must be set to 1 to enable use of the trace and debug
 * blocks:
 *
 *   - Data Watchpoint and Trace (DWT)
 *   - Instrumentation Trace Macrocell (ITM)
 *   - Embedded Trace Macrocell (ETM)
 *   - Trace Port Interface Unit (TPIU).
 */
inline constexpr unsigned core_trace_enable = 1 << 24U;

/// Mask for turning on cycle counter.
inline constexpr unsigned enable_cycle_count = 1 << 0;

/// Address of the hardware DWT registers
inline constexpr auto dwt_address = static_cast<uptr>(0xE0001000UL);

/// Address of the Cortex M CoreDebug module
inline constexpr auto core_debug_address = static_cast<uptr>(0xE000EDF0UL);

// NOLINTNEXTLINE(performance-no-int-to-ptr)
inline auto* dwt = reinterpret_cast<dwt_register_t*>(dwt_address);

inline auto* core =
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  reinterpret_cast<core_debug_registers_t*>(core_debug_address);
}  // namespace hal::cortex_m
