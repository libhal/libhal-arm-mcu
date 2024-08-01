// Copyright 2024 Khalil Estell
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
#include <cstdint>

#include <libhal/steady_clock.hpp>

namespace hal::cortex_m {
/// Structure type to access the Data Watchpoint and Trace Register (DWT).
struct dwt_register_t
{
  /// Offset: 0x000 (R/W)  Control Register
  uint32_t volatile ctrl;
  /// Offset: 0x004 (R/W)  Cycle Count Register
  uint32_t volatile cyccnt;
  /// Offset: 0x008 (R/W)  CPI Count Register
  uint32_t volatile cpicnt;
  /// Offset: 0x00C (R/W)  Exception Overhead Count Register
  uint32_t volatile exccnt;
  /// Offset: 0x010 (R/W)  Sleep Count Register
  uint32_t volatile sleepcnt;
  /// Offset: 0x014 (R/W)  LSU Count Register
  uint32_t volatile lsucnt;
  /// Offset: 0x018 (R/W)  Folded-instruction Count Register
  uint32_t volatile foldcnt;
  /// Offset: 0x01C (R/ )  Program Counter Sample Register
  uint32_t const volatile pcsr;
  /// Offset: 0x020 (R/W)  Comparator Register 0
  uint32_t volatile comp0;
  /// Offset: 0x024 (R/W)  Mask Register 0
  uint32_t volatile mask0;
  /// Offset: 0x028 (R/W)  Function Register 0
  uint32_t volatile function0;
  /// Reserved 0
  std::array<uint32_t, 1> reserved0;
  /// Offset: 0x030 (R/W)  Comparator Register 1
  uint32_t volatile comp1;
  /// Offset: 0x034 (R/W)  Mask Register 1
  uint32_t volatile mask1;
  /// Offset: 0x038 (R/W)  Function Register 1
  uint32_t volatile function1;
  /// Reserved 1
  std::array<uint32_t, 1> reserved1;
  /// Offset: 0x040 (R/W)  Comparator Register 2
  uint32_t volatile comp2;
  /// Offset: 0x044 (R/W)  Mask Register 2
  uint32_t volatile mask2;
  /// Offset: 0x048 (R/W)  Function Register 2
  uint32_t volatile function2;
  /// Reserved 2
  std::array<uint32_t, 1> reserved2;
  /// Offset: 0x050 (R/W)  Comparator Register 3
  uint32_t volatile comp3;
  /// Offset: 0x054 (R/W)  Mask Register 3
  uint32_t volatile mask3;
  /// Offset: 0x058 (R/W)  Function Register 3
  uint32_t volatile function3;
};

/// Structure type to access the Core Debug Register (CoreDebug)
struct core_debug_registers_t
{
  /// Offset: 0x000 (R/W)  Debug Halting Control and Status Register
  uint32_t volatile dhcsr;
  /// Offset: 0x004 ( /W)  Debug Core Register Selector Register
  uint32_t volatile dcrsr;
  /// Offset: 0x008 (R/W)  Debug Core Register Data Register
  uint32_t volatile dcrdr;
  /// Offset: 0x00C (R/W)  Debug Exception and Monitor Control Register
  uint32_t volatile demcr;
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
inline constexpr intptr_t dwt_address = 0xE0001000UL;

/// Address of the Cortex M CoreDebug module
inline constexpr intptr_t core_debug_address = 0xE000EDF0UL;

inline auto* dwt = reinterpret_cast<dwt_register_t*>(dwt_address);

inline auto* core =
  reinterpret_cast<core_debug_registers_t*>(core_debug_address);

}  // namespace hal::cortex_m
