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
#include <cstdint>
#include <libhal/units.hpp>

namespace hal::cortex_m {
/// Structure type to access the System Control Block (SCB).
struct scb_registers_t
{
  /// Offset: 0x000 (R/ )  CPUID Base Register
  uint32_t const volatile cpuid;
  /// Offset: 0x004 (R/W)  Interrupt Control and State Register
  uint32_t volatile icsr;
  /// Offset: 0x008 (R/W)  Vector Table Offset Register
  intptr_t volatile vtor;
  /// Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register
  uint32_t volatile aircr;
  /// Offset: 0x010 (R/W)  System Control Register
  uint32_t volatile scr;
  /// Offset: 0x014 (R/W)  Configuration Control Register
  uint32_t volatile ccr;
  /// Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 5)
  std::array<uint8_t volatile, 12U> shp;
  /// Offset: 0x024 (R/W)  System Handler Control and State Register
  uint32_t volatile shcsr;
  /// Offset: 0x028 (R/W)  Configurable Fault Status Register
  uint32_t volatile cfsr;
  /// Offset: 0x02C (R/W)  HardFault Status Register
  uint32_t volatile hfsr;
  /// Offset: 0x030 (R/W)  Debug Fault Status Register
  uint32_t volatile dfsr;
  /// Offset: 0x034 (R/W)  MemManage Fault Address Register
  uint32_t volatile mmfar;
  /// Offset: 0x038 (R/W)  BusFault Address Register
  uint32_t volatile bfar;
  /// Offset: 0x03C (R/W)  Auxiliary Fault Status Register
  uint32_t volatile afsr;
  /// Offset: 0x040 (R/ )  Processor Feature Register
  std::array<uint32_t volatile, 2U> const pfr;
  /// Offset: 0x048 (R/ )  Debug Feature Register
  uint32_t const volatile dfr;
  /// Offset: 0x04C (R/ )  Auxiliary Feature Register
  uint32_t const volatile adr;
  /// Offset: 0x050 (R/ )  Memory Model Feature Register
  std::array<uint32_t volatile, 4U> const mmfr;
  /// Offset: 0x060 (R/ )  Instruction Set Attributes Register
  std::array<uint32_t volatile, 5U> const isar;
  /// Reserved 0
  std::array<uint32_t, 5U> reserved0;
  /// Offset: 0x088 (R/W)  Coprocessor Access Control Register
  uint32_t volatile cpacr;
};

/// System control block address
inline constexpr auto scb_address = static_cast<uptr>(0xE000'ED00UL);

/// @return auto* - Address of the Cortex M system control block register
inline auto* scb =
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  reinterpret_cast<scb_registers_t*>(scb_address);
}  // namespace hal::cortex_m
