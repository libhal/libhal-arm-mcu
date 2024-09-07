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

#include <libhal-lpc40/constants.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/enum.hpp>
#include <libhal/error.hpp>
#include <libhal/units.hpp>

namespace hal::lpc40 {
/// lpc40xx system controller register map
struct system_controller_t
{
  /// Offset: 0x000 (R/W)  Flash Accelerator Configuration Register
  uint32_t volatile flashcfg;
  /// reserved 0
  std::array<uint32_t, 31> reserved0;
  /// Offset: 0x080 (R/W)  PLL0 Control Register
  uint32_t volatile pll0con;
  /// Offset: 0x084 (R/W)  PLL0 Configuration Register
  uint32_t volatile pll0cfg;
  /// Offset: 0x088 (R/ )  PLL0 Status Register
  uint32_t const volatile pll0stat;
  /// Offset: 0x08C ( /W)  PLL0 Feed Register
  uint32_t volatile pll0feed;
  /// reserved 1
  std::array<uint32_t, 4> reserved1;
  /// Offset: 0x0A0 (R/W)  PLL1 Control Register
  uint32_t volatile pll1con;
  /// Offset: 0x0A4 (R/W)  PLL1 Configuration Register
  uint32_t volatile pll1cfg;
  /// Offset: 0x0A8 (R/ )  PLL1 Status Register
  uint32_t const volatile pll1stat;
  /// Offset: 0x0AC ( /W)  PLL1 Feed Register
  uint32_t volatile pll1feed;
  /// reserved 2
  std::array<uint32_t, 4> reserved2;
  /// Offset: 0x0C0 (R/W)  Power Control Register
  uint32_t volatile power_control;
  /// Offset: 0x0C4 (R/W)  Power Control for Peripherals Register
  uint32_t volatile peripheral_power_control0;
  /// Offset: 0x0C8 (R/W)  Power Control for Peripherals Register
  uint32_t volatile peripheral_power_control1;
  /// reserved 3
  std::array<uint32_t, 13> reserved3;
  /// Offset: 0x100 (R/W)  External Memory Controller Clock Selection Register
  uint32_t volatile emmc_clock_select;
  /// Offset: 0x104 (R/W)  CPU Clock Selection Register
  uint32_t volatile cpu_clock_select;
  /// Offset: 0x108 (R/W)  USB Clock Selection Register
  uint32_t volatile usb_clock_select;
  /// Offset: 0x10C (R/W)  Clock Source Select Register
  uint32_t volatile clock_source_select;
  /// Offset: 0x110 (R/W)  CAN Sleep Clear Register
  uint32_t volatile can_sleep_clear;
  /// Offset: 0x114 (R/W)  CAN Wake-up Flags Register
  uint32_t volatile canwakeflags;
  /// reserved 4
  std::array<uint32_t, 10> reserved4;
  /// Offset: 0x140 (R/W)  External Interrupt Flag Register
  uint32_t volatile extint;
  /// reserved 5
  std::array<uint32_t, 1> reserved5;
  /// Offset: 0x148 (R/W)  External Interrupt Mode Register
  uint32_t volatile extmode;
  /// Offset: 0x14C (R/W)  External Interrupt Polarity Register
  uint32_t volatile extpolar;
  /// reserved 6
  std::array<uint32_t, 12> reserved6;
  /// Offset: 0x180 (R/W)  Reset Source Identification Register
  uint32_t volatile reset_source_id;
  /// reserved 7
  std::array<uint32_t, 7> reserved7;
  /// Offset: 0x1A0 (R/W)  System Controls and Status Register
  uint32_t volatile scs;
  /// Offset: 0x1A4 (R/W) Clock Dividers
  uint32_t volatile irctrim;
  /// Offset: 0x1A8 (R/W)  Peripheral Clock Selection Register
  uint32_t volatile peripheral_clock_select;
  /// reserved 8
  std::array<uint32_t, 1> reserved8;
  /// Offset: 0x1B0 (R/W)  Power Boost control register
  uint32_t volatile power_boost;
  /// Offset: 0x1B4 (R/W)  spifi clock select
  uint32_t volatile spifi_clock_select;
  /// Offset: 0x1B8 (R/W)  LCD Configuration and clocking control Register
  uint32_t volatile lcd_cfg;
  /// reserved 9
  std::array<uint32_t, 1> reserved9;
  /// Offset: 0x1C0 (R/W)  USB Interrupt Status Register
  uint32_t volatile usb_interrupt_status;
  /// Offset: 0x1C4 (R/W)  DMA Request Select Register
  uint32_t volatile dmareqsel;
  /// Offset: 0x1C8 (R/W)  Clock Output Configuration Register
  uint32_t volatile clkoutcfg;
  /// Offset: 0x1CC (R/W)  RESET Control0 Register
  uint32_t volatile rstcon0;
  /// Offset: 0x1D0 (R/W)  RESET Control1 Register
  uint32_t volatile rstcon1;
  /// reserved 10
  std::array<uint32_t, 2> reserved10;
  /// Offset: 0x1DC (R/W) sdram programmable delays
  uint32_t volatile sdram_delay;
  /// Offset: 0x1E0 (R/W) Calibration of programmable delays
  uint32_t volatile emmc_calibration;
};  // namespace system_controller_t

/// Namespace for PLL configuration bit masks
namespace pll_register {
/// In PLLCON register: When 1, and after a valid PLL feed, this bit
/// will activate the related PLL and allow it to lock to the requested
/// frequency.
static constexpr auto enable = bit_mask::from<0>();

/// In PLLCFG register: PLL multiplier value, the amount to multiply the
/// input frequency by.
static constexpr auto multiplier = bit_mask::from<0, 4>();

/// In PLLCFG register: PLL divider value, the amount to divide the output
/// of the multiplier stage to bring the frequency down to a
/// reasonable/usable level.
static constexpr auto divider = bit_mask::from<5, 6>();

/// In PLLSTAT register: if set to 1 by hardware, the PLL has accepted
/// the configuration and is locked.
static constexpr auto pll_lock = bit_mask::from<10>();
};  // namespace pll_register

/// Namespace of Oscillator register bit_masks
namespace oscillator {
/// IRC or Main oscillator select bit
static constexpr auto select = bit_mask::from<0>();

/// SCS: Main oscillator range select
static constexpr auto range_select = bit_mask::from<4>();

/// SCS: Main oscillator enable
static constexpr auto external_enable = bit_mask::from<5>();

/// SCS: Main oscillator ready status
static constexpr auto external_ready = bit_mask::from<6>();
};  // namespace oscillator

/// Namespace of Clock register bit_masks
namespace cpu_clock {
/// CPU clock divider amount
static constexpr auto divider = bit_mask::from<0, 4>();

/// CPU clock source select bit
static constexpr auto select = bit_mask::from<8>();
};  // namespace cpu_clock

/// Namespace of Peripheral register bit_masks
namespace peripheral_clock {
/// Main single peripheral clock divider shared across all peripherals,
/// except for USB and spifi.
static constexpr auto divider = bit_mask::from<0, 4>();
};  // namespace peripheral_clock

/// Namespace of EMC register bit_masks
namespace emc_clock {
/// EMC Clock Register divider bit
static constexpr auto divider = bit_mask::from<0>();
};  // namespace emc_clock

/// Namespace of USB register bit_masks
namespace usb_clock {
/// USB clock divider constant
static constexpr auto divider = bit_mask::from<0, 4>();

/// USB clock source select bit
static constexpr auto select = bit_mask::from<8, 9>();
};  // namespace usb_clock

/// Namespace of spifi register bit_masks
namespace spifi_clock {
/// spifi clock divider constant
static constexpr auto divider = bit_mask::from<0, 4>();

/// spifi clock source select bit
static constexpr auto select = bit_mask::from<8, 9>();
};  // namespace spifi_clock

constexpr intptr_t lpc_apb1_base = 0x40080000UL;
constexpr intptr_t lpc_sc_base = lpc_apb1_base + 0x7C000;

/// @brief Pointer to system controller register
inline system_controller_t* system_controller_reg =
  reinterpret_cast<system_controller_t*>(lpc_sc_base);
}  // namespace hal::lpc40
