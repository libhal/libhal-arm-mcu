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

#include <libhal-util/bit.hpp>
#include <libhal-util/enum.hpp>
#include <libhal/error.hpp>
#include <libhal/units.hpp>

#include "constants.hpp"

namespace hal::lpc40 {

/// The frequency of the internal RC clock and the clock frequency at startup
static constexpr hertz irc_frequency = 12'000'000.0f;
/// The default clock divider for the peripheral clock
static constexpr uint32_t default_peripheral_divider = 4;

/// USB oscillator source constants (not used)
enum class usb_clock_source : uint8_t
{
  /// Use IRC or external oscillator directly
  system_clock = 0b00,
  /// Use PLL0 main PLL as the clock source
  pll0 = 0b01,
  /// Use PLL1 alternative PLL as the clock source
  pll1 = 0b10,
};

/// USB Clock divider constants
enum class usb_divider : uint8_t
{
  divide_by1 = 0,
  divide_by2,
  divide_by3,
  divide_by4,
};

/// spifi clock options
enum class spifi_clock_source : uint8_t
{
  /// Use IRC or external oscillator directly
  system_clock = 0b00,
  /// Use PLL0 main PLL as the clock source
  pll0 = 0b01,
  /// Use PLL1 alternative PLL as the clock source
  pll1 = 0b10,
};

/// Defines the codes for the flash access clock cycles required based on the
/// CPU clocks speed.
enum class flash_configuration : uint32_t
{
  /// Flash accesses use 1 CPU clock. Use for up to 20 MHz CPU clock with
  /// power boost off.
  clock1 = 0b0000 << 12,
  /// Flash accesses use 2 CPU clocks. Use for up to 40 MHz CPU clock with
  /// power boost off.
  clock2 = 0b0001 << 12,
  /// Flash accesses use 3 CPU clocks. Use for up to 60 MHz CPU clock with
  /// power boost off.
  clock3 = 0b0010 << 12,
  /// Flash accesses use 4 CPU clocks. Use for up to 80 MHz CPU clock with
  /// power boost off.
  clock4 = 0b0011 << 12,
  /// Flash accesses use 5 CPU clocks. Use for up to 100 MHz CPU clock with
  /// power boost off. If CPU clock is above 100 MHz, use this but with power
  /// boost on.
  clock5 = 0b0100 << 12,
  /// Flash accesses use 6 CPU clocks. "Safe" setting for any allowed
  /// conditions.
  clock6 = 0b0101 << 12,
};

/// Structure representing the lpc4078 clock tree
struct clock_tree
{
  /// the frequency of the input oscillator
  hertz oscillator_frequency = irc_frequency;
  /// set to true to use external XTC
  bool use_external_oscillator = false;
  /// phase locked loops config struct
  struct pll_t
  {
    /// turn on/off a PLL
    bool enabled = false;
    /// increase the frequency of the PLL by the multiple
    uint8_t multiply = 1;
  };
  /// phase locked loops for both pll[0] and pll[1]
  std::array<pll_t, 2> pll = {};
  /// cpu clock control config struct
  struct cpu_t
  {
    /// If true, use PLL0, if false, use system clock which is defined as
    /// 12MHz
    bool use_pll0 = false;
    /// Divide the input clock from IRC or PLL0
    uint8_t divider = 1;
  };
  /// cpu clock control
  cpu_t cpu = {};

  /// usb clock control config struct
  struct usb_t
  {
    /// usb clock source
    usb_clock_source clock = usb_clock_source::system_clock;
    /// usb clock divider
    usb_divider divider = usb_divider::divide_by1;
  };
  /// usb clock control
  usb_t usb = {};

  /// spifi clock control config struct
  struct spifi_t
  {
    /// spifi clock source
    spifi_clock_source clock = spifi_clock_source::system_clock;
    /// spifi clock divider
    uint8_t divider = 1;
  };
  /// spifi clock control
  spifi_t spifi = {};

  /// Defines the peripheral clock divider amount
  uint8_t peripheral_divider = 4;

  /// Set true to make the EMC divider half as slow as the CPU divider. Set to
  /// false to set it to equal that amount.
  bool emc_half_cpu_divider = false;
};

/**
 * @brief Set the lpc40xx MCU to the maximum clock speed (120MHz) possible
 *
 * This function REQUIRES an external crystal to be used.
 *
 * - CPU clock speed set to 120MHz
 * - USB clock speed set to 120MHz
 * - Peripheral clock set to 120MHZ
 * - SPIFI clock set to 120MHz
 * - PLL0 is set to 120MHz and used for everything
 * - PLL1 is disabled and not used
 *
 * TODO(#65): explain the set of errors in better detail
 *
 * @param p_external_crystal_frequency - frequency of the crystal connected to
 * the XTAL1 & XTAL2
 */
void maximum(hertz p_external_crystal_frequency);

/**
 * @brief Get the operating frequency of the peripheral
 *
 * @param p_peripheral - id of the peripheral
 * @return frequency - operating frequency of the peripheral
 */
hertz get_frequency(peripheral p_peripheral);

/**
 * @brief Determins if the external oscillator is currently enabled and in use
 *
 * @return true - external oscillator is in use currently
 * @return false - external oscillator is NOT in use currently
 */
bool using_external_oscillator();

/**
 * @brief Apply the clock configuration to hardware
 *
 * TODO(#65): explain the set of errors in better detail
 */
void configure_clocks(clock_tree const& p_clock_tree);

}  // namespace hal::lpc40
