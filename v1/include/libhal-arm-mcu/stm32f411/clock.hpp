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

#include <libhal/error.hpp>
#include <libhal/units.hpp>

#include "constants.hpp"

namespace hal::stm32f411 {

using namespace hal::literals;

/// Constant for the frequency of the LSI
static constexpr auto internal_low_speed_oscillator = 32.0_kHz;

/// Constant for the frequency of the HSI
static constexpr auto internal_high_speed_oscillator = 16.0_MHz;

/// Constant for the frequency of the Watch Dog Peripheral
static constexpr auto watchdog_clock_rate = internal_low_speed_oscillator;

/// Available dividers for the AHB bus
enum class ahb_divider : u8
{
  divide_by_1 = 0,
  divide_by_2 = 0b1000,
  divide_by_4 = 0b1001,
  divide_by_8 = 0b1010,
  divide_by_16 = 0b1011,
  divide_by_64 = 0b1100,
  divide_by_128 = 0b1101,
  divide_by_256 = 0b1110,
  divide_by_512 = 0b1111,
};

/// Available dividers for the APB bus
enum class apb_divider : u8
{
  divide_by_1 = 0,
  divide_by_2 = 0b100,
  divide_by_4 = 0b101,
  divide_by_8 = 0b110,
  divide_by_16 = 0b111,
};

/// Available dividers for the ADC bus
enum class adc_divider : u8
{
  divide_by_2 = 0b00,
  divide_by_4 = 0b01,
  divide_by_6 = 0b10,
  divide_by_8 = 0b11,
};

/// Available clock sources available for the system clock
enum class system_clock_select : u8
{
  high_speed_internal = 0b00,
  high_speed_external = 0b01,
  pll = 0b10,
};

enum class pll_source : u8
{
  high_speed_internal = 0b00,
  high_speed_external = 0b01,
};

/// Available clock sources for the RTC
enum class rtc_source : u8
{
  no_clock = 0b00,
  low_speed_external = 0b01,
  low_speed_internal = 0b10,
  high_speed_external_divided_by_128 = 0b11,
};

/// Available sources for the I2S clocks
enum class i2s_source : u8
{
  pll_i2s_clk = 0,
  /// External I2S_CKIN pin
  i2s_external_clock = 1,
};

struct clock_tree
{
  /// Defines the frequency of the high speed external clock signal
  hal::hertz high_speed_external = 0.0_MHz;

  /// Defines the frequency of the low speed external clock signal.
  hal::hertz low_speed_external = 0.0_MHz;

  /// Defines the configuration of the PLL
  struct pll_t
  {
    bool enable = false;
    pll_source source = pll_source::high_speed_internal;
    hertz output = internal_high_speed_oscillator;
  } pll = {};

  /// Defines which clock source will be use for the system.
  /// @warning System will lock up in the following situations:
  ///          - Select PLL, but PLL is not enabled
  ///          - Select PLL, but PLL frequency is too high
  ///          - Select High Speed External, but the frequency is kept at
  ///            0_Mhz.
  system_clock_select system_clock = system_clock_select::high_speed_internal;

  /// Defines the configuration for the RTC
  struct rtc_t
  {
    bool enable = false;
    rtc_source source = rtc_source::no_clock;
  } rtc = {};

  /// Defines the configuration of the dividers beyond system clock mux.
  struct ahb_t
  {
    ahb_divider divider = ahb_divider::divide_by_1;
    /// Maximum rate of 36 MHz
    struct apb1_t
    {
      apb_divider divider = apb_divider::divide_by_1;
    } apb1 = {};

    /// Maximum rate of 72 MHz
    struct apb2_t
    {
      apb_divider divider = apb_divider::divide_by_1;
      /// Maximum of 14 MHz
      struct adc_t
      {
        adc_divider divider = adc_divider::divide_by_2;
      } adc = {};
    } apb2 = {};
  } ahb = {};
  bool usb_otg_clock_enable = false;
};

/** @attention If configuration of the system clocks is desired, one should
 * consult the user manual of the target MCU in use to determine the valid clock
 * configuration values that can/should be used. The configure_clocks() method
 * is only responsible for configuring the clock system based on configurations
 * in the clock_configuration. Incorrect configurations may result in a hard
 * fault or cause the clock system(s) to supply incorrect clock rate(s).
 *
 * @see Figure 12. Clock Tree
 * https://www.st.com/resource/en/reference_manual/rm0383-stm32f411xce-advanced-armbased-32bit-mcus-stmicroelectronics.pdf#page=93
 */
void configure_clocks(clock_tree const& p_clock_tree);

/// @return the clock rate frequency of a peripheral
hal::hertz frequency(peripheral p_id);

/**
 * @brief Sets every bus to its maximum possible frequency using the internal
 * oscillator
 *
 */
void maximum_speed_using_internal_oscillator();
}  // namespace hal::stm32f411
