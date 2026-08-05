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

module;

export module hal.arm_mcu.stm32f1:clock;

import hal;
import hal.util;

import :constants;
import :power;

namespace hal::stm32f1 {

/// Bit masks for the CFGR register
struct clock_configuration
{
  /// Controls which clock signal is sent to the MCO pin
  static constexpr auto mco = bit_mask::from<24, 26>();
  /// Sets the USB clock divider
  static constexpr auto usb_prescalar = bit_mask::from<22>();
  /// Sets the PLL multiplier
  static constexpr auto pll_mul = bit_mask::from<18, 21>();
  /// If set to 1, will divide the HSE signal by 2 before sending to PLL
  static constexpr auto hse_pre_divider = bit_mask::from<17>();
  /// Sets which source the PLL will take as input
  static constexpr auto pll_source = bit_mask::from<16>();
  /// Sets the clock divider for the ADC peripherals
  static constexpr auto adc_divider = bit_mask::from<14, 15>();
  /// Sets the divider for peripherals on the APB2 bus
  static constexpr auto apb_2_divider = bit_mask::from<11, 13>();
  /// Sets the divider for peripherals on the APB1 bus
  static constexpr auto apb_1_divider = bit_mask::from<8, 10>();
  /// Sets the divider for peripherals on the AHB bus
  static constexpr auto ahb_divider = bit_mask::from<4, 7>();
  /// Used to check if the system clock has taken the new system clock
  /// settings.
  static constexpr auto system_clock_status = bit_mask::from<2, 3>();
  /// Set which clock will be used for the system clock.
  static constexpr auto system_clock_select = bit_mask::from<0, 1>();

  static auto reg()
  {
    return hal::bit_modify(rcc->cfgr);
  }
};

/// Bit masks for the CR register
struct clock_control
{
  /// Indicates if the PLL is enabled and ready
  static constexpr auto pll_ready = bit_mask::from<25>();
  /// Used to enable the PLL
  static constexpr auto pll_enable = bit_mask::from<24>();
  /// Indicates if the external oscillator is ready for use
  static constexpr auto external_osc_ready = bit_mask::from<17>();
  /// Used to enable the external oscillator
  static constexpr auto external_osc_enable = bit_mask::from<16>();

  static auto reg()
  {
    return hal::bit_modify(rcc->cr);
  }
};

/// Bitmasks for the BDCR register
struct rtc_register
{
  /// Will reset all clock states for the RTC
  static constexpr auto backup_domain_reset = bit_mask::from<16>();
  /// Enables the RTC clock
  static constexpr auto rtc_enable = bit_mask::from<15>();
  /// Selects the clock source for the RTC
  static constexpr auto rtc_source_select = bit_mask::from<8, 9>();
  /// Indicates if the LSE is ready for use
  static constexpr auto low_speed_osc_ready = bit_mask::from<1>();
  /// Used to enable the LSE
  static constexpr auto low_speed_osc_enable = bit_mask::from<0>();

  static auto reg()
  {
    return hal::bit_modify(rcc->bdcr);
  }
};

/// Constant for the frequency of the LSE
export constexpr auto internal_low_speed_oscillator =
  20 * mp_units::si::unit_symbols::kHz;

/// Constant for the frequency of the HSE
export constexpr auto internal_high_speed_oscillator =
  8 * mp_units::si::unit_symbols::MHz;

/// Constant for the frequency of the Flash Controller
export constexpr auto flash_clock = internal_high_speed_oscillator;

/// Constant for the frequency of the Watch Dog Peripheral
export constexpr auto watchdog_clock_rate = internal_low_speed_oscillator;

/// Available dividers for the APB bus
export enum class apb_divider : u8 {
  divide_by_1 = 0,
  divide_by_2 = 0b100,
  divide_by_4 = 0b101,
  divide_by_8 = 0b110,
  divide_by_16 = 0b111,
};

/// Available dividers for the AHB bus
export enum class ahb_divider : u8 {
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

/// Available dividers for the ADC bus
export enum class adc_divider : u8 {
  divide_by_2 = 0b00,
  divide_by_4 = 0b01,
  divide_by_6 = 0b10,
  divide_by_8 = 0b11,
};

/// Available clock sources available for the system clock
export enum class system_clock_select : u8 {
  high_speed_internal = 0b00,
  high_speed_external = 0b01,
  pll = 0b10,
};

/// PLL frequency multiplication options.
export enum class pll_multiply : u8 {
  multiply_by_2 = 0b0000,
  multiply_by_3 = 0b0001,
  multiply_by_4 = 0b0010,
  multiply_by_5 = 0b0011,
  multiply_by_6 = 0b0100,
  multiply_by_7 = 0b0101,
  multiply_by_8 = 0b0110,
  multiply_by_9 = 0b0111,
  multiply_by_10 = 0b1000,
  multiply_by_11 = 0b1001,
  multiply_by_12 = 0b1010,
  multiply_by_13 = 0b1011,
  multiply_by_14 = 0b1100,
  multiply_by_15 = 0b1101,
  multiply_by_16 = 0b1110,
};

/// Available clock sources for the RTC
export enum class rtc_source : u8 {
  no_clock = 0b00,
  low_speed_internal = 0b01,
  low_speed_external = 0b10,
  high_speed_external_divided_by_128 = 0b11,
};

/// Available clock sources for the PLL
export enum class pll_source : u8 {
  internal_8mhz_divided_by_2 = 0b0,
  high_speed_external = 0b1,
  high_speed_external_divided_by_2 = 0b11,
};

/// Available dividers for the USB peripheral
export enum class usb_divider : u8 {
  /// Divide by 1.5
  divide_by_1_point_5 = 0,
  divide_by_1 = 1,
};

export struct clock_tree
{
  /// Defines the frequency of the high speed external clock signal
  hal::hertz high_speed_external = 0 * mp_units::si::unit_symbols::MHz;

  /// Defines the frequency of the low speed external clock signal.
  hal::hertz low_speed_external = 0 * mp_units::si::unit_symbols::MHz;

  /// Defines the configuration of the PLL
  struct pll_t
  {
    bool enable = false;
    pll_source source = pll_source::internal_8mhz_divided_by_2;
    pll_multiply multiply = pll_multiply::multiply_by_2;
    struct usb_divider_t
    {
      usb_divider divider = usb_divider::divide_by_1_point_5;
    } usb = {};
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
};

/// @attention If configuration of the system clocks is desired, one should
///            consult the user manual of the target MCU in use to determine
///            the valid clock configuration values that can/should be used.
///            The Initialize() method is only responsible for configuring the
///            clock system based on configurations in the
///            clock_configuration. Incorrect configurations may result in a
///            hard fault or cause the clock system(s) to supply incorrect
///            clock rate(s).
///
/// @see Figure 11. Clock Tree
///      https://www.st.com/resource/en/reference_manual/cd00171190-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf#page=126
export void configure_clocks(clock_tree p_clock_tree);

/// @return the clock rate frequency of a peripheral
export hal::hertz frequency(peripheral p_id);

/**
 * @brief Sets every bus to its maximum possible frequency using the internal
 * oscillator
 *
 * NOTE: USB cannot be used in this configuration.
 */
export void maximum_speed_using_internal_oscillator();

}  // namespace hal::stm32f1
