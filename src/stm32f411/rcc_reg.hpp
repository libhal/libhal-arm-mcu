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

#include <libhal-util/bit.hpp>

namespace hal::stm32f411 {
struct reset_and_clock_control_t
{
  /// Offset: 0x00 Clock Control Register
  std::uint32_t volatile cr;
  /// Offset: 0x04 PLL config register
  std::uint32_t volatile pllconfig;
  /// Offset: 0x08 clock config register
  std::uint32_t volatile config;
  /// Offset: 0x0c clock interrupt register
  std::uint32_t volatile cir;
  /// Offset: 0x10 ahb1 peripheral reset register
  std::uint32_t volatile ahb1rstr;
  /// Offset: 0x14 ahb2 peripheral reset register
  std::uint32_t volatile ahb2rstr;

  std::array<std::uint32_t, 2> volatile reserved0;

  /// Offset: 0x20 ahb1 peripheral reset register
  std::uint32_t volatile apb1rstr;
  /// Offset: 0x24 ahb2 peripheral reset register
  std::uint32_t volatile apb2rstr;

  std::array<std::uint32_t, 2> volatile reserved1;

  /// Offset: 0x30 ahb1 clock enable register
  std::uint32_t volatile ahb1enr;
  /// Offset: 0x34 ahb1 clock enable register
  std::uint32_t volatile ahb2enr;

  std::array<std::uint32_t, 2> volatile reserved2;

  /// Offset: 0x40 apb1 clock enable register
  std::uint32_t volatile apb1enr;
  /// Offset: 0x44 apb1 clock enable register
  std::uint32_t volatile apb2enr;

  std::array<std::uint32_t, 2> volatile reserved3;

  /// Offset: 0x50 ahb1 peripheral clock enable in low power mode register
  std::uint32_t volatile ahb1lpenr;
  /// Offset: 0x54 ahb2 peripheral clock enable in low power mode register
  std::uint32_t volatile ahb2lpenr;

  std::array<std::uint32_t, 2> volatile reserved4;

  /// Offset: 0x60 apb1 peripheral clock enable in low power mode register
  std::uint32_t volatile apb1lpenr;
  /// Offset: 0x64 apb2 peripheral clock enable in low power mode register
  std::uint32_t volatile apb2lpenr;

  std::array<std::uint32_t, 2> volatile reserved5;

  /// Offset: 0x70 backup domain control register
  std::uint32_t volatile backup_domain_control;
  /// Offset: 0x74 control and status register
  std::uint32_t volatile csr;

  std::array<std::uint32_t, 2> volatile reserved6;

  /// Offset: 0x80 spread spectrum clock generation register
  std::uint32_t volatile sscgr;
  /// Offset: 0x84 plli2s config register
  std::uint32_t volatile plli2scfgr;

  std::uint32_t volatile reserved7;

  /// Offset 0x8c dedicated clocks configuration register
  std::uint32_t volatile dckcfgr;
};

struct rcc_cr
{
  /// Internal high-speed clock enable. 0: OFF, 1: ON
  static constexpr auto high_speed_internal_enable = bit_mask::from<0>();

  /// Internal high-speed clock ready flag. 0: not ready, 1: ready
  static constexpr auto high_speed_internal_ready = bit_mask::from<1>();

  /// These bits provide an additional user-programmable trimming value that is
  /// added to the HSICAL[7:0] bits.
  static constexpr auto high_speed_internal_trim = bit_mask::from<7, 3>();

  /// Internal high-speed clock calibration
  static constexpr auto high_speed_internal_calibration =
    bit_mask::from<15, 8>();

  /// External high-speed clock enable. 0: OFF, 1: ON
  static constexpr auto high_speed_external_enable = bit_mask::from<16>();

  /// External high-speed clock ready flag. 0: not ready, 1: ready
  static constexpr auto high_speed_external_ready = bit_mask::from<17>();

  /// External high-speed clock bypass. 0: HSE oscillator not bypassed, 1: HSE
  /// oscillator bypassed with an external clock
  static constexpr auto high_speed_external_bypass = bit_mask::from<18>();

  /// Clock security system enable
  /// 0: Clock security system OFF (Clock detector OFF)
  /// 1: Clock security system ON (Clock detector ON if HSE oscillator is
  /// stable, OFF if not)
  static constexpr auto clock_security_system_enable = bit_mask::from<19>();

  /// 0: PLL OFF
  /// 1: PLL ON
  static constexpr auto main_pll_enable = bit_mask::from<24>();

  /// 0: PLL unlocked
  /// 1: PLL locked
  static constexpr auto main_pll_ready = bit_mask::from<25>();

  /// 0: PLLI2S OFF
  /// 1: PLLI2S ON
  static constexpr auto pll_i2s_enable = bit_mask::from<26>();

  /// 0: PLLI2S unlocked
  /// 1: PLLI2S locked
  static constexpr auto pll_i2s_ready = bit_mask::from<27>();
};

struct rcc_pllcnfg
{
  /// PLL Input Source divider. Must be 1 <= X <= 63
  /// VCO input = input / division_factor. VCO inp must be 1_MHz <= VCO <= 2_MHz
  static constexpr auto input_division_factor = bit_mask::from<5, 0>();

  /// Main PLL output multiplication factor. Must be 50 <= X <= 432
  /// VCO output = output * multiplication_factor. VCO out must be 100_MHz <=
  /// VCO <= 432_MHz
  static constexpr auto main_multiplication_factor = bit_mask::from<14, 6>();

  /// Main PLL output division factor. division factor = 2 + 2X
  /// Must not exceed 100_MHz
  static constexpr auto main_division_factor = bit_mask::from<17, 16>();

  /// PLL Input Source
  static constexpr auto pll_source = bit_mask::from<22>();

  /// PLL division factor for USB OTG FS, and SDIO clocks
  /// USB OTG FS requires a 48 MHz clock to work
  /// SDIO need a frequency lower than or equal to 48 MHz to work correctly
  static constexpr auto usb_sdio_dividor = bit_mask::from<16>();
};

struct rcc_config
{
  /// System clock switch
  /// 00: HSI oscillator selected as system clock
  /// 01: HSE oscillator selected as system clock
  /// 10: PLL selected as system clock
  /// 11: not allowed
  static constexpr auto system_clock_switch = bit_mask::from<1, 0>();

  /// System clock switch status
  /// 00: HSI oscillator used as the system clock
  /// 01: HSE oscillator used as the system clock
  /// 10: PLL used as the system clock
  /// 11: not applicable
  static constexpr auto system_clock_switch_status = bit_mask::from<3, 2>();

  /// AHB prescaler
  /// if less than 7, not divided
  /// else: (system clock frequency)/2**(n-7)
  static constexpr auto ahb_prescalar = bit_mask::from<7, 4>();

  /// APB1 prescaler (apb clock never exceeds 50MHz)
  static constexpr auto apb1_prescalar = bit_mask::from<12, 10>();

  /// APB1 prescaler (apb clock never exceeds 100MHz)
  static constexpr auto apb2_prescalar = bit_mask::from<15, 13>();

  /// HSE division factor for RTC clock
  /// (1MHz must be inputed in)
  /// RTC = HSE/n
  static constexpr auto hse_division_for_rtc_clock = bit_mask::from<20, 16>();

  /// Microcontroller clock output 1 (MCO1)
  /// 00: HSI clock selected
  /// 01: LSE oscillator selected
  /// 10: HSE oscillator clock selected
  /// 11: PLL clock selected
  static constexpr auto mco1_clock_select = bit_mask::from<22, 21>();

  /// I2S clock selection
  /// 0: PLLI2S clock used as I2S clock source
  /// 1: External clock mapped on the I2S_CKIN pin used as I2S clock source
  static constexpr auto i2s_clock_selection = bit_mask::from<23>();

  /// MCO1 prescaler
  /// if: n < 4, no division
  /// else: division by (n & 3) + 2
  static constexpr auto mco1_prescaler = bit_mask::from<26, 24>();

  /// MCO2 prescaler
  /// if: n < 4, no division
  /// else: division by (n & 3) + 2
  static constexpr auto mco2_prescaler = bit_mask::from<29, 27>();

  /// Microcontroller clock output 1 (MCO2)
  /// 00: System clock (SYSCLK) selected
  /// 01: PLLI2S clock selected
  /// 10: HSE oscillator clock selected
  /// 11: PLL clock selected
  static constexpr auto mco2_clock_select = bit_mask::from<31, 30>();
};

struct rcc_backup_domain_control
{
  /// 0: LSE clock OFF
  /// 1: LSE clock ON
  static constexpr auto low_speed_external_enable = bit_mask::from<0>();

  /// 0: LSE clock not ready
  /// 1: LSE clock ready
  static constexpr auto low_speed_external_ready = bit_mask::from<1>();

  /// 0: LSE oscillator not bypassed
  /// 1: LSE oscillator bypassed
  static constexpr auto low_speed_external_bypass = bit_mask::from<2>();

  /// 0: LSE oscillator “low power” mode selection
  /// 1: LSE oscillator “high drive” mode selection
  static constexpr auto low_speed_external_mode = bit_mask::from<3>();

  /// 00: No clock
  /// 01: LSE oscillator clock used as the RTC clock
  /// 10: LSI oscillator clock used as the RTC clock
  /// 11: HSE oscillator clock divided by hse_division_for_rtc_clock
  static constexpr auto rtc_clock_source = bit_mask::from<9, 8>();

  /// 0: RTC clock disabled
  /// 1: RTC clock enabled
  static constexpr auto rtc_enable = bit_mask::from<15>();

  /// 0: Reset not activated
  /// 1: Resets the entire Backup domain
  static constexpr auto backup_domain_software_reset = bit_mask::from<16>();
};

struct rcc_ahb2{
  static constexpr auto usb_otg_en = bit_mask::from<7>();
};

/**
 * @return reset_and_clock_control_t& - return reset_and_clock_control_t
 * register.
 */
inline reset_and_clock_control_t* rcc =
  reinterpret_cast<reset_and_clock_control_t*>(0x40000000 + 0x20000 + 0x3800);
}  // namespace hal::stm32f411
