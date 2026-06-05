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

#include <cmath>
#include <cstdint>

#include <libhal-arm-mcu/stm32f411/clock.hpp>
#include <libhal-arm-mcu/stm32f411/constants.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/enum.hpp>
#include <libhal/error.hpp>
#include <libhal/units.hpp>

#include "flash_reg.hpp"
#include "rcc_reg.hpp"

namespace hal::stm32f411 {

namespace {
hal::hertz rtc_clock_rate = 0.0_Hz;      // defaults to "no clock"
hal::hertz pll_clock_rate = 0.0_Hz;      // undefined until pll is enabled
hal::hertz usb_otg_clock_rate = 0.0_Hz;  // undefined until enabled
hal::hertz ahb_clock_rate = internal_high_speed_oscillator;
hal::hertz apb1_clock_rate = internal_high_speed_oscillator;
hal::hertz apb2_clock_rate = internal_high_speed_oscillator;
hal::hertz timer_apb1_clock_rate = internal_high_speed_oscillator;
hal::hertz timer_apb2_clock_rate = internal_high_speed_oscillator;
}  // namespace

void pll_valid(clock_tree const& p_clock_tree)
{
  /// Check if the VCO input can be set from 1_MHz to 2_MHz
  if (p_clock_tree.pll.source == pll_source::high_speed_external) {
    if (p_clock_tree.high_speed_external < 2.0_MHz ||
        p_clock_tree.high_speed_external > 63 * 2.0_MHz) {
      hal::safe_throw(hal::argument_out_of_domain(nullptr));
    }
  }

  /// Check the Output Frequency is within bound
  if (p_clock_tree.pll.output > (100.0_MHz) ||
      p_clock_tree.pll.output < (100.0_MHz / 8)) {
    hal::safe_throw(hal::argument_out_of_domain(nullptr));
  }
}

// TODO (#62): Add I2S to the clock configs
// TODO (#66): Add 48_MHz to the clock configs
hertz configure_pll(clock_tree const& p_clock_tree)
{
  auto vco_in = 2.0_MHz;
  float target = 0.0f;
  float output_division_factor = 0;
  if (p_clock_tree.pll.enable) {
    uint8_t input_division_factor = 2U;
    pll_valid(p_clock_tree);

    if (p_clock_tree.pll.source == pll_source::high_speed_external) {
      if (p_clock_tree.high_speed_external < 4.0_MHz) {
        vco_in = p_clock_tree.high_speed_external / 2;
        bit_modify(rcc->pllconfig)
          .insert(rcc_pllcnfg::input_division_factor, input_division_factor);
      } else {
        float division_factor = p_clock_tree.high_speed_external / 2.0_MHz;
        if ((division_factor - std::floor(division_factor)) > 0) {
          input_division_factor = static_cast<uint8_t>(division_factor) + 1;
        } else {
          input_division_factor = static_cast<uint8_t>(division_factor);
        }
        vco_in = p_clock_tree.high_speed_external / division_factor;
      }
    } else {
      input_division_factor = 8;
    }
    bit_modify(rcc->pllconfig)
      .insert(rcc_pllcnfg::input_division_factor, input_division_factor);

    /// 2 * (output_division_factor + 1) is to get the PLLP division factor
    /// Page 105 on the RM0383 User Manual
    while (p_clock_tree.pll.output * (2.0f * (output_division_factor + 1.0f)) <
           100.0_MHz) {
      output_division_factor += 1;
    }

    bit_modify(rcc->pllconfig)
      .insert(rcc_pllcnfg::main_division_factor,
              static_cast<std::uint8_t>(output_division_factor));

    /// 2 * (output_division_factor + 1) is to get the PLLN multiplicaiton
    /// factor Page 105 on the RM0383 User Manual
    target = p_clock_tree.pll.output *
             (2.0f * (output_division_factor + 1.0f)) / vco_in;
    bit_modify(rcc->pllconfig)
      .insert(rcc_pllcnfg::main_multiplication_factor,
              static_cast<uint16_t>(target));

    bit_modify(rcc->pllconfig)
      .insert(rcc_pllcnfg::pll_source, value(p_clock_tree.pll.source));

    bit_modify(rcc->cr).insert(rcc_cr::main_pll_enable,
                               p_clock_tree.pll.enable);

    while (!bit_extract<rcc_cr::main_pll_ready>(rcc->cr)) {
      continue;
    }
  }
  return (vco_in * std::floor(target) /
          (2 * (std::floor(output_division_factor + 1.0f))));
}

void configure_clocks(clock_tree const& p_clock_tree)
{
  hal::hertz system_clock = 0.0_Hz;

  // =========================================================================
  // Step 1. Select internal clock source for everything.
  //         Make sure PLLs are not clock sources for everything.
  // =========================================================================
  // Step 1.0 Turn on High speed external:
  bit_modify(rcc->cr).set(rcc_cr::high_speed_internal_enable);

  // Step 1.1 Set SystemClock to HSI
  bit_modify(rcc->config)
    .insert(rcc_config::system_clock_switch,
            value(system_clock_select::high_speed_internal));

  // Step 1.4 Reset RTC clock registers
  bit_modify(rcc->backup_domain_control)
    .set(rcc_backup_domain_control::backup_domain_software_reset);

  // Manually clear the RTC reset bit
  bit_modify(rcc->backup_domain_control)
    .clear(rcc_backup_domain_control::backup_domain_software_reset);

  // =========================================================================
  // Step 2. Disable PLL and external clock sources
  // =========================================================================
  // 2.1 Move the system_clock to HSI
  bit_modify(rcc->config).clear(rcc_config::system_clock_switch);

  while (bit_extract<rcc_config::system_clock_switch_status>(rcc->config)) {
    continue;
  }

  // 2.2 Disable LSE
  bit_modify(rcc->backup_domain_control)
    .clear(rcc_backup_domain_control::low_speed_external_enable);

  while (bit_extract<rcc_backup_domain_control::low_speed_external_ready>(
    rcc->backup_domain_control)) {
    continue;
  }

  // 2.3 Disable HSE
  bit_modify(rcc->cr).clear(rcc_cr::high_speed_external_enable);

  while (bit_extract<rcc_cr::high_speed_external_ready>(rcc->cr)) {
    continue;
  }

  // 2.4 Disable PLL
  bit_modify(rcc->cr).clear(rcc_cr::main_pll_enable);
  while (bit_extract<rcc_cr::main_pll_ready>(rcc->cr)) {
    continue;
  }

  // =========================================================================
  // Step 3. Enable External Oscillators
  // =========================================================================
  // 3.1 Enable HSE
  if (p_clock_tree.high_speed_external > 4.0_MHz) {
    bit_modify(rcc->cr).set(rcc_cr::high_speed_external_enable);
    while (!bit_extract<rcc_cr::high_speed_external_ready>(rcc->cr)) {
      continue;
    }
  }

  // 3.2 Enable LSE
  if (p_clock_tree.low_speed_external > 1.0_MHz) {
    bit_modify(rcc->backup_domain_control)
      .clear(rcc_backup_domain_control::low_speed_external_enable);

    while (bit_extract<rcc_backup_domain_control::low_speed_external_ready>(
      rcc->backup_domain_control)) {
      continue;
    }
  }

  // =========================================================================
  // Step 4. Set up PLL
  // =========================================================================
  pll_clock_rate = configure_pll(p_clock_tree);

  // =========================================================================
  // Step 5. Setup peripheral dividers
  // =========================================================================
  bit_modify(rcc->config)
    .insert(rcc_config::ahb_prescalar, value(p_clock_tree.ahb.divider))
    .insert(rcc_config::apb1_prescalar, value(p_clock_tree.ahb.apb1.divider))
    .insert(rcc_config::apb2_prescalar, value(p_clock_tree.ahb.apb2.divider));
  // =========================================================================
  // Step 6. Setup flash access
  // =========================================================================
  bit_modify(flash_config->access_control_reg)
    .set(flash_acess_control::instruction_cache_en)
    .set(flash_acess_control::data_cache_en)
    .set(flash_acess_control::prefetch_cache_en)
    .insert(flash_acess_control::latency, 2U);

  // =========================================================================
  // Step 7. Set System Clock and RTC Clock
  // =========================================================================

  std::uint32_t target_clock_source = value(p_clock_tree.system_clock);
  bit_modify(rcc->config)
    .insert(rcc_config::system_clock_switch, target_clock_source);

  while (bit_extract<rcc_config::system_clock_switch_status>(rcc->config) !=
         target_clock_source) {
    continue;
  }
  switch (p_clock_tree.system_clock) {
    case system_clock_select::high_speed_internal:
      system_clock = internal_high_speed_oscillator;
      break;
    case system_clock_select::high_speed_external:
      system_clock = p_clock_tree.high_speed_external;
      break;
    case system_clock_select::pll:
      system_clock = pll_clock_rate;
      break;
  }
  bit_modify(rcc->backup_domain_control)
    .insert(rcc_backup_domain_control::rtc_clock_source,
            value(p_clock_tree.rtc.source))
    .insert(rcc_backup_domain_control::rtc_enable, p_clock_tree.rtc.enable);

  bit_modify(rcc->ahb2enr)
    .insert(rcc_ahb2::usb_otg_en, p_clock_tree.usb_otg_clock_enable);
  // =========================================================================
  // Step 8. Define the clock rates for the system
  // =========================================================================

  switch (p_clock_tree.ahb.divider) {
    case ahb_divider::divide_by_1:
      ahb_clock_rate = system_clock / 1;
      break;
    case ahb_divider::divide_by_2:
      ahb_clock_rate = system_clock / 2;
      break;
    case ahb_divider::divide_by_4:
      ahb_clock_rate = system_clock / 4;
      break;
    case ahb_divider::divide_by_8:
      ahb_clock_rate = system_clock / 8;
      break;
    case ahb_divider::divide_by_16:
      ahb_clock_rate = system_clock / 16;
      break;
    case ahb_divider::divide_by_64:
      ahb_clock_rate = system_clock / 64;
      break;
    case ahb_divider::divide_by_128:
      ahb_clock_rate = system_clock / 128;
      break;
    case ahb_divider::divide_by_256:
      ahb_clock_rate = system_clock / 256;
      break;
    case ahb_divider::divide_by_512:
      ahb_clock_rate = system_clock / 512;
      break;
  }

  switch (p_clock_tree.ahb.apb1.divider) {
    case apb_divider::divide_by_1:
      apb1_clock_rate = ahb_clock_rate / 1;
      break;
    case apb_divider::divide_by_2:
      apb1_clock_rate = ahb_clock_rate / 2;
      break;
    case apb_divider::divide_by_4:
      apb1_clock_rate = ahb_clock_rate / 4;
      break;
    case apb_divider::divide_by_8:
      apb1_clock_rate = ahb_clock_rate / 8;
      break;
    case apb_divider::divide_by_16:
      apb1_clock_rate = ahb_clock_rate / 16;
      break;
  }

  switch (p_clock_tree.ahb.apb2.divider) {
    case apb_divider::divide_by_1:
      apb2_clock_rate = ahb_clock_rate / 1;
      break;
    case apb_divider::divide_by_2:
      apb2_clock_rate = ahb_clock_rate / 2;
      break;
    case apb_divider::divide_by_4:
      apb2_clock_rate = ahb_clock_rate / 4;
      break;
    case apb_divider::divide_by_8:
      apb2_clock_rate = ahb_clock_rate / 8;
      break;
    case apb_divider::divide_by_16:
      apb2_clock_rate = ahb_clock_rate / 16;
      break;
  }

  switch (p_clock_tree.rtc.source) {
    case rtc_source::no_clock:
      rtc_clock_rate = 0.0_Hz;
      break;
    case rtc_source::low_speed_internal:
      rtc_clock_rate = internal_low_speed_oscillator;
      break;
    case rtc_source::low_speed_external:
      rtc_clock_rate = p_clock_tree.low_speed_external;
      break;
    case rtc_source::high_speed_external_divided_by_128:
      rtc_clock_rate = p_clock_tree.high_speed_external / 128;
      break;
  }

  // This is set to the default Dedicated Clocks Configuration setting. Unsure
  // how to create an api for this register

  switch (p_clock_tree.ahb.apb1.divider) {
    case apb_divider::divide_by_1:
      timer_apb1_clock_rate = apb1_clock_rate;
      break;
    default:
      timer_apb1_clock_rate = apb1_clock_rate * 2;
      break;
  }

  switch (p_clock_tree.ahb.apb2.divider) {
    case apb_divider::divide_by_1:
      timer_apb2_clock_rate = apb2_clock_rate;
      break;
    default:
      timer_apb2_clock_rate = apb2_clock_rate * 2;
      break;
  }
  return;
}

hal::hertz frequency(peripheral p_id)
{
  switch (p_id) {
    // TODO #62: Add I2S to the clock configs
    case peripheral::i2s:
      return pll_clock_rate;

    // Arm Cortex running clock rate.
    // This code does not utilize the /8 clock for the system timer, thus the
    // clock rate for that subsystem is equal to the CPU running clock.
    case peripheral::system_timer:
      [[fallthrough]];
    case peripheral::cpu:
      return ahb_clock_rate;

    // APB1 Timers
    case peripheral::timer2:
      [[fallthrough]];
    case peripheral::timer3:
      [[fallthrough]];
    case peripheral::timer4:
      [[fallthrough]];
    case peripheral::timer5:
      return timer_apb1_clock_rate;

    // APB2 Timers
    case peripheral::timer1:
      [[fallthrough]];
    case peripheral::timer9:
      [[fallthrough]];
    case peripheral::timer10:
      [[fallthrough]];
    case peripheral::timer11:
      return timer_apb2_clock_rate;

    default: {
      auto id_bus = value(p_id) / bus_id_offset;
      switch (id_bus * bus_id_offset) {
        case ahb1_bus:
          return ahb_clock_rate;
        case ahb2_bus:
          return usb_otg_clock_rate;
        case apb1_bus:
          return apb1_clock_rate;
        case apb2_bus:
          return apb2_clock_rate;
        default:
          return 0.0_Hz;
      }
    }
  }

  return 0.0_Hz;
}

void maximum_speed_using_internal_oscillator()
{
  configure_clocks(clock_tree{
    .high_speed_external = 0.0_Hz,
    .low_speed_external = 0.0_Hz,
    .pll = { .enable = true,
             .source = pll_source::high_speed_internal,
             .output = 100.0_MHz },
    .system_clock = system_clock_select::pll,
    .ahb = { .divider = ahb_divider::divide_by_1,
             .apb1 = { .divider = apb_divider::divide_by_2 },
             .apb2 = { .divider = apb_divider::divide_by_1,
                       .adc = { .divider = adc_divider::divide_by_2 } }

    } });
}

};  // namespace hal::stm32f411
