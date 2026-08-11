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

#include <array>

module hal.arm_mcu.stm32f1;

import hal;
import hal.util;

namespace hal::stm32f1 {
using namespace mp_units::si::unit_symbols;
namespace {
// The external oscillator frequencies cannot be measured from hardware (the
// silicon has no way to know what crystal, if any, is wired to it), so these
// are the only two clock facts that must be supplied by configure_clocks()
// rather than read back from the clock control registers. Everything else
// frequency() reports is computed live from those registers on every call,
// so it stays correct even if configure_clocks() is never invoked (e.g. a
// previous firmware image already left the PLL running).
hal::hertz high_speed_external_frequency = 0 * Hz;
hal::hertz low_speed_external_frequency = 0 * Hz;
}  // namespace

struct flash_t
{
  u32 volatile acr;
  u32 volatile keyr;
  u32 volatile optkeyr;
  u32 volatile sr;
  u32 volatile cr;
  u32 volatile ar;
  u32 volatile reserved;
  u32 volatile obr;
  u32 volatile wrpr;
  std::array<u32, 8> reserved1;
  u32 volatile keyr2;
  u32 reserved2;
  u32 volatile sr2;
  u32 volatile cr2;
  u32 volatile ar2;
};

/// Pointer to the flash control register
flash_t* flash = reinterpret_cast<flash_t*>(0x4002'2000);

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
void configure_clocks(clock_tree p_clock_tree)
{
  hal::hertz pll_clock_rate = 0 * Hz;  // undefined until pll is enabled

  // =========================================================================
  // Step 1. Select internal clock source for everything.
  //         Make sure PLLs are not clock sources for everything.
  // =========================================================================
  // Step 1.1 Set SystemClock to HSI
  clock_configuration::reg().insert<clock_configuration::system_clock_select>(
    value(system_clock_select::high_speed_internal));

  // Step 1.4 Reset RTC clock registers
  rtc_register::reg().set(rtc_register::backup_domain_reset);

  // Manually clear the RTC reset bit
  rtc_register::reg().clear(rtc_register::backup_domain_reset);

  // =========================================================================
  // Step 2. Disable PLL and external clock sources
  // =========================================================================
  clock_control::reg()
    // Step 2.1 Disable PLLs
    .clear(clock_control::pll_enable)
    // Step 2.1 Disable External Oscillators
    .clear(clock_control::external_osc_enable);

  // =========================================================================
  // Step 3. Enable External Oscillators
  // =========================================================================
  // Step 3.1 Enable High speed external Oscillator
  if (p_clock_tree.high_speed_external > 1 * MHz) {
    clock_control::reg().set(clock_control::external_osc_enable);

    while (!bit_extract<clock_control::external_osc_ready>(
      clock_control::reg().get())) {
      continue;
    }
  }

  // Step 3.2 Enable Low speed external Oscillator
  if (p_clock_tree.low_speed_external > 1 * MHz) {
    rtc_register::reg().set(rtc_register::low_speed_osc_enable);

    while (!bit_extract<rtc_register::low_speed_osc_ready>(
      rtc_register::reg().get())) {
      continue;
    }
  }

  // =========================================================================
  // Step 4. Set oscillator source for PLLs
  // =========================================================================
  clock_configuration::reg()
    .insert<clock_configuration::hse_pre_divider>(
      (p_clock_tree.pll.source == pll_source::high_speed_external_divided_by_2))
    .insert<clock_configuration::pll_source>(value(p_clock_tree.pll.source));

  // =========================================================================
  // Step 5. Setup PLLs and enable them where necessary
  // =========================================================================
  if (p_clock_tree.pll.enable) {
    clock_configuration::reg().insert<clock_configuration::pll_mul>(
      value(p_clock_tree.pll.multiply));

    clock_control::reg().set<clock_control::pll_enable>();

    while (!bit_extract<clock_control::pll_ready>(clock_control::reg().get())) {
      continue;
    }

    switch (p_clock_tree.pll.source) {
      case pll_source::internal_8mhz_divided_by_2:
        pll_clock_rate = internal_high_speed_oscillator / 2;
        break;
      case pll_source::high_speed_external:
        pll_clock_rate = p_clock_tree.high_speed_external;
        break;
      case pll_source::high_speed_external_divided_by_2:
        pll_clock_rate = p_clock_tree.high_speed_external / 2;
        break;
    }

    // Multiply the PLL clock up to the correct rate.
    auto multiply = value(p_clock_tree.pll.multiply);
    pll_clock_rate = pll_clock_rate * (multiply + 2);
  }

  // =========================================================================
  // Step 6. Setup peripheral dividers
  // =========================================================================
  clock_configuration::reg()
    // Step 6.1 Set USB divider
    .insert<clock_configuration::usb_prescalar>(
      value(p_clock_tree.pll.usb.divider))
    // Step 6.2 Set AHB divider
    .insert<clock_configuration::ahb_divider>(value(p_clock_tree.ahb.divider))
    // Step 6.3 Set APB1 divider
    .insert<clock_configuration::apb_1_divider>(
      value(p_clock_tree.ahb.apb1.divider))
    // Step 6.4 Set APB2 divider
    .insert<clock_configuration::apb_2_divider>(
      value(p_clock_tree.ahb.apb2.divider))
    // Step 6.5 Set ADC divider
    .insert<clock_configuration::adc_divider>(
      value(p_clock_tree.ahb.apb2.adc.divider));

  // =========================================================================
  // Step 7. Set System Clock and RTC Clock
  // =========================================================================
  auto const target_clock_source = value(p_clock_tree.system_clock);

  // Step 7.1 Set the Flash wait states appropriately prior to setting the
  //          system clock frequency. Failure to do this will cause the system
  //          to be unable to read from flash, resulting in the platform
  //          locking up. See p.60 of RM0008 for the Flash ACR register
  if (p_clock_tree.system_clock == system_clock_select::pll) {
    constexpr auto mask = bit_mask::from<0, 2>();
    if (pll_clock_rate <= 24 * MHz) {
      // 0 Wait states
      bit_modify(flash->acr).insert<mask>(0b000U);
    } else if ((24 * MHz <= pll_clock_rate) && (pll_clock_rate <= 48 * MHz)) {
      // 1 Wait state
      bit_modify(flash->acr).insert<mask>(0b001U);
    } else {
      // 2 Wait states
      bit_modify(flash->acr).insert<mask>(0b010U);
    }
  }

  // Step 7.2 Set system clock source
  // NOTE: return error if clock = system_clock_select::high_speed_external
  // and
  //       high speed external is not enabled.
  clock_configuration::reg().insert<clock_configuration::system_clock_select>(
    value(p_clock_tree.system_clock));

  while (bit_extract<clock_configuration::system_clock_status>(
           clock_configuration::reg().get()) != target_clock_source) {
    continue;
  }

  rtc_register::reg()
    // Step 7.3 Set the RTC oscillator source
    .insert<rtc_register::rtc_source_select>(value(p_clock_tree.rtc.source))
    // Step 7.4 Enable/Disable the RTC
    .insert<rtc_register::rtc_enable>(p_clock_tree.rtc.enable);

  // =========================================================================
  // Step 8. Remember the externally-supplied oscillator frequencies
  // =========================================================================
  // These are the only clock facts frequency() cannot derive by reading the
  // clock control registers, since no register reports what crystal (if any)
  // is actually wired to the MCU.
  high_speed_external_frequency = p_clock_tree.high_speed_external;
  low_speed_external_frequency = p_clock_tree.low_speed_external;
}

namespace {
/// Converts a divider selection into its numeric divisor. Raw register
/// values that don't match a named enumerator (e.g. a prescaler's "enable"
/// bit clear but its selector bits non-zero) are hardware-equivalent to "not
/// divided", so they fall through to the divide-by-1 default below.
u32 divisor_of(ahb_divider p_divider)
{
  switch (p_divider) {
    case ahb_divider::divide_by_2:
      return 2;
    case ahb_divider::divide_by_4:
      return 4;
    case ahb_divider::divide_by_8:
      return 8;
    case ahb_divider::divide_by_16:
      return 16;
    case ahb_divider::divide_by_64:
      return 64;
    case ahb_divider::divide_by_128:
      return 128;
    case ahb_divider::divide_by_256:
      return 256;
    case ahb_divider::divide_by_512:
      return 512;
    case ahb_divider::divide_by_1:
      [[fallthrough]];
    default:
      return 1;
  }
}

u32 divisor_of(apb_divider p_divider)
{
  switch (p_divider) {
    case apb_divider::divide_by_2:
      return 2;
    case apb_divider::divide_by_4:
      return 4;
    case apb_divider::divide_by_8:
      return 8;
    case apb_divider::divide_by_16:
      return 16;
    case apb_divider::divide_by_1:
      [[fallthrough]];
    default:
      return 1;
  }
}

u32 divisor_of(adc_divider p_divider)
{
  switch (p_divider) {
    case adc_divider::divide_by_4:
      return 4;
    case adc_divider::divide_by_6:
      return 6;
    case adc_divider::divide_by_8:
      return 8;
    case adc_divider::divide_by_2:
      [[fallthrough]];
    default:
      return 2;
  }
}

/// @return the PLL's output frequency, or 0 Hz if the PLL isn't locked.
/// Reads its multiplier and source selection directly from the clock
/// configuration register, so this is correct regardless of whether
/// configure_clocks() was the one that turned the PLL on.
hal::hertz pll_frequency()
{
  if (not bit_extract<clock_control::pll_ready>(clock_control::reg().get())) {
    return 0 * Hz;
  }

  auto const clock_configuration_register = clock_configuration::reg().get();
  bool const from_external = bit_extract<clock_configuration::pll_source>(
    clock_configuration_register);
  bool const external_divided =
    bit_extract<clock_configuration::hse_pre_divider>(
      clock_configuration_register);

  hal::hertz pll_input = internal_high_speed_oscillator / 2;
  if (from_external and external_divided) {
    pll_input = high_speed_external_frequency / 2;
  } else if (from_external) {
    pll_input = high_speed_external_frequency;
  }

  auto const multiply = bit_extract<clock_configuration::pll_mul>(
    clock_configuration_register);
  return pll_input * (multiply + 2);
}

/// @return the currently active system clock frequency, read from the clock
/// configuration register's status bits rather than whatever was last
/// requested.
hal::hertz system_clock_frequency()
{
  auto const status = bit_extract<clock_configuration::system_clock_status>(
    clock_configuration::reg().get());

  switch (static_cast<system_clock_select>(status)) {
    case system_clock_select::high_speed_external:
      return high_speed_external_frequency;
    case system_clock_select::pll:
      return pll_frequency();
    case system_clock_select::high_speed_internal:
      [[fallthrough]];
    default:
      return internal_high_speed_oscillator;
  }
}

hal::hertz ahb_frequency()
{
  auto const raw = bit_extract<clock_configuration::ahb_divider>(
    clock_configuration::reg().get());
  return system_clock_frequency() / divisor_of(static_cast<ahb_divider>(raw));
}

hal::hertz apb1_frequency()
{
  auto const raw = bit_extract<clock_configuration::apb_1_divider>(
    clock_configuration::reg().get());
  return ahb_frequency() / divisor_of(static_cast<apb_divider>(raw));
}

hal::hertz apb2_frequency()
{
  auto const raw = bit_extract<clock_configuration::apb_2_divider>(
    clock_configuration::reg().get());
  return ahb_frequency() / divisor_of(static_cast<apb_divider>(raw));
}

hal::hertz adc_frequency()
{
  auto const raw = bit_extract<clock_configuration::adc_divider>(
    clock_configuration::reg().get());
  return apb2_frequency() / divisor_of(static_cast<adc_divider>(raw));
}

/// STM32F1 timers run at their APB domain's frequency when that domain's
/// prescaler is divide-by-1, and at *twice* that frequency for any other
/// prescaler setting (see RM0008's clock tree diagram).
hal::hertz timer_apb1_frequency()
{
  auto const raw = bit_extract<clock_configuration::apb_1_divider>(
    clock_configuration::reg().get());
  auto const apb1 = apb1_frequency();
  if (divisor_of(static_cast<apb_divider>(raw)) == 1) {
    return apb1;
  }
  return apb1 * 2;
}

hal::hertz timer_apb2_frequency()
{
  auto const raw = bit_extract<clock_configuration::apb_2_divider>(
    clock_configuration::reg().get());
  auto const apb2 = apb2_frequency();
  if (divisor_of(static_cast<apb_divider>(raw)) == 1) {
    return apb2;
  }
  return apb2 * 2;
}

hal::hertz usb_frequency()
{
  bool const divide_by_1 = bit_extract<clock_configuration::usb_prescalar>(
    clock_configuration::reg().get());
  auto const pll = pll_frequency();
  if (divide_by_1) {
    return pll;
  }
  return (pll * 2) / 3;
}

}  // namespace

/// @return the clock rate frequency of a peripheral
hal::hertz frequency(peripheral p_id)
{
  switch (p_id) {
    case peripheral::i2s:
      return pll_frequency();
    case peripheral::usb:
      return usb_frequency();
    case peripheral::flitf:
      return internal_high_speed_oscillator;

    // Arm Cortex running clock rate.
    // This code does not utilize the /8 clock for the system timer, thus the
    // clock rate for that subsystem is equal to the CPU running clock.
    case peripheral::system_timer:
      [[fallthrough]];
    case peripheral::cpu:
      return ahb_frequency();

    // APB1 Timers
    case peripheral::timer2:
      [[fallthrough]];
    case peripheral::timer3:
      [[fallthrough]];
    case peripheral::timer4:
      [[fallthrough]];
    case peripheral::timer5:
      [[fallthrough]];
    case peripheral::timer6:
      [[fallthrough]];
    case peripheral::timer7:
      [[fallthrough]];
    case peripheral::timer12:
      [[fallthrough]];
    case peripheral::timer13:
      [[fallthrough]];
    case peripheral::timer14:
      return timer_apb1_frequency();

    // APB2 Timers
    case peripheral::timer1:
      [[fallthrough]];
    case peripheral::timer8:
      [[fallthrough]];
    case peripheral::timer9:
      [[fallthrough]];
    case peripheral::timer10:
      [[fallthrough]];
    case peripheral::timer11:
      return timer_apb2_frequency();

    case peripheral::adc1:
      [[fallthrough]];
    case peripheral::adc2:
      [[fallthrough]];
    case peripheral::adc3:
      return adc_frequency();
    default: {
      auto id = value(p_id);

      if (id < apb1_bus) {
        return ahb_frequency();
      }

      if (apb1_bus <= id && id < apb2_bus) {
        return apb1_frequency();
      }

      if (apb2_bus <= id && id < beyond_bus) {
        return apb2_frequency();
      }

      return 0 * Hz;
    }
  }

  return 0 * Hz;
}

void maximum_speed_using_internal_oscillator()
{
  configure_clocks(clock_tree{
    .high_speed_external = 0 * Hz,
    .pll = {
      .enable = true,
      .source = pll_source::internal_8mhz_divided_by_2,
      .multiply = pll_multiply::multiply_by_16,
      .usb = { // NOTE: Cannot be used when using the internal oscillator
        .divider = usb_divider::divide_by_1_point_5,
      }
    },
    .system_clock = system_clock_select::pll,
    .ahb = {
      .divider = ahb_divider::divide_by_1,
      .apb1 = {
        .divider = apb_divider::divide_by_2,
      },
      .apb2 = {
        .divider = apb_divider::divide_by_1,
        .adc = {
          .divider = adc_divider::divide_by_6,
        }
      },
    },
  });
}
}  // namespace hal::stm32f1
