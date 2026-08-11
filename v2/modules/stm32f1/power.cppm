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

#include <cstdint>

export module hal.arm_mcu.stm32f1:power;

import hal;
import hal.util;

import :constants;

namespace hal::stm32f1 {
/// Register map for the Reset and Clock Control (RCC) peripheral
struct reset_and_clock_control_t
{
  u32 volatile cr;
  u32 volatile cfgr;
  u32 volatile cir;
  u32 volatile apb2rstr;
  u32 volatile apb1rstr;
  u32 volatile ahbenr;
  u32 volatile apb2enr;
  u32 volatile apb1enr;
  u32 volatile bdcr;
  u32 volatile csr;
  u32 volatile ahbrstr;
  u32 volatile cfgr2;
};

constexpr uptr rcc_address = 0x40000000 + 0x20000 + 0x1000;

/// Reset and Clock Control (RCC) peripheral register
// NOLINTNEXTLINE(performance-no-int-to-ptr)
auto* rcc = reinterpret_cast<reset_and_clock_control_t*>(rcc_address);

/// Information about where a peripheral's enable/reset bit lives
struct rcc_register_info
{
  u32 volatile* reg;
  hal::bit_mask mask;
};

rcc_register_info get_enable_register_info(peripheral p_peripheral)
{
  auto const peripheral_value = hal::value(p_peripheral);
  auto const bus_number = peripheral_value / bus_id_offset;
  auto const mask = bit_mask::from(peripheral_value % bus_id_offset);
  switch (bus_number) {
    case 0:
      return { .reg = &rcc->ahbenr, .mask = mask };
    case 1:
      return { .reg = &rcc->apb1enr, .mask = mask };
    case 2:
      return { .reg = &rcc->apb2enr, .mask = mask };
    default:
      throw hal::argument_out_of_domain(nullptr);
  }
}

rcc_register_info get_reset_register_info(peripheral p_peripheral)
{
  auto const peripheral_value = hal::value(p_peripheral);
  auto const bus_number = peripheral_value / bus_id_offset;
  auto const mask = bit_mask::from(peripheral_value % bus_id_offset);
  switch (bus_number) {
    case 0:
      return { .reg = &rcc->ahbrstr, .mask = mask };
    case 1:
      return { .reg = &rcc->apb1rstr, .mask = mask };
    case 2:
      [[fallthrough]];
    default:
      return { .reg = &rcc->apb2rstr, .mask = mask };
  }
}

/**
 * @brief Power on the peripheral
 *
 * This API also acts as a resource overlap detector. If this API is called
 * twice on the same peripheral, it will throw an exception. Only drivers
 * with control over the entire peripheral should call this API for their
 * respective peripheral. This allows this API to detect when two drivers
 * attempt to utilize the same resource.
 *
 * @throws hal::device_or_resource_busy - if the peripheral is already
 * powered on, constituting a violation of the 1 peripheral manager per
 * peripheral rule.
 * @throws hal::argument_out_of_domain - if the peripheral's value is
 * outside of the bounds of the enum class OR if there is no enable
 * register for that peripheral.
 */
export void power_on(peripheral p_peripheral)
{
  auto const info = get_enable_register_info(p_peripheral);

  if (hal::bit_extract(info.mask, *info.reg)) {
    throw hal::device_or_resource_busy(nullptr);
  }

  hal::bit_modify(*info.reg).set(info.mask);
}

/**
 * @brief Power off peripheral
 *
 * If the peripheral is already powered off, this does nothing.
 */
export void power_off(peripheral p_peripheral)
{
  auto const info = get_enable_register_info(p_peripheral);
  hal::bit_modify(*info.reg).clear(info.mask);
}

/**
 * @brief Check if the peripheral is powered on
 *
 * @return true - peripheral is on
 * @return false - peripheral is off
 */
export [[nodiscard]] bool is_on(peripheral p_peripheral)
{
  auto const info = get_enable_register_info(p_peripheral);
  return hal::bit_extract(info.mask, *info.reg);
}

/**
 * @brief Resets the peripheral
 *
 * This will reset all the peripheral's registers to their reset/default
 * values.
 */
export void reset_peripheral(peripheral p_peripheral)
{
  auto const info = get_reset_register_info(p_peripheral);
  hal::bit_modify(*info.reg).set(info.mask);
  hal::bit_modify(*info.reg).clear(info.mask);
}
}  // namespace hal::stm32f1
