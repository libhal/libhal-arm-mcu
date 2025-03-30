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

#include <cstdint>

#include <libhal-arm-mcu/stm32f411/pin.hpp>

#include <libhal-arm-mcu/stm32f411/constants.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/enum.hpp>
#include <libhal/units.hpp>

#include "gpio_reg.hpp"
#include "power.hpp"
#include "rcc_reg.hpp"

namespace hal::stm32f411 {

pin::pin(peripheral p_port, std::uint8_t p_pin) noexcept
  : m_port(p_port)
  , m_pin(p_pin)
{
  power(p_port).on();
}

pin const& pin::function(pin_function p_function) const noexcept
{
  auto port_reg = get_gpio_reg(m_port);
  bit_mask pin_mode_mask = { .position = static_cast<uint32_t>(m_pin) * 2U,
                             .width = 2 };

  switch (p_function) {
    case pin_function::input:
      bit_modify(port_reg->pin_mode).insert(pin_mode_mask, 0b00U);
      break;
    case pin_function::output:
      bit_modify(port_reg->pin_mode).insert(pin_mode_mask, 0b01U);
      break;
    case pin_function::analog:
      bit_modify(port_reg->pin_mode).insert(pin_mode_mask, 0b11U);
      break;
    default:
      bit_modify(port_reg->pin_mode).insert(pin_mode_mask, 0b10U);
      uint8_t alt_func = static_cast<uint8_t>(p_function) - 3U;
      bit_mask alt_func_mask = { .position =
                                   (static_cast<uint32_t>(m_pin) * 4U) % 32,
                                 .width = 4 };
      if (m_pin < 8) {
        bit_modify(port_reg->alt_function_low).insert(alt_func_mask, alt_func);
      } else {
        bit_modify(port_reg->alt_function_high).insert(alt_func_mask, alt_func);
      }
      break;
  }
  return *this;
}

pin const& pin::resistor(hal::pin_resistor p_resistor) const noexcept
{
  // modify the pull_up_pull_down reg to the enumclass of p_registor
  auto port_reg = get_gpio_reg(m_port);
  bit_mask port_mask = { .position = 2 * static_cast<uint32_t>(m_pin),
                         .width = 2 };
  switch (p_resistor) {
    case pin_resistor::none:
      hal::bit_modify(port_reg->pull_up_pull_down).insert(port_mask, 0b00U);
      break;
    case pin_resistor::pull_up:
      hal::bit_modify(port_reg->pull_up_pull_down).insert(port_mask, 0b01U);
      break;
    case pin_resistor::pull_down:
      hal::bit_modify(port_reg->pull_up_pull_down).insert(port_mask, 0b10U);
      break;
    default:
      hal::bit_modify(port_reg->pull_up_pull_down).insert(port_mask, 0b00U);
      break;
  }
  return *this;
}

pin const& pin::open_drain(bool p_enable) const noexcept
{
  // modify output_type to p_enable
  auto port_reg = get_gpio_reg(m_port);
  bit_mask pin_mask = { .position = static_cast<uint32_t>(m_pin), .width = 1 };

  bit_modify(port_reg->output_type)
    .insert(pin_mask, static_cast<uint32_t>(p_enable));
  return *this;
}

void pin::activate_mco_pc9(mco_source p_source)
{
  bit_modify(rcc->config)
    .insert(rcc_config::mco2_clock_select, value(p_source));
  bit_modify(rcc->config).insert(rcc_config::mco2_prescaler, 0b110U);
  pin a8(peripheral::gpio_c, 9);
  a8.function(pin_function::alternate0);

  return;
}

}  // namespace hal::stm32f411
