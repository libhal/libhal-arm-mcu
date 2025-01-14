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

#include <libhal-arm-mcu/stm32f411/output_pin.hpp>
#include <libhal-util/bit.hpp>

#include "gpio_reg.hpp"

namespace hal::stm32f411 {

output_pin::output_pin(hal::stm32f411::peripheral p_port,
                       std::uint8_t p_pin,
                       output_pin::settings p_settings)
  : m_port(p_port)
  , m_pin(p_pin)
{
  output_pin::driver_configure(p_settings);
}

void output_pin::driver_configure(settings const& p_settings)
{
  bit_mask pin_mode_mask = { .position = 2 * static_cast<uint32_t>(m_pin),
                             .width = 2 };
  bit_modify(get_gpio_reg(m_port)->pin_mode).insert(pin_mode_mask, 0b01U);

  pin(m_port, m_pin)
    .function(pin::pin_function::output)
    .open_drain(p_settings.open_drain)
    .resistor(p_settings.resistor);
}

void output_pin::driver_level(bool p_high)
{
  bit_mask set_bit = { .position = static_cast<uint32_t>(m_pin), .width = 1 };
  if (p_high) {
    get_gpio_reg(m_port)->set = bit_value(0U).set(set_bit).to<std::uint16_t>();
  } else {
    get_gpio_reg(m_port)->reset =
      bit_value(0U).set(set_bit).to<std::uint16_t>();
  }
}

bool output_pin::driver_level()
{
  bit_mask output_data_mask = { .position = static_cast<uint32_t>(m_pin),
                                .width = 1 };
  return bit_extract(output_data_mask, get_gpio_reg(m_port)->output_data);
}
}  // namespace hal::stm32f411
