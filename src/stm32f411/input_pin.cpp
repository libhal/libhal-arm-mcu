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

#include <cstdint>

#include <libhal-arm-mcu/stm32f411/input_pin.hpp>

#include "gpio_reg.hpp"
#include "power.hpp"
#include <libhal-util/bit.hpp>

#include "gpio_reg.hpp"

namespace hal::stm32f411 {
input_pin::input_pin(hal::stm32f411::peripheral p_port,
                     std::uint8_t p_pin,
                     settings const& p_settings)
  : m_port(p_port)
  , m_pin(p_pin)
{
  input_pin::driver_configure(p_settings);
}

void input_pin::driver_configure(settings const& p_settings)
{
  bit_mask pin_mode_mask = { .position = 2 * static_cast<uint32_t>(m_pin),
                             .width = 2 };

  bit_modify(get_reg(m_port)->pin_mode).clear(pin_mode_mask);

  pin(m_port, m_pin)
    .function(pin::pin_function::input)
    .open_drain(false)
    .resistor(p_settings.resistor);
}

bool input_pin::driver_level()
{
  bit_mask input_data_mask = { .position = static_cast<uint32_t>(m_pin),
                               .width = 1 };
  auto pin_value = bit_extract(input_data_mask, get_reg(m_port)->input_data);
  return static_cast<bool>(pin_value);
}
}  // namespace hal::stm32f411
