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

#include "libhal-arm-mcu/stm32f1/input_pin.hpp"
#include <libhal-stm32f1/input_pin.hpp>

#include <cstdint>

#include <libhal-util/bit.hpp>
#include <libhal/error.hpp>
#include <libhal/units.hpp>

#include "pin.hpp"

namespace hal::stm32f1 {
input_pin::input_pin(std::uint8_t p_port,  // NOLINT
                     std::uint8_t p_pin)   // NOLINT

  : m_port(p_port)
  , m_pin(p_pin)
{
  input_pin::driver_configure({});
}

void input_pin::driver_configure(settings const& p_settings)
{
  pin_select const pin = { .port = m_port, .pin = m_pin };
  reset_pin(pin);
  if (p_settings.resistor == pin_resistor::pull_up) {
    configure_pin(pin, input_pull_up);
  } else if (p_settings.resistor == pin_resistor::pull_down) {
    configure_pin(pin, input_pull_down);
  } else {
    configure_pin(pin, input_float);
  }
}

bool input_pin::driver_level()
{
  auto const pin_value =
    bit_extract(bit_mask::from(m_pin), gpio_reg(m_port).idr);
  return static_cast<bool>(pin_value);
}
}  // namespace hal::stm32f1
