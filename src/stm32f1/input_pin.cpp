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

#include <libhal-stm32f1/input_pin.hpp>

#include <cstdint>

#include <libhal-util/bit.hpp>
#include <libhal/error.hpp>
#include <libhal/units.hpp>

#include "libhal-arm-mcu/stm32f1/constants.hpp"
#include "pin.hpp"
#include "power.hpp"

namespace hal::stm32f1 {
input_pin::input_pin(std::uint8_t p_port,  // NOLINT
                     std::uint8_t p_pin)   // NOLINT

  : m_port(p_port)
  , m_pin(p_pin)
{
  // Ensure that AFIO is powered on before attempting to access it
  if (not is_on(peripheral::afio)) {
    power_on(peripheral::afio);
  }

  if (p_port == 'A' && not is_on(peripheral::gpio_a)) {
    power_on(peripheral::gpio_a);
  } else if (p_port == 'B' && not is_on(peripheral::gpio_b)) {
    power_on(peripheral::gpio_b);
  } else if (p_port == 'C' && not is_on(peripheral::gpio_c)) {
    power_on(peripheral::gpio_c);
  } else if (p_port == 'D' && not is_on(peripheral::gpio_d)) {
    power_on(peripheral::gpio_d);
  } else if (p_port == 'E' && not is_on(peripheral::gpio_e)) {
    power_on(peripheral::gpio_e);
  } else {
    hal::safe_throw(hal::argument_out_of_domain(this));
  }
}

void input_pin::driver_configure(settings const& p_settings)
{
  if (p_settings.resistor == pin_resistor::pull_up) {
    configure_pin({ .port = m_port, .pin = m_pin }, input_pull_up);
  } else if (p_settings.resistor == pin_resistor::pull_down) {
    configure_pin({ .port = m_port, .pin = m_pin }, input_pull_down);
  } else {
    configure_pin({ .port = m_port, .pin = m_pin }, input_float);
  }
}

bool input_pin::driver_level()
{
  auto const pin_value =
    bit_extract(bit_mask::from(m_pin), gpio_reg(m_port).idr);
  return static_cast<bool>(pin_value);
}
}  // namespace hal::stm32f1
