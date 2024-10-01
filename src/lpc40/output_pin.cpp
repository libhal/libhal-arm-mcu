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

#include <libhal-lpc40/output_pin.hpp>

#include <cstdint>

#include <libhal-lpc40/pin.hpp>

#include "gpio_reg.hpp"

namespace hal::lpc40 {
output_pin::output_pin(std::uint8_t p_port,  // NOLINT
                       std::uint8_t p_pin,
                       output_pin::settings const& p_settings)
  : m_port(p_port)
  , m_pin(p_pin)
{
  configure(p_settings);  // NOLINT
}

void output_pin::driver_configure(settings const& p_settings)
{
  bit_modify(gpio_reg[m_port]->direction).set(pin_mask(m_pin));

  pin(m_port, m_pin)
    .function(0)
    .dac(false)
    .analog(false)
    .open_drain(p_settings.open_drain)
    .resistor(p_settings.resistor);
}

void output_pin::driver_level(bool p_high)
{
  if (p_high) {
    bit_modify(gpio_reg[m_port]->pin).set(pin_mask(m_pin));
  } else {
    bit_modify(gpio_reg[m_port]->pin).clear(pin_mask(m_pin));
  }
}

bool output_pin::driver_level()
{
  auto pin_value = bit_extract(pin_mask(m_pin), gpio_reg[m_port]->pin);

  return static_cast<bool>(pin_value);
}
}  // namespace hal::lpc40
