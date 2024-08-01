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

#include <libhal-lpc40/input_pin.hpp>

#include <libhal-lpc40/clock.hpp>
#include <libhal-lpc40/constants.hpp>
#include <libhal-lpc40/pin.hpp>
#include <libhal-lpc40/power.hpp>
#include <libhal-util/bit.hpp>

#include "gpio_reg.hpp"

namespace hal::lpc40 {
input_pin::input_pin(std::uint8_t p_port,  // NOLINT
                     std::uint8_t p_pin,
                     const settings& p_settings)  // NOLINT
  : m_port(p_port)
  , m_pin(p_pin)
{
  configure(p_settings);
}

void input_pin::driver_configure(const settings& p_settings)
{
  power_on(peripheral::gpio);

  bit_modify(gpio_reg[m_port]->direction).clear(pin_mask(m_pin));

  // Pin mask is used to control which pins are updated or not through the
  // pin, set, and clear registers. The mask bit corresponding to the pin must
  // be set to 0 for the pin to be enabled.
  bit_modify(gpio_reg[m_port]->mask).clear(pin_mask(m_pin));

  pin(m_port, m_pin)
    .function(0)
    .dac(false)
    .analog(false)
    .open_drain(false)
    .resistor(p_settings.resistor);
}

bool input_pin::driver_level()
{
  auto pin_value = bit_extract(pin_mask(m_pin), gpio_reg[m_port]->pin);
  return static_cast<bool>(pin_value);
}
}  // namespace hal::lpc40
