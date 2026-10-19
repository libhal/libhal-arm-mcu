// Copyright 2026 Khalil Estell and the libhal contributors
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

module hal.arm_mcu.stm32f1;

import hal;
import hal.util;

import :pin;

namespace hal::stm32f1 {
namespace {
pin_config_t output_config(hal::pin_settings const& p_settings,
                           output_speed p_speed)
{
  return {
    .CNF1 = static_cast<u8>(p_settings.open_drain ? 1 : 0),
    .CNF0 = 0,
    .MODE = static_cast<u8>(p_speed),
    .PxODR = 0,
  };
}

bool read_level(pin p_pin)
{
  auto const& reg = gpio_reg(p_pin.port);
  auto const pin_value = bit_extract(bit_mask::from(p_pin.pin), reg.idr);
  return static_cast<bool>(pin_value);
}
}  // namespace

input_pin::input_pin(pin p_pin, pin_settings const& p_settings)
  : m_pin(p_pin)
{
  configure(p_settings);
}

void input_pin::configure(pin_settings const& p_settings)
{
  reset_pin(m_pin);

  if (p_settings.resistor == pin_resistor::pull_up) {
    configure_pin(m_pin, input_pull_up);
  } else if (p_settings.resistor == pin_resistor::pull_down) {
    configure_pin(m_pin, input_pull_down);
  } else {
    configure_pin(m_pin, input_float);
  }
}

void input_pin::driver_configure(pin_settings const& p_settings)
{
  configure(p_settings);
}

bool input_pin::driver_level()
{
  return read_level(m_pin);
}

output_pin::output_pin(pin p_pin,
                       pin_settings const& p_settings,
                       output_speed p_speed)
  : m_pin(p_pin)
  , m_speed(p_speed)
{
  throw_if_pin_is_unavailable(m_pin);
  configure(p_settings);
}

void output_pin::configure(pin_settings const& p_settings)
{
  reset_pin(m_pin);
  configure_pin(m_pin, output_config(p_settings, m_speed));
}

void output_pin::driver_configure(pin_settings const& p_settings)
{
  configure(p_settings);
}

void output_pin::driver_level(bool p_high)
{
  if (p_high) {
    // The first 16 bits of the register set the output state
    gpio_reg(m_pin.port).bsrr = 1U << m_pin.pin;
  } else {
    // The last 16 bits of the register reset the output state
    gpio_reg(m_pin.port).bsrr = 1U << (16 + m_pin.pin);
  }
}

bool output_pin::driver_level()
{
  return read_level(m_pin);
}
}  // namespace hal::stm32f1
