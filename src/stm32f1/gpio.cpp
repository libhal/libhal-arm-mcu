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

#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32f1/gpio.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/enum.hpp>
#include <libhal/error.hpp>
#include <libhal/units.hpp>

#include "pin.hpp"
#include "power.hpp"

namespace hal::stm32f1 {
namespace {
u8 peripheral_to_letter(peripheral p_peripheral)
{
  // The numeric value of `peripheral::gpio_a` to ``peripheral::gpio_g` are
  // contiguous in numeric value thus we can map letters 'A' to 'G' by doing
  // this math here.
  auto const offset = value(p_peripheral) - value(peripheral::gpio_a);
  return 'A' + offset;
}
}  // namespace

gpio_manager::gpio_manager(peripheral p_port)
  : m_port(p_port)
{
  if (not is_on(m_port)) {
    power_on(m_port);
  }
}

gpio_manager::input gpio_manager::acquire_input_pin(
  u8 p_pin,
  input_pin::settings const& p_settings)
{
  return { m_port, p_pin, p_settings };
}

gpio_manager::output gpio_manager::acquire_output_pin(
  u8 p_pin,
  output_pin::settings const& p_settings)
{
  return { m_port, p_pin, p_settings };
}

gpio_manager::input::input(peripheral p_port,
                           u8 p_pin,
                           input_pin::settings const& p_settings)
  : m_pin({ .port = peripheral_to_letter(p_port), .pin = p_pin })
{
  reset_pin(m_pin);
  gpio_manager::input::driver_configure(p_settings);
}

void gpio_manager::input::driver_configure(settings const& p_settings)
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

bool gpio_manager::input::driver_level()
{
  auto const& reg = gpio_reg(m_pin.port);
  auto const pin_value = bit_extract(bit_mask::from(m_pin.pin), reg.idr);
  return static_cast<bool>(pin_value);
}

gpio_manager::output::output(peripheral p_port,
                             u8 p_pin,
                             output_pin::settings const& p_settings)
  : m_pin({ .port = peripheral_to_letter(p_port), .pin = p_pin })
{
  throw_if_pin_is_unavailable(m_pin);
  gpio_manager::output::driver_configure(p_settings);
}

void gpio_manager::output::driver_configure(settings const& p_settings)
{
  reset_pin(m_pin);
  if (p_settings.open_drain) {
    configure_pin(m_pin, open_drain_gpio_output);
  } else {
    configure_pin(m_pin, push_pull_gpio_output);
  }
  // NOTE: The `resistor` field is ignored in this function
}

void gpio_manager::output::driver_level(bool p_high)
{
  if (p_high) {
    // The first 16 bits of the register set the output state
    gpio_reg(m_pin.port).bsrr = 1 << m_pin.pin;
  } else {
    // The last 16 bits of the register reset the output state
    gpio_reg(m_pin.port).bsrr = 1 << (16 + m_pin.pin);
  }
}

bool gpio_manager::output::driver_level()
{
  auto const& reg = gpio_reg(m_pin.port);
  auto const pin_value = bit_extract(bit_mask::from(m_pin.pin), reg.idr);
  return static_cast<bool>(pin_value);
}
}  // namespace hal::stm32f1
