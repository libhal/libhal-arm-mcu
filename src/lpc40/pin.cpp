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

#include <libhal-lpc40/pin.hpp>

#include <libhal-util/bit.hpp>
#include <libhal/units.hpp>

#include "pin_reg.hpp"

namespace hal::lpc40 {
const pin& pin::function(uint8_t p_function_code) const
{
  hal::bit_modify(pin_map->matrix[m_port][m_pin])
    .insert<pin_function>(p_function_code);
  return *this;
}

const pin& pin::resistor(hal::pin_resistor p_resistor) const
{
  uint8_t resistor_code = 0;
  switch (p_resistor) {
    case hal::pin_resistor::none:
      resistor_code = 0b00;
      break;
    case hal::pin_resistor::pull_down:
      resistor_code = 0b01;
      break;
    case hal::pin_resistor::pull_up:
      resistor_code = 0b10;
      break;
  }
  // The pin resistor enumeration matches the values for the LPC40xx so simply
  // cast the enum to an int and this will work.
  hal::bit_modify(pin_map->matrix[m_port][m_pin])
    .insert<pin_resistor>(resistor_code);
  return *this;
}

const pin& pin::hysteresis(bool p_enable) const
{
  hal::bit_modify(pin_map->matrix[m_port][m_pin])
    .insert<pin_hysteresis>(p_enable);
  return *this;
}

const pin& pin::input_invert(bool p_enable) const
{
  hal::bit_modify(pin_map->matrix[m_port][m_pin])
    .insert<pin_input_invert>(p_enable);
  return *this;
}

const pin& pin::analog(bool p_enable) const
{
  bool is_digital = !p_enable;
  hal::bit_modify(pin_map->matrix[m_port][m_pin])
    .insert<pin_analog_digital_mode>(is_digital);
  return *this;
}

const pin& pin::digital_filter(bool p_enable) const
{
  hal::bit_modify(pin_map->matrix[m_port][m_pin])
    .insert<pin_digital_filter>(p_enable);
  return *this;
}

const pin& pin::highspeed_i2c(bool p_enable) const
{
  hal::bit_modify(pin_map->matrix[m_port][m_pin])
    .insert<pin_i2c_highspeed>(p_enable);
  return *this;
}

const pin& pin::high_slew_rate(bool p_enable) const
{
  hal::bit_modify(pin_map->matrix[m_port][m_pin]).insert<pin_slew>(p_enable);
  return *this;
}

const pin& pin::i2c_high_current(bool p_enable) const
{
  hal::bit_modify(pin_map->matrix[m_port][m_pin])
    .insert<pin_i2c_high_current>(p_enable);
  return *this;
}

const pin& pin::open_drain(bool p_enable) const
{
  hal::bit_modify(pin_map->matrix[m_port][m_pin])
    .insert<pin_open_drain>(p_enable);
  return *this;
}

const pin& pin::dac(bool p_enable) const
{
  hal::bit_modify(pin_map->matrix[m_port][m_pin])
    .insert<pin_dac_enable>(p_enable);
  return *this;
}
}  // namespace hal::lpc40
