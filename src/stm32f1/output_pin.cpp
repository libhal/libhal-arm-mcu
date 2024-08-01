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

#include <libhal-stm32f1/output_pin.hpp>

#include <cstdint>

#include <libhal-util/bit.hpp>
#include <libhal/error.hpp>

#include "pin.hpp"

namespace hal::stm32f1 {
output_pin::output_pin(std::uint8_t p_port,  // NOLINT
                       std::uint8_t p_pin,   // NOLINT
                       output_pin::settings p_settings)
  : m_port(p_port)
  , m_pin(p_pin)
{
  // Ignore result as this function is infallible
  output_pin::driver_configure(p_settings);
}

void output_pin::driver_configure(settings const& p_settings)
{
  if (!p_settings.open_drain) {
    configure_pin({ .port = m_port, .pin = m_pin }, push_pull_gpio_output);
  } else if (p_settings.open_drain) {
    configure_pin({ .port = m_port, .pin = m_pin }, open_drain_gpio_output);
  }
}

void output_pin::driver_level(bool p_high)
{
  if (p_high) {
    // The first 16 bits of the register set the output state
    gpio(m_port).bsrr = 1 << m_pin;
  } else {
    // The last 16 bits of the register reset the output state
    gpio(m_port).bsrr = 1 << (16 + m_pin);
  }
}

bool output_pin::driver_level()
{
  auto pin_value = bit_extract(bit_mask::from(m_pin), gpio(m_port).idr);

  return static_cast<bool>(pin_value);
}
}  // namespace hal::stm32f1
