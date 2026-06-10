// Copyright 2024 - 2025 Khalil Estell and the libhal contributors
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

#include <libhal-arm-mcu/tca9548a.hpp>
#include <libhal-util/i2c.hpp>

namespace hal::expander {
tca9548a::tca9548a(hal::i2c& p_i2c,
                   bool p_addr_bit_0,
                   bool p_addr_bit_1,
                   bool p_addr_bit_2)
  : m_i2c(&p_i2c)
{
  // calculate address byte
  // 01110[A2][A1][A0]
  auto inputs = 0b0 | p_addr_bit_2;
  inputs = (inputs << 1) | p_addr_bit_1;
  inputs = (inputs << 1) | p_addr_bit_0;

  m_address = 0b01110000 | inputs;
}

hal::byte tca9548a::get_control_register_byte()
{
  std::array<hal::byte, 1> response;
  hal::read(*m_i2c, m_address, response);
  return response[0];
}

void tca9548a::enable_port(std::uint8_t p_port_number)
{
  hal::byte byte_to_send = 0x00;
  if (p_port_number < 8) {
    byte_to_send = byte_to_send | (1 << p_port_number);
  }

  hal::write(*m_i2c, m_address, std::array<hal::byte, 1>{ byte_to_send });
}

void tca9548a::enable_multiple_ports(std::array<bool, 8> p_ports)
{
  hal::byte byte_to_send = 0;
  for (std::int8_t i = 7; i >= 0; i--) {
    byte_to_send = (byte_to_send << 1);
    byte_to_send = byte_to_send | p_ports[i];
  }
  hal::write(*m_i2c, m_address, std::array<hal::byte, 1>{ byte_to_send });
}

void tca9548a::disable_port(std::uint8_t p_port_number)
{
  hal::byte byte_to_send = get_control_register_byte();
  if (p_port_number < 8) {
    hal::byte bit_select = 1 << p_port_number;
    hal::byte bit_mask = ~bit_select;
    byte_to_send = byte_to_send & bit_mask;
  }
  hal::write(*m_i2c, m_address, std::array<hal::byte, 1>{ byte_to_send });
}

std::array<bool, 8> tca9548a::get_ports_status()
{
  auto control_register = get_control_register_byte();
  std::array<bool, 8> ports_statuses;
  for (std::uint8_t i = 0; i < 8; i++) {
    hal::byte bit_mask = 1 << i;
    hal::byte selected_bit = control_register & bit_mask;
    ports_statuses[i] = static_cast<bool>(selected_bit >> i);
  }
  return ports_statuses;
}

}  // namespace hal::expander
