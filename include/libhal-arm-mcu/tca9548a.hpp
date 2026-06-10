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

#pragma once

#include <cstdint>

#include <libhal/i2c.hpp>

namespace hal::expander {
/**
 * @brief tca9548a driver: 8 channel i2c multiplexer using eight bidirectional
 * translating switches that can be controlled through the i2c bus
 *
 */
class tca9548a
{
public:
  /**
   * @brief Construct a new tca9548a driver object
   *
   * @param p_i2c - an i2c bus driver to communicate with
   * @param p_addr_bit_0 - bit 0 of address to be used for tca9548a
   * @param p_addr_bit_1 - bit 1 of address to be used for tca9548a
   * @param p_addr_bit_2 - bit 2 of address to be used for tca9548a
   */
  tca9548a(hal::i2c& p_i2c,
           bool p_addr_bit_0 = false,
           bool p_addr_bit_1 = false,
           bool p_addr_bit_2 = false);

  /**
   * @brief Enable a single port to read and write to over i2c
   *
   * @param p_port_number - number of port to enable, 0 - 7
   */
  void enable_port(std::uint8_t p_port_number);

  /**
   * @brief Enable a multiple ports to read and write to over i2c
   *
   * @param p_ports - array of bools representing which ports to turn on or off
   */
  void enable_multiple_ports(std::array<bool, 8> p_ports);

  /**
   * @brief Disable a single port while leaving other ports in their current
   * state
   *
   * @param p_port_number - number of port to disable, 0 - 7
   */
  void disable_port(std::uint8_t p_port_number);

  /**
   * @brief Get the status of each port
   *
   * @return std::array<bool, 8> - array of bools representing which ports are
   * on or off
   */
  std::array<bool, 8> get_ports_status();

private:
  hal::byte get_control_register_byte();

  hal::i2c* m_i2c;
  hal::byte m_address;
};
}  // namespace hal::expander
