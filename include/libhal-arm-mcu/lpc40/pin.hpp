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

#include <libhal/units.hpp>

namespace hal::lpc40 {
/**
 * @brief lpc40xx pin multiplexing and control driver used drivers and apps
 * seeking to tune the pins.
 *
 */
class pin
{
public:
  /**
   * @brief Construct a new pin mux and configuration driver
   *
   * See UM10562 page 99 for more details on which pins can be what function.
   *
   * @param p_port - selects pin port to use
   * @param p_pin - selects pin within the port to use
   */
  constexpr pin(std::uint8_t p_port, std::uint8_t p_pin)
    : m_port(p_port)
    , m_pin(p_pin)
  {
  }

  /// Default constructor
  constexpr pin() = default;

  /**
   * @brief Change the function of the pin (mux the pins function)
   *
   * @param p_function_code - the pin function code
   * @return pin& - reference to this pin for chaining
   */
  pin const& function(uint8_t p_function_code) const;

  /**
   * @brief Set the internal resistor connection for this pin
   *
   * @param p_resistor - resistor type
   * @return pin& - reference to this pin for chaining
   */
  pin const& resistor(hal::pin_resistor p_resistor) const;

  /**
   * @brief Disable or enable hysteresis mode for this pin
   *
   * @param p_enable - enable this mode, set to false to disable this mode
   * @return pin& - reference to this pin for chaining
   */
  pin const& hysteresis(bool p_enable) const;

  /**
   * @brief invert the logic for this pin in input mode
   *
   * @param p_enable - enable this mode, set to false to disable this mode
   * @return pin& - reference to this pin for chaining
   */
  pin const& input_invert(bool p_enable) const;

  /**
   * @brief enable analog mode for this pin (required for dac and adc drivers)
   *
   * @param p_enable - enable this mode, set to false to disable this mode
   * @return pin& - reference to this pin for chaining
   */
  pin const& analog(bool p_enable) const;

  /**
   * @brief enable digital filtering (filter out noise on input lines)
   *
   * @param p_enable - enable this mode, set to false to disable this mode
   * @return pin& - reference to this pin for chaining
   */
  pin const& digital_filter(bool p_enable) const;

  /**
   * @brief Enable high speed mode for i2c pins
   *
   * @param p_enable - enable this mode, set to false to disable this mode
   * @return pin& - reference to this pin for chaining
   */
  pin const& highspeed_i2c(bool p_enable = true) const;

  /**
   * @brief enable high slew rate for pin
   *
   * @param p_enable - enable this mode, set to false to disable this mode
   * @return pin& - reference to this pin for chaining
   */
  pin const& high_slew_rate(bool p_enable = true) const;

  /**
   * @brief enable high current drain for i2c lines
   *
   * @param p_enable - enable this mode, set to false to disable this mode
   * @return pin& - reference to this pin for chaining
   */
  pin const& i2c_high_current(bool p_enable = true) const;

  /**
   * @brief Make the pin open drain (required for the i2c driver)
   *
   * @param p_enable - enable this mode, set to false to disable this mode
   * @return pin& - reference to this pin for chaining
   */
  pin const& open_drain(bool p_enable = true) const;

  /**
   * @brief Enable dac mode (required for the dac driver)
   *
   * @param p_enable - enable this mode, set to false to disable this mode
   * @return pin& - reference to this pin for chaining
   */
  pin const& dac(bool p_enable = true) const;

private:
  std::uint8_t m_port{};
  std::uint8_t m_pin{};
};
}  // namespace hal::lpc40
