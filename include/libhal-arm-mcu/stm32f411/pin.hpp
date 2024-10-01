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

#pragma once

#include <cstdint>

#include <libhal/units.hpp>

#include "constants.hpp"

namespace hal::stm32f411 {
/**
 * @brief stm32f4 pin multiplexing and control driver used drivers and apps
 * seeking to tune the pins.
 *
 */
class pin
{
public:
  enum class pin_function
  {
    input,
    output,
    analog,
    alternate0,
    alternate1,
    alternate2,
    alternate3,
    alternate4,
    alternate5,
    alternate6,
    alternate7,
    alternate8,
    alternate9,
    alternate10,
    alternate11,
    alternate12,
    alternate13,
    alternate14,
    alternate15
  };

  /**
   * @brief Construct a new pin mux and configuration driver
   *
   * See UM10562 page 99 for more details on which pins can be what function.
   *
   * @param p_port - selects pin port to use
   * @param p_pin - selects pin within the port to use
   */
  pin(peripheral p_port, std::uint8_t p_pin) noexcept;

  /**
   * @brief Change the function of the pin (mux the pins function)
   *
   * @param p_function - the pin function (I,O, alternatex)
   * @return pin& - reference to this pin for chaining
   */
  pin const& function(pin_function p_function) const noexcept;

  /**
   * @brief Set the internal resistor connection for this pin
   *
   * @param p_resistor - resistor type
   * @return pin& - reference to this pin for chaining
   */
  pin const& resistor(hal::pin_resistor p_resistor) const noexcept;

  /**
   * @brief Disable or enable hysteresis mode for this pin
   *
   * @param p_enable - enable this mode, set to false to disable this mode
   * @return pin& - reference to this pin for chaining
   */
  pin const& open_drain(bool p_enable = true) const noexcept;

  enum class mco_source : std::uint8_t
  {
    system_clock = 0b00U,
    pll_i2s = 0b01U,
    high_speed_external = 0b10U,
    pll = 0b11U,
  };

  /**
   * @brief Output a clock divided by 2 on the PA8 pin
   *
   * @param p_source - source clock to channel to the PA8 pin
   */
  void activate_mco_pc9(mco_source p_source);

private:
  peripheral m_port{};
  std::uint8_t m_pin{};
};
}  // namespace hal::stm32f411
