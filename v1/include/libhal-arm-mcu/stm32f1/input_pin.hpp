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

#include <libhal/input_pin.hpp>
#include <libhal/units.hpp>

namespace hal::stm32f1 {
/**
 * @brief Input pin implementation for the stm32f10x
 * @deprecated Use the `hal::gpio<peripheral>` class instead. This will be
 * removed in the next later stage.
 *
 * Input pin only seems to support floating pins, does not support pull up or
 * pull down
 *
 */
class input_pin final : public hal::input_pin
{
public:
  /**
   * @brief Get the input pin object
   *
   * @param p_port - selects pin port to use
   * @param p_pin - selects which pin within the port to use
   * @throws hal::argument_out_of_domain - if the port and pin are not valid
   */
  input_pin(u8 p_port,  // NOLINT
            u8 p_pin);  // NOLINT

private:
  void driver_configure(settings const& p_settings) override;
  bool driver_level() override;

  u8 m_port{};
  u8 m_pin{};
};
}  // namespace hal::stm32f1
