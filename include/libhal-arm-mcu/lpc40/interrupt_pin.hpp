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

#include <libhal/interrupt_pin.hpp>
#include <libhal/units.hpp>

namespace hal::lpc40 {
/**
 * @brief Interrupt pin implementation for the lpc40xx
 *
 */
class interrupt_pin final : public hal::interrupt_pin
{
public:
  /**
   * @brief Construct a new interrupt pin object
   *
   * @param port - selects pin port to use
   * @param pin - selects pin within the port to use
   * @param p_settings - initial pin settings
   */
  interrupt_pin(u8 port,  // NOLINT
                u8 pin,
                settings const& p_settings = {});
  interrupt_pin(interrupt_pin const& p_other) = delete;
  interrupt_pin& operator=(interrupt_pin const& p_other) = delete;
  interrupt_pin(interrupt_pin&& p_other) noexcept = delete;
  interrupt_pin& operator=(interrupt_pin&& p_other) noexcept = delete;
  ~interrupt_pin() override;

private:
  void driver_configure(settings const& p_settings) override;
  void driver_on_trigger(hal::callback<handler> p_callback) override;

  u8 m_port;
  u8 m_pin;
};
}  // namespace hal::lpc40
