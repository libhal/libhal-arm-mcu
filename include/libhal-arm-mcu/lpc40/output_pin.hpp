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

#include <libhal/output_pin.hpp>
#include <libhal/units.hpp>

namespace hal::lpc40 {
/**
 * @brief Output pin implementation for the lpc40xx
 *
 */
class output_pin final : public hal::output_pin
{
public:
  /**
   * @brief Construct a new output pin object
   *
   * @param p_port - selects pin port to use
   * @param p_pin - selects which pin within the port to use
   * @param p_settings - initial pin settings
   */
  output_pin(u8 p_port, u8 p_pin, output_pin::settings const& p_settings = {});

  output_pin(output_pin const& p_other) = delete;
  output_pin& operator=(output_pin const& p_other) = delete;
  output_pin(output_pin&& p_other) noexcept = delete;
  output_pin& operator=(output_pin&& p_other) noexcept = delete;
  ~output_pin() override = default;

private:
  void driver_configure(settings const& p_settings) override;
  void driver_level(bool p_high) override;
  bool driver_level() override;

  u8 m_port{};
  u8 m_pin{};
};
}  // namespace hal::lpc40
