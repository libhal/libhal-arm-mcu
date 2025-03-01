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

#include <libhal/input_pin.hpp>
#include <libhal/output_pin.hpp>

#include "constants.hpp"

namespace hal::stm32f1 {

class gpio_manager
{
public:
  template<peripheral select>
  friend class gpio;

  gpio_manager(gpio_manager&) = delete;
  gpio_manager& operator=(gpio_manager&) = delete;
  gpio_manager(gpio_manager&&) noexcept = default;
  gpio_manager& operator=(gpio_manager&&) noexcept = default;
  ~gpio_manager();

  class input;
  class output;

  input acquire_input_pin(u8 p_pin, input_pin::settings const& p_settings = {});
  output acquire_output_pin(u8 p_pin,
                            output_pin::settings const& p_settings = {});

private:
  gpio_manager(peripheral p_select);
  peripheral m_port;
};

template<peripheral select>
class gpio final : public gpio_manager
{
public:
  static_assert(select == peripheral::gpio_a or /* line break */
                  select == peripheral::gpio_b or
                  select == peripheral::gpio_c or
                  select == peripheral::gpio_d or
                  select == peripheral::gpio_e or
                  select == peripheral::gpio_f or /* line break */
                  select == peripheral::gpio_g,
                "Only gpio_(a to g) is allowed for this class");
  gpio()
    : gpio_manager(select)
  {
  }
  ~gpio() = default;
};

class gpio_manager::input : public hal::input_pin
{
public:
private:
  friend class gpio_manager;

  input(peripheral p_port, u8 p_pin, settings const& p_settings);

  void driver_configure(settings const& p_settings) override;
  bool driver_level() override;

  u8 m_port;
  u8 m_pin;
};

class gpio_manager::output : public hal::output_pin
{
private:
  friend class gpio_manager;

  output(peripheral p_port, u8 p_pin, settings const& p_settings);

  void driver_configure(settings const& p_settings) override;
  void driver_level(bool p_high) override;
  bool driver_level() override;

  hal::u8 m_port;
  hal::u8 m_pin;
};
}  // namespace hal::stm32f1
