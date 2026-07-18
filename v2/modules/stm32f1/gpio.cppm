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

module;

#include <memory_resource>

export module hal.arm_mcu.stm32f1:gpio;

import hal;
import hal.util;

import :constants;
import :power;
import :pin;

namespace hal::stm32f1 {
/**
 * @brief Implementation of the GPIO port manager class
 *
 * This class has a private constructor and can only be used via its derived
 * class `gpio<peripheral>` class.
 *
 */
export class gpio_manager
{
public:
  template<peripheral select>
  friend class gpio;

  gpio_manager(gpio_manager&) = delete;
  gpio_manager& operator=(gpio_manager&) = delete;
  gpio_manager(gpio_manager&&) noexcept = default;
  gpio_manager& operator=(gpio_manager&&) noexcept = default;
  /**
   * @brief Destroy the gpio port manager object
   *
   * This actually does nothing as this driver cannot disable the GPIO port
   * peripherals if other pins are used within the application.
   */
  ~gpio_manager() = default;

  class input;
  class output;

  input acquire_input_pin(u8 p_pin,
                          hal::input_pin::settings const& p_settings = {});
  output acquire_output_pin(u8 p_pin,
                            hal::output_pin::settings const& p_settings = {});

private:
  gpio_manager(peripheral p_select);
  peripheral m_port;
};

/**
 * @brief Gpio manager for the gpio ports A through to G.
 *
 * Use the acquire APIs in order to get input and output pins. If a pin is
 * already in use, the `hal::device_or_resource_busy` will be thrown.
 *
 * @tparam select - gpio peripheral port selection. Only peripheral::gpio_a to
 * peripheral::gpio_g.
 */
export template<peripheral select>
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
                "Only peripheral gpio_(a to g) is allowed for this class");
  gpio()
    : gpio_manager(select)
  {
  }
  ~gpio() = default;
};

class gpio_manager::input final : public hal::input_pin
{
public:
  template<peripheral port>
  input(gpio<port> const&, u8 p_pin, settings const& p_settings = {})
    : input(port, p_pin, p_settings)
  {
  }

private:
  friend class gpio_manager;

  friend mem::strong_ptr<hal::input_pin> acquire_input_pin(
    std::pmr::polymorphic_allocator<> p_allocator,
    mem::strong_ptr<gpio_manager> const& p_manager,
    u8 p_pin,
    hal::input_pin::settings const& p_settings);

  input(peripheral p_port, u8 p_pin, settings const& p_settings);

  async::future<void> driver_configure(async::context& p_context,
                                       settings const& p_settings) override;
  async::future<bool> driver_level(async::context& p_context) override;

  pin m_pin;
};

class gpio_manager::output final : public hal::output_pin
{
public:
  template<peripheral port>
  output(gpio<port> const&, u8 p_pin, settings const& p_settings = {})
    : output(port, p_pin, p_settings)
  {
  }

private:
  friend class gpio_manager;

  friend mem::strong_ptr<hal::output_pin> acquire_output_pin(
    std::pmr::polymorphic_allocator<> p_allocator,
    mem::strong_ptr<gpio_manager> const& p_manager,
    u8 p_pin,
    hal::output_pin::settings const& p_settings);

  output(peripheral p_port, u8 p_pin, settings const& p_settings);

  async::future<void> driver_configure(async::context& p_context,
                                       settings const& p_settings) override;
  async::future<void> driver_level(async::context& p_context,
                                   bool p_high) override;
  async::future<bool> driver_level(async::context& p_context) override;

  pin m_pin;
};
}  // namespace hal::stm32f1
