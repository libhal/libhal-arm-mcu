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
 * Manages a single gpio port (A through G). Use the `create()` factory
 * functions to construct one, then use the acquire APIs to obtain input and
 * output pins.
 *
 */
export class gpio_manager : public hal::pimpl<gpio_manager>
{
public:
  /// Forward declaration only. Defined in gpio.cpp.
  struct impl;

  gpio_manager(gpio_manager&) = delete;
  gpio_manager& operator=(gpio_manager&) = delete;
  gpio_manager(gpio_manager&&) noexcept = delete;
  gpio_manager& operator=(gpio_manager&&) noexcept = delete;
  /**
   * @brief Destroy the gpio port manager object
   *
   * This actually does nothing as this driver cannot disable the GPIO port
   * peripherals if other pins are used within the application.
   */
  ~gpio_manager() = default;

  /**
   * @brief Create a gpio port manager with compile time peripheral
   * validation
   *
   * @tparam select - gpio peripheral port selection. Only peripheral::gpio_a
   * to peripheral::gpio_g.
   * @param p_allocator - allocator used to allocate the memory for the
   * object
   * @return hal::ptr<gpio_manager> - manager for the selected gpio port.
   */
  template<peripheral select>
  static hal::ptr<gpio_manager> create(hal::allocator p_allocator)
  {
    static_assert(
      select == peripheral::gpio_a or /* line break */
        select == peripheral::gpio_b or select == peripheral::gpio_c or
        select == peripheral::gpio_d or select == peripheral::gpio_e or
        select == peripheral::gpio_f or /* line break */
        select == peripheral::gpio_g,
      "Only peripheral gpio_(a to g) is allowed for this class");
    return create(p_allocator, select);
  }

  hal::ptr<hal::input_pin> acquire_input_pin(
    hal::allocator p_allocator,
    u8 p_pin,
    hal::input_pin::settings const& p_settings = {});
  hal::ptr<hal::output_pin> acquire_output_pin(
    hal::allocator p_allocator,
    u8 p_pin,
    hal::output_pin::settings const& p_settings = {});

  gpio_manager(private_key, hal::allocator p_allocator, peripheral p_select);

private:
  /**
   * @brief Create a gpio port manager
   *
   * @param p_allocator - allocator used to allocate the memory for the
   * object
   * @param p_select - gpio peripheral port selection. Only peripheral::gpio_a
   * to peripheral::gpio_g.
   * @return hal::ptr<gpio_manager> - manager for the selected gpio port.
   */
  static hal::ptr<gpio_manager> create(hal::allocator p_allocator,
                                       peripheral p_select);
  peripheral m_peripheral;
};
}  // namespace hal::stm32f1
