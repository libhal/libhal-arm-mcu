// Copyright 2026 Khalil Estell and the libhal contributors
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

import :pin;

namespace hal::stm32f1 {

/**
 * @brief Maximum output slew-rate for a push-pull/open-drain output pin
 *
 * Values match the MODE[1:0] bits for output pins (see "Table 21. Output
 * MODE bits" in RM0008).
 */
export enum class output_speed : hal::u8 {
  max_10_mhz = 0b01,
  max_2_mhz = 0b10,
  max_50_mhz = 0b11,
};

/**
 * @brief STM32F1 input pin factory
 */
export struct input_pin
{
  /**
   * @brief Acquire an input pin
   *
   * @param p_allocator - allocator used to allocate the memory for the object
   * @param p_pin - port & pin selection
   * @param p_settings - settings to apply to the input pin
   * @return hal::ptr<hal::input_pin> - type erased input pin driver
   */
  static hal::ptr<hal::input_pin> create(
    hal::allocator p_allocator,
    pin p_pin,
    hal::pin_settings const& p_settings = {});
};

/**
 * @brief STM32F1 output pin factory
 */
export struct output_pin
{
  /**
   * @brief Acquire an output pin
   *
   * @param p_allocator - allocator used to allocate the memory for the object
   * @param p_pin - port & pin selection
   * @param p_settings - settings to apply to the output pin
   * @param p_speed - maximum output slew-rate for the pin
   * @return hal::ptr<hal::output_pin> - type erased output pin driver
   */
  static hal::ptr<hal::output_pin> create(
    hal::allocator p_allocator,
    pin p_pin,
    hal::pin_settings const& p_settings = {},
    output_speed p_speed = output_speed::max_50_mhz);
};
}  // namespace hal::stm32f1
