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

export module hal.arm_mcu.stm32f1:adc;

import hal;
import hal.util;

import :constants;

namespace hal::stm32f1 {

/**
 * @brief Analog input pins available to `adc1` and `adc2`
 */
export enum class adc_pins : hal::u8 {
  pa0 = 0,
  pa1 = 1,
  pa2 = 2,
  pa3 = 3,
  pa4 = 4,
  pa5 = 5,
  pa6 = 6,
  pa7 = 7,
  pb0 = 8,
  pb1 = 9,
  pc0 = 10,
  pc1 = 11,
  pc2 = 12,
  pc3 = 13,
  pc4 = 14,
  pc5 = 15,
};

/**
 * @brief STM32F1 ADC peripheral manager
 *
 * Owns one of the STM32F1's ADC hardware blocks: its register bank and the
 * mutual exclusion needed to serialize conversions requested by channels
 * acquired from it, since only one conversion can run at a time. Only
 * `adc1` and `adc2` are supported.
 */
export class adc
  : public hal::pimpl<adc>
  , public mem::enable_strong_from_this<adc>
{
public:
  struct impl;

  /**
   * @brief Acquire an ADC peripheral manager
   *
   * Powers on the ADC peripheral and runs its self-calibration routine.
   *
   * @param p_allocator - allocator used to allocate the memory for the
   * object
   * @param p_id - which ADC peripheral to manage. Must be `adc1` or `adc2`.
   * @return hal::ptr<adc> - ADC peripheral manager
   * @throws hal::operation_not_supported - if `p_id` is not `adc1` or
   * `adc2`, or if the ADC peripheral's clock rate exceeds its maximum rated
   * frequency of 14 MHz.
   * @throws hal::device_or_resource_busy - if the peripheral is already
   * powered on and managed elsewhere.
   */
  [[nodiscard]] static hal::ptr<adc> create(hal::allocator p_allocator,
                                            peripheral p_id);

  /**
   * @brief Acquire an ADC channel resource
   *
   * Sets the given pin to analog input mode.
   *
   * @param p_pin - analog input pin to sample
   * @return hal::ptr<hal::adc16> - type erased 16-bit ADC channel driver
   */
  [[nodiscard]] hal::ptr<hal::adc16> acquire_channel(adc_pins p_pin);

  /**
   * @private
   * @brief Sample an analog input pin
   *
   * Serializes concurrent access from every channel resource acquired from
   * this manager, since the ADC's conversion hardware is shared and only
   * one conversion can run at a time.
   *
   * @param p_context - async context for coroutine suspension and
   * resumption
   * @param p_pin - analog input pin to sample
   * @return async::future<hal::u16> - the sampled value upscaled to 16-bits
   */
  [[nodiscard]] async::future<hal::u16> read(async::context& p_context,
                                             adc_pins p_pin);

  /// @private
  adc(private_key, hal::allocator p_allocator, peripheral p_id);

  adc(adc const&) = delete;
  adc& operator=(adc const&) = delete;
  adc(adc&&) = delete;
  adc& operator=(adc&&) = delete;

  ~adc();
};
}  // namespace hal::stm32f1
