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

module;

#include <atomic>

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
 * Owns one of the STM32F1's ADC hardware blocks and keeps it powered for as
 * long as the manager lives. Its channels are handed out by value from
 * `acquire_channel()`. Every channel shares the one converter, which runs one
 * conversion at a time. Only `adc1` and `adc2` are supported.
 *
 * Every read holds the manager's lock for its whole conversion, so channels
 * read from different contexts (RTOS threads, or a thread and an interrupt)
 * take turns. If every channel is read from one context, pass a
 * `hal::no_op_lock`. A read that interrupts another read of the same ADC
 * then throws `hal::device_or_resource_busy` instead of switching the channel
 * under it.
 *
 * ```cpp
 * static hal::no_op_lock single_context;
 * static hal::stm32f1::adc adc1(
 *   hal::stm32f1::peripheral::adc1, clock, single_context);
 * static auto battery = adc1.acquire_channel(hal::stm32f1::adc_pins::pa0);
 * static auto throttle = adc1.acquire_channel(hal::stm32f1::adc_pins::pb1);
 * ```
 */
export class adc
{
public:
  class channel;

  /**
   * @brief Claim an ADC peripheral, power it on, and calibrate it
   *
   * The peripheral is powered off and released when the manager is
   * destroyed. The manager must outlive every channel acquired from it.
   *
   * @param p_id - which ADC peripheral to manage. Must be `adc1` or `adc2`.
   * @param p_clock - times the converter's power-up before calibration. It
   * is only used during construction, so it needn't outlive the manager.
   * @param p_lock - lock held for each conversion, which takes a few
   * microseconds. It must outlive the manager. If an interrupt reads this
   * ADC, the lock must be safe to take from that interrupt, which most RTOS
   * mutexes aren't. Use `hal::no_op_lock` if every channel is read from one
   * context.
   * @throws hal::operation_not_supported - if `p_id` is not `adc1` or
   * `adc2`, or if the ADC clock is faster than its maximum rated frequency of
   * 14 MHz.
   * @throws hal::device_or_resource_busy - if another driver has claimed the
   * peripheral (see `claim()`), for example a second `adc` for it.
   */
  adc(peripheral p_id,
      steady_clock& p_clock,
      hal::ref<basic_lock> p_lock [[clang::lifetimebound]]);

  adc(adc const&) = delete;
  adc& operator=(adc const&) = delete;
  adc(adc&&) = delete;
  adc& operator=(adc&&) = delete;

  /// Power off the peripheral
  ~adc();

  /**
   * @brief Acquire a channel of this ADC
   *
   * Claims the pin and sets it to analog input mode.
   *
   * @param p_pin - analog input pin to sample
   * @return channel - the channel, which can't be copied or moved. Store it
   * where it is returned, or use `hal::make_owner_from()` to put it on the
   * heap.
   * @throws hal::device_or_resource_busy - if another driver has claimed the
   * pin
   */
  [[nodiscard]] channel acquire_channel(adc_pins p_pin)
    [[clang::lifetimebound]];

private:
  /// @throws hal::device_or_resource_busy - if another read of this ADC is
  /// in progress, which only a lock that doesn't block lets through
  [[nodiscard]] u16 read(adc_pins p_pin);

  hal::ref<basic_lock> m_lock;
  peripheral m_id;
  /// Set while a read is converting
  std::atomic_flag m_converting;
};

/**
 * @brief An analog input pin sampled by an STM32F1 ADC
 *
 * Each read starts a conversion and waits for it to finish, without
 * interrupts. If the manager's lock doesn't block, such as a
 * `hal::no_op_lock`, a read that interrupts another read of the same ADC
 * throws `hal::device_or_resource_busy`.
 *
 * Only `adc::acquire_channel()` creates one.
 */
class adc::channel final : public hal::adc16
{
public:
  channel(channel const&) = delete;
  channel& operator=(channel const&) = delete;
  channel(channel&&) = delete;
  channel& operator=(channel&&) = delete;

  /// Return the pin to its reset state and release it
  ~channel();

private:
  friend class adc;

  channel(adc& p_manager, adc_pins p_pin);

  u16 driver_read() override;

  hal::ref<adc> m_manager;
  adc_pins m_pin;
};
}  // namespace hal::stm32f1
