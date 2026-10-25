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

export module hal.arm_mcu.stm32f1:i2c;

import hal;
import hal.util;
import hal.arm_mcu.stm32_generic;

import :constants;

namespace hal::stm32f1 {

/**
 * @brief STM32F1 I2C controller
 *
 * Owns one of the STM32F1's I2C hardware blocks and its SCL and SDA pins, and
 * keeps it powered for as long as it lives. It is the bus controller (master)
 * only. Only `i2c1` and `i2c2` are supported.
 *
 * Every transaction holds the lock passed to the constructor, so drivers in
 * different contexts (RTOS threads, or a thread and an interrupt) can share
 * the bus. If every user of the bus is in one context, pass a
 * `hal::no_op_lock`. A transaction that interrupts another one on the same
 * bus then throws `hal::device_or_resource_busy` instead of corrupting it.
 *
 * A transaction sleeps the calling context on its waiter while the bus
 * works, so a waiter provider must be installed (see `hal::current_waiter()`)
 * and transactions can't run inside an interrupt. The bytes move by DMA when
 * the transaction can claim the peripheral's DMA channels: DMA1 channels 6
 * (transmit) and 7 (receive) for `i2c1`, 4 and 5 for `i2c2`. They are
 * claimed for each transaction and released after it, so other drivers on
 * those channels take turns with this one. If another driver holds a channel,
 * such as a USART receiving on it, the bytes in that direction move without
 * DMA, waking the context once per byte.
 *
 * A transaction throws `hal::timed_out` if the bus makes no progress for
 * 25 ms, for example while a device holds SCL low.
 *
 * ```cpp
 * static hal::no_op_lock single_context;
 * static hal::stm32f1::i2c i2c1(
 *   hal::stm32f1::peripheral::i2c1, clock, single_context);
 * auto const found = hal::probe(i2c1, 0x68);
 * ```
 */
export class i2c final : public hal::i2c
{
public:
  /**
   * @brief Claim an I2C peripheral and its pins, power it on, and configure
   * it
   *
   * Claims and configures SCL and SDA as open-drain outputs: PB6 and PB7 for
   * `i2c1`, PB10 and PB11 for `i2c2`. The bus needs pull-up resistors on both
   * lines.
   *
   * The peripheral is powered off and released, and its pins are reset and
   * released, when the controller is destroyed.
   *
   * @param p_id - which I2C peripheral to use. Must be `i2c1` or `i2c2`.
   * @param p_clock - times the short waits on the bus that raise no
   * interrupt. It must outlive the controller.
   * @param p_lock - lock held for each transaction. It must outlive the
   * controller. If an interrupt uses this bus, the lock must be safe to take
   * from that interrupt, which most RTOS mutexes aren't. Use
   * `hal::no_op_lock` if every user of the bus is in one context.
   * @param p_settings - initial bus settings
   * @throws hal::operation_not_supported - if `p_id` is not `i2c1` or `i2c2`,
   * or if `p_settings` can't be achieved (see `configure()`).
   * @throws hal::device_or_resource_busy - if another driver has claimed the
   * peripheral or one of its pins.
   */
  i2c(peripheral p_id,
      hal::ref<hal::steady_clock> p_clock [[clang::lifetimebound]],
      hal::ref<basic_lock> p_lock [[clang::lifetimebound]],
      hal::i2c::settings const& p_settings = {});

  i2c(i2c const&) = delete;
  i2c& operator=(i2c const&) = delete;
  i2c(i2c&&) = delete;
  i2c& operator=(i2c&&) = delete;

  /// Power off the peripheral and release its pins
  ~i2c();

private:
  /// @throws hal::operation_not_supported - if the clock rate is zero or the
  /// bus can't run as slowly as requested. A rate above 400 kHz runs at
  /// 400 kHz.
  void driver_configure(hal::i2c::settings const& p_settings) override;
  /// @throws hal::device_or_resource_busy - if another transaction on this
  /// bus is in progress, which only a lock that doesn't block lets through
  void driver_transaction(hal::byte p_address,
                          mem::scatter_span<hal::byte const> p_data_out,
                          mem::scatter_span<hal::byte> p_data_in) override;

  hal::ref<hal::steady_clock> m_clock;
  hal::ref<basic_lock> m_lock;
  peripheral m_id;
  hal::stm32_generic::i2c m_driver;
  /// Set while a transaction is in progress
  std::atomic_flag m_busy;
};
}  // namespace hal::stm32f1
