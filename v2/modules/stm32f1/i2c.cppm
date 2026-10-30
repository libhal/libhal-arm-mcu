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
 * @brief STM32F1 I2C controller that polls every byte
 *
 * Owns one of the STM32F1's I2C hardware blocks and its SCL and SDA pins, and
 * keeps it powered for as long as it lives. It is the bus controller (master)
 * only. Only `i2c1` and `i2c2` are supported.
 *
 * A transaction spins on the peripheral's status flags until it finishes, so
 * it needs no interrupts, no DMA channels and no waiter, but keeps the CPU
 * busy for the whole transaction: about 25 microseconds per byte at 400 kHz.
 *
 * The controller doesn't lock the bus. Drivers in one context can share it
 * as is. A transaction that starts while another is in progress, from
 * another context or an interrupt, throws `hal::device_or_resource_busy`
 * instead of corrupting it. To use the bus from several threads or contexts,
 * wrap it in a `hal::thread_safe_i2c`, which makes callers wait instead.
 *
 * A transaction throws `hal::timed_out` if the bus makes no progress for at
 * least 25 ms, for example while a device holds SCL low. The bound is a
 * number of polls of the peripheral, so it needs no clock. In practice a
 * poll takes longer than the bound assumes, so a stall takes longer to
 * detect.
 *
 * ```cpp
 * static hal::stm32f1::polled_i2c i2c1(hal::stm32f1::peripheral::i2c1);
 * auto const found = hal::probe(i2c1, 0x68);
 * ```
 */
export class polled_i2c final : public hal::i2c
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
   * @param p_settings - bus settings. The clock rate must be at or below
   * the limit of every device on the bus, which `hal::i2c_bus_rate()`
   * computes at compile time. The default, 100 kHz, is safe for any device.
   * @throws hal::operation_not_supported - if `p_id` is not `i2c1` or `i2c2`,
   * or if `p_settings` can't be achieved (see `configure()`).
   * @throws hal::device_or_resource_busy - if another driver has claimed the
   * peripheral or one of its pins.
   */
  explicit polled_i2c(peripheral p_id,
                      hal::i2c_settings const& p_settings = {});

  polled_i2c(polled_i2c const&) = delete;
  polled_i2c& operator=(polled_i2c const&) = delete;
  polled_i2c(polled_i2c&&) = delete;
  polled_i2c& operator=(polled_i2c&&) = delete;

  /// Power off the peripheral and release its pins
  ~polled_i2c();

  /**
   * @brief Change the bus's clock rate
   *
   * @warning Every device on the bus sees the new rate. A device that sees a
   * rate faster than it supports can misread traffic meant for another
   * device and corrupt the bus. Keep the rate at or below the slowest
   * device's limit, and call this only from the application, which knows
   * every device on the bus. Device drivers take a `hal::i2c`, which has no
   * way to change the rate.
   *
   * The bus never runs faster than requested. A rate above 400 kHz runs at
   * 400 kHz.
   *
   * @param p_settings - the new bus settings
   * @throws hal::operation_not_supported - if the clock rate is zero or the
   * bus can't run as slowly as requested.
   * @throws hal::device_or_resource_busy - if a transaction on this bus is in
   * progress.
   */
  void configure(hal::i2c_settings const& p_settings);

private:
  /// @throws hal::device_or_resource_busy - if another transaction on this
  /// bus is in progress
  void driver_transaction(hal::byte p_address,
                          mem::scatter_span<hal::byte const> p_data_out,
                          mem::scatter_span<hal::byte> p_data_in) override;

  peripheral m_id;
  hal::stm32_generic::i2c m_driver;
  /// Set while a transaction or configure() is in progress
  std::atomic_flag m_busy;
};

}  // namespace hal::stm32f1
