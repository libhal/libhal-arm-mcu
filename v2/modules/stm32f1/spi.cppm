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

export module hal.arm_mcu.stm32f1:spi;

import hal;
import hal.util;

import :constants;

namespace hal::stm32f1 {

/**
 * @brief STM32F1 SPI peripheral manager
 *
 * Owns one of the STM32F1's SPI hardware blocks: its register bank, its
 * clock/CIPO/COPI pin trio, and the mutual exclusion needed to serialize bus
 * access between channels acquired from it, since only one channel may hold
 * the bus (and assert its chip select) at a time. Only `spi1`, `spi2`, and
 * `spi3` are supported.
 */
export class spi
  : public hal::pimpl<spi>
  , public mem::enable_strong_from_this<spi>
{
public:
  struct impl;

  /**
   * @brief Acquire an SPI peripheral manager
   *
   * Configures the bus's clock/CIPO/COPI pins and powers on the peripheral.
   *
   * @param p_allocator - allocator used to allocate the memory for the
   * object
   * @param p_id - which SPI peripheral to manage. Must be `spi1`, `spi2`, or
   * `spi3`.
   * @return hal::ptr<spi> - SPI peripheral manager
   * @throws hal::operation_not_supported - if `p_id` is not `spi1`, `spi2`,
   * or `spi3`.
   * @throws hal::device_or_resource_busy - if the peripheral is already
   * powered on and managed elsewhere.
   */
  [[nodiscard]] static hal::ptr<spi> create(hal::allocator p_allocator,
                                            peripheral p_id);

  /**
   * @brief Acquire a channel resource representing one device on this bus
   *
   * @param p_chip_select - output pin used to select this channel's device.
   * Driven high (deselected) immediately and whenever this channel does not
   * hold the bus, and low (selected) while it does.
   * @param p_settings - spi settings to apply whenever this channel acquires
   * the bus
   * @return hal::ptr<hal::spi_channel> - type erased spi channel driver
   */
  [[nodiscard]] hal::ptr<hal::spi_channel> acquire_channel(
    hal::ptr<hal::output_pin> const& p_chip_select,
    hal::spi_channel::settings const& p_settings = {});

  /**
   * @private
   * @brief Acquire or release exclusive access over the bus on behalf of a
   * channel, driving its chip select pin to match.
   *
   * @param p_context - async context for coroutine suspension and resumption
   * @param p_chip_select - the calling channel's chip select pin
   * @param p_settings - the calling channel's settings, applied to the bus
   * when `p_select` is true
   * @param p_select - true to acquire the bus and assert chip select, false
   * to de-assert chip select and release the bus
   */
  [[nodiscard]] async::future<void> chip_select(
    async::context& p_context,
    hal::ptr<hal::output_pin> const& p_chip_select,
    hal::spi_channel::settings const& p_settings,
    bool p_select);

  /**
   * @private
   * @brief Perform a transfer on behalf of a channel that currently holds
   * the bus
   *
   * @param p_context - async context for coroutine suspension and resumption
   * @param p_data_out - outgoing data
   * @param p_data_in - incoming data
   * @param p_filler - output filler bytes if the outgoing data runs out
   * before the incoming data
   */
  [[nodiscard]] async::future<void> transfer(
    async::context& p_context,
    mem::scatter_span<hal::byte const> p_data_out,
    mem::scatter_span<hal::byte> p_data_in,
    hal::byte p_filler);

  /**
   * @private
   * @brief Compute the clock rate a channel with `p_settings` would achieve
   * without touching hardware state
   *
   * @param p_settings - the calling channel's settings
   */
  [[nodiscard]] hal::hertz achievable_clock_rate(
    hal::spi_channel::settings const& p_settings);

  /// @private
  spi(private_key, hal::allocator p_allocator, peripheral p_id);

  spi(spi const&) = delete;
  spi& operator=(spi const&) = delete;
  spi(spi&&) = delete;
  spi& operator=(spi&&) = delete;

  ~spi();
};
}  // namespace hal::stm32f1
