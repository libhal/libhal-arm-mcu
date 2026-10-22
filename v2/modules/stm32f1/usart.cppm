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

#include <span>

export module hal.arm_mcu.stm32f1:usart;

import hal;
import hal.util;
import hal.arm_mcu.stm32_generic;

import :constants;
import :pin;

namespace hal::stm32f1 {

/**
 * @brief STM32F1 USART peripheral manager
 *
 * Owns one of the STM32F1's USART hardware blocks and keeps it powered for as
 * long as the manager lives. Its serial port is handed out by value from
 * `acquire_serial()`. Only `usart1`, `usart2`, and `usart3` are supported.
 * `uart4` and `uart5` use a different pin/DMA mapping that is not yet
 * implemented by this driver.
 *
 * ```cpp
 * static hal::stm32f1::usart usart1(hal::stm32f1::peripheral::usart1);
 * static std::array<hal::byte, 128> receive_buffer{};
 *
 * // static storage
 * static auto console = usart1.acquire_serial(receive_buffer);
 *
 * // heap storage, owned as the interface
 * auto console = hal::make_owner_from<hal::serial>(allocator, [&] {
 *   return usart1.acquire_serial(receive_buffer);
 * });
 * ```
 */
export class usart
{
public:
  class serial;

  /**
   * @brief Claim a USART peripheral, power it on, and reset it
   *
   * The peripheral is powered off and released when the manager is
   * destroyed. The manager must outlive the serial port acquired from it.
   *
   * @param p_id - which USART peripheral to manage. Must be `usart1`,
   * `usart2`, or `usart3`.
   * @throws hal::operation_not_supported - if `p_id` is not `usart1`,
   * `usart2`, or `usart3`.
   * @throws hal::device_or_resource_busy - if another driver has claimed the
   * peripheral (see `claim()`), for example a second `usart` for it.
   */
  explicit usart(peripheral p_id);

  usart(usart const&) = delete;
  usart& operator=(usart const&) = delete;
  usart(usart&&) = delete;
  usart& operator=(usart&&) = delete;

  /// Power off the peripheral
  ~usart();

  /**
   * @brief Acquire the serial port of this USART peripheral
   *
   * Configures the TX/RX pins for this peripheral and sets up its receive
   * DMA channel to continuously write incoming bytes into `p_buffer` in a
   * circular fashion.
   *
   * @param p_buffer - buffer that the receive DMA channel writes into. Must
   * outlive the returned serial port.
   * @param p_settings - serial settings to apply
   * @return serial - the serial port, which can't be copied or moved. Store
   * it where it is returned, or use `hal::make_owner_from()` to put it on the
   * heap.
   * @throws hal::device_or_resource_busy - if a serial port acquired from this
   * manager still exists (destroy it before acquiring another), or if another
   * driver has claimed the TX or RX pin.
   * @throws hal::operation_not_supported - if `p_buffer` is larger than the
   * DMA controller can address (`max_dma_length`).
   */
  [[nodiscard]] serial acquire_serial(
    std::span<hal::byte> p_buffer [[clang::lifetimebound]],
    hal::serial::settings const& p_settings = {}) [[clang::lifetimebound]];

private:
  peripheral m_id;
  bool m_serial_acquired = false;
};

/**
 * @brief The serial port of an STM32F1 USART peripheral
 *
 * Writes block until every byte is sent. Received bytes are written into the
 * receive buffer by DMA without any CPU involvement; `receive_cursor()`
 * reports how far DMA has written.
 *
 * Only `usart::acquire_serial()` creates one.
 */
class usart::serial final : public hal::serial
{
public:
  serial(serial const&) = delete;
  serial& operator=(serial const&) = delete;
  serial(serial&&) = delete;
  serial& operator=(serial&&) = delete;

  /// Stop the receive DMA channel and return the pins to their reset state
  ~serial();

private:
  friend class usart;

  serial(usart& p_manager,
         std::span<hal::byte> p_buffer,
         hal::serial::settings const& p_settings);

  void driver_configure(hal::serial::settings const& p_settings) override;
  void driver_write(mem::scatter_span<hal::byte const> p_data) override;
  hal::circular_span<hal::byte const> driver_receive_buffer() override;
  hal::usize driver_receive_cursor() override;

  hal::ref<usart> m_manager;
  peripheral m_id;
  pin m_tx{};
  pin m_rx{};
  u8 m_dma_channel = 0;
  hal::stm32_generic::uart m_uart;
};
}  // namespace hal::stm32f1
