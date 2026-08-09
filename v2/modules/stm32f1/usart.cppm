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

import :constants;

namespace hal::stm32f1 {

/**
 * @brief STM32F1 USART peripheral manager
 *
 * Owns one of the STM32F1's USART hardware blocks: its register bank, its
 * receive DMA channel, and its TX/RX pin pair. Only `usart1`, `usart2`, and
 * `usart3` are supported. `uart4` and `uart5` use a different pin/DMA
 * mapping that is not yet implemented by this driver.
 */
export class usart
  : public hal::pimpl<usart>
  , public mem::enable_strong_from_this<usart>
{
public:
  struct impl;

  /**
   * @brief Acquire a USART peripheral manager
   *
   * This powers on the USART peripheral. The peripheral is powered off when
   * the returned manager, and every serial resource acquired from it, has
   * been released.
   *
   * @param p_allocator - allocator used to allocate the memory for the
   * object
   * @param p_id - which USART peripheral to manage. Must be `usart1`,
   * `usart2`, or `usart3`.
   * @return hal::ptr<usart> - USART peripheral manager
   * @throws hal::operation_not_supported - if `p_id` is not `usart1`,
   * `usart2`, or `usart3`.
   * @throws hal::device_or_resource_busy - if the peripheral is already
   * powered on and managed elsewhere.
   */
  [[nodiscard]] static hal::ptr<usart> create(hal::allocator p_allocator,
                                              peripheral p_id);

  /**
   * @brief Acquire the serial resource for this USART peripheral
   *
   * Configures the TX/RX pins for this peripheral and sets up its receive
   * DMA channel to continuously write incoming bytes into `p_buffer` in a
   * circular fashion.
   *
   * @param p_buffer - buffer that the receive DMA channel is configured to
   * write into. Must outlive the returned serial resource.
   * @param p_settings - serial settings to apply
   * @return hal::ptr<hal::serial> - type erased serial driver
   * @throws hal::operation_not_supported - if `p_buffer` is larger than the
   * DMA controller can address, or if the requested baud rate cannot be
   * achieved.
   */
  [[nodiscard]] hal::ptr<hal::serial> acquire_serial(
    std::span<hal::byte> p_buffer,
    hal::serial::settings const& p_settings = {});

  /// @private
  usart(private_key, hal::allocator p_allocator, peripheral p_id);

  usart(usart const&) = delete;
  usart& operator=(usart const&) = delete;
  usart(usart&&) = delete;
  usart& operator=(usart&&) = delete;

  ~usart();
};
}  // namespace hal::stm32f1
