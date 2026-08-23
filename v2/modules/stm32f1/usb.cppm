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

#include <array>

export module hal.arm_mcu.stm32f1:usb;

import hal;
import hal.util;

import :constants;
import :pin;

namespace hal::stm32f1 {

/**
 * @brief USB interrupt endpoint pair returned from
 * `usb::acquire_interrupt_endpoint()`
 *
 * Both endpoints share the same logical endpoint number (one direction
 * each), as required by the STM32F1's USB peripheral.
 */
export struct usb_interrupt_endpoint_pair
{
  /// The OUT (host-to-device) half of this endpoint pair
  hal::ptr<hal::usb::interrupt_out_endpoint> out;
  /// The IN (device-to-host) half of this endpoint pair
  hal::ptr<hal::usb::interrupt_in_endpoint> in;
};

/**
 * @brief USB bulk endpoint pair returned from `usb::acquire_bulk_endpoint()`
 *
 * Both endpoints share the same logical endpoint number (one direction
 * each), as required by the STM32F1's USB peripheral.
 */
export struct usb_bulk_endpoint_pair
{
  /// The OUT (host-to-device) half of this endpoint pair
  hal::ptr<hal::usb::bulk_out_endpoint> out;
  /// The IN (device-to-host) half of this endpoint pair
  hal::ptr<hal::usb::bulk_in_endpoint> in;
};

/**
 * @brief STM32F1 USB device-controller peripheral manager
 *
 * Owns the STM32F1's USB register bank and its 512-byte packet memory area
 * (PMA), vending the control endpoint plus interrupt/bulk endpoint pairs.
 * The STM32F1's USB peripheral physically shares its packet memory with
 * CAN1, so only one of `hal::stm32f1::usb` or `hal::stm32f1::can` may be
 * powered on at a time.
 *
 * Every endpoint is fixed at 16 bytes (RM0008's example configuration); the
 * peripheral supports at most `endpoint_count` (8) logical endpoints,
 * including endpoint 0 which is always reserved for the control endpoint.
 */
export class usb
  : public hal::pimpl<usb>
  , public mem::enable_strong_from_this<usb>
{
public:
  struct impl;

  /// Number of logical endpoints supported by the STM32F1 USB peripheral,
  /// including endpoint 0 (the control endpoint).
  static constexpr hal::u8 endpoint_count = 8;

  /**
   * @brief Acquire a USB peripheral manager
   *
   * Powers on the USB peripheral and performs its power-up sequence
   * (RM0008's PDWN/FRES timed transition), which requires suspending for a
   * few fixed settle-time delays — hence this factory is a coroutine rather
   * than a synchronous call.
   *
   * @param p_context - async context for coroutine suspension and resumption
   * @param p_allocator - allocator used to allocate the memory for the
   * object. Resources later acquired from this manager (control/interrupt/
   * bulk endpoints) are allocated from this same allocator via
   * `memory_resource()`.
   * @return hal::future_ptr<usb> - completes with the USB peripheral manager
   * @throws hal::operation_not_supported - if the USB peripheral's input
   * clock is not exactly 48 MHz.
   * @throws hal::device_or_resource_busy - if USB or CAN1 is already powered
   * on and managed elsewhere (they share the same packet memory).
   */
  [[nodiscard]] static hal::future_ptr<usb> create(async::context& p_context,
                                                    hal::allocator p_allocator);

  /**
   * @brief Acquire the control endpoint (endpoint 0)
   *
   * @return hal::ptr<hal::usb::control_endpoint> - type erased control
   * endpoint
   */
  [[nodiscard]] hal::ptr<hal::usb::control_endpoint> acquire_control_endpoint();

  /**
   * @brief Acquire an interrupt endpoint pair
   *
   * Claims the next unused logical endpoint number for both directions.
   *
   * @return usb_interrupt_endpoint_pair - IN and OUT interrupt endpoints
   * sharing one logical endpoint number
   * @throws hal::resource_unavailable_try_again - if every logical endpoint
   * number has already been claimed
   */
  [[nodiscard]] usb_interrupt_endpoint_pair acquire_interrupt_endpoint();

  /**
   * @brief Acquire a bulk endpoint pair
   *
   * Claims the next unused logical endpoint number for both directions.
   *
   * @return usb_bulk_endpoint_pair - IN and OUT bulk endpoints sharing one
   * logical endpoint number
   * @throws hal::resource_unavailable_try_again - if every logical endpoint
   * number has already been claimed
   */
  [[nodiscard]] usb_bulk_endpoint_pair acquire_bulk_endpoint();

  /**
   * @private
   * @brief Write data to an IN endpoint, suspending until each 16-byte chunk
   * has been transmitted
   *
   * Called by the endpoint resource types acquired from this manager.
   */
  [[nodiscard]] async::future<void> write(
    async::context& p_context,
    hal::u8 p_endpoint,
    mem::scatter_span<hal::byte const> p_data);

  /**
   * @private
   * @brief Suspend until data becomes available on an OUT endpoint
   *
   * Called by the endpoint resource types acquired from this manager. Only
   * a single waiting context is supported per endpoint.
   */
  [[nodiscard]] async::future<void> wait_for_receive(async::context& p_context,
                                                      hal::u8 p_endpoint);

  /**
   * @private
   * @brief Suspend until the next bus-level event occurs
   *
   * Called by the control endpoint resource acquired from this manager.
   * Only a single waiting context is supported.
   */
  [[nodiscard]] async::future<hal::usb::bus_event> wait_for_bus_event(
    async::context& p_context);

  /**
   * @private
   * @brief Claim the next unused logical endpoint number
   *
   * Called by `acquire_interrupt_endpoint()`/`acquire_bulk_endpoint()`.
   *
   * @throws hal::resource_unavailable_try_again - if every logical endpoint
   * number has already been claimed
   */
  [[nodiscard]] hal::u8 allocate_endpoint_number();

  /**
   * @private
   * @brief Grant or revoke remote wakeup permission
   */
  void remote_wakeup_enable(bool p_enabled);

  /**
   * @private
   * @brief Query the current remote wakeup permission
   */
  [[nodiscard]] bool remote_wakeup_granted();

  /**
   * @private
   * @brief Query whether the last endpoint-0 packet was a SETUP packet
   *
   * Called by the control endpoint resource acquired from this manager.
   */
  [[nodiscard]] bool setup_packet_pending();

  /**
   * @private
   * @brief Clear the setup-packet flag once its 8 bytes have been consumed
   *
   * Called by the control endpoint resource acquired from this manager.
   */
  void clear_setup_packet_flag();

  /// @private
  usb(private_key, hal::allocator p_allocator);

  usb(usb const&) = delete;
  usb& operator=(usb const&) = delete;
  usb(usb&&) = delete;
  usb& operator=(usb&&) = delete;

  ~usb();
};
}  // namespace hal::stm32f1
