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

export module hal.arm_mcu.stm32f1:can;

import hal;
import hal.util;

import :constants;
import :pin;

namespace hal::stm32f1 {

/**
 * @brief Selects which of the bxCAN's two receive FIFOs a filter bank
 * delivers matching messages into.
 */
export enum class can_fifo : hal::u8 {
  select1 = 0,
  select2 = 1,
};

/// Configuration for `can::create()`, beyond the baud rate. Declared at
/// namespace scope (rather than nested in `can`) because a nested struct's
/// default member initializers aren't available yet for use as a default
/// argument on a member function of the same enclosing class.
export struct can_settings
{
  /// TX/RX pin pair to route the CAN1 peripheral to.
  can_pins pins = can_pins::pa11_pa12;

  /// Number of `hal::can_message`s the receive buffer can hold.
  hal::usize message_count = 8;

  /// Enable loopback self-test mode at construction. Messages sent by this
  /// device's transceiver are received back by this same device.
  bool enable_self_test = false;
};

/**
 * @brief STM32F1 CAN (bxCAN) peripheral manager
 *
 * Owns the STM32F1's CAN1 register bank, its 28 filter banks, and the
 * receive message buffer. Vends `hal::can_transceiver`, `hal::can_bus_manager`
 * and filter resources from that shared hardware. Only `can1` is supported;
 * `can2` shares CAN1's filter banks on connectivity-line parts and is not yet
 * implemented by this driver.
 */
export class can
  : public hal::pimpl<can>
  , public mem::enable_strong_from_this<can>
{
public:
  struct impl;

  /**
   * @brief Acquire a CAN peripheral manager
   *
   * Powers on the CAN1 peripheral, configures its bit timing for the
   * requested baud rate, and routes it to the requested TX/RX pins.
   *
   * @param p_allocator - allocator used to allocate the memory for the
   * object and its receive message buffer. Resources later acquired from
   * this manager (transceiver, bus manager, filters) are allocated from this
   * same allocator via `memory_resource()`.
   * @param p_baud_rate - bus baud rate in hertz
   * @param p_settings - pin selection, receive buffer size, and self-test
   * @return hal::ptr<can> - CAN peripheral manager
   * @throws hal::operation_not_supported - if `p_baud_rate` cannot be
   * achieved with the CAN1 peripheral's input clock.
   * @throws hal::device_or_resource_busy - if CAN1 is already powered on and
   * managed elsewhere.
   */
  [[nodiscard]] static hal::ptr<can> create(
    hal::allocator p_allocator,
    hal::u32 p_baud_rate,
    can_settings const& p_settings = {});

  /**
   * @brief Acquire the transceiver resource for this CAN peripheral
   *
   * @return hal::ptr<hal::can_transceiver> - type erased CAN transceiver
   */
  [[nodiscard]] hal::ptr<hal::can_transceiver> acquire_transceiver();

  /**
   * @private
   * @brief Send a can message
   *
   * Called by the transceiver resource acquired from this manager.
   *
   * @param p_context - async context for coroutine suspension and resumption
   * @param p_message - message to be sent over the can network
   * @throws hal::operation_not_permitted - if the device is bus-off
   * @throws hal::resource_unavailable_try_again - if no transmit mailbox
   * becomes available in time
   */
  [[nodiscard]] async::future<void> send(async::context& p_context,
                                         hal::can_message const& p_message);

  /**
   * @private
   * @brief Returns this manager's message receive buffer
   */
  [[nodiscard]] hal::circular_span<hal::can_message const> receive_buffer();

  /**
   * @private
   * @brief Returns the current write position of the receive buffer
   */
  [[nodiscard]] hal::usize receive_cursor();

  /**
   * @brief Acquire the bus configuration/control resource for this CAN
   * peripheral
   *
   * @return hal::ptr<hal::can_bus_manager> - type erased CAN bus manager
   */
  [[nodiscard]] hal::ptr<hal::can_bus_manager> acquire_bus_manager();

  /**
   * @private
   * @brief Set the bus baud rate
   *
   * Called by the bus manager resource acquired from this manager.
   *
   * @throws hal::operation_not_supported - if `p_hertz` cannot be achieved
   */
  [[nodiscard]] async::future<void> set_baud_rate(async::context& p_context,
                                                  hal::u32 p_hertz);

  /**
   * @private
   * @brief Suspend until the device enters the bus-off state
   *
   * Called by the bus manager resource acquired from this manager. Only one
   * waiter is supported at a time, matching `hal::can_bus_manager`'s
   * documented single-owner usage.
   */
  [[nodiscard]] async::future<void> wait_for_bus_off(async::context& p_context);

  /**
   * @private
   * @brief Leave the bus-off state, if in it
   *
   * Called by the bus manager resource acquired from this manager.
   */
  [[nodiscard]] async::future<void> bus_on(async::context& p_context);

  /**
   * @private
   * @brief Claim an unused filter bank
   *
   * Called by the filter resource types acquired from this manager.
   *
   * @throws hal::resource_unavailable_try_again - if every filter bank is
   * already claimed
   */
  [[nodiscard]] hal::u8 acquire_filter_bank();

  /**
   * @private
   * @brief Release a previously claimed filter bank
   *
   * Called by the filter resource types acquired from this manager.
   */
  void release_filter_bank(hal::u8 p_index);

  /**
   * @brief Acquire a set of 4x standard (11-bit) identifier list filters
   *
   * Claims one bxCAN filter bank in dual 16-bit list mode.
   *
   * @param p_fifo - which receive FIFO matching messages are delivered into
   * @return std::array<hal::ptr<hal::can_id_filter>, 4> - a set of 4x
   * identifier filters, co-owning the filter bank they share. Releases the
   * bank once every filter in the set has been destroyed.
   * @throws hal::resource_unavailable_try_again - if no filter banks are
   * available
   */
  [[nodiscard]] std::array<hal::ptr<hal::can_id_filter>, 4>
  acquire_identifier_filter(can_fifo p_fifo = can_fifo::select1);

  /**
   * @brief Acquire a pair of extended (29-bit) identifier list filters
   *
   * Claims one bxCAN filter bank in single 32-bit list mode.
   *
   * @param p_fifo - which receive FIFO matching messages are delivered into
   * @return std::array<hal::ptr<hal::can_id_ext_filter>, 2> - a pair of
   * extended identifier filters, co-owning the filter bank they share.
   * @throws hal::resource_unavailable_try_again - if no filter banks are
   * available
   */
  [[nodiscard]] std::array<hal::ptr<hal::can_id_ext_filter>, 2>
  acquire_extended_identifier_filter(can_fifo p_fifo = can_fifo::select1);

  /**
   * @brief Acquire a pair of standard (11-bit) mask filters
   *
   * Claims one bxCAN filter bank in dual 16-bit mask mode.
   *
   * @param p_fifo - which receive FIFO matching messages are delivered into
   * @return std::array<hal::ptr<hal::can_mask_filter>, 2> - a pair of mask
   * filters, co-owning the filter bank they share.
   * @throws hal::resource_unavailable_try_again - if no filter banks are
   * available
   */
  [[nodiscard]] std::array<hal::ptr<hal::can_mask_filter>, 2>
  acquire_mask_filter(can_fifo p_fifo = can_fifo::select1);

  /**
   * @brief Acquire an extended (29-bit) mask filter
   *
   * Claims one bxCAN filter bank in single 32-bit mask mode.
   *
   * @param p_fifo - which receive FIFO matching messages are delivered into
   * @return hal::ptr<hal::can_mask_ext_filter> - extended mask filter
   * @throws hal::resource_unavailable_try_again - if no filter banks are
   * available
   */
  [[nodiscard]] hal::ptr<hal::can_mask_ext_filter> acquire_extended_mask_filter(
    can_fifo p_fifo = can_fifo::select1);

  /// @private
  can(private_key,
      hal::allocator p_allocator,
      hal::u32 p_baud_rate,
      can_settings const& p_settings);

  can(can const&) = delete;
  can& operator=(can const&) = delete;
  can(can&&) = delete;
  can& operator=(can&&) = delete;

  ~can();
};
}  // namespace hal::stm32f1
