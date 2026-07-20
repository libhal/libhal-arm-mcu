// Copyright 2024 - 2025 Khalil Estell and the libhal contributors
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

#pragma once

#include <array>
#include <bitset>
#include <memory_resource>

#include <libhal/can.hpp>
#include <libhal/circular_buffer.hpp>
#include <libhal/pointers.hpp>
#include <libhal/units.hpp>

#include "constants.hpp"
#include "pin.hpp"

namespace hal::lpc40 {
/**
 * @brief Manager for the LPC40xx CAN peripheral (CAN1 or CAN2)
 *
 * The LPC40xx CAN peripheral's acceptance filter is always configured to
 * accept every message on the bus. Hardware ID filtering is not implemented
 * by this driver. Filter objects acquired from this manager (identifier,
 * mask, and extended variants) are accepted for API compatibility but do not
 * change which messages are received.
 */
class can_peripheral_manager_v2
{
public:
  /// Contains all of the information needed to control and configure a CAN
  /// BUS port on the LPC40xx platform.
  struct port
  {
    /// Reference to transmit pin object
    pin td;
    /// Pin function code for transmit
    std::uint8_t td_function_code;
    /// Reference to read pin object
    pin rd;
    /// Pin function code for receive
    std::uint8_t rd_function_code;
    /// Peripheral's ID
    peripheral id;
    /// IRQ
    irq irq_number;
  };

  /**
   * @brief Construct a new can peripheral manager for CAN1 or CAN2
   *
   * @param p_message_count - number of messages the receive buffer will hold
   * @param p_allocator - allocator used to allocate the receive buffer
   * @param p_baud_rate - set baud rate of the device
   * @param p_can_port_number - selects which CAN port to use, must be either
   * `1` (CAN1) or `2` (CAN2).
   * @throws hal::operation_not_supported - if `p_can_port_number` is not `1`
   * or `2`, or if the baud rate is not achievable.
   */
  can_peripheral_manager_v2(
    hal::usize p_message_count,
    std::pmr::polymorphic_allocator<> p_allocator,
    hal::u32 p_baud_rate,
    std::uint8_t p_can_port_number = 1);

  /**
   * @brief Construct a new can peripheral manager using a custom port
   * configuration
   *
   * @param p_message_count - number of messages the receive buffer will hold
   * @param p_allocator - allocator used to allocate the receive buffer
   * @param p_baud_rate - set baud rate of the device
   * @param p_port - CAN port pin & peripheral information
   * @throws hal::operation_not_supported - if the baud rate is not achievable
   */
  can_peripheral_manager_v2(hal::usize p_message_count,
                             std::pmr::polymorphic_allocator<> p_allocator,
                             hal::u32 p_baud_rate,
                             port const& p_port);

  can_peripheral_manager_v2(can_peripheral_manager_v2 const&) = delete;
  can_peripheral_manager_v2& operator=(can_peripheral_manager_v2 const&) =
    delete;
  can_peripheral_manager_v2(can_peripheral_manager_v2&&) = delete;
  can_peripheral_manager_v2& operator=(can_peripheral_manager_v2&&) = delete;
  ~can_peripheral_manager_v2();

  /**
   * @brief Set can peripheral's baud rate
   *
   * @param p_hertz - baud rate in hertz
   */
  void baud_rate(hal::u32 p_hertz);

  /**
   * @brief Get can peripheral's baud rate
   *
   * @return hal::u32 - baud rate in hertz
   */
  hal::u32 baud_rate() const;

  /**
   * @brief Send a can message
   *
   * @param p_message - message to send
   * @throws hal::operation_not_permitted - if the device is in the "bus-off"
   * state.
   */
  void send(can_message const& p_message);

  /**
   * @brief Set callback on message reception
   *
   * @param p_callback - callback to be called on each received message
   */
  void on_receive(
    hal::can_interrupt::optional_receive_handler const& p_callback);

  /**
   * @brief Set callback for when the device enters the "bus-off" state
   *
   * @param p_callback - callback to be called when the device goes bus-off
   */
  void on_bus_off(hal::can_bus_manager::optional_bus_off_handler p_callback);

  /**
   * @brief Exit "bus-off" state if the device is in that state.
   *
   * If the device is already bus-on then nothing happens.
   */
  void bus_on();

  /**
   * @brief Get a view of the circular receive buffer
   *
   * @return std::span<can_message const> - span of received messages
   */
  std::span<can_message const> receive_buffer()
  {
    return { m_buffer.data(), m_buffer.capacity() };
  }

  /**
   * @brief Get the current write index of the circular receive buffer
   *
   * @return std::size_t - current write position in the buffer
   */
  std::size_t receive_cursor()
  {
    return m_buffer.write_index();
  }

  /**
   * @brief Get the total count of received can messages
   *
   * This number increases monotonically until it overflows
   *
   * @return std::size_t - total number of received can messages
   */
  std::size_t receive_count() const
  {
    return m_message_count;
  }

  /**
   * @brief Scan through the acquired filter slots and return the index to an
   * available one.
   *
   * NOTE: The LPC40xx driver does not implement hardware ID filtering. These
   * slots do not correspond to real hardware resources; they simply prevent
   * more filter objects than the platform's stm32f1 counterpart would allow
   * from being acquired at once.
   *
   * @return hal::u8 - returns the index of the filter slot that has been
   * acquired for the caller.
   * @throws hal::resource_unavailable_try_again - if no filter slots are
   * available
   */
  [[nodiscard(
    "This value must be saved in order to be cleared later")]] hal::u8
  available_filter();

  /**
   * @brief Release filter slot
   *
   * NOTE: This should only be used by the internal drivers and not by
   * callers otherwise, resources can be forgotten.
   *
   * @param p_filter_bank - filter slot to release
   */
  void release_filter(hal::u8 p_filter_bank);

private:
  void setup(port const& p_port, hal::u32 p_baud_rate);
  void configure_baud_rate(hal::u32 p_baud_rate);
  void install_interrupt();

  port m_port{};
  std::bitset<32> m_acquired_banks{};
  hal::v5::circular_buffer<hal::can_message> m_buffer;
  hal::u32 m_current_baud_rate = 0;
  hal::usize m_message_count = 0;
  can_interrupt::optional_receive_handler m_receive_handler{};
  hal::can_bus_manager::optional_bus_off_handler m_bus_off_handler{};
};

/**
 * @brief Acquire a `hal::can_transceiver` from the lpc40 can_peripheral_manager
 *
 * @param p_allocator - allocator used to allocate the driver
 * @param p_manager - Manager for which to extract the can_transceiver
 * @return hal::v5::strong_ptr<hal::can_transceiver> - can transceiver
 */
hal::v5::strong_ptr<hal::can_transceiver> acquire_can_transceiver(
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::v5::strong_ptr<can_peripheral_manager_v2> const& p_manager);

/**
 * @brief Acquire a `hal::can_bus_manager` from the lpc40
 * can_peripheral_manager
 *
 * @param p_allocator - allocator used to allocate the driver
 * @param p_manager - Manager for which to extract the can_bus_manager
 * @return hal::v5::strong_ptr<hal::can_bus_manager>
 */
hal::v5::strong_ptr<hal::can_bus_manager> acquire_can_bus_manager(
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::v5::strong_ptr<can_peripheral_manager_v2> const& p_manager);

/**
 * @brief Acquire an `hal::can_interrupt` implementation
 *
 * @param p_allocator - allocator used to allocate the driver
 * @param p_manager - Manager for which to extract the can_interrupt
 * @return hal::v5::strong_ptr<hal::can_interrupt> - object implementing the
 * `hal::can_interrupt` interface for this can peripheral.
 */
hal::v5::strong_ptr<hal::can_interrupt> acquire_can_interrupt(
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::v5::strong_ptr<can_peripheral_manager_v2> const& p_manager);

/**
 * @brief Acquire a set of 4x standard identifier filters
 *
 * NOTE: This platform's CAN peripheral does not support hardware ID
 * filtering. The returned filters accept the `allow()` call, but it has no
 * effect: every message on the bus is received regardless of filter state.
 *
 * @param p_allocator - allocator used to allocate the driver
 * @param p_manager - Manager for which to extract the filter
 * @return std::array<hal::v5::strong_ptr<hal::can_identifier_filter>, 4> - A
 * set of 4x identifier filters. When destroyed, releases the filter slot it
 * held on to.
 */
std::array<hal::v5::strong_ptr<hal::can_identifier_filter>, 4>
acquire_can_identifier_filter(
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::v5::strong_ptr<can_peripheral_manager_v2> const& p_manager);

/**
 * @brief Acquire a pair of two extended identifier filters
 *
 * NOTE: See `acquire_can_identifier_filter()` for details on why hardware
 * filtering is not implemented for this platform.
 *
 * @param p_allocator - allocator used to allocate the driver
 * @param p_manager - Manager for which to extract the filter
 * @return std::array<hal::v5::strong_ptr<hal::can_extended_identifier_filter>,
 * 2> - A set of 2x extended identifier filters.
 */
std::array<hal::v5::strong_ptr<hal::can_extended_identifier_filter>, 2>
acquire_can_extended_identifier_filter(
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::v5::strong_ptr<can_peripheral_manager_v2> const& p_manager);

/**
 * @brief Acquire a pair of mask filters
 *
 * NOTE: See `acquire_can_identifier_filter()` for details on why hardware
 * filtering is not implemented for this platform.
 *
 * @param p_allocator - allocator used to allocate the driver
 * @param p_manager - Manager for which to extract the filter
 * @return std::array<hal::v5::strong_ptr<hal::can_mask_filter>, 2> - A set of
 * 2x standard mask filters
 */
std::array<hal::v5::strong_ptr<hal::can_mask_filter>, 2>
acquire_can_mask_filter(
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::v5::strong_ptr<can_peripheral_manager_v2> const& p_manager);

/**
 * @brief Acquire a single extended identifier mask filter
 *
 * NOTE: See `acquire_can_identifier_filter()` for details on why hardware
 * filtering is not implemented for this platform.
 *
 * @param p_allocator - allocator used to allocate the driver
 * @param p_manager - Manager for which to extract the filter
 * @return hal::v5::strong_ptr<hal::can_extended_mask_filter> - extended mask
 * filter
 */
hal::v5::strong_ptr<hal::can_extended_mask_filter>
acquire_can_extended_mask_filter(
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::v5::strong_ptr<can_peripheral_manager_v2> const& p_manager);

}  // namespace hal::lpc40

namespace hal {
using lpc40::acquire_can_bus_manager;
using lpc40::acquire_can_extended_identifier_filter;
using lpc40::acquire_can_extended_mask_filter;
using lpc40::acquire_can_identifier_filter;
using lpc40::acquire_can_interrupt;
using lpc40::acquire_can_mask_filter;
using lpc40::acquire_can_transceiver;
}  // namespace hal
