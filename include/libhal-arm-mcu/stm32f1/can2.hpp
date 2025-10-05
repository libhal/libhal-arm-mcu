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

#include <bitset>
#include <chrono>
#include <memory_resource>

#include <libhal/can.hpp>
#include <libhal/circular_buffer.hpp>
#include <libhal/pointers.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>

#include "pin.hpp"

namespace hal::stm32f1 {
/**
 * @brief Defines what a "disabled" ID would be
 *
 * The stm32 bxCAN system uses a series of filter bank registers. Each can be
 * used to either create 4x identifier filters, 2x mask filters, 2x extended
 * identifier filters or 1x extended mask filter. Entire filter bank can be
 * disabled/enabled, but not on a sub-bank basis. So to support "disabling a
 * filter, we simply provide IDs that we expect to never appear on the CAN bus
 * and use those as a "disabled" filter state.
 *
 */
enum class can_fifo : std::uint8_t
{
  select1 = 0,
  select2 = 1,
};

/**
 * @brief Defines what a "disabled" ID would be
 *
 * The stm32 bxCAN system uses a series of filter bank registers. Each can be
 * used to either create 4x identifier filters, 2x mask filters, 2x extended
 * identifier filters or 1x extended mask filter. Entire filter bank can be
 * disabled/enabled, but not on a sub-bank basis. So to support "disabling a
 * filter, we simply provide IDs that we expect to never appear on the CAN bus
 * and use those as a "disabled" filter state.
 *
 * If {0, 0} doesn't work for you, you can choose your own sentinel value as a
 * "disable id"
 *
 */
struct can_disable_ids
{
  u16 standard = 0;
  u32 extended = 0;
};

/**
 * @brief Named parameter for enabling self test at driver construction
 *
 */
enum class can_self_test : hal::u8
{
  off = 0,
  on = 1,
};

/**
 * @brief Contains a filter and word index, used by filter implementations to
 * determine which bits in the filter registers to modify.
 *
 */
struct can_filter_resource
{
  hal::u8 filter_index = 0;
  hal::u8 word_index = 0;
  bool filter_for_remote_request = false;
};

class can_peripheral_manager_v2
{
public:
  /**
   * @brief Construct a new can peripheral manager v2 object
   *
   * @param p_allocator - allocator used to allocate receive buffer
   * @param p_message_count - number of messages the receive buffer will hold
   * @param p_clock - a steady clock used to determine if initialization has
   * exceeded the p_timeout_time.
   * @param p_timeout_time - the amount of time to wait for initialization
   * before throwing `hal::timed_out`.
   * @param p_baud_rate - set baud rate of the device
   * @param p_pins - CAN bus RX and TX pin selection
   * @param p_enable_self_test - determines if self test is enabled at
   * construction.
   * @param p_disabled_ids - IDs to use as filtering values when a filter is set
   * to be disabled. Choose IDs you expect will never appear on the CAN BUS.
   * @throw hal::operation_not_supported - if the baud rate is not usable
   * @throw hal::timed_out - if can peripheral initialization
   */
  can_peripheral_manager_v2(
    hal::usize p_message_count,
    std::pmr::polymorphic_allocator<> p_allocator,
    hal::u32 p_baud_rate,
    hal::steady_clock& p_clock,
    hal::time_duration p_timeout_time = std::chrono::milliseconds(1),
    can_pins p_pins = can_pins::pa11_pa12,
    can_self_test p_enable_self_test = can_self_test::off,
    can_disable_ids p_disabled_ids = can_disable_ids{ .standard = 0,
                                                      .extended = 0 });

  can_peripheral_manager_v2(can_peripheral_manager_v2 const&) = delete;
  can_peripheral_manager_v2& operator=(can_peripheral_manager_v2 const&) =
    delete;
  can_peripheral_manager_v2(can_peripheral_manager_v2&&) = delete;
  can_peripheral_manager_v2& operator=(can_peripheral_manager_v2&&) = delete;
  ~can_peripheral_manager_v2();

  /**
   * @brief Enable/disable self test mode
   *
   * Self test mode allows message loopback. This mode allows messages sent from
   * this device's can transceiver to be received back by this device.
   *
   * @param p_enable - enable self test mode
   */
  void enable_self_test(bool p_enable);

  /**
   * @brief Set can peripheral's baud rate
   *
   * @param p_hertz - baud rate in hertz
   */
  void baud_rate(hal::u32 p_hertz);

  /**
   * @brief Get can peripheral's baud rate
   *
   * @returns hal::u32 - baud rate in hertz
   */
  hal::u32 baud_rate() const;

  /**
   * @brief Send a can message
   *
   * @param p_message
   */
  void send(can_message const& p_message);

  /**
   * @brief Set callback on message reception
   *
   * @param p_callback - callback to be called on each received message
   */
  void on_receive(hal::can_interrupt::optional_receive_handler p_callback);

  /**
   * @brief Exit "bus-off" state if the device is in that state.
   *
   * If the device is already bus-on then nothing happens.
   *
   */
  void bus_on();

  std::span<can_message const> receive_buffer()
  {
    return { m_buffer.data(), m_buffer.capacity() };
  }

  std::size_t receive_cursor()
  {
    return m_buffer.write_index();
  }

private:
  /**
   * @brief Scan through the acquired banks and return the index to an available
   * one.
   *
   * @return hal::u8 - returns the index of the filter banks that has been
   * acquired for the caller.
   * @throws hal::resource_unavailable_try_again - if no filter banks are
   * available
   */
  hal::u8 available_filter();

  hal::v5::circular_buffer<hal::can_message> m_buffer;
  hal::u32 m_current_baud_rate = 0;
  can_disable_ids m_disable_id{};
  can_interrupt::optional_receive_handler m_receive_handler{};
  std::bitset<28> m_acquired_banks{};
};

/**
 * @brief Acquire a `hal::can_transceiver` from the stm32f1
 * can_peripheral_manager
 *
 * @param p_allocator - allocator for the object
 * @param p_manager - stm32f1 can manager object
 * @param p_message_buffer_size - number of can messages the can transceiver
 * circular buffer in can hold.
 * @return hal::v5::strong_ptr<hal::can_transceiver> - can transceiver
 */
hal::v5::strong_ptr<hal::can_transceiver> acquire_transceiver(
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::v5::strong_ptr<can_peripheral_manager_v2> p_manager,
  hal::usize p_message_buffer_size);

/**
 * @brief Acquire a `hal::can_bus_manager` from the stm32f1
 * can_peripheral_manager.
 *
 *
 * @param p_allocator - allocator for the object
 * @param p_manager - stm32f1 can manager object
 * @return hal::v5::strong_ptr<hal::can_bus_manager>
 */
hal::v5::strong_ptr<hal::can_bus_manager> acquire_bus_manager(
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::v5::strong_ptr<can_peripheral_manager_v2> p_manager);

/**
 * @brief Acquire an `hal::can_interrupt` implementation
 *
 * @return interrupt - object implementing the `hal::can_interrupt` interface
 * for this can peripheral.
 */
hal::v5::strong_ptr<hal::can_interrupt> acquire_interrupt(
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::v5::strong_ptr<can_peripheral_manager_v2> p_manager);

/**
 * @brief Acquire a set of 4x standard identifier filters
 *
 * @return identifier_filter_set - A set of 4x identifier filters. When
 * destroyed, releases the filter resource it held on to.
 */
std::array<hal::v5::strong_ptr<hal::can_identifier_filter>, 4>
acquire_identifier_filter(
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::v5::strong_ptr<can_peripheral_manager_v2> p_manager,
  can_fifo p_fifo = can_fifo::select1);

/**
 * @brief Acquire a pair of two extended identifier filters
 *
 * @return extended_identifier_filter_set - A set of 2x extended identifier
 * filters.
 */
std::array<hal::v5::strong_ptr<hal::can_extended_identifier_filter>, 2>
acquire_extended_identifier_filter(
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::v5::strong_ptr<can_peripheral_manager_v2> p_manager,
  can_fifo p_fifo = can_fifo::select1);

/**
 * @brief Acquire a pair of mask filters
 *
 * @param p_manager - Manager for which to extract the filter
 * @param p_fifo - Select the FIFO to store the received message
 * @return hal::v5::strong_ptr<can_mask_filter_set> - A set of 2x standard mask
 * filters
 */
std::array<hal::v5::strong_ptr<hal::can_mask_filter>, 2> acquire_mask_filter(
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::v5::strong_ptr<can_peripheral_manager_v2> p_manager,
  can_fifo p_fifo = can_fifo::select1);

/**
 * @brief Acquire an extended mask filter
 *
 * @param p_manager - Manager for which to extract the filter
 * @param p_fifo - Select the FIFO to store the received message
 * @return hal::v5::strong_ptr<hal::can_extended_mask_filter> - An extended mask
 * filter
 */
hal::v5::strong_ptr<hal::can_extended_mask_filter> acquire_extended_mask_filter(
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::v5::strong_ptr<can_peripheral_manager_v2> p_manager,
  can_fifo p_fifo = can_fifo::select1);

}  // namespace hal::stm32f1
