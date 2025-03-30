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

#include <chrono>

#include <libhal/can.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>

#include "pin.hpp"

namespace hal::stm32f1 {
class can final : public hal::can
{
public:
  can(can::settings const& p_settings = {},
      can_pins p_pins = can_pins::pa11_pa12);
  void enable_self_test(bool p_enable);
  ~can() override;

private:
  void driver_configure(settings const& p_settings) override;
  void driver_bus_on() override;
  void driver_send(message_t const& p_message) override;
  void driver_on_receive(hal::callback<handler> p_handler) override;
};

/**
 * @brief Peripheral manager for the sole CAN BUS port on the stm32f1xx MCU
 *
 */
struct can_peripheral_manager final
{
  /**
   * @brief Named parameter for enabling self test at driver construction
   *
   */
  enum class self_test : hal::u8
  {
    off = 0,
    on = 1,
  };

  /**
   * @brief Contains a filter and word index, used by filter implementations to
   * determine which bits in the filter registers to modify.
   *
   */
  struct filter_resource
  {
    hal::u8 filter_index = 0;
    hal::u8 word_index = 0;
    bool filter_for_remote_request = false;
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
   */

  /// This enumeration labels the selected FIFO.
  /// Used with CAN FIFO Assignment Register (CAN_FFA1R) (pg. 693).
  enum class fifo_assignment : std::uint8_t
  {
    /// FIFO 1
    fifo1 = 0,
    /// FIFO 2
    fifo2 = 1,
  };

  /**
   * @brief Implementation of the `hal::can_transceiver` interface.
   *
   */
  class transceiver : public hal::can_transceiver
  {
  public:
    transceiver(transceiver const&) = delete;
    transceiver& operator=(transceiver const&) = delete;
    transceiver(transceiver&&) = default;
    transceiver& operator=(transceiver&&) = default;
    ~transceiver() override;

  private:
    friend struct can_peripheral_manager;

    explicit transceiver(std::span<can_message> p_receive_buffer);

    u32 driver_baud_rate() override;
    void driver_send(can_message const& p_message) override;
    std::span<can_message const> driver_receive_buffer() override;
    std::size_t driver_receive_cursor() override;
  };

  /**
   * @brief Implementation of the `hal::can_interrupt` interface.
   *
   */
  class interrupt : public hal::can_interrupt
  {
  public:
    interrupt(interrupt const&) = delete;
    interrupt& operator=(interrupt const&) = delete;
    interrupt(interrupt&&) = default;
    interrupt& operator=(interrupt&&) = default;
    ~interrupt() override;

  private:
    friend struct can_peripheral_manager;

    interrupt();

    void driver_on_receive(optional_receive_handler p_callback) override;
  };

  /**
   * @brief Implementation of the `hal::can_bus_manager` interface.
   *
   */
  class bus_manager : public hal::can_bus_manager
  {
  public:
    bus_manager(bus_manager const&) = delete;
    bus_manager& operator=(bus_manager const&) = delete;
    bus_manager(bus_manager&&) = default;
    bus_manager& operator=(bus_manager&&) = default;
    ~bus_manager() override;

  private:
    friend struct can_peripheral_manager;

    bus_manager() = default;

    void driver_baud_rate(hal::u32 p_hertz) override;
    /**
     * @brief Selects the filtering mode for the device
     * @deprecated Do not use this API. It does nothing when called.
     *
     * @param p_accept - acceptance mode
     */
    void driver_filter_mode(accept p_accept) override;
    void driver_on_bus_off(optional_bus_off_handler p_callback) override;
    void driver_bus_on() override;
  };

  /**
   * @brief Implementation of the `hal::can_identifier_filter` interface.
   *
   */
  class identifier_filter : public hal::can_identifier_filter
  {
  public:
    identifier_filter(identifier_filter const&) = delete;
    identifier_filter& operator=(identifier_filter const&) = delete;
    identifier_filter(identifier_filter&&) = default;
    identifier_filter& operator=(identifier_filter&&) = default;

  private:
    friend struct can_peripheral_manager;

    identifier_filter() = default;

    explicit identifier_filter(filter_resource p_resource);
    void driver_allow(std::optional<u16> p_id) override;

    filter_resource m_resource;
  };

  /**
   * @brief Manages a single filter bank which supports up to 4x identifier
   * filters.
   *
   * When this object is destroyed, the filter ban is deactivated and the filter
   * bank is freed to be used by other filters.
   *
   */
  class identifier_filter_set
  {
  public:
    /**
     * @brief 4x identifier filters
     *
     * Each is given a portion of the filter bank to use for their own filtering
     * needs.
     *
     */
    std::array<identifier_filter, 4> filter;

    identifier_filter_set(identifier_filter_set const&) = delete;
    identifier_filter_set& operator=(identifier_filter_set const&) = delete;
    identifier_filter_set(identifier_filter_set&&) = default;
    identifier_filter_set& operator=(identifier_filter_set&&) = default;
    ~identifier_filter_set();

  private:
    friend struct can_peripheral_manager;

    explicit identifier_filter_set(hal::u8 p_filter_index,
                                   fifo_assignment p_fifo);
  };

  /**
   * @brief Implementation of the `hal::can_extended_identifier_filter`
   * interface.
   *
   */
  class extended_identifier_filter : public hal::can_extended_identifier_filter
  {
  public:
    extended_identifier_filter(extended_identifier_filter const&) = delete;
    extended_identifier_filter& operator=(extended_identifier_filter const&) =
      delete;
    extended_identifier_filter(extended_identifier_filter&&) = default;
    extended_identifier_filter& operator=(extended_identifier_filter&&) =
      default;

  private:
    friend struct can_peripheral_manager;

    explicit extended_identifier_filter(filter_resource p_resource);
    void driver_allow(std::optional<u32> p_id) override;

    filter_resource m_resource;
  };

  /**
   * @brief Manages a single filter bank which supports up to 2x extended
   * identifier filters.
   *
   * When this object is destroyed, the filter ban is deactivated and the filter
   * bank is freed to be used by other filters.
   *
   */
  class extended_identifier_filter_set
  {
  public:
    /**
     * @brief 2x extended identifier filters
     *
     * Each is given a portion of the filter bank to use for their own filtering
     * needs.
     *
     */
    std::array<extended_identifier_filter, 2> filter;

    extended_identifier_filter_set(extended_identifier_filter_set const&) =
      delete;
    extended_identifier_filter_set& operator=(
      extended_identifier_filter_set const&) = delete;
    extended_identifier_filter_set(extended_identifier_filter_set&&) = default;
    extended_identifier_filter_set& operator=(
      extended_identifier_filter_set&&) = default;
    ~extended_identifier_filter_set();

  private:
    friend struct can_peripheral_manager;
    explicit extended_identifier_filter_set(hal::u8 p_filter_index,
                                            fifo_assignment p_fifo);
  };

  /**
   * @brief Implementation of the `hal::can_mask_filter` interface.
   *
   */
  class mask_filter : public hal::can_mask_filter
  {
  public:
    mask_filter(mask_filter const&) = delete;
    mask_filter& operator=(mask_filter const&) = delete;
    mask_filter(mask_filter&&) = default;
    mask_filter& operator=(mask_filter&&) = default;

  private:
    friend struct can_peripheral_manager;

    explicit mask_filter(filter_resource p_resource);
    void driver_allow(std::optional<pair> p_filter_pair) override;

    filter_resource m_resource;
  };

  class mask_filter_set
  {
  public:
    /**
     * @brief 2x mask filters
     *
     * Each is given a portion of the filter bank to use for their own filtering
     * needs.
     *
     */
    std::array<mask_filter, 2> filter;

    mask_filter_set(mask_filter_set const&) = delete;
    mask_filter_set& operator=(mask_filter_set const&) = delete;
    mask_filter_set(mask_filter_set&&) = default;
    mask_filter_set& operator=(mask_filter_set&&) = default;
    ~mask_filter_set();

  private:
    friend struct can_peripheral_manager;
    explicit mask_filter_set(hal::u8 p_filter_index, fifo_assignment p_fifo);
  };

  /**
   * @brief Implementation of the `hal::can_extended_mask_filter` interface.
   *
   * NOTE: extended_mask_filter does not have a "set" object because this
   * implementation requires the full filter bank.
   */
  class extended_mask_filter : public hal::can_extended_mask_filter
  {
  public:
    extended_mask_filter(extended_mask_filter const&) = delete;
    extended_mask_filter& operator=(extended_mask_filter const&) = delete;
    extended_mask_filter(extended_mask_filter&&) = default;
    extended_mask_filter& operator=(extended_mask_filter&&) = default;
    ~extended_mask_filter() override;

  private:
    friend struct can_peripheral_manager;
    explicit extended_mask_filter(hal::u8 p_filter_index,
                                  fifo_assignment p_fifo);

    void driver_allow(std::optional<pair> p_filter_pair) override;

    hal::u8 m_filter_index;
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
   */
  struct disable_ids
  {
    u16 standard = 0;
    u32 extended = 0;
  };

  /**
   * @brief Construct a new can peripheral manager object
   *
   * @deprecated Calling this constructor is problematic as it can halt the
   * system if a hardware can transceiver is not connected to the CAN RX and CAN
   * TX pins. Use the constructor that accepts a `hal::steady_clock` and timeout
   * time instead.
   *
   * @param p_baud_rate - set baud rate of the device
   * @param p_pins - CAN bus RX and TX pin selection
   * @param p_disabled_ids - IDs to use as filtering values when a filter is set
   * to be disabled. Choose IDs you expect will never appear on the CAN BUS.
   * @throw hal::operation_not_supported - if the baud rate is not usable
   */
  [[deprecated("Calling this constructor is problematic as it can halt the "
               "system if a hardware can transceiver is not connected to the "
               "CAN RX and CAN TX pins. Use the constructor that accepts a "
               "`hal::steady_clock` and timeout time instead.")]]
  can_peripheral_manager(
    hal::u32 p_baud_rate,
    can_pins p_pins = can_pins::pa11_pa12,
    disable_ids p_disabled_ids = disable_ids{ .standard = 0, .extended = 0 });

  /**
   * @brief Construct a new can peripheral manager object
   *
   * @param p_baud_rate - set baud rate of the device
   * @param p_clock - a steady clock used to determine if initialization has
   * exceeded the p_timeout_time.
   * @param p_timeout_time - the amount of time to wait for initialization
   * before throwing `hal::timed_out`.
   * @param p_pins - CAN bus RX and TX pin selection
   * @param p_enable_self_test - determines if self test is enabled at
   * construction.
   * @param p_disabled_ids - IDs to use as filtering values when a filter is set
   * to be disabled. Choose IDs you expect will never appear on the CAN BUS.
   * @throw hal::operation_not_supported - if the baud rate is not usable
   * @throw hal::timed_out - if can peripheral initialization
   */
  can_peripheral_manager(
    hal::u32 p_baud_rate,
    hal::steady_clock& p_clock,
    hal::time_duration p_timeout_time = std::chrono::milliseconds(1),
    can_pins p_pins = can_pins::pa11_pa12,
    self_test p_enable_self_test = self_test::off,
    disable_ids p_disabled_ids = disable_ids{ .standard = 0, .extended = 0 });

  can_peripheral_manager(can_peripheral_manager const&) = delete;
  can_peripheral_manager& operator=(can_peripheral_manager const&) = delete;
  can_peripheral_manager(can_peripheral_manager&&) = delete;
  can_peripheral_manager& operator=(can_peripheral_manager&&) = delete;

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
   * @brief Acquire an `hal::can_transceiver` implementation
   *
   * @return transceiver - object implementing the `hal::can_transceiver`
   * interface for this can peripheral.
   */
  transceiver acquire_transceiver(std::span<can_message> p_receive_buffer);

  /**
   * @brief Acquire an `hal::can_bus_manager` implementation
   *
   * @return bus_manager - object implementing the `hal::can_bus_manager`
   * interface for this can peripheral.
   */
  bus_manager acquire_bus_manager();

  /**
   * @brief Acquire an `hal::can_interrupt` implementation
   *
   * @return interrupt - object implementing the `hal::can_interrupt` interface
   * for this can peripheral.
   */
  interrupt acquire_interrupt();

  /**
   * @brief Acquire a set of 4x standard identifier filters
   *
   * @return identifier_filter_set - A set of 4x identifier filters. When
   * destroyed, releases the filter resource it held on to.
   */
  identifier_filter_set acquire_identifier_filter(
    fifo_assignment p_fifo = fifo_assignment::fifo1);

  /**
   * @brief Acquire a pair of two extended identifier filters
   *
   * @return extended_identifier_filter_set - A set of 2x extended identifier
   * filters.
   */
  extended_identifier_filter_set acquire_extended_identifier_filter(
    fifo_assignment p_fifo = fifo_assignment::fifo1);

  /**
   * @brief Acquire a pair of mask filters
   *
   * @return mask_filter_set - A set of 2x standard mask filters
   */
  mask_filter_set acquire_mask_filter(
    fifo_assignment p_fifo = fifo_assignment::fifo1);

  /**
   * @brief An extended mask filters
   *
   * @return extended_mask_filter - An extended mask filter
   */
  extended_mask_filter acquire_extended_mask_filter(
    fifo_assignment p_fifo = fifo_assignment::fifo1);

  ~can_peripheral_manager();
};
}  // namespace hal::stm32f1
