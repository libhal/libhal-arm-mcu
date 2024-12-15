// Copyright 2024 Khalil Estell
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

#include <libhal/can.hpp>
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

class can_peripheral_manager final
{
public:
  class transceiver : public hal::can_transceiver
  {
  public:
    ~transceiver() override;

  private:
    friend class can_peripheral_manager;

    explicit transceiver(std::span<can_message> p_receive_buffer);
    u32 driver_baud_rate() override;
    void driver_send(can_message const& p_message) override;
    std::span<can_message const> driver_receive_buffer() override;
    std::size_t driver_receive_cursor() override;
  };

  class interrupt : public hal::can_interrupt
  {
  public:
    ~interrupt() override;

  private:
    friend class can_peripheral_manager;

    interrupt();

    void driver_on_receive(optional_receive_handler p_callback) override;
  };
  class bus_manager : public can_bus_manager
  {
  public:
    ~bus_manager() override;

  private:
    friend class can_peripheral_manager;

    void driver_baud_rate(hal::u32 p_hertz) override;
    void driver_filter_mode(accept p_accept) override;
    void driver_on_bus_off(optional_bus_off_handler p_callback) override;
    void driver_bus_on() override;
  };

  struct [[gnu::packed]] filter_resource
  {
    hal::u8 filter_index;
    hal::u8 word_index;
  };

  class identifier_filter : public can_identifier_filter
  {
  private:
    friend class can_peripheral_manager;

    explicit identifier_filter(filter_resource p_resource);
    void driver_allow(std::optional<u16> p_id) override;
    filter_resource m_resource;
  };

  class identifier_filter_set
  {
  public:
    std::array<identifier_filter, 4> filters;
    ~identifier_filter_set();

  private:
    friend class can_peripheral_manager;
    explicit identifier_filter_set(hal::u8 p_filter_index);
  };

  class extended_identifier_filter : public can_extended_identifier_filter
  {
  public:
    ~extended_identifier_filter() override;

  private:
    friend class can_peripheral_manager;

    explicit extended_identifier_filter(filter_resource p_resource);

    void driver_allow(std::optional<u32> p_id) override;
    filter_resource m_resource;
  };

  class mask_filter : public can_mask_filter
  {
  public:
    ~mask_filter() override;

  private:
    friend class can_peripheral_manager;

    explicit mask_filter(filter_resource p_resource);

    void driver_allow(std::optional<pair> p_filter_pair) override;

    filter_resource m_resource;
  };

  class extended_mask_filter : public can_extended_mask_filter
  {
  public:
    ~extended_mask_filter() override;

  private:
    friend class can_peripheral_manager;
    explicit extended_mask_filter(filter_resource p_resource);

    void driver_allow(std::optional<pair> p_filter_pair) override;

    filter_resource m_resource;
  };

  struct disable_ids
  {
    u16 standard = 0;
    u32 extended = 0;
  };

  /**
   * @brief Construct a new can peripheral manager object
   *
   * @param p_baud_rate - set baud rate of the device
   * @param p_pins - CAN bus RX and TX pin selection
   * @throw hal::operation_not_supported - if the baud rate is not usable
   */
  can_peripheral_manager(
    hal::u32 p_baud_rate,
    can_pins p_pins = can_pins::pa11_pa12,
    disable_ids p_disabled_ids = disable_ids{ .standard = 0, .extended = 0 });

  /**
   * @brief Enable/disable self test mode
   *
   * Self test mode allows message loopback. This mode allows messages sent from
   * this device's can transceiver to be received back by this device.
   *
   * @param p_enable - enable self test mode
   */
  void enable_self_test(bool p_enable);

  transceiver acquire_transceiver(std::span<can_message> p_receive_buffer);
  bus_manager acquire_bus_manager();
  interrupt acquire_interrupt();

  /**
   * @brief A set of 4x standard identifier filters
   *
   * @return identifier_filter_set - A set of 4x identifier filters. When
   * destroyed, releases the filter resource it held on to.
   */
  identifier_filter_set acquire_identifier_filter();

  /**
   * @brief A pair of two extended identifier filters
   *
   * @return std::array<extended_identifier_filter, 2> - 2x extended identifier
   * filters.
   */
  std::array<extended_identifier_filter, 2>
  acquire_extended_identifier_filter();

  std::array<mask_filter, 2> acquire_mask_filter();
  extended_mask_filter acquire_extended_mask_filter();

  ~can_peripheral_manager();

private:
  u32 m_baud_rate = 0;
};
}  // namespace hal::stm32f1
