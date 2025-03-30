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

#include <cstdint>

#include <libhal/can.hpp>

#include "constants.hpp"
#include "pin.hpp"

namespace hal::lpc40 {
class can final : public hal::can
{
public:
  /// Contains all of the information for to control and configure a CAN BUS bus
  /// on the LPC40xx platform.
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
    /// Number of time quanta for sync bits - 1
    std::uint8_t sync_jump = 0;
    /// Number of time quanta for tseg1 - 1
    std::uint8_t tseg1 = 6;
    /// Number of time quanta for tseg2 - 1
    std::uint8_t tseg2 = 1;
  };

  can(std::uint8_t p_port, can::settings const& p_settings = {});
  can(port const& p_port, can::settings const& p_settings = {});

  can(can const& p_other) = delete;
  can& operator=(can const& p_other) = delete;
  can(can&& p_other) noexcept = delete;
  can& operator=(can&& p_other) noexcept = delete;
  virtual ~can();

private:
  void driver_configure(settings const& p_settings) override;
  void driver_bus_on() override;
  void driver_send(message_t const& p_message) override;

  void setup(can::port const& p_port, can::settings const& p_settings);
  void configure_baud_rate(can::port const& p_port,
                           can::settings const& p_settings);
  /**
   * @note This interrupt handler is used by both CAN1 and CAN2. This should
   *     only be called for a single CAN port to service both receive
   * handlers.
   */
  void driver_on_receive(
    hal::callback<can::handler> p_receive_handler) override;

  port m_port;
  hal::callback<can::handler> m_receive_handler;
};
}  // namespace hal::lpc40
