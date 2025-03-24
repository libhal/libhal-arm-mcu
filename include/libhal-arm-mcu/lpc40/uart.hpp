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

#include <libhal/serial.hpp>
#include <libhal/zero_copy_serial.hpp>
#include <nonstd/ring_span.hpp>

#include "constants.hpp"
#include "pin.hpp"

namespace hal::lpc40 {
/**
 * @brief Implementation of the UART peripheral for the LPC40xx family of
 * microcontrollers.
 * @deprecated Use `hal::uart_manager` instead
 *
 * @note that the baud rates less than or equal to the peripheral clock
 * frequency / 48. Otherwise this peripheral cannot guarantee proper
 * transmission or receive of bytes.
 */
class uart final : public hal::serial
{
public:
  /// Port contains all of the information that the lpc40 uart port needs to
  /// operate.
  struct port
  {
    /// Resource ID of the UART peripheral to power on at initialization.
    peripheral id;
    /// Interrupt request number
    irq irq_number;
    /// Reference to a uart transmitter pin
    pin tx;
    /// Reference to a uart receiver pin
    pin rx;
    /// Function code to set the transmit pin to uart transmitter
    std::uint8_t tx_function;
    /// Function code to set the receive pin to uart receiver
    std::uint8_t rx_function;
  };

  /**
   * @brief Retrieve a UART serial port
   *
   * @param p_port_number - which uart port number to return
   * @param p_receive_working_buffer - uart serial receive working buffer
   * @param p_settings - the initial settings for the uart driver
   */
  uart(std::uint8_t p_port_number,
       std::span<hal::byte> p_receive_working_buffer,
       serial::settings const& p_settings = {});
  /**
   * @brief Construct a new uart object
   *
   * @param p_port
   * @param p_receive_working_buffer
   * @param p_settings
   */
  uart(uart::port const& p_port,
       std::span<hal::byte> p_receive_working_buffer,
       serial::settings const& p_settings = {});

  uart(uart const& p_other) = delete;
  uart& operator=(uart const& p_other) = delete;
  uart(uart&& p_other) noexcept = delete;
  uart& operator=(uart&& p_other) noexcept = delete;
  virtual ~uart() = default;

private:
  void driver_configure(settings const& p_settings) override;
  write_t driver_write(std::span<hal::byte const> p_data) override;
  read_t driver_read(std::span<hal::byte> p_data) override;
  void driver_flush() override;

  void setup_receive_interrupt();
  void interrupt_handler();

  port m_port;
  nonstd::ring_span<hal::byte> m_receive_buffer;
};

class uart_manager_impl
{
public:
  class serial;
  class zero_copy_serial;

protected:
  uart_manager_impl(peripheral p_id);

  peripheral p_id;
};

template<peripheral id>
class uart_manager : public uart_manager_impl
{
  uart_manager()
    : uart_manager_impl(id)
  {
  }
};

class uart_manager_impl::serial : public hal::serial
{
public:
  serial(serial const& p_other) = delete;
  serial& operator=(serial const& p_other) = delete;
  serial(serial&& p_other) noexcept = default;
  serial& operator=(serial&& p_other) noexcept = default;

private:
  serial();
  ~serial() override = default;
  void driver_configure(settings const& p_settings) override;
  write_t driver_write(std::span<byte const> p_data) override;
  read_t driver_read(std::span<byte> p_data) override;
  void driver_flush() override;
};

class uart_manager_impl::zero_copy_serial : public hal::zero_copy_serial
{
public:
  zero_copy_serial(zero_copy_serial const& p_other) = delete;
  zero_copy_serial& operator=(zero_copy_serial const& p_other) = delete;
  zero_copy_serial(zero_copy_serial&& p_other) noexcept = default;
  zero_copy_serial& operator=(zero_copy_serial&& p_other) noexcept = default;

private:
  zero_copy_serial();
  ~zero_copy_serial() override = default;
  void driver_configure(hal::serial::settings const& p_settings) override;
  void driver_write(std::span<hal::byte const> p_data) override;
  std::span<hal::byte const> driver_receive_buffer() override;
  std::size_t driver_cursor() override;
};
}  // namespace hal::lpc40
