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

#include <libhal/initializers.hpp>
#include <libhal/serial.hpp>

namespace hal::stm32_generic {
class uart
{
public:
  uart(void* p_uart, std::span<hal::byte> p_receive_buffer);
  /**
   * @brief STM32 Common write function
   *
   * @param p_data Writes the data to the UART registers
   * @return serial::write_t
   */
  serial::write_t uart_write(std::span<hal::byte const> p_data);
  /**
   * @brief
   *
   * @param p_data Where the data gets copied to
   * @param p_dma_cursor_position Where the DMA is set to
   * @return serial::read_t
   */
  serial::read_t uart_read(std::span<hal::byte>& p_data,
                           u32 const& p_dma_cursor_position);
  /**
   * @brief Configures the serial port based on the settings
   *
   * @param p_settings - serial settings
   * @param p_frequency - frequency of the input clock to the peripheral
   */
  void configure(serial::settings const& p_settings, hertz p_frequency);

  u32 volatile* data_register();
  void flush(u32 p_dma_cursor_position);
  u32 buffer_size();

private:
  void configure_baud_rate(hal::hertz p_frequency,
                           serial::settings const& p_settings);
  void configure_format(serial::settings const& p_settings);

  void* m_uart;
  std::span<hal::byte> m_receive_buffer;
  std::uint16_t m_read_index;
};

class zero_copy_usart
{
public:
  zero_copy_usart(void* p_uart, std::span<hal::byte> p_receive_buffer);
  /**
   * @brief STM32 Common write function
   *
   * @param p_data Writes the data to the UART registers
   * @return serial::write_t
   */
  serial::write_t uart_write(std::span<hal::byte const> p_data);
  /**
   * @brief
   *
   * @param p_data Where the data gets copied to
   * @param p_dma_cursor_position Where the DMA is set to
   * @return serial::read_t
   */
  serial::read_t uart_read(std::span<hal::byte>& p_data,
                           u32 const& p_dma_cursor_position);
  /**
   * @brief Configures the serial port based on the settings
   *
   * @param p_settings - serial settings
   * @param p_frequency - frequency of the input clock to the peripheral
   */
  void configure(serial::settings const& p_settings, u32 p_frequency);

  u32 volatile* data_tx_register();
  bool tx_busy();
  u32 volatile* data_rx_register();

private:
  void configure_baud_rate(hal::hertz p_frequency,
                           serial::settings const& p_settings);
  void configure_format(serial::settings const& p_settings);

  void* m_uart;
  std::span<hal::byte> m_receive_buffer;
  std::uint16_t m_read_index;
};
}  // namespace hal::stm32_generic
