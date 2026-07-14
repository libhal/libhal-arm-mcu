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

#include <array>

#include <libhal-arm-mcu/stm32_generic/uart.hpp>
#include <libhal-arm-mcu/stm32f411/clock.hpp>
#include <libhal-arm-mcu/stm32f411/constants.hpp>
#include <libhal-arm-mcu/stm32f411/pin.hpp>
#include <libhal-arm-mcu/stm32f411/uart.hpp>
#include <libhal/units.hpp>

#include "dma.hpp"
#include "power.hpp"

namespace hal::stm32f411 {
namespace {
inline auto usart1 = reinterpret_cast<void*>(0x4001'1000);
inline auto usart2 = reinterpret_cast<void*>(0x4000'4400);
inline auto usart6 = reinterpret_cast<void*>(0x4001'1400);
}  // namespace
uart::uart(hal::runtime,
           std::uint8_t p_port,
           std::span<hal::byte> p_buffer,
           serial::settings const& p_settings)
  : uart(p_port, p_buffer, p_settings)
{
}

uart::uart(std::uint8_t p_port,
           std::span<hal::byte> p_buffer,
           serial::settings const& p_settings)
  : m_stm32_uart(usart1, p_buffer)
  , m_dma(peripheral::dma2)
  , m_id{}
{
  if (p_buffer.size() > max_dma_length) {
    hal::safe_throw(hal::operation_not_supported(this));
  }
  std::array<dma_channel_stream_t, 2> possible_streams;
  switch (p_port) {
    case 1:
      m_id = peripheral::usart1;
      m_dma = peripheral::dma2;
      m_stm32_uart = stm32_generic::uart(usart1, p_buffer);
      {
        pin tx(peripheral::gpio_a, 9);
        pin rx(peripheral::gpio_a, 10);
        tx.function(pin::pin_function::alternate7)
          .open_drain(false)
          .resistor(pin_resistor::none);
        rx.function(pin::pin_function::alternate7)
          .open_drain(false)
          .resistor(pin_resistor::none);
      }
      possible_streams = { dma_channel_stream_t{ .stream = 2, .channel = 4 },
                           dma_channel_stream_t{ .stream = 5, .channel = 4 } };
      break;
    case 2:
      m_dma = peripheral::dma1;
      m_id = peripheral::usart2;
      m_stm32_uart = stm32_generic::uart(usart2, p_buffer);
      {
        pin tx(peripheral::gpio_a, 2);
        pin rx(peripheral::gpio_a, 3);
        tx.function(pin::pin_function::alternate7)
          .open_drain(false)
          .resistor(pin_resistor::none);
        rx.function(pin::pin_function::alternate7)
          .open_drain(false)
          .resistor(pin_resistor::none);
      }
      possible_streams = { dma_channel_stream_t{ .stream = 5, .channel = 4 },
                           dma_channel_stream_t{ .stream = 7, .channel = 6 } };
      break;
    case 6:
      m_dma = peripheral::dma2;
      m_id = peripheral::usart6;
      m_stm32_uart = stm32_generic::uart(usart6, p_buffer);
      {
        pin tx(peripheral::gpio_a, 11);
        pin rx(peripheral::gpio_a, 12);
        tx.function(pin::pin_function::alternate8)
          .open_drain(false)
          .resistor(pin_resistor::none);
        rx.function(pin::pin_function::alternate8)
          .open_drain(false)
          .resistor(pin_resistor::none);
      }
      possible_streams = { dma_channel_stream_t{ .stream = 2, .channel = 5 },
                           dma_channel_stream_t{ .stream = 1, .channel = 5 } };
      break;
    default:
      hal::safe_throw(hal::operation_not_supported(this));
  }
  // Power on the usart/uart id
  power(m_id).on();

  /// configure dma here
  dma_settings_t dma_setting = {
    .source = m_stm32_uart.data_register(),
    .destination = p_buffer.data(),

    .transfer_length = p_buffer.size(),
    .flow_controller = dma_flow_controller::dma_controls_flow,
    .transfer_type = dma_transfer_type::peripheral_to_memory,
    .circular_mode = true,
    .peripheral_address_increment = false,
    .memory_address_increment = true,
    .peripheral_data_size = dma_transfer_size::byte,
    .memory_data_size = dma_transfer_size::byte,
    .priority_level = dma_priority_level::high,
    .double_buffer_mode = false,
    .current_target = false,
    .peripheral_burst_size = dma_burst_size::single_transfer,
    .memory_burst_size = dma_burst_size::single_transfer
  };

  m_dma_stream =
    setup_dma_transfer(m_dma, possible_streams, dma_setting, []() {}).stream;
  driver_configure(p_settings);
}

std::uint32_t uart::dma_cursor_position()
{
  std::uint32_t receive_amount =
    get_dma_reg(m_dma)->stream[m_dma_stream].transfer_count;
  std::uint32_t write_position = m_stm32_uart.buffer_size() - receive_amount;
  return write_position % m_stm32_uart.buffer_size();
}

void uart::driver_configure(serial::settings const& p_settings)
{
  m_stm32_uart.configure(p_settings, frequency(m_id));
}

serial::read_t uart::driver_read(std::span<hal::byte> p_data)
{
  return m_stm32_uart.uart_read(p_data, dma_cursor_position());
}
serial::write_t uart::driver_write(std::span<hal::byte const> p_data)
{
  return m_stm32_uart.uart_write(p_data);
}

void uart::driver_flush()
{
  m_stm32_uart.flush(dma_cursor_position());
}
}  // namespace hal::stm32f411
