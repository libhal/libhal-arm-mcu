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

#include <cmath>

#include <libhal-arm-mcu/stm32f1/clock.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32f1/uart.hpp>
#include <libhal-util/bit.hpp>
#include <libhal/error.hpp>

#include "dma.hpp"
#include "pin.hpp"
#include "power.hpp"
#include "usart_reg.hpp"

namespace hal::stm32f1 {
namespace {
void configure_baud_rate(usart_t& p_usart,
                         peripheral p_peripheral,
                         serial::settings const& p_settings)
{
  auto const clock_frequency = frequency(p_peripheral);
  float usart_divider = clock_frequency / (16.0f * p_settings.baud_rate);

  // Truncate off the decimal values
  auto mantissa = static_cast<uint16_t>(usart_divider);
  // Subtract the whole number to leave just the decimal
  auto fraction = usart_divider - static_cast<float>(mantissa);
  auto fractional_int = static_cast<uint16_t>(std::roundf(fraction * 16));

  if (fractional_int >= 16) {
    mantissa = static_cast<uint16_t>(mantissa + 1U);
    fractional_int = 0;
  }

  p_usart.baud_rate = hal::bit_value()
                        .insert<baud_rate_reg::mantissa>(mantissa)
                        .insert<baud_rate_reg::fraction>(fractional_int)
                        .to<std::uint16_t>();
}

void configure_format(usart_t& p_usart, serial::settings const& p_settings)
{
  constexpr auto parity_selection = bit_mask::from<9>();
  constexpr auto parity_control = bit_mask::from<10>();
  constexpr auto word_length = bit_mask::from<12>();
  constexpr auto stop = bit_mask::from<12, 13>();

  bool parity_enable = (p_settings.parity != serial::settings::parity::none);
  bool parity = (p_settings.parity == serial::settings::parity::odd);
  bool double_stop = (p_settings.stop == serial::settings::stop_bits::two);
  std::uint16_t stop_value = (double_stop) ? 0b10U : 0b00U;

  // Parity codes are: 0 for Even and 1 for Odd, thus the expression above
  // sets the bool to TRUE when odd and zero when something else. This value
  // is ignored if the parity is NONE since parity_enable will be zero.

  bit_modify(p_usart.control1)
    .insert<parity_control>(parity_enable)
    .insert<parity_selection>(parity)
    .insert<word_length>(0U);

  bit_modify(p_usart.control2).insert<stop>(stop_value);
}
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
  : m_uart(nullptr)
  , m_receive_buffer(p_buffer)
  , m_read_index(0)
  , m_dma(0)
  , m_id{}
{
  if (p_buffer.size() > max_dma_length) {
    hal::safe_throw(hal::operation_not_supported(this));
  }

  switch (p_port) {
    case 1:
      m_port_tx = 'A';
      m_pin_tx = 9;
      m_port_rx = 'A';
      m_pin_rx = 10;
      m_id = peripheral::usart1;
      m_dma = 5;
      break;
    case 2:
      m_port_tx = 'A';
      m_pin_tx = 2;
      m_port_rx = 'A';
      m_pin_rx = 3;
      m_dma = 6;
      m_id = peripheral::usart2;
      break;
    case 3:
      m_port_tx = 'B';
      m_pin_tx = 10;
      m_port_rx = 'B';
      m_pin_rx = 11;
      m_dma = 3;
      m_id = peripheral::usart3;
      break;
    default:
      hal::safe_throw(hal::operation_not_supported(this));
  }

  m_uart = reinterpret_cast<void*>(peripheral_to_register(m_id));  // NOLINT

  // Power on the usart/uart id
  power_on(m_id);

  // Power on dma1 which has the usart channels
  // TODO(): DMA1 is shared across multiple peripherals
  if (not is_on(peripheral::dma1)) {
    power_on(peripheral::dma1);
  }

  auto& uart_reg = *to_usart(m_uart);

  // Setup RX DMA channel
  auto const data_address = reinterpret_cast<iptr>(&uart_reg.data);
  auto const queue_address = reinterpret_cast<iptr>(p_buffer.data());
  auto const data_address_int = static_cast<u32>(data_address);
  auto const queue_address_int = static_cast<u32>(queue_address);

  dma::dma1->channel[m_dma - 1].transfer_amount = p_buffer.size();
  dma::dma1->channel[m_dma - 1].peripheral_address = data_address_int;
  dma::dma1->channel[m_dma - 1].memory_address = queue_address_int;
  dma::dma1->channel[m_dma - 1].configuration = uart_dma_settings1;

  // Setup UART Control Settings 1
  uart_reg.control1 = control_reg::control_settings1;

  // NOTE: We leave control settings 2 alone as it is for features beyond
  //       basic UART such as USART clock, USART port network (LIN), and other
  //       things.

  // Setup UART Control Settings 3
  uart_reg.control3 = control_reg::control_settings3;

  uart::driver_configure(p_settings);

  configure_pin({ .port = m_port_tx, .pin = m_pin_tx },
                push_pull_alternative_output);
  configure_pin({ .port = m_port_rx, .pin = m_pin_rx }, input_pull_up);
}

uart::~uart()
{
  reset_pin({ .port = m_port_tx, .pin = m_pin_tx });
  reset_pin({ .port = m_port_rx, .pin = m_pin_rx });
}

u32 uart::dma_cursor_position()
{
  u32 receive_amount = dma::dma1->channel[m_dma - 1].transfer_amount;
  u32 write_position = m_receive_buffer.size() - receive_amount;
  return write_position % m_receive_buffer.size();
}

void uart::driver_configure(serial::settings const& p_settings)
{
  auto& uart_reg = *to_usart(m_uart);
  configure_baud_rate(uart_reg, m_id, p_settings);
  configure_format(uart_reg, p_settings);
}

serial::write_t uart::driver_write(std::span<hal::byte const> p_data)
{
  auto& uart_reg = *to_usart(m_uart);

  for (auto const& byte : p_data) {
    while (not bit_extract<status_reg::transit_empty>(uart_reg.status)) {
      continue;
    }
    // Load the next byte into the data register
    uart_reg.data = byte;
  }

  return {
    .data = p_data,
  };
}

serial::read_t uart::driver_read(std::span<hal::byte> p_data)
{
  size_t count = 0;

  for (auto& byte : p_data) {
    if (m_read_index == dma_cursor_position()) {
      break;
    }
    byte = m_receive_buffer[m_read_index++];
    m_read_index = m_read_index % m_receive_buffer.size();
    count++;
  }

  return {
    .data = p_data.first(count),
    .available = 1,
    .capacity = m_receive_buffer.size(),
  };
}

void uart::driver_flush()
{
  m_read_index = dma_cursor_position();
}
}  // namespace hal::stm32f1
