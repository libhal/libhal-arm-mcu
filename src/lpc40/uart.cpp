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

#include <libhal-arm-mcu/lpc40/uart.hpp>

#include <cstdint>
#include <span>

#include <libhal-arm-mcu/lpc40/clock.hpp>
#include <libhal-arm-mcu/lpc40/constants.hpp>
#include <libhal-arm-mcu/lpc40/interrupt.hpp>
#include <libhal-arm-mcu/lpc40/power.hpp>
#include <libhal-armcortex/interrupt.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/enum.hpp>
#include <libhal-util/static_callable.hpp>

#include "uart_clock.hpp"
#include "uart_reg.hpp"

namespace hal::lpc40 {

uart_reg_t* get_uart_reg(peripheral p_id)
{
  switch (p_id) {
    case peripheral::uart0:
      return uart_reg0;
    case peripheral::uart1:
      return uart_reg1;
    case peripheral::uart2:
      return uart_reg2;
    case peripheral::uart3:
      return uart_reg3;
    case peripheral::uart4:
    default:
      return uart_reg4;
  }
}

void configure_baud_rate(uart_reg_t* p_reg, uart_baud_t p_calibration)
{
  static constexpr auto divisor_access = bit_mask::from<7>();

  auto divisor_latch_msb =
    static_cast<uint8_t>((p_calibration.divider >> 8) & 0xFF);
  auto divisor_latch_lsb = static_cast<uint8_t>(p_calibration.divider & 0xFF);
  auto fractional_divider = static_cast<uint8_t>(
    (p_calibration.numerator & 0xF) | (p_calibration.denominator & 0xF) << 4);

  bit_modify(p_reg->line_control).set(divisor_access);
  p_reg->group1.divisor_latch_lsb = divisor_latch_lsb;
  p_reg->group2.divisor_latch_msb = divisor_latch_msb;
  p_reg->fractional_divider = fractional_divider;
  bit_modify(p_reg->line_control).clear(divisor_access);
}

uint8_t get_line_control(serial::settings const& p_settings)
{
  bit_value<std::uint8_t> line_control_object(0);

  // Set stop bit length
  switch (p_settings.stop) {
    case serial::settings::stop_bits::one:
      line_control_object.clear<uart_line_control::stop>();
      break;
    case serial::settings::stop_bits::two:
      line_control_object.set<uart_line_control::stop>();
      break;
  }

  // Set frame size to 8 = 0x3
  line_control_object.insert<uart_line_control::word_length>(0x3U);

  // Preset the parity enable and disable it if the parity is set to none
  line_control_object.set<uart_line_control::parity_enable>();

  // Set frame parity
  switch (p_settings.parity) {
    case serial::settings::parity::odd:
      line_control_object.insert<uart_line_control::parity>(0x0U);
      break;
    case serial::settings::parity::even:
      line_control_object.insert<uart_line_control::parity>(0x1U);
      break;
    case serial::settings::parity::forced1:
      line_control_object.insert<uart_line_control::parity>(0x2U);
      break;
    case serial::settings::parity::forced0:
      line_control_object.insert<uart_line_control::parity>(0x3U);
      break;
    case serial::settings::parity::none:
      // Turn off parity if the parity is set to none
      line_control_object.clear<uart_line_control::parity_enable>();
      break;
  }

  return line_control_object.get();
}

void reset_uart_queue(uart_reg_t* p_reg)
{
  bit_modify(p_reg->group3.fifo_control)
    .set<uart_fifo_control::rx_fifo_clear>()
    .set<uart_fifo_control::tx_fifo_clear>();
}

inline bool has_data(uart_reg_t* p_reg)
{
  return bit_extract<bit_mask::from<0U>()>(p_reg->line_status);
}

void uart::interrupt_handler()
{
  auto* reg = get_uart_reg(m_port.id);
  [[maybe_unused]] auto line_status_value = reg->line_status;
  auto interrupt_type =
    bit_extract<uart_interrupt_id::id>(reg->group3.interrupt_id);
  if (interrupt_type == 0x2 || interrupt_type == 0x6) {
    while (has_data(reg)) {
      hal::byte new_byte{ reg->group1.receive_buffer };
      if (!m_receive_buffer.full()) {
        m_receive_buffer.push_back(hal::byte{ new_byte });
      }
    }
  }
}

void uart::setup_receive_interrupt()
{
  auto* reg = get_uart_reg(m_port.id);
  // Create a lambda to call the interrupt() method
  auto isr = [this]() { interrupt_handler(); };

  // A pointer to save the static_callable isr address to.
  cortex_m::interrupt_pointer handler;

  switch (m_port.irq_number) {
    case irq::uart0:
      handler = static_callable<uart, 0, void(void)>(isr).get_handler();
      break;
    case irq::uart1:
      handler = static_callable<uart, 1, void(void)>(isr).get_handler();
      break;
    case irq::uart2:
      handler = static_callable<uart, 2, void(void)>(isr).get_handler();
      break;
    case irq::uart3:
      handler = static_callable<uart, 3, void(void)>(isr).get_handler();
      break;
    case irq::uart4:
    default:
      handler = static_callable<uart, 4, void(void)>(isr).get_handler();
      break;
  }

  // Enable interrupt service routine.
  cortex_m::enable_interrupt(m_port.irq_number, handler);

  // Enable uart interrupt signal
  bit_modify(reg->group2.interrupt_enable)
    .set<uart_interrupt_enable::receive_interrupt>();
  // 0x3 = 14 bytes in fifo before triggering a receive interrupt.
  // 0x2 = 8
  // 0x1 = 4
  // 0x0 = 1
  bit_modify(reg->group3.fifo_control)
    .insert<uart_fifo_control::rx_trigger_level>(0x3U);
}

uart::uart(std::uint8_t p_port_number,
           std::span<hal::byte> p_receive_working_buffer,
           serial::settings const& p_settings)
  : m_port{}
  , m_receive_buffer(p_receive_working_buffer.begin(),
                     p_receive_working_buffer.end())
{
  if (p_port_number == 0) {
    // NOTE: required since LPC_UART0 is of type LPC_UART0_TypeDef in
    // lpc17xx
    // and LPC_UART_TypeDef in lpc40xx causing a "useless cast" warning when
    // compiled for, some odd reason, for either one being compiled, which
    // would make more sense if it only warned us with lpc40xx.
    m_port = uart::port{
      .id = peripheral::uart0,
      .irq_number = irq::uart0,
      .tx = pin(0, 2),
      .rx = pin(0, 3),
      .tx_function = 0b001,
      .rx_function = 0b001,
    };
  } else if (p_port_number == 1) {
    m_port = uart::port{
      .id = peripheral::uart1,
      .irq_number = irq::uart1,
      .tx = pin(2, 0),
      .rx = pin(2, 1),
      .tx_function = 0b010,
      .rx_function = 0b010,
    };
  } else if (p_port_number == 2) {
    m_port = uart::port{
      .id = peripheral::uart2,
      .irq_number = irq::uart2,
      .tx = pin(2, 8),
      .rx = pin(2, 9),
      .tx_function = 0b010,
      .rx_function = 0b010,
    };
  } else if (p_port_number == 3) {
    m_port = uart::port{
      .id = peripheral::uart3,
      .irq_number = irq::uart3,
      .tx = pin(4, 28),
      .rx = pin(4, 29),
      .tx_function = 0b010,
      .rx_function = 0b010,
    };
  } else if (p_port_number == 4) {
    m_port = uart::port{
      .id = peripheral::uart4,
      .irq_number = irq::uart4,
      .tx = pin(1, 28),
      .rx = pin(2, 9),
      .tx_function = 0b101,
      .rx_function = 0b011,
    };
  } else {
    // "Support UART ports for LPC40xx are UART0, UART2, UART3, and UART4.";
    hal::safe_throw(hal::operation_not_supported(this));
  }

  initialize_interrupts();
  uart::driver_configure(p_settings);
}

uart::uart(uart::port const& p_port,
           std::span<hal::byte> p_receive_working_buffer,
           serial::settings const& p_settings)
  : m_port(p_port)
  , m_receive_buffer(p_receive_working_buffer.begin(),
                     p_receive_working_buffer.end())
{
  initialize_interrupts();
  uart::driver_configure(p_settings);
}

void uart::driver_configure(settings const& p_settings)
{
  auto* reg = get_uart_reg(m_port.id);

  // Validate the settings before configuring any hardware
  auto baud_rate = static_cast<std::uint32_t>(p_settings.baud_rate);
  auto uart_frequency = get_frequency(m_port.id);
  auto uart_frequency_hz = static_cast<std::uint32_t>(uart_frequency);
  auto baud_settings = calculate_baud(baud_rate, uart_frequency_hz);

  // For proper operation of the UART port, the divider must be greater than 2
  // If it is not the cause that means that the baud rate is too high for this
  // device.
  if (baud_settings.divider <= 2) {
    safe_throw(hal::operation_not_supported(this));
  }

  // Power on UART peripheral
  power_on(m_port.id);

  // Enable fifo for receiving bytes and to enable full access of the FCR
  // register.
  bit_modify(reg->group3.fifo_control).set<uart_fifo_control::fifo_enable>();
  reg->line_control = get_line_control(p_settings);

  configure_baud_rate(reg, baud_settings);

  pin(m_port.tx).function(m_port.tx_function);
  pin(m_port.rx)
    .function(m_port.rx_function)
    .resistor(hal::pin_resistor::pull_up);

  setup_receive_interrupt();

  // Clear the buffer
  uart::driver_flush();

  // Reset the UART queues
  reset_uart_queue(reg);
}

bool finished_sending(uart_reg_t* p_reg)
{
  return bit_extract<bit_mask::from<5U>()>(p_reg->line_status);
}

serial::write_t uart::driver_write(std::span<hal::byte const> p_data)
{
  auto* reg = get_uart_reg(m_port.id);

  for (auto const& byte : p_data) {
    reg->group1.transmit_buffer = byte;
    while (!finished_sending(reg)) {
      continue;
    }
  }
  return write_t{ .data = p_data };
}

serial::read_t uart::driver_read(std::span<hal::byte> p_data)
{
  size_t count = 0;
  for (auto& byte : p_data) {
    if (m_receive_buffer.empty()) {
      break;
    }

    byte = m_receive_buffer.pop_front();
    count++;
  }

  return read_t{
    .data = p_data.subspan(0, count),
    .available = m_receive_buffer.size(),
    .capacity = m_receive_buffer.capacity(),
  };
}

void uart::driver_flush()
{
  while (!m_receive_buffer.empty()) {
    m_receive_buffer.pop_back();
  }
}
}  // namespace hal::lpc40
