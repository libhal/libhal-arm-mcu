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

#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-util/bit.hpp>
#include <libhal/serial.hpp>
#include <libhal/units.hpp>

#include "dma.hpp"

namespace hal::stm32f1 {

/// Namespace for the status registers (SR) bit masks
struct status_reg  // NOLINT
{
  /// Indicates if the transmit data register is empty and can be loaded with
  /// another byte.
  static constexpr auto transit_empty = hal::bit_mask::from<7>();
};

/// Namespace for the control registers (CR1, CR3) bit masks and predefined
/// settings constants.
struct control_reg  // NOLINT
{
  /// When this bit is cleared the USART prescalers and outputs are stopped
  /// and the end of the current byte transfer in order to reduce power
  /// consumption. (CR1)
  static constexpr auto usart_enable = hal::bit_mask::from<13>();

  /// Enables DMA receiver (CR3)
  static constexpr auto dma_receiver_enable = hal::bit_mask::from<6>();

  /// This bit enables the transmitter. (CR1)
  static constexpr auto transmitter_enable = hal::bit_mask::from<3>();

  /// This bit enables the receiver. (CR1)
  static constexpr auto receive_enable = hal::bit_mask::from<2>();

  /// Enable USART + Enable Receive + Enable Transmitter
  static constexpr auto control_settings1 =
    hal::bit_value(0UL)
      .set<control_reg::usart_enable>()
      .set<control_reg::receive_enable>()
      .set<control_reg::transmitter_enable>()
      .to<std::uint16_t>();

  /// Make sure that DMA is enabled for receive only
  static constexpr auto control_settings3 =
    hal::bit_value(0UL)
      .set<control_reg::dma_receiver_enable>()
      .to<std::uint16_t>();
};

/// Namespace for the baud rate (BRR) registers bit masks
struct baud_rate_reg  // NOLINT
{
  /// Mantissa of USART DIV
  static constexpr auto mantissa = hal::bit_mask::from<4, 15>();

  /// Fraction of USART DIV
  static constexpr auto fraction = hal::bit_mask::from<0, 3>();
};

struct usart_t
{
  u32 volatile status;
  u32 volatile data;
  u32 volatile baud_rate;
  u32 volatile control1;
  u32 volatile control2;
  u32 volatile control3;
  u32 volatile guard_time_and_prescale;
};

inline hal::uptr peripheral_to_register(peripheral p_select)
{
  // See Chapter 3.3 "Memory" page 50 in RM0008 for these magic numbers
  switch (p_select) {
    case peripheral::usart1:
      return 0x4001'3800;
    case peripheral::usart2:
      return 0x4000'4400;
    case peripheral::usart3:
      return 0x4000'4800;
    case peripheral::uart4:
      return 0x4000'4C00;
    case peripheral::uart5:
      return 0x4000'5000;
    default:
      hal::safe_throw(hal::argument_out_of_domain(nullptr));
  }
}

static constexpr auto uart_dma_settings1 =
  hal::bit_value()
    .clear<dma::transfer_complete_interrupt_enable>()
    .clear<dma::half_transfer_interrupt_enable>()
    .clear<dma::transfer_error_interrupt_enable>()
    .clear<dma::data_transfer_direction>()  // Read from peripheral
    .set<dma::circular_mode>()
    .clear<dma::peripheral_increment_enable>()
    .set<dma::memory_increment_enable>()
    .clear<dma::memory_to_memory>()
    .set<dma::enable>()
    .insert<dma::peripheral_size, 0b00U>()   // size = 8 bits
    .insert<dma::memory_size, 0b00U>()       // size = 8 bits
    .insert<dma::channel_priority, 0b10U>()  // Low Medium [High] Very_High
    .to<std::uint32_t>();

inline usart_t* to_usart(void* p_uart)
{
  return reinterpret_cast<usart_t*>(p_uart);
}
inline usart_t* to_usart(uptr p_uart)
{
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  return reinterpret_cast<usart_t*>(p_uart);
}
}  // namespace hal::stm32f1
