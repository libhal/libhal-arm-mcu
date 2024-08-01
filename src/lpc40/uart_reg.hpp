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

#include <array>
#include <cstdint>

#include <libhal-util/bit.hpp>

namespace hal::lpc40 {

/// peripheral register map
struct uart_reg_t
{
  /// Union of registers overlapping offset address 0x000
  union union1
  {
    /// (R/ ) Contains the next received character to be read
    const volatile uint8_t receive_buffer;
    /// ( /W) The next character to be transmitted is written here (DLAB = 0)
    volatile uint8_t transmit_buffer;
    /// (R/W) Least significant byte of the baud rate divisor value. The full
    /// divisor is used to generate a baud rate from the fractional rate
    /// divider (DLAB = 1)
    volatile uint8_t divisor_latch_lsb;
    /// Simply here to expand the size of the first union to 32-bits
    uint32_t reserved0;
  };
  /// Offset: 0x000 Registers overlapping offset address 0x000
  union1 group1;
  /// Union of registers overlapping offset address 0x004
  union union2
  {
    /// (R/W) Most significant byte of the baud rate divisor value. The full
    /// divisor is used to generate a baud rate from the fractional rate
    /// divider (DLAB = 1)
    volatile uint8_t divisor_latch_msb;
    /// (R/W) Contains individual interrupt enable bits for the 7 potential
    /// UART interrupts (DLAB =0).
    volatile uint32_t interrupt_enable;
  };
  /// Offset: 0x004 Registers overlapping offset address 0x004
  union2 group2;
  /// Union of registers overlapping offset address 0x008
  union union3
  {
    /// (R/ ) Identifies which interrupt(s) are pending.
    const volatile uint32_t interrupt_id;
    /// ( /W) Controls UART FIFO usage and modes.
    volatile uint8_t fifo_control;
  };
  /// Offset: 0x008 Registers overlapping offset address 0x000
  union3 group3;
  /// Offset: 0x00C (R/W) Contains controls for frame formatting and break
  /// generation
  volatile uint8_t line_control;
  /// reserved 1
  std::array<uint8_t, 7> reserved1;
  /// Offset: 0x014 (R/ ) Contains flags for transmit and receive status,
  /// including line errors
  const volatile uint8_t line_status;
  /// reserved 2
  std::array<uint8_t, 7> reserved2;
  /// Offset: 0x01C (R/W) 8-bit temporary storage for software
  volatile uint8_t scratch_pad;
  /// reserved 3
  std::array<uint8_t, 3> reserved3;
  /// Offset: 0x020 (R/W) Contains controls for the auto-baud feature.
  volatile uint32_t autobaud_control;
  /// Offset:
  volatile uint8_t icr;
  /// reserved 4
  std::array<uint8_t, 3> reserved4;
  /// Offset: 0x028 (R/W) Generates a clock input for the baud rate divider.
  volatile uint8_t fractional_divider;
  /// reserved 5
  std::array<uint8_t, 7> reserved5;
  /// Offset: 0x030 (R/W) Turns off UART transmitter for use with software
  /// flow control.
  volatile uint8_t transmit_enable;
};

/// Line control bit fields
namespace uart_line_control {
/// Word Length Select: Reset = 0
/// - 0x0 5-bit character
/// - 0x1 6-bit character
/// - 0x2 7-bit character
/// - 0x3 8-bit character
static constexpr auto word_length = bit_mask::from<0, 1>();
/// Stop Bit Select: Reset 0
/// - 0 1 stop bit.
/// - 1 2 stop bits. (1.5 if UnLCR[1:0]=00).)
static constexpr auto stop = bit_mask::from<2>();
/// Parity Enable: Reset 0
/// - 0 Disable parity generation and checking.
/// - 1 Enable parity generation and checking.
static constexpr auto parity_enable = bit_mask::from<3>();
/// Parity Select 0
/// - 0x0 Odd parity. Number of 1s in the transmitted character and the
///   attached parity bit will be odd.
/// - 0x1 Even Parity. Number of 1s in the transmitted character and the
///   attached parity bit will be even.
/// - 0x2 Forced 1 stick parity.
/// - 0x3 Forced 0 stick parity.
static constexpr auto parity = bit_mask::from<4, 5>();
};  // namespace uart_line_control

/// Interrupt enable bit fields
namespace uart_interrupt_enable {
/// RBR Interrupt Enable. Enables the Receive Data Available interrupt for
/// UARTn: Reset 0 It also controls the Character Receive Time-out
/// interrupt.
/// - 0 Disable the RDA interrupts.
/// - 1 Enable the RDA interrupts.
static constexpr auto receive_interrupt = bit_mask::from<0>();
};  // namespace uart_interrupt_enable

/// Interrupt ID bit fields
namespace uart_interrupt_id {
/// Interrupt identification. UnIER[3:1] identifies an interrupt
/// corresponding to the UARTn Rx or TX FIFO. All other combinations of
/// UnIER[3:1] not listed below are reserved (000,100,101,111).
/// - 0x3 1 - Receive Line Status (RLS).
/// - 0x2 2a - Receive Data Available (RDA).
/// - 0x6 2b - Character Time-out Indicator (CTI).
/// - 0x1 3 - THRE Interrupt
static constexpr auto id = bit_mask::from<1, 3>();
};  // namespace uart_interrupt_id

/// FIFO control bit fields
namespace uart_fifo_control {
/// FIFO Enable: Reset 0
/// - 0 UARTn FIFOs are disabled. Must not be used in the application.
/// - 1 Active high enable for both UARTn Rx and TX FIFOs and UnFCR[7:1]
/// access. This bit must be set for proper UART operation. Any transition
/// on this bit will automatically clear the related UART FIFOs.
static constexpr auto fifo_enable = bit_mask::from<0>();
/// RX FIFO Reset: Reset 0
/// - 0 No impact on either of UARTn FIFOs.
/// - 1 Writing a logic 1 to UnFCR[1] will clear all bytes in UARTn Rx FIFO,
/// reset the pointer logic. This bit is self-clearing.
static constexpr auto rx_fifo_clear = bit_mask::from<1>();
/// TX FIFO Reset: Reset 0
/// - 0 No impact on either of UARTn FIFOs.
/// - 1 Writing a logic 1 to UnFCR[2] will clear all bytes in UARTn TX FIFO,
/// reset the pointer logic. This bit is self-clearing.
static constexpr auto tx_fifo_clear = bit_mask::from<2>();
/// RX Trigger Level. These two bits determine how many receiver UARTn FIFO
/// characters must be written before an interrupt or DMA request is
/// activated: Reset 0
/// - 0x0 Trigger level 0 (1 character or 0x01).
/// - 0x1 Trigger level 1 (4 characters or 0x04).
/// - 0x2 Trigger level 2 (8 characters or 0x08).
/// - 0x3 Trigger level 3 (14 characters or 0x0E).
static constexpr auto rx_trigger_level = bit_mask::from<6, 7>();
};  // namespace uart_fifo_control

inline uart_reg_t* uart_reg0 = reinterpret_cast<uart_reg_t*>(0x4000'C000);
inline uart_reg_t* uart_reg1 = reinterpret_cast<uart_reg_t*>(0x4001'0000);
inline uart_reg_t* uart_reg2 = reinterpret_cast<uart_reg_t*>(0x4008'8000);
inline uart_reg_t* uart_reg3 = reinterpret_cast<uart_reg_t*>(0x4009'C000);
inline uart_reg_t* uart_reg4 = reinterpret_cast<uart_reg_t*>(0x400A'4000);
}  // namespace hal::lpc40