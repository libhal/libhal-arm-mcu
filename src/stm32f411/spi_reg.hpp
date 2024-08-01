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

#include <cstdint>

#include <libhal-util/bit.hpp>

namespace hal::stm32f411 {
struct spi_reg_t
{
  /*!< Offset: 0x000 Control Register 1 (R/W) */
  uint32_t volatile cr1;
  /*!< Offset: 0x004 Control Register 2 (R/W) */
  uint32_t volatile cr2;
  /*!< Offset: 0x008 Status Register (R/W) */
  uint32_t volatile sr;
  /*!< Offset: 0x00C Data Register (R/W) */
  uint32_t volatile dr;
  /*!< Offset: 0x010 CRC polynomial register (R/ ) */
  uint32_t const volatile crcpr;
  /*!< Offset: 0x014 RX CRC register (R/W) */
  uint32_t volatile rxcrcr;
  /*!< Offset: 0x018 TX CRC Register (R/W) */
  uint32_t volatile txcrcr;
  /*!< Offset: 0x01C configuration register (R/W) */
  uint32_t volatile i2scfgr;
  /*!< Offset: 0x020 prescaler register (R/W) */
  uint32_t volatile i2spr;
};

/// SPI Control Register 1
struct control_register1
{
  /// 0: first clock transistion is the first capture edge
  /// 1: second clock transition is the first data capture edge
  static constexpr auto clock_phase = bit_mask::from<0>();

  /// 0: clock to 0 when idle
  /// 1: clock to 1 when idle
  static constexpr auto clock_polarity = bit_mask::from<1>();

  /// 0: slave, 1: master
  static constexpr auto master_selection = bit_mask::from<2>();

  /// baudrate control: sets the clock rate to:
  /// (peripheral clock frequency)/2**(n+1)
  static constexpr auto baud_rate_control = bit_mask::from<5, 3>();

  /// Peripheral Enable
  /// 0: disable, 1: enable
  static constexpr auto enable = bit_mask::from<6>();

  /// Frame Format
  /// 0: msb transmitted first
  /// 1: lsb tranmitted first
  static constexpr auto frame_format = bit_mask::from<7>();

  /// internal slave select
  static constexpr auto internal_slave_select = bit_mask::from<8>();

  /// Software slave management
  /// 0: disable, 1: enable
  static constexpr auto software_slave_management = bit_mask::from<9>();

  /// Recieve only
  /// 0: Full Duplex, 1: Output disable
  static constexpr auto rx_only = bit_mask::from<10>();

  /// Data frame format
  /// 0: 8-bits, 1: 16-bit
  static constexpr auto data_frame_format = bit_mask::from<11>();

  /// CRC transfer next
  /// 0: No CRC phase, 1: transfer CRC next
  static constexpr auto crc_transfer_next = bit_mask::from<12>();

  /// CRC enable
  /// 0: disable, 1: enable
  static constexpr auto crc_enable = bit_mask::from<13>();

  /// Output enable in bidirectional mode
  /// 0: output disabled, 1: output enable
  static constexpr auto bidirectional_output_enable = bit_mask::from<14>();

  /// Bidirectional data mode enable
  /// 0: full-duplex, 1: half-duplex
  static constexpr auto bidirectional_mode_enable = bit_mask::from<15>();
};

/// SPI Control Register 2
struct control_register2
{
  /// Rx buffer DMA enable
  static constexpr auto rx_dma_enable = bit_mask::from<0>();

  /// Tx buffer DMA enable
  static constexpr auto tx_dma_enable = bit_mask::from<1>();

  /// Slave select output enable
  /// 0: use a GPIO, 1: use the NSS pin
  static constexpr auto slave_select_output_enable = bit_mask::from<2>();

  /// Frame format
  /// 0: Motorola mode, 1: TI mode
  static constexpr auto frame_format = bit_mask::from<4>();

  /// Error interupt enable
  static constexpr auto error_interrupt_enable = bit_mask::from<5>();

  /// Rx buffer empty interrupt enable
  static constexpr auto rx_buffer_empty_interrupt_enable = bit_mask::from<6>();

  /// Tx buffer empty interrupt enable
  static constexpr auto tx_buffer_empty_interrupt_enable = bit_mask::from<7>();
};

/// SPI Status Register
struct status_register
{
  /// Recieve buffer not empty
  static constexpr auto rx_buffer_not_empty = bit_mask::from<0>();

  /// Transmit buffer not empty
  static constexpr auto tx_buffer_empty = bit_mask::from<1>();

  /// Channel side (i2s only)
  /// 0: left has been transmitted/recieved
  /// 1: right has been transmitted/recieved
  static constexpr auto i2s_channel_side = bit_mask::from<2>();

  /// Underrun flag
  static constexpr auto underrun_flag = bit_mask::from<3>();

  /// CRC error flag
  static constexpr auto crc_error_flag = bit_mask::from<4>();

  /// Mode fault flag
  static constexpr auto mode_fault_flag = bit_mask::from<5>();

  /// Overrun flag
  static constexpr auto overrun_flag = bit_mask::from<6>();

  /// Busy flag
  static constexpr auto busy_flag = bit_mask::from<7>();

  /// frame format error flag
  static constexpr auto frame_format_error_flag = bit_mask::from<8>();
};

inline constexpr intptr_t apb1_base = 0x4000'0000UL;
inline constexpr intptr_t apb2_base = 0x4001'0000UL;

inline spi_reg_t* spi_reg1 = reinterpret_cast<spi_reg_t*>(apb2_base + 0x3000);
inline spi_reg_t* spi_reg2 = reinterpret_cast<spi_reg_t*>(apb1_base + 0x3800);
inline spi_reg_t* spi_reg3 = reinterpret_cast<spi_reg_t*>(apb1_base + 0x3C00);
inline spi_reg_t* spi_reg4 = reinterpret_cast<spi_reg_t*>(apb2_base + 0x3400);
inline spi_reg_t* spi_reg5 = reinterpret_cast<spi_reg_t*>(apb2_base + 0x5000);
}  // namespace hal::stm32f411
