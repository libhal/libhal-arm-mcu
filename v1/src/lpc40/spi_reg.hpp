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

#include <libhal-util/bit.hpp>

namespace hal::lpc40 {
struct spi_reg_t
{
  /*!< Offset: 0x000 Control Register 0 (R/W) */
  uint32_t volatile cr0;
  /*!< Offset: 0x004 Control Register 1 (R/W) */
  uint32_t volatile cr1;
  /*!< Offset: 0x008 Data Register (R/W) */
  uint32_t volatile dr;
  /*!< Offset: 0x00C Status Register (R/ ) */
  uint32_t const volatile sr;
  /*!< Offset: 0x010 Clock Prescale Register (R/W) */
  uint32_t volatile cpsr;
  /*!< Offset: 0x014 Interrupt Mask Set and Clear Register (R/W) */
  uint32_t volatile imsc;
  /*!< Offset: 0x018 Raw Interrupt Status Register (R/W) */
  uint32_t volatile ris;
  /*!< Offset: 0x01C Masked Interrupt Status Register (R/W) */
  uint32_t volatile mis;
  /*!< Offset: 0x020 SSPICR Interrupt Clear Register (R/W) */
  uint32_t volatile icr;
  /*!< Offset: 0x024 SSPnDMACR DMA control register (R/W) */
  uint32_t volatile dmacr;
};

/// SSPn Control Register 0
struct control_register0  // NOLINT
{
  /// Data Size Select. This field controls the number of bits transferred in
  /// each frame. Values 0000-0010 are not supported and should not be used.
  static constexpr auto data_bit = bit_mask::from<0, 3>();

  /// Frame Format bitmask.
  /// 00 = SPI, 01 = TI, 10 = Microwire, 11 = Invalid
  static constexpr auto frame_bit = bit_mask::from<4, 5>();

  /// If bit is set to 0 SSP controller maintains the bus clock low between
  /// frames.
  ///
  /// If bit is set to 1 SSP controller maintains the bus clock high between
  /// frames.
  static constexpr auto polarity_bit = bit_mask::from<6>();

  /// If bit is set to 0 SSP controller captures serial data on the first
  /// clock transition of the frame, that is, the transition away from the
  /// inter-frame state of the clock line.
  ///
  /// If bit is set to 1 SSP controller captures serial data on the second
  /// clock transition of the frame, that is, the transition back to the
  /// inter-frame state of the clock line.
  static constexpr auto phase_bit = bit_mask::from<7>();

  /// Bitmask for dividing the peripheral clock to set the SPI clock
  /// frequency.
  static constexpr auto divider_bit = bit_mask::from<8, 15>();
};

/// SSPn Control Register 1
struct control_register1  // NOLINT
{
  /// Setting this bit to 1 will enable the peripheral for communication.
  static constexpr auto spi_enable = bit_mask::from<1>();

  /// Setting this bit to 1 will enable spi slave mode.
  static constexpr auto slave_mode_bit = bit_mask::from<2>();
};

/// SSPn Status Register
struct status_register  // NOLINT
{
  static constexpr auto transmit_fifo_not_full = bit_mask::from<1>();
  static constexpr auto receive_fifo_not_empty = bit_mask::from<2>();
  /// This bit is 0 if the SSPn controller is idle, or 1 if it is currently
  /// sending/receiving a frame and/or the Tx FIFO is not empty.
  static constexpr auto data_line_busy_bit = bit_mask::from<4>();
};

/// SSPn dma Register
struct dma_register  // NOLINT
{
  static constexpr auto receive_dma_enable = bit_mask::from<0>();
  static constexpr auto transmit_dma_enable = bit_mask::from<1>();
};

inline spi_reg_t* spi_reg0 = reinterpret_cast<spi_reg_t*>(0x40088000);
inline spi_reg_t* spi_reg1 = reinterpret_cast<spi_reg_t*>(0x40030000);
inline spi_reg_t* spi_reg2 = reinterpret_cast<spi_reg_t*>(0x400AC000);
}  // namespace hal::lpc40
