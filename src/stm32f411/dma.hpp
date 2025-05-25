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

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include <libhal-arm-mcu/stm32f411/constants.hpp>
#include <libhal-arm-mcu/stm32f411/dma.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/enum.hpp>
#include <libhal/functional.hpp>

namespace hal::stm32f411 {
enum class dma_transfer_type : std::uint8_t
{
  peripheral_to_memory = 0b00U,
  memory_to_peripheral = 0b01U,
  memory_to_memory = 0b10U
};
enum class dma_transfer_size : std::uint8_t
{
  byte = 0b00U,
  half_word = 0b01U,
  word = 0b10U
};

enum class dma_burst_size : std::uint8_t
{
  single_transfer = 0b00U,
  transfer_4_beats = 0b01U,
  transfer_8_beats = 0b10U,
  transfer_16_beats = 0b11U,
};

enum class dma_priority_level : std::uint8_t
{
  low = 0b00U,
  medium = 0b01U,
  high = 0b10U,
  very_high = 0b11U,
};

enum class dma_flow_controller : u8
{
  dma_controls_flow = 0b0U,
  peripheral_controls_flow = 0b1U,
};

struct dma_channel_stream_t
{
  uint8_t stream;
  uint8_t channel;
};
/// DMA register map
struct stream_config_t
{
  /// Offset 0x00: This register is used to configure the concerned stream
  std::uint32_t volatile configure;
  /// Offset 0x04: Number of data items to be transferred (0 up to 65535). This
  /// register can be written only when the stream is disabled
  std::uint32_t volatile transfer_count;
  /// Offset 0x08: Base address of the peripheral data register from/to which
  /// the data will be read/written. These bits are write-protected and can be
  /// written only when bit EN = '0' in the DMA_SxCR register.
  std::uintptr_t volatile peripheral_address;
  /// Offset 0x0C: Base address of Memory area 0 from/to which the data will be
  /// read/written
  std::uintptr_t volatile memory_0_address;
  /// Offset 0x10: Base address of Memory area 1 from/to which the data will be
  /// read/written
  std::uintptr_t volatile memory_1_address;
  /// Offset 0x14: fifo_control
  std::uint32_t volatile fifo_control;
};
struct dma_config_t
{
  /// Offset 0x00: low interrupt status register
  std::uint32_t volatile interupt_status_low;
  /// Offset 0x04: high interrupt status register
  std::uint32_t volatile interupt_status_high;
  /// Offset 0x08: low interrupt flag clear register
  std::uint32_t volatile interupt_status_clear_low;
  /// Offset 0x0C: low interrupt flag clear register
  std::uint32_t volatile interupt_status_clear_high;
  std::array<stream_config_t, 7> stream;
};
struct dma_stream_config
{
  static constexpr auto stream_enable = bit_mask::from<0>();
  static constexpr auto direct_mode_error_interupt_enable = bit_mask::from<1>();
  static constexpr auto transfer_error_interupt_enable = bit_mask::from<2>();
  static constexpr auto half_transfer_interupt_enable = bit_mask::from<3>();
  static constexpr auto transfer_complete_interupt_enable = bit_mask::from<4>();
  /// 0: DMA is the flow controller
  /// 1: peripheral is the flow controller
  static constexpr auto peripheral_flow_controller = bit_mask::from<5>();
  /// 00: Peripheral-to-memory
  /// 01: Memory-to-peripheral
  /// 10: Memory-to-memory
  static constexpr auto data_transfer_direction = bit_mask::from<7, 6>();
  static constexpr auto circular_mode_enable = bit_mask::from<8>();
  /// 0: peripheral address pointer is fixed
  /// 1: peripheral address pointer is incremented after each data transfer (set
  /// by peripheral_data_size)
  static constexpr auto peripheral_increment_mode = bit_mask::from<9>();
  /// 0: memory address pointer is fixed
  /// 1: memory address pointer is incremented after each data transfer (set by
  /// memory_data_size)
  static constexpr auto memory_increment_mode = bit_mask::from<10>();
  /// 00: byte (8-bit)
  /// 01: half-word (16-bit)
  /// 10: word (32-bit)
  static constexpr auto peripheral_data_size = bit_mask::from<12, 11>();
  /// 00: byte (8-bit)
  /// 01: half-word (16-bit)
  /// 10: word (32-bit)
  static constexpr auto memory_data_size = bit_mask::from<14, 13>();
  /// 0: The offset size for the peripheral address calculation is linked to the
  /// PSIZE 1: The offset size for the peripheral address calculation is fixed
  /// to 4 (32-bit alignment).
  static constexpr auto peripheral_offset_size = bit_mask::from<15>();
  /// 00: Low
  /// 01: Medium
  /// 10: High
  /// 11: Very high
  static constexpr auto priority_level = bit_mask::from<17, 16>();
  /// 0: No buffer switching at the end of transfer
  /// 1: Memory target switched at the end of the DMA transfer
  static constexpr auto double_buffer_mode = bit_mask::from<18>();
  /// 0: The current target memory is Memory 0 (addressed by the DMA_SxM0AR
  /// pointer) 1: The current target memory is Memory 1 (addressed by the
  /// DMA_SxM1AR pointer)
  static constexpr auto current_target = bit_mask::from<19>();
  /// 00: single transfer
  /// 01: INCR4 (incremental burst of 4 beats)
  /// 10: INCR8 (incremental burst of 8 beats)
  /// 11: INCR16 (incremental burst of 16 beats)
  static constexpr auto peripheral_burst_transfer = bit_mask::from<22, 21>();
  /// 00: single transfer
  /// 01: INCR4 (incremental burst of 4 beats)
  /// 10: INCR8 (incremental burst of 8 beats)
  /// 11: INCR16 (incremental burst of 16 beats)
  static constexpr auto memory_burst_transfer = bit_mask::from<24, 23>();
  /// Refer to Table 27(DMA 1) or 28(DMA 2) of the user manual for channel
  /// selection
  static constexpr auto channel_select = bit_mask::from<27, 25>();
};
struct dma_fifo_config
{
  static constexpr auto direct_mode_disable = bit_mask::from<2>();
  static constexpr auto threshold_select = bit_mask::from<1, 0>();
};
inline constexpr intptr_t ahb_base = 0x4002'0000UL;
inline constexpr intptr_t dma_base = ahb_base + 0x6000;
static inline dma_config_t* get_dma_reg(peripheral p_dma)
{
  auto const dma_index =
    static_cast<u8>(hal::value(p_dma) - hal::value(peripheral::dma1));

  // STM has dedicated memory blocks where every 2^10 is a new DMA register
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  return reinterpret_cast<dma_config_t*>(dma_base + (dma_index << 10));
}

struct dma_settings_t
{
  void const volatile* source;
  void volatile* destination;

  size_t transfer_length;
  dma_flow_controller flow_controller;
  dma_transfer_type transfer_type;
  bool circular_mode;
  bool peripheral_address_increment;
  bool memory_address_increment;
  dma_transfer_size peripheral_data_size;
  dma_transfer_size memory_data_size;
  dma_priority_level priority_level;
  bool double_buffer_mode;
  bool current_target;
  dma_burst_size peripheral_burst_size;
  dma_burst_size memory_burst_size;
};

/**
 * @brief Setup and start a dma transfer
 *
 * @param p_configuration - dma configuration
 * @param p_interrupt_callback - callback when dma has completed
 */

dma_channel_stream_t setup_dma_transfer(
  peripheral p_dma,
  std::span<dma_channel_stream_t> const& p_possible_streams,
  dma_settings_t const& p_configuration,
  hal::callback<void()> p_interrupt_callback);

}  // namespace hal::stm32f411
