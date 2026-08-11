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

module;

#include <array>

export module hal.arm_mcu.stm32f1:dma;

import hal;
import hal.util;

namespace hal::stm32f1 {
/// Maximum length of a buffer that the stm32f1xxx series dma controller can
/// handle.
export constexpr u32 max_dma_length = 65'535;

/// Namespace for the DMA controller's register map and channel configuration
/// (CCR) bit masks.
namespace dma {
/// Declare this channel for Memory to memory mode
constexpr auto memory_to_memory = hal::bit_mask::from<14>();

/// Configure the channel priority for this channel.
/// 0b00: Low
/// 0b01: Medium
/// 0b10: High
/// 0b11: Very high
constexpr auto channel_priority = hal::bit_mask::from<12, 13>();

/// The size of each element of the memory.
/// 0b00: 8-bits
/// 0b01: 16-bits
/// 0b10: 32-bits
/// 0b11: Reserved
constexpr auto memory_size = hal::bit_mask::from<10, 11>();

/// The peripheral register size.
/// 0b00: 8-bits
/// 0b01: 16-bits
/// 0b10: 32-bits
/// 0b11: Reserved
constexpr auto peripheral_size = hal::bit_mask::from<8, 9>();

/// Activate memory increment mode, which will increment the memory address
/// with each transfer
constexpr auto memory_increment_enable = hal::bit_mask::from<7>();

/// Activate memory increment mode, which will increment the peripheral
/// address with each transfer
constexpr auto peripheral_increment_enable = hal::bit_mask::from<6>();

/// DMA will continuous load bytes into the buffer supplied in a circular
/// buffer manner.
constexpr auto circular_mode = hal::bit_mask::from<5>();

/// Data transfer direction
/// 0: Read from peripheral
/// 1: Read from memory
constexpr auto data_transfer_direction = hal::bit_mask::from<4>();

/// Enable interrupt on transfer error
constexpr auto transfer_error_interrupt_enable = hal::bit_mask::from<3>();

/// Enable interrupt on half of data transferred
constexpr auto half_transfer_interrupt_enable = hal::bit_mask::from<2>();

/// Enable interrupt on complete transfer
constexpr auto transfer_complete_interrupt_enable = hal::bit_mask::from<1>();

/// Enable this DMA channel
constexpr auto enable = hal::bit_mask::from<0>();

/// Register map for a single DMA channel
struct dma_channel_t
{
  u32 volatile configuration;
  u32 volatile transfer_amount;
  u32 volatile peripheral_address;
  u32 volatile memory_address;
  u32 volatile reserved;
};

/// Register map for the DMA controller
struct dma_t
{
  u32 volatile interrupt_status;
  u32 volatile interrupt_flag_clear;
  std::array<dma_channel_t, 7> channel;
};

/// DMA1 controller address
constexpr auto dma1_addr = static_cast<uptr>(0x4002'0000);
/// DMA2 controller address
constexpr auto dma2_addr = static_cast<uptr>(0x4002'0400);
// NOLINTBEGIN(performance-no-int-to-ptr)
/// DMA1 controller register
inline auto* dma1 = reinterpret_cast<dma_t*>(dma1_addr);
/// DMA2 controller register
inline auto* dma2 = reinterpret_cast<dma_t*>(dma2_addr);
// NOLINTEND(performance-no-int-to-ptr)
}  // namespace dma
}  // namespace hal::stm32f1
