#pragma once

#include <array>

#include <libhal-util/bit.hpp>

namespace hal::stm32f1::dma {
/// Namespace for the control registers (DMA->CCR) bit masks and predefined
/// settings constants.

/// Declare this channel for Memory to memory mode
static constexpr auto memory_to_memory = hal::bit_mask::from<14>();

/// Configure the channel priority for this channel.
/// 0b00: Low
/// 0b01: Medium
/// 0b10: High
/// 0b11: Very high
static constexpr auto channel_priority = hal::bit_mask::from<12, 13>();

/// The size of each element of the memory.
/// 0b00: 8-bits
/// 0b01: 16-bits
/// 0b10: 32-bits
/// 0b11: Reserved
static constexpr auto memory_size = hal::bit_mask::from<10, 11>();

/// The peripheral register size.
/// 0b00: 8-bits
/// 0b01: 16-bits
/// 0b10: 32-bits
/// 0b11: Reserved
static constexpr auto peripheral_size = hal::bit_mask::from<8, 9>();

/// Activate memory increment mode, which will increment the memory address
/// with each transfer
static constexpr auto memory_increment_enable = hal::bit_mask::from<7>();

/// Activate memory increment mode, which will increment the peripheral
/// address with each transfer
static constexpr auto peripheral_increment_enable = hal::bit_mask::from<6>();

/// DMA will continuous load bytes into the buffer supplied in a circular
/// buffer manner.
static constexpr auto circular_mode = hal::bit_mask::from<5>();

/// Data transfer direction
/// 0: Read from peripheral
/// 1: Read from memory
static constexpr auto data_transfer_direction = hal::bit_mask::from<4>();

/// Enable interrupt on transfer error
static constexpr auto transfer_error_interrupt_enable =
  hal::bit_mask::from<3>();

/// Enable interrupt on half of data transferred
static constexpr auto half_transfer_interrupt_enable = hal::bit_mask::from<2>();

/// Enable interrupt on complete transfer
static constexpr auto transfer_complete_interrupt_enable =
  hal::bit_mask::from<1>();

/// Enable this DMA channel
static constexpr auto enable = hal::bit_mask::from<0>();

struct dma_channel_t
{
  std::uint32_t volatile configuration;
  std::uint32_t volatile transfer_amount;
  std::uint32_t volatile peripheral_address;
  std::uint32_t volatile memory_address;
  std::uint32_t volatile reserved;
};

struct dma_t
{
  std::uint32_t volatile interrupt_status;
  std::uint32_t volatile interrupt_flag_clear;
  std::array<dma_channel_t, 7> channel;
};

inline auto* dma1 = reinterpret_cast<dma_t*>(0x4002'0000);
inline auto* dma2 = reinterpret_cast<dma_t*>(0x4002'0400);
}  // namespace hal::stm32f1::dma
