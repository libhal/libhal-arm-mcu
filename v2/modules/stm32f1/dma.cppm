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
#include <cstddef>
#include <optional>
#include <span>

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

/**
 * @brief A peripheral's DMA request line
 *
 * Each DMA channel serves a fixed set of peripheral requests (RM0008 tables
 * 78 and 79), so naming the request picks the channel. Requests that share a
 * channel, such as `i2c1_tx` and `usart2_rx`, can't run at the same time.
 * `dma_channel` claims the channel, so that is detected.
 *
 * `spi3_rx` and `spi3_tx` are on DMA2, which only high density and larger
 * parts have. Every other request is on DMA1.
 */
export enum class dma_request : u8
{
  /// DMA1 channel 1
  adc1,
  /// DMA1 channel 2
  spi1_rx,
  /// DMA1 channel 3
  spi1_tx,
  /// DMA1 channel 4
  spi2_rx,
  /// DMA1 channel 5
  spi2_tx,
  /// DMA2 channel 1
  spi3_rx,
  /// DMA2 channel 2
  spi3_tx,
  /// DMA1 channel 5
  usart1_rx,
  /// DMA1 channel 4
  usart1_tx,
  /// DMA1 channel 6
  usart2_rx,
  /// DMA1 channel 7
  usart2_tx,
  /// DMA1 channel 3
  usart3_rx,
  /// DMA1 channel 2
  usart3_tx,
  /// DMA1 channel 7
  i2c1_rx,
  /// DMA1 channel 6
  i2c1_tx,
  /// DMA1 channel 5
  i2c2_rx,
  /// DMA1 channel 4
  i2c2_tx,
};

/**
 * @brief Exclusive use of one DMA channel, moving bytes between a peripheral
 * register and memory
 *
 * A driver claims the channel for a request, starts transfers on it, and
 * releases it when the object is destroyed. Claims are tracked in software,
 * so two drivers whose requests share a channel can't clobber each other's
 * transfers.
 *
 * A driver that only needs the channel during a transfer claims it with
 * `try_claim()` at the start of the transfer and falls back to moving bytes
 * itself if another driver holds it. That way peripherals that share a
 * channel can both use it, one transfer at a time. A driver that needs the
 * channel for its whole life, such as a USART receiving into a circular
 * buffer, claims it with the constructor.
 *
 * One-shot transfers notify a `hal::notifier` when they finish or fail, so
 * the driver can sleep with `hal::notifier::scope` while the bytes move.
 * Drivers must re-check `remaining()` and `failed()` after every wakeup.
 *
 * Transfers move 8-bit values. The memory passed to a transfer must stay
 * valid until the transfer finishes or `stop()` is called. The channel stops
 * when it is destroyed.
 */
export class dma_channel
{
  /// Only `try_claim()` can make one, so only it can call the constructor
  /// that assumes the channel is already claimed.
  struct claimed_key
  {
    explicit claimed_key() = default;
  };

public:
  /**
   * @brief Claim the channel for a request
   *
   * Powers on the DMA controller, which is shared, so it is never powered off.
   *
   * @param p_request - the peripheral request this channel will serve
   * @throws hal::device_or_resource_busy - if a driver holds the channel,
   * for this request or another one on the same channel.
   */
  explicit dma_channel(dma_request p_request);

  /**
   * @brief Claim the channel for a request if it's free
   *
   * @param p_request - the peripheral request this channel will serve
   * @return the claimed channel, or `std::nullopt` if a driver holds it
   */
  [[nodiscard]] static std::optional<dma_channel> try_claim(
    dma_request p_request) noexcept;

  /**
   * @brief Take over a channel `try_claim()` already claimed
   *
   * Only `try_claim()` can call this, since only it can make a `claimed_key`.
   *
   * @param p_request - the request the channel was claimed for
   */
  dma_channel(claimed_key, dma_request p_request) noexcept;

  dma_channel(dma_channel const&) = delete;
  dma_channel& operator=(dma_channel const&) = delete;
  dma_channel(dma_channel&&) = delete;
  dma_channel& operator=(dma_channel&&) = delete;

  /// Stop any transfer and release the channel
  ~dma_channel();

  /**
   * @brief Copy bytes from a peripheral register into a buffer, once
   *
   * Replaces any transfer in progress. An empty buffer starts nothing and
   * leaves `remaining()` at zero.
   *
   * @param p_register - the peripheral's data register
   * @param p_buffer - where the bytes go
   * @param p_notifier - notified when the transfer finishes or fails
   * @throws hal::operation_not_supported - if the buffer is longer than
   * `max_dma_length`.
   */
  void receive(u32 const volatile& p_register,
               std::span<hal::byte> p_buffer,
               hal::notifier& p_notifier);

  /**
   * @brief Read a peripheral register a number of times, keeping only the
   * last byte
   *
   * For the bytes a full duplex bus receives that nobody asked for.
   *
   * @param p_register - the peripheral's data register
   * @param p_sink - the byte each read overwrites
   * @param p_count - how many bytes to read
   * @param p_notifier - notified when the transfer finishes or fails
   * @throws hal::operation_not_supported - if `p_count` is more than
   * `max_dma_length`.
   */
  void receive_discarding(u32 const volatile& p_register,
                          hal::byte& p_sink,
                          std::size_t p_count,
                          hal::notifier& p_notifier);

  /**
   * @brief Copy bytes from a peripheral register into a circular buffer,
   * forever
   *
   * No notification is sent. Read `remaining()` to find where the next byte
   * will be written.
   *
   * @param p_register - the peripheral's data register
   * @param p_buffer - the circular buffer
   * @throws hal::operation_not_supported - if the buffer is longer than
   * `max_dma_length`.
   */
  void receive_circular(u32 const volatile& p_register,
                        std::span<hal::byte> p_buffer);

  /**
   * @brief Copy bytes from memory into a peripheral register, once
   *
   * Replaces any transfer in progress. Empty data starts nothing and leaves
   * `remaining()` at zero.
   *
   * @param p_register - the peripheral's data register
   * @param p_data - the bytes to send
   * @param p_notifier - notified when the transfer finishes or fails
   * @throws hal::operation_not_supported - if the data is longer than
   * `max_dma_length`.
   */
  void transmit(u32 volatile& p_register,
                std::span<hal::byte const> p_data,
                hal::notifier& p_notifier);

  /**
   * @brief Write the same byte into a peripheral register a number of times
   *
   * For the filler bytes a full duplex bus sends while it only receives.
   *
   * @param p_register - the peripheral's data register
   * @param p_value - the byte to send
   * @param p_count - how many times to send it
   * @param p_notifier - notified when the transfer finishes or fails
   * @throws hal::operation_not_supported - if `p_count` is more than
   * `max_dma_length`.
   */
  void transmit_repeated(u32 volatile& p_register,
                         hal::byte const& p_value,
                         std::size_t p_count,
                         hal::notifier& p_notifier);

  /// Stop the transfer in progress, if any. The notifier is no longer
  /// notified.
  void stop() noexcept;

  /**
   * @return std::size_t - how many bytes the transfer has left to move. Zero
   * when a one-shot transfer has finished.
   */
  [[nodiscard]] std::size_t remaining() const noexcept;

  /**
   * @return true - the transfer ended before moving every byte, because of
   * a bus error or because it was stopped
   * @return false - the transfer is in progress or finished
   */
  [[nodiscard]] bool failed() const noexcept;

private:
  void start(uptr p_register,
             uptr p_memory,
             std::size_t p_count,
             u32 p_configuration,
             hal::notifier* p_notifier);

  u8 m_index;
};
}  // namespace hal::stm32f1
