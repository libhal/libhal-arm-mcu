// Copyright 2026 Khalil Estell and the libhal contributors
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

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <optional>
#include <span>
#include <utility>

module hal.arm_mcu.stm32f1;

import hal;
import hal.util;
import hal.arm_mcu.cortex_m;

import :constants;
import :power;
import :dma;

namespace hal::stm32f1 {
namespace {
/// Channels the requests use: DMA1 channels 1 to 7 are indexes 0 to 6, DMA2
/// channels 1 and 2 are indexes 7 and 8. Supporting a request on DMA2
/// channel 4 or 5 needs more care, since high density parts share one
/// interrupt between them.
constexpr std::size_t channel_count = 9;
constexpr std::size_t dma1_channel_count = 7;

/// @return the index of the channel that serves the request (RM0008 tables
/// 78 and 79)
constexpr u8 channel_index(dma_request p_request)
{
  switch (p_request) {
    case dma_request::adc1:
      return 0;
    case dma_request::spi1_rx:
    case dma_request::usart3_tx:
      return 1;
    case dma_request::spi1_tx:
    case dma_request::usart3_rx:
      return 2;
    case dma_request::spi2_rx:
    case dma_request::usart1_tx:
    case dma_request::i2c2_tx:
      return 3;
    case dma_request::spi2_tx:
    case dma_request::usart1_rx:
    case dma_request::i2c2_rx:
      return 4;
    case dma_request::usart2_rx:
    case dma_request::i2c1_tx:
      return 5;
    case dma_request::usart2_tx:
    case dma_request::i2c1_rx:
      return 6;
    case dma_request::spi3_rx:
      return 7;
    case dma_request::spi3_tx:
      return 8;
  }
  std::unreachable();
}

[[nodiscard]] dma::dma_t& controller(u8 p_index)
{
  return (p_index < dma1_channel_count) ? *dma::dma1 : *dma::dma2;
}

[[nodiscard]] u8 controller_channel(u8 p_index)
{
  return (p_index < dma1_channel_count) ? p_index
                                        : (p_index - dma1_channel_count);
}

[[nodiscard]] dma::dma_channel_t& registers(u8 p_index)
{
  return controller(p_index).channel[controller_channel(p_index)];
}

[[nodiscard]] irq interrupt_for(u8 p_index)
{
  if (p_index < dma1_channel_count) {
    return static_cast<irq>(hal::value(irq::dma1_channel1) + p_index);
  }
  return static_cast<irq>((hal::value(irq::dma2_channel1) + p_index) -
                          dma1_channel_count);
}

/// Clear every interrupt flag of a channel (CGIF, CTCIF, CHTIF, CTEIF)
void clear_flags(u8 p_index)
{
  constexpr u32 channel_flags = 0b1111;
  constexpr u32 bits_per_channel = 4;
  controller(p_index).interrupt_flag_clear =
    channel_flags << (bits_per_channel * controller_channel(p_index));
}

/// One bit per channel, set while a driver holds the channel. Atomic, so
/// drivers in different contexts can claim channels at the same time.
constinit std::atomic<u32> claimed_channels{ 0 };

/// What each channel notifies when its one-shot transfer ends
constinit std::array<hal::notifier* volatile, channel_count> notifiers{};

[[nodiscard]] bool claim_channel(u8 p_index) noexcept
{
  auto const bit = u32{ 1 } << p_index;
  return (claimed_channels.fetch_or(bit, std::memory_order_acquire) & bit) == 0;
}

void release_channel(u8 p_index) noexcept
{
  auto const bit = u32{ 1 } << p_index;
  claimed_channels.fetch_and(~bit, std::memory_order_release);
}

template<u8 Index>
void channel_isr()
{
  // The driver finds out what happened from the channel's registers, so the
  // flags are only cleared to stop the interrupt from firing again.
  clear_flags(Index);
  if (auto* const notifier = notifiers[Index]) {
    notifier->notify();
  }
}

template<std::size_t... Index>
consteval auto make_isrs(std::index_sequence<Index...>)
{
  return std::array<cortex_m::interrupt_pointer, sizeof...(Index)>{
    &channel_isr<Index>...
  };
}

constexpr auto channel_isrs =
  make_isrs(std::make_index_sequence<channel_count>{});

/// Settings shared by every transfer: 8-bit peripheral and memory accesses.
/// Receiving gets the higher priority, so a full duplex bus empties its data
/// register before the next byte arrives.
constexpr auto receive_settings =
  hal::bit_value()
    .clear<dma::data_transfer_direction>()  // Read from peripheral
    .insert<dma::peripheral_size, 0b00U>()
    .insert<dma::memory_size, 0b00U>()
    .insert<dma::channel_priority, 0b10U>()  // Low Medium [High] Very_High
    .set<dma::enable>()
    .to<u32>();

constexpr auto transmit_settings =
  hal::bit_value()
    .set<dma::data_transfer_direction>()  // Read from memory
    .insert<dma::peripheral_size, 0b00U>()
    .insert<dma::memory_size, 0b00U>()
    .insert<dma::channel_priority, 0b01U>()  // Low [Medium] High Very_High
    .set<dma::enable>()
    .to<u32>();

/// One-shot transfers notify when they finish or fail
constexpr auto notify_settings =
  hal::bit_value()
    .set<dma::transfer_complete_interrupt_enable>()
    .set<dma::transfer_error_interrupt_enable>()
    .to<u32>();

constexpr auto increment_memory =
  hal::bit_value().set<dma::memory_increment_enable>().to<u32>();

constexpr auto circular = hal::bit_value().set<dma::circular_mode>().to<u32>();

/// Memory to memory: the "peripheral" address is the source, and both
/// addresses increment. Copies get the lowest priority, so they never delay
/// a peripheral's transfer.
constexpr auto copy_settings =
  hal::bit_value()
    .set<dma::memory_to_memory>()
    .clear<dma::data_transfer_direction>()  // Read from "peripheral"
    .set<dma::peripheral_increment_enable>()
    .set<dma::memory_increment_enable>()
    .insert<dma::channel_priority, 0b00U>()  // [Low] Medium High Very_High
    .set<dma::enable>()
    .to<u32>();

constexpr auto word_size = hal::bit_value()
                             .insert<dma::peripheral_size, 0b10U>()
                             .insert<dma::memory_size, 0b10U>()
                             .to<u32>();

[[nodiscard]] uptr address_of(void const volatile* p_address)
{
  return reinterpret_cast<uptr>(p_address);
}
}  // namespace

dma_channel::dma_channel(dma_request p_request)
  : m_registers(&registers(channel_index(p_request)))
  , m_index(channel_index(p_request))
{
  if (not claim_channel(m_index)) {
    throw hal::device_or_resource_busy(this);
  }
  power_on((m_index < dma1_channel_count) ? peripheral::dma1
                                          : peripheral::dma2);
  cortex_m::enable_interrupt(interrupt_for(m_index), channel_isrs[m_index]);
}

std::optional<dma_channel> dma_channel::try_claim(
  dma_request p_request) noexcept
{
  if (not claim_channel(channel_index(p_request))) {
    return std::nullopt;
  }
  return std::optional<dma_channel>(
    std::in_place, claimed_key{}, channel_index(p_request));
}

std::optional<dma_channel> dma_channel::try_claim_any() noexcept
{
  for (u8 index = 0; index < dma1_channel_count; index++) {
    if (claim_channel(index)) {
      return std::optional<dma_channel>(std::in_place, claimed_key{}, index);
    }
  }
  return std::nullopt;
}

dma_channel::dma_channel(claimed_key, u8 p_index) noexcept
  : m_registers(&registers(p_index))
  , m_index(p_index)
{
  power_on((m_index < dma1_channel_count) ? peripheral::dma1
                                          : peripheral::dma2);
  cortex_m::enable_interrupt(interrupt_for(m_index), channel_isrs[m_index]);
}

dma_channel::~dma_channel()
{
  stop();
  cortex_m::disable_interrupt(interrupt_for(m_index));
  release_channel(m_index);
}

void dma_channel::receive(u32 const volatile& p_register,
                          std::span<hal::byte> p_buffer,
                          hal::notifier& p_notifier)
{
  start(address_of(&p_register),
        address_of(p_buffer.data()),
        p_buffer.size(),
        receive_settings | increment_memory | notify_settings,
        &p_notifier);
}

void dma_channel::receive_discarding(u32 const volatile& p_register,
                                     hal::byte& p_sink,
                                     std::size_t p_count,
                                     hal::notifier& p_notifier)
{
  start(address_of(&p_register),
        address_of(&p_sink),
        p_count,
        receive_settings | notify_settings,
        &p_notifier);
}

void dma_channel::receive_circular(u32 const volatile& p_register,
                                   std::span<hal::byte> p_buffer)
{
  start(address_of(&p_register),
        address_of(p_buffer.data()),
        p_buffer.size(),
        receive_settings | increment_memory | circular,
        nullptr);
}

void dma_channel::transmit(u32 volatile& p_register,
                           std::span<hal::byte const> p_data,
                           hal::notifier& p_notifier)
{
  start(address_of(&p_register),
        address_of(p_data.data()),
        p_data.size(),
        transmit_settings | increment_memory | notify_settings,
        &p_notifier);
}

void dma_channel::transmit_repeated(u32 volatile& p_register,
                                    hal::byte const& p_value,
                                    std::size_t p_count,
                                    hal::notifier& p_notifier)
{
  start(address_of(&p_register),
        address_of(&p_value),
        p_count,
        transmit_settings | notify_settings,
        &p_notifier);
}

void dma_channel::copy(std::span<hal::byte const> p_source,
                       std::span<hal::byte> p_destination,
                       hal::notifier& p_notifier)
{
  constexpr std::size_t word = 4;
  auto const length = std::min(p_source.size(), p_destination.size());
  auto const source = address_of(p_source.data());
  auto const destination = address_of(p_destination.data());

  if (((source | destination | length) % word) == 0) {
    start(source,
          destination,
          length / word,
          copy_settings | word_size | notify_settings,
          &p_notifier);
  } else {
    start(source,
          destination,
          length,
          copy_settings | notify_settings,
          &p_notifier);
  }
}

void dma_channel::start(uptr p_register,
                        uptr p_memory,
                        std::size_t p_count,
                        u32 p_configuration,
                        hal::notifier* p_notifier)
{
  if (p_count > max_dma_length) {
    throw hal::operation_not_supported(this);
  }

  // The other registers can only be written while the channel is disabled.
  stop();
  auto& channel = *m_registers;
  channel.peripheral_address = static_cast<u32>(p_register);
  channel.memory_address = static_cast<u32>(p_memory);
  channel.transfer_amount = static_cast<u32>(p_count);
  if (p_count == 0) {
    return;
  }
  notifiers[m_index] = p_notifier;
  // Writes to the memory being sent must happen before the channel can read
  // it.
  std::atomic_signal_fence(std::memory_order_seq_cst);
  channel.configuration = p_configuration;
}

void dma_channel::stop() noexcept
{
  // Also disables the channel's interrupts.
  m_registers->configuration = 0;
  clear_flags(m_index);
  notifiers[m_index] = nullptr;
}

std::size_t dma_channel::remaining() const noexcept
{
  auto const count = m_registers->transfer_amount;
  // Reads of the memory being received into must happen after the channel
  // has written it.
  std::atomic_signal_fence(std::memory_order_seq_cst);
  return count;
}

bool dma_channel::failed() const noexcept
{
  // The channel disables itself on a transfer error, and stop() disables it.
  // A transfer that finished stays enabled with nothing left to move.
  auto& channel = *m_registers;
  return not hal::bit_extract<dma::enable>(channel.configuration) and
         (channel.transfer_amount != 0);
}
}  // namespace hal::stm32f1
