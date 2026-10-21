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

#include <span>

module hal.arm_mcu.stm32f1;

import hal;
import hal.util;
import hal.arm_mcu.stm32_generic;

import :constants;
import :pin;
import :power;
import :clock;
import :dma;

namespace hal::stm32f1 {
namespace {
/// TX/RX pin pair and receive DMA channel for a supported USART peripheral
struct usart_resources
{
  pin tx;
  pin rx;
  u8 dma_channel;
};

/// @throws hal::operation_not_supported - if `p_id` is not `usart1`,
/// `usart2`, or `usart3`.
uptr peripheral_to_register(peripheral p_id)
{
  // See Chapter 3.3 "Memory" page 50 in RM0008 for these magic numbers
  switch (p_id) {
    case peripheral::usart1:
      return 0x4001'3800;
    case peripheral::usart2:
      return 0x4000'4400;
    case peripheral::usart3:
      return 0x4000'4800;
    default:
      throw hal::operation_not_supported(nullptr);
  }
}

/// @throws hal::operation_not_supported - if `p_id` is not `usart1`,
/// `usart2`, or `usart3`.
usart_resources resources_for(peripheral p_id)
{
  switch (p_id) {
    case peripheral::usart1:
      return { .tx = { .port = 'A', .pin = 9 },
               .rx = { .port = 'A', .pin = 10 },
               .dma_channel = 5 };
    case peripheral::usart2:
      return { .tx = { .port = 'A', .pin = 2 },
               .rx = { .port = 'A', .pin = 3 },
               .dma_channel = 6 };
    case peripheral::usart3:
      return { .tx = { .port = 'B', .pin = 10 },
               .rx = { .port = 'B', .pin = 11 },
               .dma_channel = 3 };
    default:
      throw hal::operation_not_supported(nullptr);
  }
}

/// Configure the receive DMA channel to continuously (circular mode) copy
/// incoming bytes from the USART data register into memory, 8-bits at a
/// time.
constexpr auto uart_dma_settings1 =
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
    .to<u32>();

/// @return dma::dma_channel_t& - the DMA1 channel that receives for this USART
dma::dma_channel_t& receive_channel(u8 p_dma_channel)
{
  return dma::dma1->channel[p_dma_channel - 1];
}
}  // namespace

usart::usart(peripheral p_id)
  : m_id(p_id)
{
  // Rejects unsupported peripherals before anything is claimed.
  [[maybe_unused]] auto const reg = peripheral_to_register(p_id);
  claim(p_id);
  power_on(p_id);
  // Start from the reset state, whatever ran before (e.g. a bootloader) left.
  reset_peripheral(p_id);
}

usart::~usart()
{
  power_off(m_id);
  release(m_id);
}

usart::serial usart::acquire_serial(std::span<hal::byte> p_buffer,
                                    hal::serial::settings const& p_settings)
{
  if (m_serial_acquired) {
    throw hal::device_or_resource_busy(this);
  }
  if (p_buffer.size() > max_dma_length) {
    throw hal::operation_not_supported(this);
  }
  return { *this, p_buffer, p_settings };
}

usart::serial::serial(usart& p_manager,
                      std::span<hal::byte> p_buffer,
                      hal::serial::settings const& p_settings)
  : m_manager(p_manager)
  , m_id(p_manager.m_id)
  , m_uart(
      // NOLINTNEXTLINE(performance-no-int-to-ptr)
      reinterpret_cast<void*>(peripheral_to_register(p_manager.m_id)),
      p_buffer)
{
  auto const resources = resources_for(m_id);
  m_tx = resources.tx;
  m_rx = resources.rx;
  m_dma_channel = resources.dma_channel;

  // NOTE: DMA1 is shared across multiple peripherals
  if (not is_on(peripheral::dma1)) {
    power_on(peripheral::dma1);
  }

  // TODO(#219): The stm32f1 platform needs APIs to determine if a DMA channel
  // is already taken and to hold that resource until destruction.
  auto const data_register_address =
    reinterpret_cast<uptr>(m_uart.data_register());
  auto const buffer_address = reinterpret_cast<uptr>(p_buffer.data());

  auto& channel = receive_channel(m_dma_channel);
  channel.transfer_amount = static_cast<u32>(p_buffer.size());
  channel.peripheral_address = static_cast<u32>(data_register_address);
  channel.memory_address = static_cast<u32>(buffer_address);
  channel.configuration = uart_dma_settings1;

  m_uart.configure(p_settings, frequency(m_id));

  configure_pin(m_tx, push_pull_alternative_output);
  configure_pin(m_rx, input_pull_up);

  // Set last, so a constructor that throws leaves the manager free.
  m_manager->m_serial_acquired = true;
}

usart::serial::~serial()
{
  // Stop DMA writing into the receive buffer, which may be freed next.
  hal::bit_modify(receive_channel(m_dma_channel).configuration)
    .clear<dma::enable>();
  reset_pin(m_tx);
  reset_pin(m_rx);
  m_manager->m_serial_acquired = false;
}

void usart::serial::driver_configure(hal::serial::settings const& p_settings)
{
  m_uart.configure(p_settings, frequency(m_id));
}

void usart::serial::driver_write(mem::scatter_span<hal::byte const> p_data)
{
  m_uart.write(p_data);
}

hal::circular_span<hal::byte const> usart::serial::driver_receive_buffer()
{
  return m_uart.receive_buffer();
}

hal::usize usart::serial::driver_receive_cursor()
{
  auto const buffer_size = m_uart.receive_buffer().size();
  return buffer_size - receive_channel(m_dma_channel).transfer_amount;
}
}  // namespace hal::stm32f1
