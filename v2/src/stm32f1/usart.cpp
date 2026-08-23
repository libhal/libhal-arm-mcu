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

class serial_driver final : public hal::serial
{
public:
  serial_driver(hal::ptr<usart> p_manager,
                peripheral p_id,
                usart_resources const& p_resources,
                void* p_register,
                std::span<hal::byte> p_buffer,
                hal::serial::settings const& p_settings)
    : m_manager(p_manager)
    , m_id(p_id)
    , m_tx(p_resources.tx)
    , m_rx(p_resources.rx)
    , m_dma_channel(p_resources.dma_channel)
    , m_uart(p_register, p_buffer)
  { 
    // NOTE: DMA1 is shared across multiple peripherals
    if (not is_on(peripheral::dma1)) {
      power_on(peripheral::dma1);
    }

    // TODO(#219): The stm32f1 platform needs APIs to determine if a DMA channel
    // is already taken and to hold that resource until destruction.
    auto const data_register_address =
      reinterpret_cast<uptr>(m_uart.data_register());
    auto const buffer_address = reinterpret_cast<uptr>(p_buffer.data());

    auto& channel = dma::dma1->channel[m_dma_channel - 1];
    channel.transfer_amount = static_cast<u32>(p_buffer.size());
    channel.peripheral_address = static_cast<u32>(data_register_address);
    channel.memory_address = static_cast<u32>(buffer_address);
    channel.configuration = uart_dma_settings1;

    m_uart.configure(p_settings, frequency(m_id));

    configure_pin(m_tx, push_pull_alternative_output);
    configure_pin(m_rx, input_pull_up);
  }

  serial_driver(serial_driver const&) = delete;
  serial_driver& operator=(serial_driver const&) = delete;
  serial_driver(serial_driver&&) = delete;
  serial_driver& operator=(serial_driver&&) = delete;

  ~serial_driver()
  {
    reset_pin(m_tx);
    reset_pin(m_rx);
  }

private:
  async::future<void> driver_configure(
    async::context&,
    hal::serial::settings const& p_settings) override
  {
    m_uart.configure(p_settings, frequency(m_id));
    return {};
  }

  async::future<void> driver_write(
    async::context&,
    mem::scatter_span<hal::byte const> p_data) override
  {
    m_uart.write(p_data);
    return {};
  }

  hal::circular_span<hal::byte const> driver_receive_buffer() override
  {
    return m_uart.receive_buffer();
  }

  hal::usize driver_receive_cursor() override
  {
    auto const buffer_size = m_uart.receive_buffer().size();
    auto const& channel = dma::dma1->channel[m_dma_channel - 1];
    return buffer_size - channel.transfer_amount;
  }

  // Co-owns the manager so the USART register bank and its power state
  // outlive this resource.
  hal::ptr<usart> m_manager;
  peripheral m_id;
  pin m_tx;
  pin m_rx;
  u8 m_dma_channel;
  hal::stm32_generic::uart m_uart;
};
}  // namespace

struct usart::impl
{
  peripheral id;
  void* reg;
};

hal::ptr<usart> usart::create(hal::allocator p_allocator, peripheral p_id)
{
  return hal::allocate<usart>(p_allocator, private_key{}, p_allocator, p_id);
}

usart::usart(private_key, hal::allocator p_allocator, peripheral p_id)
  : pimpl(p_allocator,
          impl{ .id = p_id,
                // NOLINTNEXTLINE(performance-no-int-to-ptr)
                .reg = reinterpret_cast<void*>(peripheral_to_register(p_id)) })
{
  power_on(p_id);
}

usart::~usart()
{
  power_off(inner().id);
}

hal::ptr<hal::serial> usart::acquire_serial(
  std::span<hal::byte> p_buffer,
  hal::serial::settings const& p_settings)
{
  if (p_buffer.size() > max_dma_length) {
    throw hal::operation_not_supported(this);
  }

  auto const id = inner().id;
  auto const resources = resources_for(id);

  return hal::allocate<serial_driver>(memory_resource(),
                                      strong_from_this(),
                                      id,
                                      resources,
                                      inner().reg,
                                      p_buffer,
                                      p_settings);
}
}  // namespace hal::stm32f1
