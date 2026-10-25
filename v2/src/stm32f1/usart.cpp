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
/// TX/RX pin pair and receive DMA request for a supported USART peripheral
struct usart_resources
{
  pin tx;
  pin rx;
  dma_request receive_request;
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
               .receive_request = dma_request::usart1_rx };
    case peripheral::usart2:
      return { .tx = { .port = 'A', .pin = 2 },
               .rx = { .port = 'A', .pin = 3 },
               .receive_request = dma_request::usart2_rx };
    case peripheral::usart3:
      return { .tx = { .port = 'B', .pin = 10 },
               .rx = { .port = 'B', .pin = 11 },
               .receive_request = dma_request::usart3_rx };
    default:
      throw hal::operation_not_supported(nullptr);
  }
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
  // Claims the channel first, so its destructor releases it if a pin claim
  // below throws.
  , m_receive_dma(resources_for(p_manager.m_id).receive_request)
  , m_uart(
      // NOLINTNEXTLINE(performance-no-int-to-ptr)
      reinterpret_cast<void*>(peripheral_to_register(p_manager.m_id)),
      p_buffer)
{
  auto const resources = resources_for(m_id);
  m_tx = resources.tx;
  m_rx = resources.rx;

  // Claim both pins or neither: check RX first, so a TX claim that succeeds
  // is never left behind by an RX claim that throws.
  if (is_pin_claimed(m_rx)) {
    throw hal::device_or_resource_busy(this);
  }
  claim_pin(m_tx);
  claim_pin(m_rx);

  m_receive_dma.receive_circular(*m_uart.data_register(), p_buffer);

  m_uart.configure(p_settings, frequency(m_id));

  configure_pin(m_tx, push_pull_alternative_output);
  configure_pin(m_rx, input_pull_up);

  // Set last, so a constructor that throws leaves the manager free.
  m_manager->m_serial_acquired = true;
}

usart::serial::~serial()
{
  // Stop DMA writing into the receive buffer, which may be freed next.
  m_receive_dma.stop();
  reset_pin(m_tx);
  reset_pin(m_rx);
  release_pin(m_tx);
  release_pin(m_rx);
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
  return buffer_size - m_receive_dma.remaining();
}
}  // namespace hal::stm32f1
