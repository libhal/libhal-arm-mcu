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

#include <array>
#include <optional>

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
/// Clock, CIPO and COPI pins of a supported SPI peripheral
struct spi_pins
{
  pin clock;
  pin cipo;
  pin copi;
};

/// @throws hal::operation_not_supported - if `p_id` is not `spi1`, `spi2`,
/// or `spi3`.
uptr peripheral_to_register(peripheral p_id)
{
  // See Chapter 3.3 "Memory map" in RM0008 for these magic numbers
  switch (p_id) {
    case peripheral::spi1:
      return 0x4001'3000;
    case peripheral::spi2:
      return 0x4000'3800;
    case peripheral::spi3:
      return 0x4000'3C00;
    default:
      throw hal::operation_not_supported(nullptr);
  }
}

/// @throws hal::operation_not_supported - if `p_id` is not `spi1`, `spi2`,
/// or `spi3`.
spi_pins pins_for(peripheral p_id)
{
  // See the pin definitions table in the STM32F103 datasheets (DS5319,
  // DS5792). SPI3 only exists on high density and larger parts, where it is
  // fixed to PB3, PB4 and PB5.
  switch (p_id) {
    case peripheral::spi1:
      return { .clock = { .port = 'A', .pin = 5 },
               .cipo = { .port = 'A', .pin = 6 },
               .copi = { .port = 'A', .pin = 7 } };
    case peripheral::spi2:
      return { .clock = { .port = 'B', .pin = 13 },
               .cipo = { .port = 'B', .pin = 14 },
               .copi = { .port = 'B', .pin = 15 } };
    case peripheral::spi3:
      return { .clock = { .port = 'B', .pin = 3 },
               .cipo = { .port = 'B', .pin = 4 },
               .copi = { .port = 'B', .pin = 5 } };
    default:
      throw hal::operation_not_supported(nullptr);
  }
}

void* register_address(peripheral p_id)
{
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  return reinterpret_cast<void*>(peripheral_to_register(p_id));
}

/// DMA requests of a supported SPI peripheral
struct spi_requests
{
  dma_request transmit;
  dma_request receive;
};

spi_requests requests_for(peripheral p_id)
{
  switch (p_id) {
    case peripheral::spi1:
      return { .transmit = dma_request::spi1_tx,
               .receive = dma_request::spi1_rx };
    case peripheral::spi2:
      return { .transmit = dma_request::spi2_tx,
               .receive = dma_request::spi2_rx };
    default:
      return { .transmit = dma_request::spi3_tx,
               .receive = dma_request::spi3_rx };
  }
}
}  // namespace

spi::spi(peripheral p_id)
  : m_id(p_id)
  , m_driver(register_address(p_id))  // Rejects unsupported peripherals
{
  auto const pins = pins_for(p_id);

  // Claim every pin or none: check them all before claiming any.
  for (auto const& selected : std::array{ pins.clock, pins.cipo, pins.copi }) {
    if (is_pin_claimed(selected)) {
      throw hal::device_or_resource_busy(this);
    }
  }
  claim(p_id);
  claim_pin(pins.clock);
  claim_pin(pins.cipo);
  claim_pin(pins.copi);

  power_on(p_id);
  // Start from the reset state, whatever ran before (e.g. a bootloader) left.
  reset_peripheral(p_id);

  if (p_id == peripheral::spi1) {
    // AFIO is shared, so it is powered on, never claimed.
    power_on(peripheral::afio);
    bit_modify(alternative_function_io->mapr).clear<pin_remap::spi1_remap>();
  }

  configure_pin(pins.clock, push_pull_alternative_output);
  configure_pin(pins.cipo, input_float);
  configure_pin(pins.copi, push_pull_alternative_output);
}

spi::~spi()
{
  auto const pins = pins_for(m_id);
  for (auto const& selected : std::array{ pins.clock, pins.cipo, pins.copi }) {
    reset_pin(selected);
    release_pin(selected);
  }
  // Disables the peripheral. m_driver doesn't touch the registers when it is
  // destroyed after this.
  reset_peripheral(m_id);
  power_off(m_id);
  release(m_id);
}

void spi::driver_configure(hal::spi_settings const& p_settings)
{
  m_driver.configure(p_settings, frequency(m_id));
}

hal::hertz spi::driver_clock_rate(hal::spi_settings const& p_settings)
{
  return hal::stm32_generic::achievable_clock_rate(frequency(m_id),
                                                   p_settings.clock_rate);
}

void spi::driver_transfer(mem::scatter_span<hal::byte const> p_data_out,
                          mem::scatter_span<hal::byte> p_data_in,
                          hal::byte p_filler)
{
  // Channels are claimed per transfer, so drivers whose requests share a
  // channel take turns. If another driver holds either one, the transfer
  // polls instead.
  auto const requests = requests_for(m_id);
  auto transmit = dma_channel::try_claim(requests.transmit);
  auto receive = dma_channel::try_claim(requests.receive);
  if (transmit and receive) {
    m_driver.transfer(p_data_out, p_data_in, p_filler, *transmit, *receive);
  } else {
    m_driver.transfer(p_data_out, p_data_in, p_filler);
  }
}
}  // namespace hal::stm32f1
