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
#include <cstddef>
#include <optional>
#include <span>

module arm_mcu_demos;

import hal;
import hal.arm_mcu.stm32f1;
import hal.arm_mcu.cortex_m;

namespace {
/// Copies memory with a DMA1 channel, claimed for as long as it lives
class dma_copier final : public memory_copier
{
public:
  dma_copier()
    : m_channel(hal::stm32f1::dma_channel::try_claim_any())
  {
    if (not m_channel) {
      throw hal::device_or_resource_busy(this);
    }
  }

private:
  void driver_copy(std::span<hal::byte const> p_source,
                   std::span<hal::byte> p_destination) override
  {
    // One DMA transfer moves at most 65,535 bytes or words. A multiple of 4
    // keeps every chunk word aligned.
    constexpr std::size_t chunk = 65'532;
    auto const length = std::min(p_source.size(), p_destination.size());

    hal::notifier::scope operation(m_notifier,
                                   [this]() noexcept { m_channel->stop(); });
    for (std::size_t offset = 0; offset < length; offset += chunk) {
      auto const count = std::min(chunk, length - offset);
      m_channel->copy(p_source.subspan(offset, count),
                      p_destination.subspan(offset, count),
                      m_notifier);
      while (m_channel->remaining() != 0) {
        if (m_channel->failed()) {
          throw hal::io_error(this);
        }
        operation.block();
      }
    }
  }

  std::optional<hal::stm32f1::dma_channel> m_channel;
  hal::notifier m_notifier;
};
}  // namespace

namespace resources {
hal::static_ref<memory_copier> copier()
{
  static dma_copier copier;
  return copier;
}

hal::static_ref<hal::steady_clock> clock()
{
  static hal::cortex_m::dwt_counter counter(
    hal::stm32f1::frequency(hal::stm32f1::peripheral::cpu));
  return counter;
}

hal::static_ref<hal::output_pin> status_led()
{
  static hal::stm32f1::output_pin led({ .port = 'C', .pin = 13 });
  return led;
}

hal::static_ref<hal::input_pin> input_pin()
{
  static hal::stm32f1::input_pin pin({ .port = 'B', .pin = 4 });
  return pin;
}

hal::static_ref<hal::serial> console()
{
  static hal::stm32f1::usart usart1(hal::stm32f1::peripheral::usart1);
  static std::array<hal::byte, 128> receive_buffer{};
  static auto serial = usart1.acquire_serial(receive_buffer);
  return serial;
}

hal::static_ref<hal::adc16> adc()
{
  static hal::no_op_lock single_context;
  static hal::stm32f1::adc adc1(
    hal::stm32f1::peripheral::adc1, *clock(), single_context);
  static auto channel = adc1.acquire_channel(hal::stm32f1::adc_pins::pb0);
  return channel;
}

hal::static_ref<hal::i2c> i2c()
{
  static hal::stm32f1::i2c i2c1(hal::stm32f1::peripheral::i2c1, clock());
  return i2c1;
}

hal::static_ref<hal::spi_channel> spi_channel()
{
  using namespace mp_units::si::unit_symbols;
  static hal::no_op_lock single_context;
  static hal::spi_bus_manager spi1(
    [] { return hal::stm32f1::spi(hal::stm32f1::peripheral::spi1); },
    single_context);
  static hal::stm32f1::output_pin chip_select({ .port = 'A', .pin = 4 });
  static auto channel =
    spi1.acquire_channel(chip_select, { .clock_rate = 1 * MHz });
  return channel;
}
}  // namespace resources

void initialize_platform()
{
  // std::set_terminate(resources::terminate_handler);
  hal::cortex_m::initialize_interrupts<hal::stm32f1::irq::max>();

  // Set the MCU to the maximum clock speed
#if 0
  hal::stm32f1::configure_clocks(hal::stm32f1::clock_tree{
    .high_speed_external = 8 * mp_units::si::unit_symbols::MHz,
    .pll = {
      .enable = true,
      .source = hal::stm32f1::pll_source::high_speed_external,
      .multiply = hal::stm32f1::pll_multiply::multiply_by_9,
      .usb = {
        .divider = hal::stm32f1::usb_divider::divide_by_1_point_5,
      }
    },
    .system_clock = hal::stm32f1::system_clock_select::pll,
    .ahb = {
      .divider = hal::stm32f1::ahb_divider::divide_by_1,
      .apb1 = {
        .divider = hal::stm32f1::apb_divider::divide_by_2,
      },
      .apb2 = {
        .divider = hal::stm32f1::apb_divider::divide_by_1,
        .adc = {
          .divider = hal::stm32f1::adc_divider::divide_by_6,
        }
      },
    },
  });
#endif

  hal::stm32f1::activate_mco_pa8(
    hal::stm32f1::mco_source::pll_clock_divided_by_2);

  hal::stm32f1::release_jtag_pins();

  // Blocking calls such as hal::sleep_for() sleep the CPU with WFI, woken by
  // SysTick. Constructed after the clocks are configured, since SysTick counts
  // CPU cycles. The waiter installs itself when constructed.
  static hal::cortex_m::systick_timer timer(
    hal::stm32f1::frequency(hal::stm32f1::peripheral::cpu));
  static hal::cortex_m::wfi_waiter waiter(timer);
}
