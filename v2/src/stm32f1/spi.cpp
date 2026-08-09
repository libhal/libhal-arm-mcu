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

#include <atomic>
#include <coroutine>
#include <optional>

module hal.arm_mcu.stm32f1;

import hal;
import hal.util;
import hal.arm_mcu.stm32_generic;

import :constants;
import :pin;
import :power;
import :clock;

namespace hal::stm32f1 {
namespace {
/// @throws hal::operation_not_supported - if `p_id` is not `spi1`, `spi2`,
/// or `spi3`.
uptr peripheral_to_register(peripheral p_id)
{
  // See Chapter 4: Pin definition, Table 9 in RM0008 for these magic numbers
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
void configure_bus_pins(peripheral p_id)
{
  // Datasheet: Chapter 4: Pin definition Table 9
  switch (p_id) {
    case peripheral::spi1:
      bit_modify(alternative_function_io->mapr).clear<pin_remap::spi1_remap>();
      configure_pin({ .port = 'A', .pin = 5 },
                    push_pull_alternative_output);            // clock
      configure_pin({ .port = 'A', .pin = 6 }, input_float);  // cipo
      configure_pin({ .port = 'A', .pin = 7 },
                    push_pull_alternative_output);  // copi
      break;
    case peripheral::spi2:
      configure_pin({ .port = 'B', .pin = 13 },
                    push_pull_alternative_output);             // clock
      configure_pin({ .port = 'B', .pin = 14 }, input_float);  // cipo
      configure_pin({ .port = 'B', .pin = 15 },
                    push_pull_alternative_output);  // copi
      break;
    case peripheral::spi3:
      bit_modify(alternative_function_io->mapr2).set<pin_remap2::spi3_remap>();
      configure_pin({ .port = 'C', .pin = 10 },
                    push_pull_alternative_output);             // clock
      configure_pin({ .port = 'C', .pin = 11 }, input_float);  // cipo
      configure_pin({ .port = 'C', .pin = 12 },
                    push_pull_alternative_output);  // copi
      break;
    default:
      throw hal::operation_not_supported(nullptr);
  }
}

class channel final : public hal::spi_channel
{
public:
  channel(hal::ptr<spi> p_manager,
          hal::ptr<hal::output_pin> p_chip_select,
          hal::spi_channel::settings const& p_settings)
    : m_manager(p_manager)
    , m_chip_select(p_chip_select)
    , m_settings(p_settings)
  {
  }

  channel(channel const&) = delete;
  channel& operator=(channel const&) = delete;
  channel(channel&&) = delete;
  channel& operator=(channel&&) = delete;

  ~channel() override = default;

private:
  async::future<void> driver_configure(
    async::context&,
    hal::spi_channel::settings const& p_settings) override
  {
    m_settings = p_settings;
    return {};
  }

  async::future<hal::hertz> driver_clock_rate(async::context&) override
  {
    return m_manager->achievable_clock_rate(m_settings);
  }

  async::future<void> driver_chip_select(async::context& p_context,
                                         bool p_select) override
  {
    co_await m_manager->chip_select(
      p_context, m_chip_select, m_settings, p_select);
    m_asserted = p_select;
  }

  async::future<void> driver_transfer(
    async::context& p_context,
    mem::scatter_span<hal::byte const> p_data_out,
    mem::scatter_span<hal::byte> p_data_in,
    hal::byte p_filler) override
  {
    if (m_asserted) {
      co_await m_manager->transfer(p_context, p_data_out, p_data_in, p_filler);
      co_return;
    }

    co_await driver_chip_select(p_context, true);
    co_await m_manager->transfer(p_context, p_data_out, p_data_in, p_filler);
    co_await driver_chip_select(p_context, false);
  }

  hal::ptr<spi> m_manager;
  hal::ptr<hal::output_pin> m_chip_select;
  hal::spi_channel::settings m_settings;
  std::atomic<bool> m_asserted = false;
};
}  // namespace

struct spi::impl
{
  impl(peripheral p_id, void* p_reg)
    : id(p_id)
    , driver(p_reg)
  {
  }

  peripheral id;
  hal::stm32_generic::spi driver;
  async::mutex bus_lock;
};

hal::ptr<spi> spi::create(hal::allocator p_allocator, peripheral p_id)
{
  return hal::allocate<spi>(p_allocator, private_key{}, p_allocator, p_id);
}

spi::spi(private_key, hal::allocator p_allocator, peripheral p_id)
  // NOLINTBEGIN(performance-no-int-to-ptr)
  : pimpl(p_allocator,
          p_id,
          reinterpret_cast<void*>(peripheral_to_register(p_id)))
// NOLINTEND(performance-no-int-to-ptr)
{
  configure_bus_pins(p_id);
  power_on(p_id);
}

spi::~spi()
{
  power_off(inner().id);
}

hal::ptr<hal::spi_channel> spi::acquire_channel(
  hal::ptr<hal::output_pin> const& p_chip_select,
  hal::spi_channel::settings const& p_settings)
{
  return hal::allocate<channel>(
    memory_resource(), strong_from_this(), p_chip_select, p_settings);
}

async::future<void> spi::chip_select(
  async::context& p_context,
  hal::ptr<hal::output_pin> const& p_chip_select,
  hal::spi_channel::settings const& p_settings,
  bool p_select)
{
  auto& state = inner();

  if (p_select) {
    auto guard = co_await state.bus_lock.lock(p_context);
    state.driver.configure(p_settings, frequency(state.id));
    // Chip select is active low.
    co_await p_chip_select->level(p_context, false);
  } else {
    co_await p_chip_select->level(p_context, true);
  }
}

async::future<void> spi::transfer(async::context&,
                                  mem::scatter_span<hal::byte const> p_data_out,
                                  mem::scatter_span<hal::byte> p_data_in,
                                  hal::byte p_filler)
{
  // TODO(kammce): Convert generic stm32 spi into a coroutine
  inner().driver.transfer(p_data_out, p_data_in, p_filler);
  return {};
}

hal::hertz spi::achievable_clock_rate(
  hal::spi_channel::settings const& p_settings)
{
  return hal::stm32_generic::achievable_clock_rate(frequency(inner().id),
                                                   p_settings.clock_rate);
}
}  // namespace hal::stm32f1
