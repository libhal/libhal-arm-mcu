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
#include <cstdint>

module hal.arm_mcu.stm32f1;

import hal;
import hal.util;
import hal.arm_mcu.cortex_m;
import hal.arm_mcu.stm32_generic;

import :constants;
import :power;
import :clock;
import :timer;

namespace hal::stm32f1 {
namespace {
/// Where each supported timer lives and which interrupt reports its updates
struct timer_route
{
  peripheral id;
  irq interrupt;
  std::uintptr_t address;
};

/// TIM1 to TIM8 (RM0008 section 3.3, memory map). TIM1 and TIM8 have their
/// own update interrupt; the others have one interrupt for everything.
constexpr std::array<timer_route, 8> routes{ {
  { .id = peripheral::timer1,
    .interrupt = irq::tim1_up,
    .address = 0x4001'2C00 },
  { .id = peripheral::timer2, .interrupt = irq::tim2, .address = 0x4000'0000 },
  { .id = peripheral::timer3, .interrupt = irq::tim3, .address = 0x4000'0400 },
  { .id = peripheral::timer4, .interrupt = irq::tim4, .address = 0x4000'0800 },
  { .id = peripheral::timer5, .interrupt = irq::tim5, .address = 0x4000'0C00 },
  { .id = peripheral::timer6, .interrupt = irq::tim6, .address = 0x4000'1000 },
  { .id = peripheral::timer7, .interrupt = irq::tim7, .address = 0x4000'1400 },
  { .id = peripheral::timer8,
    .interrupt = irq::tim8_up,
    .address = 0x4001'3400 },
} };

/// The driver each timer's interrupt reports to
constinit std::array<timer*, routes.size()> drivers{};

/// @throws hal::operation_not_supported - if `p_id` isn't TIM1 to TIM8
u8 route_index(peripheral p_id)
{
  for (usize index = 0; index < routes.size(); index++) {
    if (routes[index].id == p_id) {
      // NOTE: Safe to cast without narrowing: there are 8 routes.
      return static_cast<u8>(index);
    }
  }
  throw hal::operation_not_supported(nullptr);
}

void* timer_address(u8 p_index)
{
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  return reinterpret_cast<void*>(routes[p_index].address);
}
}  // namespace

timer::timer(peripheral p_id)
  : m_frequency(frequency(p_id))
  , m_driver(timer_address(route_index(p_id)))
  , m_index(route_index(p_id))
{
  // The last call that can throw, so nothing needs undoing after it
  claim(p_id);
  power_on(p_id);
  // Start from the reset state, whatever ran before (e.g. a bootloader) left.
  reset_peripheral(p_id);

  drivers[m_index] = this;
  cortex_m::enable_interrupt(routes[m_index].interrupt, &isr);
}

timer::~timer()
{
  m_driver.stop();
  cortex_m::disable_interrupt(routes[m_index].interrupt);
  drivers[m_index] = nullptr;
  power_off(routes[m_index].id);
  release(routes[m_index].id);
}

void timer::isr()
{
  // One handler serves every timer: update() only reports an expiry for a
  // timer whose own update flag is set, so asking each driver is safe. It
  // links one handler instead of one per timer.
  for (auto* const driver : drivers) {
    if (driver == nullptr or not driver->m_driver.update()) {
      continue;
    }
    // Read the callback once: it can be cleared between expirations.
    auto* const callback = driver->m_callback;
    if (callback != nullptr) {
      callback->callback();
    }
  }
}

bool timer::driver_scheduled()
{
  return m_driver.armed();
}

void timer::driver_schedule(hal::time_duration p_delay, hal::timer_mode p_mode)
{
  // Rounds up, so the delay is never shorter than asked. A delay that isn't
  // positive gives 0, which runs as the shortest period.
  auto const cycles = hal::cycles_per(m_frequency, p_delay);
  m_driver.start(hal::stm32_generic::timer_periods(cycles), p_mode);
}

void timer::driver_cancel() noexcept
{
  m_driver.stop();
}

void timer::driver_on_expire(hal::timed_callback& p_callback)
{
  if (m_callback != nullptr) {
    throw hal::subscription_busy(this);
  }
  m_callback = &p_callback;
}

void timer::driver_clear_expire() noexcept
{
  m_driver.stop();
  m_callback = nullptr;
}
}  // namespace hal::stm32f1
