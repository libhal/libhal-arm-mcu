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
import :pin;
import :timer;

namespace hal::stm32f1 {
namespace {
/// What each timer's update interrupt calls
struct timer_slot
{
  timer_handler handler = nullptr;
  void* object = nullptr;
};

constinit std::array<timer_slot, timer_routes.size()> timer_slots{};

/**
 * One interrupt handler serves every timer. Each driver's handler checks its
 * own timer's update flag, so calling every routed handler is safe, and it
 * links one handler instead of one per timer.
 */
void timer_isr()
{
  for (auto const& slot : timer_slots) {
    if (slot.handler != nullptr) {
      slot.handler(slot.object);
    }
  }
}

/// @throws hal::operation_not_supported - if `p_id` isn't TIM1 to TIM8
u8 timer_index(peripheral p_id)
{
  for (usize index = 0; index < timer_routes.size(); index++) {
    if (timer_routes[index].id == p_id) {
      // NOTE: Safe to cast without narrowing: there are 8 timers.
      return static_cast<u8>(index);
    }
  }
  throw hal::operation_not_supported(nullptr);
}
}  // namespace

u8 claim_timer(peripheral p_id)
{
  auto const index = timer_index(p_id);
  // The last call that can throw, so a driver has nothing to undo after it
  claim(p_id);
  power_on(p_id);
  // Start from the reset state, whatever ran before (e.g. a bootloader) left.
  reset_peripheral(p_id);
  return index;
}

void release_timer(u8 p_index) noexcept
{
  auto const& route = timer_routes[p_index];
  cortex_m::disable_interrupt(route.interrupt);
  timer_slots[p_index] = {};
  power_off(route.id);
  release(route.id);
}

void route_timer_interrupt(u8 p_index, timer_handler p_handler, void* p_object)
{
  timer_slots[p_index] = { .handler = p_handler, .object = p_object };
  cortex_m::enable_interrupt(timer_routes[p_index].interrupt, &timer_isr);
}

void apply_timer_remap(peripheral p_id, u32 p_value)
{
  switch (p_id) {
    case peripheral::timer1:
      write_remap<pin_remap::tim1_rempap>(p_value);
      break;
    case peripheral::timer2:
      write_remap<pin_remap::tim2_rempap>(p_value);
      break;
    case peripheral::timer3:
      write_remap<pin_remap::tim3_rempap>(p_value);
      break;
    case peripheral::timer4:
      write_remap<pin_remap::tim4_rempap>(p_value);
      break;
    default:
      // TIM5 and TIM8 have no remap
      break;
  }
}

void* timer_address(u8 p_index)
{
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  return reinterpret_cast<void*>(timer_routes[p_index].address);
}

timer::timer(peripheral p_id)
  : timer(claim_timer(p_id))
{
}

timer::timer(u8 p_index)
  : m_frequency(frequency(timer_routes[p_index].id))
  , m_driver(timer_address(p_index))
  , m_index(p_index)
{
  route_timer_interrupt(m_index, &handle_update, this);
}

timer::~timer()
{
  m_driver.stop();
  release_timer(m_index);
}

void timer::handle_update(void* p_self) noexcept
{
  auto& self = *static_cast<timer*>(p_self);
  if (not self.m_driver.update()) {
    return;
  }
  // Read the callback once: it can be cleared between expirations.
  auto* const callback = self.m_callback;
  if (callback != nullptr) {
    callback->callback();
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
