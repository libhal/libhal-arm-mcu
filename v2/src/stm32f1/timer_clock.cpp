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

module hal.arm_mcu.stm32f1;

import hal;
import hal.util;
import hal.arm_mcu.stm32_generic;

import :constants;
import :clock;
import :power;
import :timer;

namespace hal::stm32f1 {
namespace {
/// @throws hal::argument_out_of_domain - if `p_max_interrupt_rate` is 0 Hz
u32 prescaler_for(hal::hertz p_timer_clock, hal::hertz p_max_interrupt_rate)
{
  return hal::stm32_generic::counter_prescaler(
    { .clock_hz = p_timer_clock.numerical_value_in(mp_units::si::hertz),
      .max_interrupt_rate_hz =
        p_max_interrupt_rate.numerical_value_in(mp_units::si::hertz) });
}

/// Which timer of a pair counts the clock, which counts its wraps, and the
/// high timer's trigger input wired to the low one (RM0008 Tables 82 and 86)
struct timer_chain
{
  peripheral low;
  peripheral high;
  u8 trigger_input;
};

/// Indexed by `timer_pair`
constexpr std::array<timer_chain, 14> timer_chains{ {
  { .low = peripheral::timer1, .high = peripheral::timer2, .trigger_input = 0 },
  { .low = peripheral::timer1, .high = peripheral::timer3, .trigger_input = 0 },
  { .low = peripheral::timer1, .high = peripheral::timer4, .trigger_input = 0 },
  { .low = peripheral::timer2, .high = peripheral::timer3, .trigger_input = 1 },
  { .low = peripheral::timer2, .high = peripheral::timer4, .trigger_input = 1 },
  { .low = peripheral::timer3, .high = peripheral::timer4, .trigger_input = 2 },
  // TIM5 can't count TIM1, so TIM1 counts TIM5
  { .low = peripheral::timer5, .high = peripheral::timer1, .trigger_input = 0 },
  { .low = peripheral::timer1, .high = peripheral::timer8, .trigger_input = 0 },
  { .low = peripheral::timer2, .high = peripheral::timer5, .trigger_input = 0 },
  { .low = peripheral::timer2, .high = peripheral::timer8, .trigger_input = 1 },
  { .low = peripheral::timer3, .high = peripheral::timer5, .trigger_input = 1 },
  { .low = peripheral::timer4, .high = peripheral::timer5, .trigger_input = 2 },
  { .low = peripheral::timer4, .high = peripheral::timer8, .trigger_input = 2 },
  { .low = peripheral::timer5, .high = peripheral::timer8, .trigger_input = 3 },
} };
}  // namespace

timer_clock_16bit::timer_clock_16bit(
  peripheral p_id,
  timer_clock_16bit_settings const& p_settings)
  : timer_clock_16bit(claim_timer(p_id), p_settings)
{
}

timer_clock_16bit::timer_clock_16bit(
  u8 p_index,
  timer_clock_16bit_settings const& p_settings)
  : m_counter(timer_address(p_index))
  , m_index(p_index)
{
  route_timer_interrupt(m_index, &handle_update, this);
  m_counter.start(prescaler_for(stm32f1::frequency(timer_routes[m_index].id),
                                p_settings.max_interrupt_rate));
}

timer_clock_16bit::~timer_clock_16bit()
{
  m_counter.stop();
  release_timer(m_index);
}

void timer_clock_16bit::handle_update(void* p_self) noexcept
{
  static_cast<timer_clock_16bit*>(p_self)->m_counter.update();
}

hal::hertz timer_clock_16bit::driver_frequency()
{
  return stm32f1::frequency(timer_routes[m_index].id) / m_counter.prescaler();
}

u64 timer_clock_16bit::driver_uptime()
{
  return m_counter.uptime();
}

claimed_timer_pair claim_timer_pair(timer_pair p_pair)
{
  auto const& chain = timer_chains[hal::value(p_pair)];
  // Checked first, so neither claim below can throw and leave the other
  // claimed
  if (is_claimed(chain.low) or is_claimed(chain.high)) {
    throw hal::device_or_resource_busy(nullptr);
  }
  return { .low_index = claim_timer(chain.low),
           .high_index = claim_timer(chain.high),
           .trigger_input = chain.trigger_input };
}

chained_32bit_timer_clock::chained_32bit_timer_clock(timer_pair p_pair)
  : chained_32bit_timer_clock(claim_timer_pair(p_pair))
{
}

chained_32bit_timer_clock::chained_32bit_timer_clock(claimed_timer_pair p_pair)
  : m_counter(timer_address(p_pair.low_index), timer_address(p_pair.high_index))
  , m_low_index(p_pair.low_index)
  , m_high_index(p_pair.high_index)
{
  route_timer_interrupt(m_high_index, &handle_update, this);
  m_counter.start(p_pair.trigger_input);
}

chained_32bit_timer_clock::~chained_32bit_timer_clock()
{
  m_counter.stop();
  release_timer(m_high_index);
  release_timer(m_low_index);
}

void chained_32bit_timer_clock::handle_update(void* p_self) noexcept
{
  static_cast<chained_32bit_timer_clock*>(p_self)->m_counter.update();
}

hal::hertz chained_32bit_timer_clock::driver_frequency()
{
  // The low timer counts every cycle of its clock
  return stm32f1::frequency(timer_routes[m_low_index].id);
}

u64 chained_32bit_timer_clock::driver_uptime()
{
  return m_counter.uptime();
}
}  // namespace hal::stm32f1
