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
  auto const prescaler = hal::stm32_generic::counter_prescaler(
    p_timer_clock.numerical_value_in(mp_units::si::hertz),
    p_max_interrupt_rate.numerical_value_in(mp_units::si::hertz));
  if (prescaler == 0) {
    throw hal::argument_out_of_domain(nullptr);
  }
  return prescaler;
}

/// The timers each timer can count through its internal trigger inputs
/// ITR0 to ITR3 (RM0008 Tables 82 and 86)
struct trigger_inputs
{
  peripheral high;
  std::array<peripheral, 4> lows;
};

constexpr std::array<trigger_inputs, 6> trigger_table{ {
  { .high = peripheral::timer1,
    .lows = { peripheral::timer5,
              peripheral::timer2,
              peripheral::timer3,
              peripheral::timer4 } },
  { .high = peripheral::timer8,
    .lows = { peripheral::timer1,
              peripheral::timer2,
              peripheral::timer4,
              peripheral::timer5 } },
  { .high = peripheral::timer2,
    .lows = { peripheral::timer1,
              peripheral::timer8,
              peripheral::timer3,
              peripheral::timer4 } },
  { .high = peripheral::timer3,
    .lows = { peripheral::timer1,
              peripheral::timer2,
              peripheral::timer5,
              peripheral::timer4 } },
  { .high = peripheral::timer4,
    .lows = { peripheral::timer1,
              peripheral::timer2,
              peripheral::timer3,
              peripheral::timer8 } },
  { .high = peripheral::timer5,
    .lows = { peripheral::timer2,
              peripheral::timer3,
              peripheral::timer4,
              peripheral::timer8 } },
} };

/**
 * @brief Check that the pair can chain, and that both are free
 *
 * Runs before either timer is claimed, so nothing needs undoing.
 *
 * @return u32 - the high timer's trigger input that sees the low timer
 * @throws hal::operation_not_supported - if `p_high` can't count `p_low`
 * @throws hal::device_or_resource_busy - if either timer is claimed
 */
u32 check_chain(peripheral p_low, peripheral p_high)
{
  for (auto const& entry : trigger_table) {
    if (entry.high != p_high) {
      continue;
    }
    for (u32 input = 0; input < entry.lows.size(); input++) {
      if (entry.lows[input] != p_low) {
        continue;
      }
      if (is_claimed(p_low) or is_claimed(p_high)) {
        throw hal::device_or_resource_busy(nullptr);
      }
      return input;
    }
  }
  throw hal::operation_not_supported(nullptr);
}
}  // namespace

timer_clock_16bit::timer_clock_16bit(
  peripheral p_id,
  timer_clock_16bit_settings const& p_settings)
  : m_prescaler(
      prescaler_for(stm32f1::frequency(p_id), p_settings.max_interrupt_rate))
  , m_index(claim_timer(p_id))
  , m_frequency(stm32f1::frequency(p_id) / m_prescaler)
  , m_counter(timer_address(m_index))
{
  route_timer_interrupt(m_index, &handle_update, this);
  m_counter.start(m_prescaler);
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
  return m_frequency;
}

u64 timer_clock_16bit::driver_uptime()
{
  return m_counter.uptime();
}

chained_32bit_timer_clock::chained_32bit_timer_clock(peripheral p_low,
                                                     peripheral p_high)
  : m_trigger_input(check_chain(p_low, p_high))
  // Neither claim can throw now: check_chain() found both free.
  , m_low_index(claim_timer(p_low))
  , m_high_index(claim_timer(p_high))
  , m_frequency(stm32f1::frequency(p_low))
  , m_counter(timer_address(m_low_index), timer_address(m_high_index))
{
  route_timer_interrupt(m_high_index, &handle_update, this);
  m_counter.start(m_trigger_input);
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
  return m_frequency;
}

u64 chained_32bit_timer_clock::driver_uptime()
{
  return m_counter.uptime();
}
}  // namespace hal::stm32f1
