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

module hal.arm_mcu.stm32f1;

import hal;
import hal.util;
import hal.arm_mcu.stm32_generic;

import :constants;
import :clock;
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
}  // namespace hal::stm32f1
