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
import :pin;
import :power;
import :timer;
import :pwm;

namespace hal::stm32f1 {
namespace {
hal::stm32_generic::pwm_period period_for(hal::hertz p_clock,
                                          hal::hertz p_frequency)
{
  return hal::stm32_generic::pwm_periods(
    { .clock_hz = p_clock.numerical_value_in(mp_units::si::hertz),
      .frequency_hz = p_frequency.numerical_value_in(mp_units::si::hertz) });
}
}  // namespace

pwm_base::pwm_base(peripheral p_id,
                   u8 p_remap_value,
                   pwm_settings const& p_settings)
  // claim_timer() is the last call that can throw, so nothing needs undoing
  // after it
  : pwm_base(claim_timer(p_id), p_settings)
{
  // AFIO is shared, so it is powered on, never claimed. No pin is an output
  // yet, so the remap can follow the counter's start.
  power_on(peripheral::afio);
  apply_timer_remap(p_id, p_remap_value);
}

pwm_base::pwm_base(u8 p_index, pwm_settings const& p_settings)
  : m_driver(timer_address(p_index))
  , m_clock(stm32f1::frequency(timer_routes[p_index].id))
  , m_period(period_for(m_clock, p_settings.frequency))
  , m_index(p_index)
{
  auto const id = timer_routes[m_index].id;
  m_driver.start(m_period);
  if ((id == peripheral::timer1) or (id == peripheral::timer8)) {
    m_driver.enable_main_output();
  }
}

pwm_base::~pwm_base()
{
  m_driver.stop();
  release_timer(m_index);
}

void pwm_base::driver_frequency(hal::hertz p_frequency)
{
  m_period = period_for(m_clock, p_frequency);
  m_driver.period(m_period);
  // Keep each duty cycle at the new number of steps. A channel without a pin
  // has its output off, so its compare value doesn't matter.
  for (usize index = 0; index < m_duty_cycles.size(); index++) {
    m_driver.compare(
      static_cast<timer_channel>(index),
      hal::stm32_generic::pwm_compare(m_duty_cycles[index], m_period.reload));
  }
}

pwm_base::channel::channel(hal::ref<pwm_base> p_pwm,
                           timer_channel p_channel,
                           pin p_pin)
  : m_pwm(p_pwm)
  , m_pin(p_pin)
  , m_channel(p_channel)
{
  // A channel's pin is fixed by the remap, so a claimed pin also means a
  // claimed channel.
  claim_pin(m_pin);
  configure_pin(m_pin, push_pull_alternative_output);
  m_pwm->m_duty_cycles[hal::value(m_channel)] = 0;
  m_pwm->m_driver.enable(m_channel);
}

pwm_base::channel::~channel()
{
  m_pwm->m_driver.disable(m_channel);
  reset_pin(m_pin);
  release_pin(m_pin);
}

hal::hertz pwm_base::channel::driver_frequency()
{
  // Rounded to the nearest hertz: 50 Hz at 72 MHz is 49.9997 Hz, which
  // rounding down would report as 49.
  auto const& period = m_pwm->m_period;
  auto const hertz = hal::divide<hal::rounding::round, u64>(
    { .num = m_pwm->m_clock.numerical_value_in(mp_units::si::hertz),
      .den = u64{ period.prescaler } * period.reload });
  // NOTE: Safe to cast without narrowing: at most half the timer clock.
  return static_cast<u32>(hertz) * mp_units::si::hertz;
}

void pwm_base::channel::driver_duty_cycle(u16 p_duty_cycle)
{
  m_pwm->m_duty_cycles[hal::value(m_channel)] = p_duty_cycle;
  m_pwm->m_driver.compare(
    m_channel,
    hal::stm32_generic::pwm_compare(p_duty_cycle, m_pwm->m_period.reload));
}
}  // namespace hal::stm32f1
