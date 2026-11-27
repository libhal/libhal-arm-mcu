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
#include <bitset>

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
/// @throws hal::operation_not_supported - see `timer_pins()`
peripheral checked_remap(peripheral p_id, timer_remap p_remap)
{
  static_cast<void>(timer_pins(p_id, p_remap));
  return p_id;
}

hal::stm32_generic::pwm_period period_for(hal::hertz p_clock,
                                          hal::hertz p_frequency)
{
  return hal::stm32_generic::pwm_periods(
    { .clock_hz = p_clock.numerical_value_in(mp_units::si::hertz),
      .frequency_hz = p_frequency.numerical_value_in(mp_units::si::hertz) });
}
}  // namespace

pwm::pwm(peripheral p_id, pwm_settings const& p_settings)
  // claim_timer() is the last call that can throw, so nothing needs undoing
  // after it
  : pwm(claim_timer(checked_remap(p_id, p_settings.remap)), p_settings)
{
}

pwm::pwm(u8 p_index, pwm_settings const& p_settings)
  : m_driver(timer_address(p_index))
  , m_remap(p_settings.remap)
  , m_index(p_index)
{
  auto const id = timer_routes[m_index].id;
  // AFIO is shared, so it is powered on, never claimed.
  power_on(peripheral::afio);
  apply_timer_remap(id, timer_pins(id, m_remap).remap_value);
  m_driver.start(period_for(clock(), p_settings.frequency));
  if ((id == peripheral::timer1) or (id == peripheral::timer8)) {
    m_driver.enable_main_output();
  }
}

pwm::~pwm()
{
  m_driver.stop();
  release_timer(m_index);
}

hal::hertz pwm::clock() const
{
  return stm32f1::frequency(timer_routes[m_index].id);
}

void pwm::driver_frequency(hal::hertz p_frequency)
{
  auto const old_reload = m_driver.period().reload;
  auto const period = period_for(clock(), p_frequency);
  m_driver.period(period);
  // Keep each duty cycle: rescale its compare value to the new number of
  // steps. 100% stays 100%: the old reload maps to the new one.
  for (usize index = 0; index < m_claimed_channels.size(); index++) {
    if (not m_claimed_channels.test(index)) {
      continue;
    }
    auto const channel = static_cast<timer_channel>(index);
    auto const scaled = hal::divide<hal::rounding::round, u64>(
      { .num = u64{ m_driver.compare(channel) } * period.reload,
        .den = old_reload });
    // NOTE: Safe to cast without narrowing: at most the new reload.
    m_driver.compare(channel, static_cast<u32>(scaled));
  }
}

pwm::channel::channel(hal::ref<pwm> p_pwm, timer_channel p_channel)
  : m_pwm(p_pwm)
  , m_channel(p_channel)
{
  auto const id = timer_routes[m_pwm->m_index].id;
  auto const output =
    timer_pins(id, m_pwm->m_remap).channels[hal::value(m_channel)];
  if (m_pwm->m_claimed_channels.test(hal::value(m_channel)) or
      is_pin_claimed(output)) {
    throw hal::device_or_resource_busy(this);
  }
  claim_pin(output);
  configure_pin(output, push_pull_alternative_output);
  m_pwm->m_claimed_channels.set(hal::value(m_channel));
  m_pwm->m_driver.enable(m_channel);
}

pwm::channel::~channel()
{
  m_pwm->m_driver.disable(m_channel);
  m_pwm->m_claimed_channels.reset(hal::value(m_channel));
  auto const id = timer_routes[m_pwm->m_index].id;
  auto const output =
    timer_pins(id, m_pwm->m_remap).channels[hal::value(m_channel)];
  reset_pin(output);
  release_pin(output);
}

hal::hertz pwm::channel::driver_frequency()
{
  // Rounded to the nearest hertz: 50 Hz at 72 MHz is 49.9997 Hz, which
  // rounding down would report as 49.
  auto const period = m_pwm->m_driver.period();
  auto const hertz = hal::divide<hal::rounding::round, u64>(
    { .num = m_pwm->clock().numerical_value_in(mp_units::si::hertz),
      .den = u64{ period.prescaler } * period.reload });
  // NOTE: Safe to cast without narrowing: at most half the timer clock.
  return static_cast<u32>(hertz) * mp_units::si::hertz;
}

void pwm::channel::driver_duty_cycle(u16 p_duty_cycle)
{
  m_pwm->m_driver.compare(m_channel,
                          hal::stm32_generic::pwm_compare(
                            p_duty_cycle, m_pwm->m_driver.period().reload));
}
}  // namespace hal::stm32f1
