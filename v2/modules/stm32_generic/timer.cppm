// Copyright 2024 - 2026 Khalil Estell and the libhal contributors
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

export module hal.arm_mcu.stm32_generic:timer;

import hal;
import hal.util;

import :registers;

namespace hal::stm32_generic {
/// How a timer counts out a delay: `counts` equal periods, each
/// `prescaler * reload` timer clock cycles long
export struct timer_period
{
  /// How many periods the delay takes
  u64 counts;
  /// The prescaler's divider, 1 to 65,536 (PSC + 1)
  u32 prescaler;
  /// Prescaled ticks per period, 2 to 65,536 (ARR + 1)
  u32 reload;
};

/**
 * @brief Split a delay into equal timer periods
 *
 * One period counts at most 65,536 * 65,536 cycles, with a 16-bit prescaler
 * and a 16-bit counter. A longer delay runs as the fewest equal periods that
 * fit, so a periodic timer keeps reloading in hardware and never drifts. With
 * two or more, each is over half the longest period, so the interrupt has
 * plenty of time to run before the next one ends.
 *
 * Each period picks the smallest prescaler that fits, for the finest ticks,
 * and rounds up. The delay never ends early, and runs fewer than `prescaler`
 * cycles per period longer than asked. A delay of fewer than 2 cycles runs as
 * 2, since a counter with a reload of 0 doesn't count.
 *
 * @param p_cycles - the delay in timer clock cycles
 * @return timer_period - the periods that count out the delay
 */
export [[nodiscard]] constexpr timer_period timer_periods(u64 p_cycles)
{
  constexpr u64 max_divider = 65'536;
  constexpr u64 max_period = max_divider * max_divider;
  constexpr u64 min_period = 2;

  u64 counts = 1;
  u64 period = p_cycles;
  if (p_cycles > max_period) {
    counts = hal::divide<hal::rounding::ceil, u64>(
      { .num = p_cycles, .den = max_period });
    period =
      hal::divide<hal::rounding::ceil, u64>({ .num = p_cycles, .den = counts });
  }
  period = std::max(period, min_period);

  auto const prescaler = hal::divide<hal::rounding::ceil, u64>(
    { .num = period, .den = max_divider });
  auto const reload =
    hal::divide<hal::rounding::ceil, u64>({ .num = period, .den = prescaler });

  // NOTE: Safe to cast without narrowing: both are at most 65,536.
  return { .counts = counts,
           .prescaler = static_cast<u32>(prescaler),
           .reload = static_cast<u32>(reload) };
}

/**
 * @brief Delay counting shared by the STM32 basic, general-purpose and
 * advanced timers
 *
 * The counter counts up from 0 to ARR and raises an update interrupt each time
 * it wraps. The platform driver claims and powers the timer, routes that
 * interrupt, and calls `update()` from it.
 *
 * This class is meant to only be used by platform libraries or drivers.
 */
export class timer
{
public:
  /**
   * @brief Construct over a timer's registers
   *
   * Doesn't touch the hardware.
   *
   * @param p_address - the timer's base address
   */
  explicit timer(void* p_address)
    : m_reg(timer_reg::from(p_address))
  {
  }

  /**
   * @brief Count out a delay, stopping any delay already counting
   *
   * Leaves the update interrupt enabled.
   *
   * @param p_period - the periods to count, from `timer_periods()`
   * @param p_mode - whether to stop after the delay or count it again
   * @pre The timer is powered on.
   */
  void start(timer_period p_period, hal::timer_mode p_mode) noexcept
  {
    stop();

    m_mode = p_mode;
    m_counts = p_period.counts;
    m_counts_left = p_period.counts;
    m_armed = true;

    // Count up, without one-pulse mode, and only raise an update interrupt
    // when the counter wraps, not on the update generated below. ARR is
    // preloaded so a new reload takes effect cleanly at the next update.
    bit_modify(m_reg->control_register)
      .set<control::auto_reload_preload>()
      .clear<control::center_aligned_mode>()
      .clear<control::direction>()
      .clear<control::one_pulse_mode>()
      .set<control::update_request_source>();
    m_reg->prescale_register = p_period.prescaler - 1U;
    m_reg->auto_reload_register = p_period.reload - 1U;
    // Load the prescaler and reload, and restart the counter from 0
    m_reg->event_generator_register = update_generation.value<u32>();
    clear_update_flag();
    bit_modify(m_reg->interrupt_enable_register).set<update_interrupt_enable>();
    bit_modify(m_reg->control_register).set<control::counter_enable>();
  }

  /**
   * @brief Stop counting
   *
   * Does nothing if no delay is counting. A pending update interrupt is
   * cleared, so `update()` won't report an expiry for it.
   *
   * @pre The timer is powered on.
   */
  void stop() noexcept
  {
    bit_modify(m_reg->control_register).clear<control::counter_enable>();
    clear_update_flag();
    m_armed = false;
    m_counts_left = 0;
  }

  /**
   * @brief Handle the update interrupt
   *
   * Call from the timer's update interrupt. A periodic delay starts counting
   * again, and a one-shot delay stops, before this returns, so the caller is
   * free to start or stop the timer from its callback.
   *
   * @return true - the delay has run out
   * @return false - more periods are left, or the interrupt wasn't an update
   * @pre The timer is powered on.
   */
  [[nodiscard]] bool update() noexcept
  {
    if (not bit_is_set<update_interrupt_flag>(m_reg->status_register)) {
      return false;
    }
    clear_update_flag();
    if (m_counts_left == 0) {
      return false;
    }

    // The counter has already wrapped and is counting the next period.
    m_counts_left = m_counts_left - 1U;
    if (m_counts_left > 0) {
      return false;
    }

    if (m_mode == hal::timer_mode::periodic) {
      m_counts_left = m_counts;
    } else {
      bit_modify(m_reg->control_register).clear<control::counter_enable>();
      m_armed = false;
    }
    return true;
  }

  /**
   * @brief Whether a delay is counting
   *
   * @return true - a one-shot delay hasn't run out, or a periodic delay is
   * counting
   */
  [[nodiscard]] bool armed() const noexcept
  {
    return m_armed;
  }

private:
  using control = timer_bits::control;
  static constexpr auto update_interrupt_enable =
    timer_bits::update_interrupt_enable;
  static constexpr auto update_interrupt_flag =
    timer_bits::update_interrupt_flag;
  static constexpr auto update_generation = timer_bits::update_generation;

  void clear_update_flag() noexcept
  {
    // The status flags are cleared by writing 0. Writing 1 leaves them as
    // they are, so only UIF clears.
    m_reg->status_register = ~update_interrupt_flag.value<u32>();
  }

  timer_reg* m_reg;
  u64 m_counts = 0;
  u64 m_counts_left = 0;
  hal::timer_mode m_mode = hal::timer_mode::one_shot;
  bool m_armed = false;
};
}  // namespace hal::stm32_generic
