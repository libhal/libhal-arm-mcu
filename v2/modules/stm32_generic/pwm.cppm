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

export module hal.arm_mcu.stm32_generic:pwm;

import hal;
import hal.util;

import :registers;
import :timer;

namespace hal::stm32_generic {
/// The counter settings for a PWM frequency
export struct pwm_period
{
  /// The prescaler's divider, 1 to 65,536 (PSC + 1)
  u32 prescaler;
  /// Prescaled ticks per period, 2 to 65,535 (ARR + 1). It is also how many
  /// duty cycle steps the channels have.
  u32 reload;
};

/// A timer clock and a PWM frequency, for `pwm_periods()`
export struct pwm_rate
{
  /// The timer clock
  u64 clock_hz;
  /// The PWM frequency
  u64 frequency_hz;
};

/**
 * @brief The counter settings closest to a PWM frequency
 *
 * Picks the smallest prescaler that fits, so the channels get the most duty
 * cycle steps, then the period closest to the request. Frequencies above half
 * the timer clock run at half the timer clock, the fastest a 2-step period
 * goes, and 0 Hz runs at the lowest frequency.
 *
 * @param p_rate - the timer clock and the PWM frequency
 * @return pwm_period - the prescaler and reload
 */
export [[nodiscard]] constexpr pwm_period pwm_periods(pwm_rate p_rate)
{
  constexpr u64 max_prescaler = 65'536;
  // Not 65,536: 100% duty needs a compare value of ARR + 1, and CCRx is 16
  // bits.
  constexpr u64 max_reload = 65'535;
  constexpr u64 min_reload = 2;
  if (p_rate.frequency_hz == 0) {
    return { .prescaler = max_prescaler, .reload = max_reload };
  }
  // The period in timer clock cycles, rounded to the nearest
  auto const cycles =
    std::max<u64>(hal::divide<hal::rounding::round, u64>(
                    { .num = p_rate.clock_hz, .den = p_rate.frequency_hz }),
                  min_reload);
  auto const prescaler = std::min<u64>(
    hal::divide<hal::rounding::ceil, u64>({ .num = cycles, .den = max_reload }),
    max_prescaler);
  auto const reload = std::clamp<u64>(
    hal::divide<hal::rounding::round, u64>({ .num = cycles, .den = prescaler }),
    min_reload,
    max_reload);
  // NOTE: Safe to cast without narrowing: both are at most 65,536.
  return { .prescaler = static_cast<u32>(prescaler),
           .reload = static_cast<u32>(reload) };
}

/**
 * @brief The compare value for a duty cycle
 *
 * Rounds to the nearest step. 0 keeps the output low, and 65,535 keeps it
 * high for the whole period.
 *
 * @param p_duty_cycle - 0 to 65,535 for 0% to 100%
 * @param p_reload - prescaled ticks per period (ARR + 1)
 * @return u32 - the value for CCRx, 0 to `p_reload`
 */
export [[nodiscard]] constexpr u32 pwm_compare(u16 p_duty_cycle, u32 p_reload)
{
  // NOTE: Safe to cast without narrowing: the result is at most p_reload.
  return static_cast<u32>(hal::divide<hal::rounding::round, u64>(
    { .num = u64{ p_duty_cycle } * p_reload, .den = 65'535 }));
}

/**
 * @brief PWM output shared by the STM32 general-purpose and advanced timers
 *
 * The counter counts up from 0 to ARR, and each channel in PWM mode 1 drives
 * its output high while the counter is below its compare value. ARR and the
 * compare values are preloaded, so a new frequency or duty cycle takes effect
 * at the next period, without a glitch.
 *
 * This class is meant to only be used by platform libraries or drivers.
 */
export class pwm
{
public:
  /**
   * @brief Construct over a timer's registers
   *
   * Doesn't touch the hardware.
   *
   * @param p_address - the timer's base address
   */
  explicit pwm(void* p_address)
    : m_reg(timer_reg::from(p_address))
  {
  }

  /**
   * @brief Start the counter with every channel's output off
   *
   * @param p_period - the counter settings, from `pwm_periods()`
   * @pre The timer is powered on.
   */
  void start(pwm_period p_period) noexcept
  {
    bit_modify(m_reg->control_register)
      .set<control::auto_reload_preload>()
      .clear<control::center_aligned_mode>()
      .clear<control::direction>()
      .clear<control::one_pulse_mode>();
    m_reg->cc_enable_register = 0;
    m_reg->prescale_register = p_period.prescaler - 1U;
    m_reg->auto_reload_register = p_period.reload - 1U;
    // Load the prescaler and reload, and restart the counter from 0
    m_reg->event_generator_register = update_generation.value<u32>();
    bit_modify(m_reg->control_register).set<control::counter_enable>();
  }

  /**
   * @brief Stop the counter and turn every channel's output off
   *
   * @pre The timer is powered on.
   */
  void stop() noexcept
  {
    bit_modify(m_reg->control_register).clear<control::counter_enable>();
    m_reg->cc_enable_register = 0;
  }

  /**
   * @brief Enable the advanced timer outputs (MOE)
   *
   * TIM1 and TIM8 drive no output until this is set, whatever the channels
   * are set to. Not for the general-purpose timers, which don't have it.
   *
   * @pre The timer is powered on.
   */
  void enable_main_output() noexcept
  {
    bit_modify(m_reg->break_and_deadtime_register).set<main_output_enable>();
  }

  /**
   * @brief Change the period, from the next period on
   *
   * The prescaler always waits for the next update, and so does ARR, which
   * is preloaded. Call `compare()` for each channel after this.
   *
   * @param p_period - the counter settings, from `pwm_periods()`
   * @pre The timer is powered on.
   */
  void period(pwm_period p_period) noexcept
  {
    m_reg->prescale_register = p_period.prescaler - 1U;
    m_reg->auto_reload_register = p_period.reload - 1U;
  }

  /**
   * @brief The period last set, read back from PSC and ARR
   *
   * Both read back the value written, even before it takes effect at the
   * next period.
   *
   * @return pwm_period - the prescaler and reload
   * @pre The timer is powered on.
   */
  [[nodiscard]] pwm_period period() const noexcept
  {
    return { .prescaler = (m_reg->prescale_register & 0xFFFFU) + 1U,
             .reload = (m_reg->auto_reload_register & 0xFFFFU) + 1U };
  }

  /**
   * @brief Drive a channel's output in PWM mode 1, starting low
   *
   * @param p_channel - the channel
   * @pre The timer is powered on.
   */
  void enable(timer_channel p_channel) noexcept
  {
    compare(p_channel, 0);
    auto const index = hal::value(p_channel);
    // CCMR1 holds CH1 and CH2, CCMR2 holds CH3 and CH4, 8 bits each:
    // CCxS = 0b00 (output), OCxPE = 1 (preload), OCxM = 0b110 (PWM mode 1)
    auto& mode = (index < 2U) ? m_reg->capture_compare_mode_register
                              : m_reg->capture_compare_mode_register_2;
    auto const shift = (index % 2U) * 8U;
    auto const field = hal::bit_mask{ .position = shift, .width = 8 };
    bit_modify(mode).insert(field, output_mode_pwm1);
    // CCxE, active high
    auto const enable_bit = hal::bit_mask{ .position = index * 4U, .width = 2 };
    bit_modify(m_reg->cc_enable_register).insert(enable_bit, 0b01U);
  }

  /**
   * @brief Turn a channel's output off
   *
   * @param p_channel - the channel
   * @pre The timer is powered on.
   */
  void disable(timer_channel p_channel) noexcept
  {
    auto const enable_bit =
      hal::bit_mask{ .position = hal::value(p_channel) * 4U, .width = 2 };
    bit_modify(m_reg->cc_enable_register).insert(enable_bit, 0b00U);
    compare(p_channel, 0);
  }

  /**
   * @brief Set a channel's compare value, from the next period on
   *
   * @param p_channel - the channel
   * @param p_value - the value for CCRx, from `pwm_compare()`
   * @pre The timer is powered on.
   */
  void compare(timer_channel p_channel, u32 p_value) noexcept
  {
    switch (p_channel) {
      case timer_channel::ch1:
        m_reg->capture_compare_register = p_value;
        break;
      case timer_channel::ch2:
        m_reg->capture_compare_register_2 = p_value;
        break;
      case timer_channel::ch3:
        m_reg->capture_compare_register_3 = p_value;
        break;
      case timer_channel::ch4:
        m_reg->capture_compare_register_4 = p_value;
        break;
    }
  }

  /**
   * @brief A channel's compare value last set, read back from CCRx
   *
   * CCRx reads back the value written, even before it takes effect at the
   * next period.
   *
   * @param p_channel - the channel
   * @return u32 - the value in CCRx
   * @pre The timer is powered on.
   */
  [[nodiscard]] u32 compare(timer_channel p_channel) const noexcept
  {
    switch (p_channel) {
      case timer_channel::ch1:
        return m_reg->capture_compare_register & 0xFFFFU;
      case timer_channel::ch2:
        return m_reg->capture_compare_register_2 & 0xFFFFU;
      case timer_channel::ch3:
        return m_reg->capture_compare_register_3 & 0xFFFFU;
      case timer_channel::ch4:
        break;
    }
    return m_reg->capture_compare_register_4 & 0xFFFFU;
  }

private:
  using control = timer_bits::control;
  static constexpr auto update_generation = timer_bits::update_generation;
  static constexpr auto main_output_enable = timer_bits::main_output_enable;
  /// One channel's 8 bits of CCMRx: OCxM = 0b110, OCxPE = 1, CCxS = 0b00
  static constexpr u32 output_mode_pwm1 = 0b0110'1000;

  timer_reg* m_reg;
};
}  // namespace hal::stm32_generic
