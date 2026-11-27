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

export module hal.arm_mcu.stm32f1:pwm;

import hal;
import hal.util;
import hal.arm_mcu.stm32_generic;

import :constants;
import :timer;

namespace hal::stm32f1 {
/// Settings for a `pwm`
export struct pwm_settings
{
  /// Which pins the channels use: see `timer_remap`
  timer_remap remap = timer_remap::none;
  /// The frequency every channel runs at
  hal::hertz frequency = 20'000 * mp_units::si::hertz;
};

/**
 * @brief One STM32F1 timer's PWM: the frequency its four channels share
 *
 * Claims the whole timer: a timer that runs PWM can't also count delays or a
 * quadrature encoder, since those use the same counter. The channels are
 * `pwm::channel`s, constructed from this.
 *
 * Each frequency gets the smallest prescaler that fits, so the channels get as
 * many duty cycle steps as the timer clock allows: the timer clock divided by
 * the frequency, up to 65,535. At a 72 MHz timer clock, 20 kHz (the default)
 * has 3,600 steps, about 11.8 bits, and 1.1 kHz or less has the full 65,535.
 * `frequency()` asks for the closest frequency the timer can make: a channel's
 * `frequency()` reports what it got, rounded to the nearest hertz. 0 Hz gets
 * the lowest, one period every 59.6 s at 72 MHz, and anything above half the
 * timer clock gets half the timer clock.
 *
 * A new frequency or duty cycle takes effect at the start of the next period,
 * and changing the frequency keeps every channel's duty cycle.
 *
 * TIM1 to TIM5 and TIM8 have PWM outputs; TIM6 and TIM7 don't. TIM5 and TIM8
 * exist only on high-density and larger parts, such as the STM32F103ZG. On
 * other parts, constructing them is undefined behavior.
 */
export class pwm final : public hal::pwm_group_manager
{
public:
  class channel;

  /**
   * @brief Claim a timer, set its pin remap and start its counter
   *
   * No channel drives its pin until a `pwm::channel` claims it.
   *
   * @param p_id - the timer: TIM1 to TIM5 or TIM8
   * @param p_settings - the channels' pins and frequency
   * @throws hal::operation_not_supported - if the timer has no PWM outputs or
   * no such remap
   * @throws hal::device_or_resource_busy - if another driver has claimed the
   * timer
   */
  explicit pwm(peripheral p_id, pwm_settings const& p_settings = {});

  pwm(pwm const&) = delete;
  pwm& operator=(pwm const&) = delete;
  pwm(pwm&&) = delete;
  pwm& operator=(pwm&&) = delete;

  /// Stop the counter, turn the outputs off, power the timer off and release
  /// it. Destroy the channels first.
  ~pwm();

private:
  void driver_frequency(hal::hertz p_frequency) override;

  timer_remap m_remap;
  u8 m_index;
  hal::hertz m_clock;
  hal::stm32_generic::pwm_period m_period;
  hal::stm32_generic::pwm m_driver;
  std::array<u16, 4> m_duty_cycles{};
  std::bitset<4> m_claimed_channels{};
};

/**
 * @brief One channel of an STM32F1 timer's PWM
 *
 * Claims its pin and drives it in PWM mode 1, active high. It starts at a 0%
 * duty cycle.
 */
class pwm::channel final : public hal::pwm16_channel
{
public:
  /**
   * @brief Claim a channel and its pin, and start it at 0%
   *
   * @param p_pwm - the timer's PWM, which must outlive the channel
   * @param p_channel - which channel
   * @throws hal::device_or_resource_busy - if the channel or its pin is
   * already claimed
   */
  channel(hal::ref<pwm> p_pwm [[clang::lifetimebound]],
          timer_channel p_channel);

  channel(channel const&) = delete;
  channel& operator=(channel const&) = delete;
  channel(channel&&) = delete;
  channel& operator=(channel&&) = delete;

  /// Turn the output off and release the channel and its pin
  ~channel();

private:
  hal::hertz driver_frequency() override;
  void driver_duty_cycle(u16 p_duty_cycle) override;

  hal::ref<pwm> m_pwm;
  u8 m_channel;
};
}  // namespace hal::stm32f1
