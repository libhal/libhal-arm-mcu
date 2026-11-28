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

export module hal.arm_mcu.stm32f1:pwm;

import hal;
import hal.util;
import hal.arm_mcu.stm32_generic;

import :constants;
import :pin;
import :timer;

namespace hal::stm32f1 {
/// Settings for a `pwm`
export struct pwm_settings
{
  /// The frequency every channel runs at
  hal::hertz frequency = 20'000 * mp_units::si::hertz;
};

export template<pin Pin>
class pwm_channel;

/**
 * @brief The part of `pwm` that isn't a template
 *
 * Holds the timer and implements `hal::pwm_group_manager`. Construct a `pwm`,
 * which picks the pins.
 */
export class pwm_base : public hal::pwm_group_manager
{
public:
  class channel;

  pwm_base(pwm_base const&) = delete;
  pwm_base& operator=(pwm_base const&) = delete;
  pwm_base(pwm_base&&) = delete;
  pwm_base& operator=(pwm_base&&) = delete;

  /// Stop the counter, turn the outputs off, power the timer off and release
  /// it. Destroy the channels first.
  ~pwm_base();

protected:
  /**
   * @brief Claim a timer, set its pin remap and start its counter
   *
   * @param p_id - the timer
   * @param p_remap_value - the value for the timer's AFIO MAPR field
   * @param p_settings - the frequency
   * @throws hal::device_or_resource_busy - if another driver has claimed the
   * timer
   */
  pwm_base(peripheral p_id, u8 p_remap_value, pwm_settings const& p_settings);

private:
  /// Takes the index `claim_timer()` returned
  pwm_base(u8 p_index, pwm_settings const& p_settings);

  void driver_frequency(hal::hertz p_frequency) override;

  hal::stm32_generic::pwm m_driver;
  hal::hertz m_clock;
  hal::stm32_generic::pwm_period m_period;
  /// Each channel's duty cycle, so a frequency change keeps it at full
  /// precision
  std::array<u16, 4> m_duty_cycles{};
  u8 m_index;
};

/**
 * @brief One STM32F1 timer's PWM: the frequency its channels share
 *
 * The timer and the pins its channels will drive are template arguments, and
 * the timer's AFIO remap is picked from the pins at compile time. A timer
 * with no remap that has all the pins doesn't compile, so neither does a
 * timer without PWM outputs (TIM6, TIM7). Each pin then gets a
 * `pwm_channel`:
 *
 * ```cpp
 * hal::stm32f1::pwm<hal::stm32f1::peripheral::timer3,
 *                   hal::stm32f1::pb4,
 *                   hal::stm32f1::pb1>
 *   motors;  // TIM3's partial remap
 * hal::stm32f1::pwm_channel<hal::stm32f1::pb4> left(motors);
 * hal::stm32f1::pwm_channel<hal::stm32f1::pb1> right(motors);
 * ```
 *
 * Claims the whole timer: a timer that runs PWM can't also count delays or a
 * quadrature encoder, since those use the same counter.
 *
 * The pins of each timer's CH1, CH2, CH3 and CH4, by remap (RM0008 Tables 43
 * to 46):
 *
 * - TIM1: PA8, PA9, PA10, PA11. Full remap: PE9, PE11, PE13, PE14.
 * - TIM2: PA0, PA1, PA2, PA3. Partial remaps: PA15, PB3, PA2, PA3 and PA0,
 *   PA1, PB10, PB11. Full remap: PA15, PB3, PB10, PB11.
 * - TIM3: PA6, PA7, PB0, PB1. Partial remap: PB4, PB5, PB0, PB1. Full
 *   remap: PC6, PC7, PC8, PC9.
 * - TIM4: PB6, PB7, PB8, PB9. Full remap: PD12, PD13, PD14, PD15.
 * - TIM5: PA0, PA1, PA2, PA3.
 * - TIM8: PC6, PC7, PC8, PC9.
 *
 * The full remaps of TIM1 and TIM4 exist only on 100- and 144-pin packages,
 * and TIM3's on 64-pin and larger. TIM5 and TIM8 exist only on high-density
 * and larger parts, such as the STM32F103ZG; on other parts, constructing
 * them is undefined behavior. PA15, PB3 and PB4 are JTAG pins: they drive
 * nothing until the debug port is switched to SWD only or off.
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
 * @tparam Timer - the timer: TIM1 to TIM5 or TIM8
 * @tparam Pins - the pins the channels will drive, in any order
 */
export template<peripheral Timer, pin... Pins>
class pwm final : public pwm_base
{
public:
  static_assert(sizeof...(Pins) > 0, "List the pins the channels will drive");
  static_assert(not has_duplicate_pins<sizeof...(Pins)>({ Pins... }),
                "A pin is listed more than once");
  static_assert(find_timer_pin_set<sizeof...(Pins)>(Timer, { Pins... }) <
                  timer_pin_sets.size(),
                "No remap of this timer has all of these pins, or the timer "
                "has no PWM outputs");

  /**
   * @brief Claim the timer, set its pin remap and start its counter
   *
   * No channel drives its pin until a `pwm_channel` claims it.
   *
   * @param p_settings - the frequency
   * @throws hal::device_or_resource_busy - if another driver has claimed the
   * timer
   */
  explicit pwm(pwm_settings const& p_settings = {})
    : pwm_base(Timer, pins.remap_value, p_settings)
  {
  }

private:
  template<pin>
  friend class pwm_channel;

  /// The timer's channel pins with the remap picked, found at compile time
  static constexpr timer_pin_set pins =
    timer_pin_sets[find_timer_pin_set<sizeof...(Pins)>(Timer, { Pins... })];

  /// The channel that drives `Pin`, or 4 if no channel does
  template<pin Pin>
  static consteval timer_channel channel_of()
  {
    u8 index = 0;
    while ((index < pins.channels.size()) and (pins.channels[index] != Pin)) {
      index++;
    }
    return static_cast<timer_channel>(index);
  }
};

/**
 * @brief The part of `pwm_channel` that isn't a template
 *
 * Construct a `pwm_channel`, which picks the pin.
 */
class pwm_base::channel : public hal::pwm16_channel
{
public:
  channel(channel const&) = delete;
  channel& operator=(channel const&) = delete;
  channel(channel&&) = delete;
  channel& operator=(channel&&) = delete;

  /// Turn the output off and release the pin
  ~channel();

protected:
  /**
   * @brief Claim a pin and drive it from its channel, starting at 0%
   *
   * @param p_pwm - the timer's PWM, which must outlive the channel
   * @param p_channel - the channel that drives the pin
   * @param p_pin - the pin
   * @throws hal::device_or_resource_busy - if the pin is already claimed
   */
  channel(hal::ref<pwm_base> p_pwm [[clang::lifetimebound]],
          timer_channel p_channel,
          pin p_pin);

private:
  hal::hertz driver_frequency() override;
  void driver_duty_cycle(u16 p_duty_cycle) override;

  hal::ref<pwm_base> m_pwm;
  pin m_pin;
  timer_channel m_channel;
};

/**
 * @brief One pin of a timer's PWM
 *
 * Claims its pin and drives it in PWM mode 1, active high. It starts at a 0%
 * duty cycle. The pin must be one the `pwm` lists, or it doesn't compile.
 *
 * @tparam Pin - the pin, such as `hal::stm32f1::pb1`
 */
export template<pin Pin>
class pwm_channel final : public pwm_base::channel
{
public:
  /**
   * @brief Claim the pin and start it at 0%
   *
   * @param p_pwm - the timer's PWM, which must outlive the channel
   * @throws hal::device_or_resource_busy - if the pin is already claimed
   */
  template<peripheral Timer, pin... Pins>
  explicit pwm_channel(pwm<Timer, Pins...>& p_pwm [[clang::lifetimebound]])
    : pwm_base::channel(p_pwm,
                        pwm<Timer, Pins...>::template channel_of<Pin>(),
                        Pin)
  {
    static_assert(((Pin == Pins) or ...),
                  "This pin isn't one of the pwm's pins");
  }
};
}  // namespace hal::stm32f1
