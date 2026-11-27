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

export module hal.arm_mcu.stm32f1:timer;

import hal;
import hal.util;
import hal.arm_mcu.stm32_generic;

import :constants;

namespace hal::stm32f1 {
/**
 * @brief One STM32F1 timer, TIM1 to TIM8, as a `hal::timed_interrupt`
 *
 * Claims the whole timer: a timer that counts delays can't also run PWM or a
 * quadrature encoder, since those use the same counter.
 *
 * A periodic timer reloads in hardware, so it keeps its period exactly. One
 * period counts up to 65,536 * 65,536 timer clock cycles, about 59.6 s at
 * 72 MHz. A longer delay runs as several equal periods, counted by the update
 * interrupt. Each period rounds up to a whole tick, so a delay runs less than
 * one prescaled tick per period longer than asked, and never ends early.
 *
 * TIM5 to TIM8 exist only on high-density and larger parts, such as the
 * STM32F103ZG. On other parts, constructing them is undefined behavior.
 */
export class timer final : public hal::timed_interrupt
{
public:
  /**
   * @brief Claim a timer, power it on and route its update interrupt
   *
   * Leaves the timer stopped.
   *
   * @param p_id - the timer: `peripheral::timer1` to `peripheral::timer8`
   * @throws hal::operation_not_supported - if `p_id` isn't TIM1 to TIM8
   * @throws hal::device_or_resource_busy - if another driver has claimed the
   * timer
   */
  explicit timer(peripheral p_id);

  timer(timer const&) = delete;
  timer& operator=(timer const&) = delete;
  timer(timer&&) = delete;
  timer& operator=(timer&&) = delete;

  /// Stop the timer, disable its interrupt, power it off and release it
  ~timer();

private:
  static void isr();

  bool driver_scheduled() override;
  void driver_schedule(hal::time_duration p_delay,
                       hal::timer_mode p_mode) override;
  void driver_cancel() noexcept override;
  void driver_on_expire(hal::timed_callback& p_callback) override;
  void driver_clear_expire() noexcept override;

  u8 m_index;
  hal::hertz m_frequency;
  hal::stm32_generic::timer m_driver;
  hal::timed_callback* volatile m_callback = nullptr;
};
}  // namespace hal::stm32f1
