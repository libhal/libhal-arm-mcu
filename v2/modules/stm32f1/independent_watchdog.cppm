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

export module hal.arm_mcu.stm32f1:independent_watchdog;

import hal;
import hal.util;

import :constants;
import :power;

namespace hal::stm32f1 {

/**
 * @brief STM32F1 Independent Watchdog (IWDG)
 *
 * Owns the STM32F1's independent watchdog. The IWDG is clocked from the Low
 * Speed Internal (LSI) oscillator and has no clock-enable bit, so there is
 * nothing to power on. Only one may exist at a time.
 *
 * Once started, the watchdog can't be stopped, not even by destroying this
 * object: only a system reset stops it. Call `reset()` often enough to keep
 * the countdown from reaching zero.
 *
 * ```cpp
 * static hal::stm32f1::independent_watchdog watchdog;
 * watchdog.set_countdown_time(100ms);
 * watchdog.start();
 * while (true) {
 *   do_work();
 *   watchdog.reset();
 * }
 * ```
 */
export class independent_watchdog
{
public:
  /**
   * @brief Take control of the independent watchdog
   *
   * Claims `peripheral::independent_watchdog` (see `claim()`), and releases
   * it when destroyed.
   *
   * @throws hal::device_or_resource_busy - if another `independent_watchdog`
   * exists
   */
  independent_watchdog();

  independent_watchdog(independent_watchdog const&) = delete;
  independent_watchdog& operator=(independent_watchdog const&) = delete;
  independent_watchdog(independent_watchdog&&) = delete;
  independent_watchdog& operator=(independent_watchdog&&) = delete;

  /// Give up control of the watchdog. A started watchdog keeps counting.
  ~independent_watchdog();

  /**
   * @brief Start the watchdog countdown
   *
   * Enables the watchdog and begins the countdown. Once started, the watchdog
   * must be periodically refreshed using `reset()` to avoid a system reset.
   */
  void start();

  /**
   * @brief Resets (refreshes) the watchdog countdown
   *
   * Reloads the counter with the value set by `set_countdown_time()`. This
   * must be called periodically to avoid a system reset.
   */
  void reset();

  /**
   * @brief Configures the watchdog countdown time
   *
   * Sets the prescaler and reload values for the watchdog countdown. The
   * actual wait time may be 66%-133% of the specified time due to LSI clock
   * variance (section 7.2.5, RM0008).
   *
   * The hardware takes a few LSI cycles to apply new values, and only while
   * the LSI runs, which starts with `start()`. Configure the countdown once
   * before `start()`, or wait at least a few milliseconds between changes.
   *
   * @param p_wait_time - countdown time till the watchdog triggers a reset
   * @throws hal::operation_not_supported - if the requested time cannot be
   * achieved with the available prescaler/reload combinations.
   * @throws hal::device_or_resource_busy - if the previous countdown change
   * hasn't been applied yet
   */
  void set_countdown_time(hal::time_duration p_wait_time);

  /**
   * @brief Checks if the last reset was caused by the independent watchdog
   *
   * @return true - the IWDG reset flag is set in the reset status register
   * @return false - the IWDG did not cause a reset since the flags were last
   * cleared
   */
  [[nodiscard]] bool check_flag();

  /**
   * @brief Clears the reset flags
   *
   * Clears every reset flag in the reset status register, not just the IWDG
   * one: the hardware only clears them together.
   */
  void clear_flag();
};

}  // namespace hal::stm32f1
