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
 * @brief STM32F1 Independent Watchdog (IWDG) peripheral manager
 *
 * Owns the STM32F1's independent watchdog hardware block: its register bank.
 * The IWDG is clocked from the Low Speed Internal (LSI) oscillator and does
 * not require explicit power management.
 */
export struct independent_watchdog
  : public hal::pimpl<independent_watchdog>
  , public mem::enable_strong_from_this<independent_watchdog>
{
public:
  struct impl;

  /**
   * @brief Acquire an independent watchdog peripheral manager
   *
   * @param p_allocator - allocator used to allocate the memory for the object
   * @return hal::ptr<independent_watchdog> - independent watchdog manager
   */
  [[nodiscard]] static hal::ptr<independent_watchdog> create(
    hal::allocator p_allocator);

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
   * @param p_wait_time - countdown time till the watchdog triggers a reset
   * @throws hal::operation_not_supported - if the requested time cannot be
   *         achieved with the available prescaler/reload combinations.
   */
  void set_countdown_time(hal::time_duration p_wait_time);

  /**
   * @brief Checks if a reset flag is set
   *
   * Returns true if the independent watchdog reset flag is set in the
   * reset status register.
   *
   * @return true if the IWDG reset flag is set, false otherwise
   */
  bool check_flag();

  /**
   * @brief Clears the reset flags
   *
   * Clears the reset flag bits in the reset status register.
   */
  void clear_flag();

  /// @private
  independent_watchdog(private_key, hal::allocator p_allocator);

  independent_watchdog(independent_watchdog const&) = delete;
  independent_watchdog& operator=(independent_watchdog const&) = delete;
  independent_watchdog(independent_watchdog&&) = delete;
  independent_watchdog& operator=(independent_watchdog&&) = delete;

  ~independent_watchdog();
};

}  // namespace hal::stm32f1
