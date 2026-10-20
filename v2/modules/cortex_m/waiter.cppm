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

#include <algorithm>
#include <chrono>
#include <utility>

export module hal.arm_mcu.cortex_m:waiter;

import hal;
import hal.util;

import :system_control;

namespace hal::cortex_m {
/**
 * @brief Bare-metal waiter that sleeps the CPU with WFI while blocked.
 *
 * This is the waiter provider for single-context ARM Cortex-M applications:
 * everything runs from `main()` and interrupts. A context that blocks puts the
 * CPU to sleep until an interrupt arrives, then re-checks whether it was
 * notified. Timeouts and sleeps are measured by a `hal::timed_interrupt`,
 * whose interrupt wakes the CPU when the time is up.
 *
 * The timing granularity of `block_for()` and `sleep_for()` is the tick period
 * of the timer.
 *
 * Usage:
 *
 * @code
 * static hal::cortex_m::systick_timer timer(cpu_frequency);
 * static hal::cortex_m::wfi_waiter waiter(timer);  // installs itself
 * hal::sleep_for(10ms);
 * @endcode
 */
export class wfi_waiter final : public hal::blocking_waiter
{
public:
  /**
   * @brief Construct a waiter and install it as the waiter of every context.
   *
   * After construction, `hal::current_waiter()` returns this waiter, so
   * construct it before any driver that blocks. Drivers keep a pointer to the
   * waiter that was current when they were constructed, so this waiter must
   * outlive every driver that can block or notify. In practice that means it
   * needs static storage.
   *
   * @param p_timer - timer used to end `block_for()` timeouts and
   * `sleep_for()` durations. The waiter registers its own callback with it,
   * so it must not already have one, and it must outlive the waiter.
   * @throws hal::subscription_busy - if p_timer already has a callback
   */
  explicit wfi_waiter(hal::timed_interrupt& p_timer [[clang::lifetimebound]])
    : m_timer(p_timer)
    , m_wakeup_subscription(p_timer.on_expire(m_wakeup))
  {
    // Set s_installed first, so the provider never returns a null waiter.
    m_previous_installed = std::exchange(s_installed, this);
    m_previous_provider = hal::set_waiter_provider(&provider);
  }

  wfi_waiter(wfi_waiter const&) = delete;
  wfi_waiter& operator=(wfi_waiter const&) = delete;
  wfi_waiter(wfi_waiter&&) = delete;
  wfi_waiter& operator=(wfi_waiter&&) = delete;

  /**
   * @brief Restore the waiter that was installed before this one.
   *
   * Waiters are expected to be destroyed in the reverse order of their
   * construction, as static and local objects are.
   */
  ~wfi_waiter()
  {
    hal::set_waiter_provider(m_previous_provider);
    s_installed = m_previous_installed;
  }

private:
  /// Sets a flag when the timer expires. The timer interrupt itself is what
  /// wakes the CPU from WFI.
  struct wakeup : public hal::timed_callback
  {
    void callback() noexcept override
    {
      m_expired = true;
    }

    bool volatile m_expired = false;
  };

  static hal::blocking_waiter& provider()
  {
    return *s_installed;
  }

  void driver_notify() noexcept override
  {
    m_notified = true;
  }

  void driver_block() override
  {
    sleep_until([this]() { return m_notified; });
    m_notified = false;
  }

  bool driver_block_for(time_duration p_timeout) override
  {
    start_timer(p_timeout);
    sleep_until([this]() { return m_notified or m_wakeup.m_expired; });
    stop_timer();

    bool const notified = m_notified;
    m_notified = false;
    return notified;
  }

  void driver_sleep_for(time_duration p_duration) override
  {
    if (p_duration <= time_duration::zero()) {
      return;
    }
    start_timer(p_duration);
    sleep_until([this]() { return m_wakeup.m_expired; });
  }

  void driver_clear() noexcept override
  {
    m_notified = false;
  }

  void start_timer(time_duration p_delay)
  {
    m_wakeup.m_expired = false;
    m_timer->schedule(std::max(p_delay, time_duration::zero()),
                      hal::timer_mode::one_shot);
  }

  void stop_timer()
  {
    m_timer->cancel();
  }

  inline static wfi_waiter* s_installed = nullptr;

  hal::ref<hal::timed_interrupt> m_timer;
  hal::waiter_provider m_previous_provider = nullptr;
  wfi_waiter* m_previous_installed = nullptr;
  wakeup m_wakeup{};
  bool volatile m_notified = false;
  // Last, so the timer stops calling m_wakeup before it is destroyed.
  hal::subscription m_wakeup_subscription;
};
}  // namespace hal::cortex_m
