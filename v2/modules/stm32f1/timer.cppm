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
#include <cstdint>

export module hal.arm_mcu.stm32f1:timer;

import hal;
import hal.util;
import hal.arm_mcu.stm32_generic;

import :constants;

namespace hal::stm32f1 {
/**
 * @brief Two timers that can chain into a `chained_32bit_timer_clock`
 *
 * One timer counts the timer clock and the other counts its wraps through an
 * internal trigger input, which only some pairs are wired for (RM0008 Tables
 * 82 and 86). The clock picks which timer does which, so every pair here
 * works. TIM6 and TIM7 can't chain, and neither can TIM3 with TIM8.
 *
 * The `_high_density` pairs use TIM5 or TIM8, which exist only on
 * high-density and larger parts, such as the STM32F103ZG. On other parts,
 * constructing a clock from one of them is undefined behavior.
 */
export enum class timer_pair : u8 {
  timer1_timer2,
  timer1_timer3,
  timer1_timer4,
  timer2_timer3,
  timer2_timer4,
  timer3_timer4,
  timer1_timer5_high_density,
  timer1_timer8_high_density,
  timer2_timer5_high_density,
  timer2_timer8_high_density,
  timer3_timer5_high_density,
  timer4_timer5_high_density,
  timer4_timer8_high_density,
  timer5_timer8_high_density,
};

// ---- Shared by the timer drivers, not exported ----------------------------

/// Where a timer lives and which interrupt reports its updates
struct timer_route
{
  peripheral id;
  irq interrupt;
  std::uintptr_t address;
};

/// TIM1 to TIM8 (RM0008 section 3.3, memory map). TIM1 and TIM8 have their
/// own update interrupt; the others have one interrupt for everything.
constexpr std::array<timer_route, 8> timer_routes{ {
  { .id = peripheral::timer1,
    .interrupt = irq::tim1_up,
    .address = 0x4001'2C00 },
  { .id = peripheral::timer2, .interrupt = irq::tim2, .address = 0x4000'0000 },
  { .id = peripheral::timer3, .interrupt = irq::tim3, .address = 0x4000'0400 },
  { .id = peripheral::timer4, .interrupt = irq::tim4, .address = 0x4000'0800 },
  { .id = peripheral::timer5, .interrupt = irq::tim5, .address = 0x4000'0C00 },
  { .id = peripheral::timer6, .interrupt = irq::tim6, .address = 0x4000'1000 },
  { .id = peripheral::timer7, .interrupt = irq::tim7, .address = 0x4000'1400 },
  { .id = peripheral::timer8,
    .interrupt = irq::tim8_up,
    .address = 0x4001'3400 },
} };

/// Called from a timer's interrupt with the object it was routed with
using timer_handler = void (*)(void* p_object) noexcept;

/**
 * @brief Claim a timer, power it on and reset it
 *
 * @param p_id - the timer
 * @return u8 - the timer's index in `timer_routes`
 * @throws hal::operation_not_supported - if `p_id` isn't TIM1 to TIM8
 * @throws hal::device_or_resource_busy - if another driver has claimed it
 */
u8 claim_timer(peripheral p_id);

/// Stop routing the timer's interrupt, power it off and release it
void release_timer(u8 p_index) noexcept;

/// Send the timer's update interrupt to `p_handler(p_object)`, and enable it
void route_timer_interrupt(u8 p_index, timer_handler p_handler, void* p_object);

/// The timer's base address
void* timer_address(u8 p_index);

/// Two claimed timers chained by `claim_timer_pair()`
struct claimed_timer_pair
{
  /// The timer that counts the clock, as an index in `timer_routes`
  u8 low_index;
  /// The timer that counts the low timer's wraps
  u8 high_index;
  /// The high timer's trigger input (ITR0 to ITR3) wired to the low timer
  u8 trigger_input;
};

/**
 * @brief Claim both timers of a pair, power them on and reset them
 *
 * @throws hal::device_or_resource_busy - if another driver has claimed
 * either timer. Neither is claimed then.
 */
claimed_timer_pair claim_timer_pair(timer_pair p_pair);

// ---- Drivers ---------------------------------------------------------------

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
  /// Takes the index `claim_timer()` returned
  explicit timer(u8 p_index);

  static void handle_update(void* p_self) noexcept;

  bool driver_scheduled() override;
  void driver_schedule(hal::time_duration p_delay,
                       hal::timer_mode p_mode) override;
  void driver_cancel() noexcept override;
  void driver_on_expire(hal::timed_callback& p_callback) override;
  void driver_clear_expire() noexcept override;

  hal::hertz m_frequency;
  hal::stm32_generic::timer m_driver;
  hal::timed_callback* volatile m_callback = nullptr;
  u8 m_index;
};

/// Settings for a `timer_clock_16bit`
export struct timer_clock_16bit_settings
{
  /// The most update interrupts a second the clock may take. The clock picks
  /// the finest tick rate that stays at or under it.
  hal::hertz max_interrupt_rate = 20 * mp_units::si::hertz;
};

/**
 * @brief One STM32F1 timer, TIM1 to TIM8, as a 64-bit `hal::steady_clock`
 *
 * Claims the whole timer. Its 16-bit counter counts prescaled ticks, and the
 * update interrupt counts each wrap, so `uptime()` never needs to be called
 * on a schedule to keep the time right.
 *
 * Finer ticks mean more interrupts: the counter wraps `frequency() / 65,536`
 * times a second. The settings give the most interrupts a second the
 * application accepts, and the clock picks the finest tick that stays under
 * it. At a 72 MHz timer clock:
 *
 * | max_interrupt_rate | tick rate    | tick    | interrupts/s |
 * | ------------------ | ------------ | ------- | ------------ |
 * | 1,099 Hz or more   | 72 MHz       | 13.9 ns | 1,098.6      |
 * | 20 Hz (default)    | 1.309 MHz    | 0.76 us | 19.98        |
 * | 15 Hz              | 0.973 MHz    | 1.03 us | 14.85        |
 * | 1 Hz               | 65,514 Hz    | 15.3 us | 1.0          |
 *
 * The rate also bounds how long interrupts may stay masked: if they are
 * masked for longer than one wrap period (1 / the interrupt rate), a wrap is
 * lost and the clock falls behind by 65,536 ticks.
 *
 * `frequency()` reports the tick rate rounded down to a whole hertz, which
 * reads it less than 1 Hz low: under 16 ppm at 65 kHz, and under 1 ppm at
 * 1 MHz or more.
 *
 * TIM5 to TIM8 exist only on high-density and larger parts, such as the
 * STM32F103ZG. On other parts, constructing them is undefined behavior.
 */
export class timer_clock_16bit final : public hal::steady_clock
{
public:
  /**
   * @brief Claim a timer, power it on and start it counting
   *
   * @param p_id - the timer: `peripheral::timer1` to `peripheral::timer8`
   * @param p_settings - the most interrupts a second to accept. A rate too low
   * to reach, 0 Hz included, gets the fewest: one wrap every 59.6 s at 72 MHz.
   * @throws hal::operation_not_supported - if `p_id` isn't TIM1 to TIM8
   * @throws hal::device_or_resource_busy - if another driver has claimed the
   * timer
   */
  explicit timer_clock_16bit(peripheral p_id,
                             timer_clock_16bit_settings const& p_settings = {});

  timer_clock_16bit(timer_clock_16bit const&) = delete;
  timer_clock_16bit& operator=(timer_clock_16bit const&) = delete;
  timer_clock_16bit(timer_clock_16bit&&) = delete;
  timer_clock_16bit& operator=(timer_clock_16bit&&) = delete;

  /// Stop the timer, disable its interrupt, power it off and release it
  ~timer_clock_16bit();

private:
  /// Takes the index `claim_timer()` returned
  timer_clock_16bit(u8 p_index, timer_clock_16bit_settings const& p_settings);

  static void handle_update(void* p_self) noexcept;

  hal::hertz driver_frequency() override;
  u64 driver_uptime() override;

  hal::stm32_generic::free_running_counter m_counter;
  u8 m_index;
};

/**
 * @brief Two STM32F1 timers chained into a 64-bit `hal::steady_clock`
 *
 * Claims both timers. The low timer counts every timer clock cycle and the
 * high timer counts its wraps in hardware, so together they count 32 bits at
 * the full timer clock: 13.9 ns ticks at 72 MHz. Only the 32-bit count's wraps
 * raise an interrupt, once every 59.6 s at 72 MHz, which extends it to 64
 * bits. `uptime()` stays right however rarely it is called, and interrupts may
 * stay masked for up to one wrap, 59.6 s at 72 MHz.
 *
 * The pair must be one whose timers are wired to chain: see `timer_pair`.
 */
export class chained_32bit_timer_clock final : public hal::steady_clock
{
public:
  /**
   * @brief Claim two timers, chain them and start them counting
   *
   * @param p_pair - the two timers
   * @throws hal::device_or_resource_busy - if another driver has claimed
   * either timer
   */
  explicit chained_32bit_timer_clock(timer_pair p_pair);

  chained_32bit_timer_clock(chained_32bit_timer_clock const&) = delete;
  chained_32bit_timer_clock& operator=(chained_32bit_timer_clock const&) =
    delete;
  chained_32bit_timer_clock(chained_32bit_timer_clock&&) = delete;
  chained_32bit_timer_clock& operator=(chained_32bit_timer_clock&&) = delete;

  /// Stop both timers, disable the interrupt, power them off and release them
  ~chained_32bit_timer_clock();

private:
  explicit chained_32bit_timer_clock(claimed_timer_pair p_pair);

  static void handle_update(void* p_self) noexcept;

  hal::hertz driver_frequency() override;
  u64 driver_uptime() override;

  hal::stm32_generic::chained_counter m_counter;
  u8 m_low_index;
  u8 m_high_index;
};
}  // namespace hal::stm32f1
