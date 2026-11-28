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
import hal.arm_mcu.cortex_m;

import :registers;

namespace hal::stm32_generic {
/// One of a timer's four channels
export enum class timer_channel : u8 {
  ch1,
  ch2,
  ch3,
  ch4,
};

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

/// A timer clock and the most interrupts a second to allow, for
/// `counter_prescaler()`
export struct counter_rate
{
  /// The timer clock
  u64 clock_hz;
  /// The most wraps a second to allow. 0 asks for as few as possible.
  u64 max_interrupt_rate_hz;
};

/**
 * @brief The prescaler for the finest free-running count under an interrupt
 * rate
 *
 * A 16-bit counter wraps every 65,536 ticks, raising an interrupt each time.
 * This picks the smallest prescaler, so the finest ticks, whose wraps come at
 * most `max_interrupt_rate_hz` times a second. A rate too low to reach, 0 Hz
 * included, gets the largest prescaler, so the fewest wraps.
 *
 * @param p_rate - the timer clock and the interrupt rate
 * @return u32 - the prescaler's divider, 1 to 65,536 (PSC + 1)
 */
export [[nodiscard]] constexpr u32 counter_prescaler(counter_rate p_rate)
{
  constexpr u64 counter_ticks_per_wrap = 65'536;
  if (p_rate.max_interrupt_rate_hz == 0) {
    return counter_ticks_per_wrap;
  }
  auto const divider = hal::divide<hal::rounding::ceil, u64>(
    { .num = p_rate.clock_hz,
      .den = p_rate.max_interrupt_rate_hz * counter_ticks_per_wrap });
  // NOTE: Safe to cast without narrowing: the clamp is at most 65,536.
  return static_cast<u32>(std::clamp<u64>(divider, 1, counter_ticks_per_wrap));
}

/// Whether the wrap flag was set when a free-running counter was read
export enum class pending_wrap : u8 {
  /// Not set: every wrap so far has been counted
  none,
  /// Set before the counter was read: the count is after a wrap the interrupt
  /// hasn't counted yet
  before_read,
  /// Set while the counter was read: the count is from just before or just
  /// after the wrap
  during_read,
};

/// One read of a free-running counter
export struct counter_reading
{
  /// Wraps the interrupt has counted
  u64 wraps;
  /// The counter: 16 bits for one timer, 32 for two chained
  u32 count;
  /// Whether a wrap was waiting to be counted
  pending_wrap pending;
};

/**
 * @brief Combine a free-running counter's reading into 64-bit ticks
 *
 * A wrap the interrupt hasn't counted yet still counts. If it happened while
 * the counter was read, the count tells which side it's on: a few ticks after
 * a wrap it's near 0, and a few ticks before, near its maximum.
 *
 * @tparam Bits - the counter's width: 16 for one timer, 32 for two chained
 * @param p_reading - the wraps, the count and the wrap flag
 * @return u64 - ticks since the counter started
 */
export template<u8 Bits>
[[nodiscard]] constexpr u64 counter_ticks(counter_reading p_reading)
{
  static_assert((Bits == 16U) or (Bits == 32U));
  constexpr u64 half = 1ULL << (Bits - 1U);
  bool const uncounted = (p_reading.pending == pending_wrap::before_read) or
                         ((p_reading.pending == pending_wrap::during_read) and
                          (p_reading.count < half));
  auto const wraps = p_reading.wraps + (uncounted ? 1U : 0U);
  return (wraps << Bits) | p_reading.count;
}

/**
 * @brief A free-running count shared by the STM32 timers, for steady clocks
 *
 * The 16-bit counter counts prescaled ticks from 0 to 65,535 and wraps,
 * raising an update interrupt. The platform driver calls `update()` from that
 * interrupt, which counts the wrap, and `uptime()` combines the wraps with the
 * counter into 64 bits.
 *
 * This class is meant to only be used by platform libraries or drivers.
 */
export class free_running_counter
{
public:
  /**
   * @brief Construct over a timer's registers
   *
   * Doesn't touch the hardware.
   *
   * @param p_address - the timer's base address
   */
  explicit free_running_counter(void* p_address)
    : m_reg(timer_reg::from(p_address))
  {
  }

  /**
   * @brief Start counting from 0
   *
   * @param p_prescaler - the prescaler's divider, 1 to 65,536 (PSC + 1)
   * @pre The timer is powered on.
   */
  void start(u32 p_prescaler) noexcept
  {
    stop();
    m_wraps = 0;

    // Count up from 0 to 65,535 over and over, and only raise an update
    // interrupt when the counter wraps, not on the update generated below.
    bit_modify(m_reg->control_register)
      .clear<control::center_aligned_mode>()
      .clear<control::direction>()
      .clear<control::one_pulse_mode>()
      .set<control::update_request_source>();
    m_reg->prescale_register = p_prescaler - 1U;
    m_reg->auto_reload_register = 0xFFFF;
    // Load the prescaler, and restart the counter from 0
    m_reg->event_generator_register = update_generation.value<u32>();
    clear_wrap_flag();
    bit_modify(m_reg->interrupt_enable_register).set<update_interrupt_enable>();
    bit_modify(m_reg->control_register).set<control::counter_enable>();
  }

  /**
   * @brief Stop counting. `uptime()` keeps returning the last count.
   *
   * @pre The timer is powered on.
   */
  void stop() noexcept
  {
    bit_modify(m_reg->control_register).clear<control::counter_enable>();
  }

  /**
   * @brief The prescaler's divider `start()` set
   *
   * @return u32 - 1 to 65,536 (PSC + 1)
   * @pre The timer is powered on.
   */
  [[nodiscard]] u32 prescaler() const noexcept
  {
    return (m_reg->prescale_register & 0xFFFFU) + 1U;
  }

  /**
   * @brief Handle the update interrupt: count a wrap
   *
   * Call from the timer's update interrupt. Does nothing if the interrupt
   * wasn't a wrap.
   *
   * @pre The timer is powered on.
   */
  void update() noexcept
  {
    if (not wrap_pending()) {
      return;
    }
    // A higher-priority interrupt reading uptime() between these two lines
    // would see neither the flag nor the count, and run 65,536 ticks behind.
    cortex_m::interrupt_mask_guard const mask;
    clear_wrap_flag();
    m_wraps = m_wraps + 1U;
  }

  /**
   * @brief Ticks since `start()`
   *
   * Safe to call from any context, including with interrupts masked, as long
   * as they haven't been masked for longer than one wrap.
   *
   * @return u64 - ticks since the counter started
   * @pre The timer is powered on.
   */
  [[nodiscard]] u64 uptime() const noexcept
  {
    while (true) {
      auto const wraps = m_wraps;
      bool const before = wrap_pending();
      auto const count = m_reg->counter_register & 0xFFFFU;
      bool const after = wrap_pending();
      // The interrupt counted a wrap in between, so read again.
      if (wraps != m_wraps) {
        continue;
      }
      auto pending = pending_wrap::none;
      if (before) {
        pending = pending_wrap::before_read;
      } else if (after) {
        pending = pending_wrap::during_read;
      }
      return counter_ticks<16>(
        { .wraps = wraps, .count = count, .pending = pending });
    }
  }

private:
  using control = timer_bits::control;
  static constexpr auto update_interrupt_enable =
    timer_bits::update_interrupt_enable;
  static constexpr auto update_interrupt_flag =
    timer_bits::update_interrupt_flag;
  static constexpr auto update_generation = timer_bits::update_generation;

  [[nodiscard]] bool wrap_pending() const noexcept
  {
    return bit_is_set<update_interrupt_flag>(m_reg->status_register);
  }

  void clear_wrap_flag() noexcept
  {
    // The status flags are cleared by writing 0. Writing 1 leaves them as
    // they are, so only UIF clears.
    m_reg->status_register = ~update_interrupt_flag.value<u32>();
  }

  timer_reg* m_reg;
  u64 volatile m_wraps = 0;
};

/**
 * @brief Two STM32 timers chained into a 32-bit free-running count
 *
 * The low timer counts every timer clock cycle from 0 to 65,535. Each time it
 * wraps, it pulses its TRGO output, which the chip wires to an internal
 * trigger input (ITRx) of the high timer. The high timer, in external clock
 * mode 1, counts those pulses, so together they count 32 bits in hardware:
 * the high timer holds the upper 16 bits and the low timer the lower 16. Only
 * the high timer's wraps raise an interrupt: once every 4,294,967,296
 * cycles, 59.6 s at 72 MHz. The platform driver calls `update()` from that
 * interrupt, which counts the wrap.
 *
 * This class is meant to only be used by platform libraries or drivers.
 */
export class chained_counter
{
public:
  /**
   * @brief Construct over two timers' registers
   *
   * Doesn't touch the hardware.
   *
   * @param p_low - the base address of the timer that counts the clock
   * @param p_high - the base address of the timer that counts its wraps
   */
  chained_counter(void* p_low, void* p_high)
    : m_low(timer_reg::from(p_low))
    , m_high(timer_reg::from(p_high))
  {
  }

  /**
   * @brief Start counting from 0
   *
   * @param p_trigger_input - which internal trigger input (ITR0 to ITR3) of
   * the high timer receives the low timer's TRGO
   * @pre Both timers are powered on.
   */
  void start(u32 p_trigger_input) noexcept
  {
    stop();
    m_wraps = 0;

    // Low: count every cycle from 0 to 65,535. The update generated here
    // loads the prescaler. It may pulse TRGO, but the high timer isn't
    // counting yet, and its own update below resets it anyway.
    bit_modify(m_low->control_register)
      .clear<control::center_aligned_mode>()
      .clear<control::direction>()
      .clear<control::one_pulse_mode>();
    m_low->prescale_register = 0;
    m_low->auto_reload_register = 0xFFFF;
    m_low->event_generator_register = update_generation.value<u32>();
    bit_modify(m_low->control_register_2)
      .insert<master_mode>(master_mode_update);

    // High: count the low timer's wraps from 0 to 65,535, and only raise an
    // update interrupt when the counter wraps.
    bit_modify(m_high->control_register)
      .clear<control::center_aligned_mode>()
      .clear<control::direction>()
      .clear<control::one_pulse_mode>()
      .set<control::update_request_source>();
    bit_modify(m_high->peripheral_control_register)
      .insert<trigger_selection>(p_trigger_input)
      .insert<slave_mode>(slave_mode_external_clock);
    m_high->prescale_register = 0;
    m_high->auto_reload_register = 0xFFFF;
    m_high->event_generator_register = update_generation.value<u32>();
    clear_wrap_flag();
    bit_modify(m_high->interrupt_enable_register)
      .set<update_interrupt_enable>();
    bit_modify(m_high->control_register).set<control::counter_enable>();

    // Start the low timer last, so both count from 0 together
    m_low->counter_register = 0;
    bit_modify(m_low->control_register).set<control::counter_enable>();
  }

  /**
   * @brief Stop counting. `uptime()` keeps returning the last count.
   *
   * @pre Both timers are powered on.
   */
  void stop() noexcept
  {
    bit_modify(m_low->control_register).clear<control::counter_enable>();
    bit_modify(m_high->control_register).clear<control::counter_enable>();
  }

  /**
   * @brief Handle the high timer's update interrupt: count a wrap
   *
   * Does nothing if the interrupt wasn't a wrap.
   *
   * @pre Both timers are powered on.
   */
  void update() noexcept
  {
    if (not wrap_pending()) {
      return;
    }
    // A higher-priority interrupt reading uptime() between these two lines
    // would see neither the flag nor the count, and run 2^32 ticks behind.
    cortex_m::interrupt_mask_guard const mask;
    clear_wrap_flag();
    m_wraps = m_wraps + 1U;
  }

  /**
   * @brief Timer clock cycles since `start()`
   *
   * Safe to call from any context, including with interrupts masked, as long
   * as they haven't been masked for longer than one wrap of the 32-bit count.
   *
   * @return u64 - cycles since the counter started
   * @pre Both timers are powered on.
   */
  [[nodiscard]] u64 uptime() const noexcept
  {
    // The high timer counts a low wrap a few clock cycles after it happens,
    // once TRGO is resynchronized to its clock. A low count this small may
    // be from before the high count caught up, so read again.
    constexpr u32 settle_ticks = 8;
    while (true) {
      auto const wraps = m_wraps;
      bool const before = wrap_pending();
      auto const high = m_high->counter_register & 0xFFFFU;
      auto const low = m_low->counter_register & 0xFFFFU;
      auto const high_again = m_high->counter_register & 0xFFFFU;
      bool const after = wrap_pending();
      if ((wraps != m_wraps) or (high != high_again) or (low < settle_ticks)) {
        continue;
      }
      auto pending = pending_wrap::none;
      if (before) {
        pending = pending_wrap::before_read;
      } else if (after) {
        pending = pending_wrap::during_read;
      }
      return counter_ticks<32>(
        { .wraps = wraps, .count = (high << 16U) | low, .pending = pending });
    }
  }

private:
  using control = timer_bits::control;
  static constexpr auto master_mode = timer_bits::control_2::master_mode;
  static constexpr auto master_mode_update =
    timer_bits::control_2::master_mode_update;
  static constexpr auto slave_mode = timer_bits::slave_mode_control::slave_mode;
  static constexpr auto slave_mode_external_clock =
    timer_bits::slave_mode_control::slave_mode_external_clock;
  static constexpr auto trigger_selection =
    timer_bits::slave_mode_control::trigger_selection;
  static constexpr auto update_interrupt_enable =
    timer_bits::update_interrupt_enable;
  static constexpr auto update_interrupt_flag =
    timer_bits::update_interrupt_flag;
  static constexpr auto update_generation = timer_bits::update_generation;

  [[nodiscard]] bool wrap_pending() const noexcept
  {
    return bit_is_set<update_interrupt_flag>(m_high->status_register);
  }

  void clear_wrap_flag() noexcept
  {
    // The status flags are cleared by writing 0. Writing 1 leaves them as
    // they are, so only UIF clears.
    m_high->status_register = ~update_interrupt_flag.value<u32>();
  }

  timer_reg* m_low;
  timer_reg* m_high;
  u64 volatile m_wraps = 0;
};
}  // namespace hal::stm32_generic
