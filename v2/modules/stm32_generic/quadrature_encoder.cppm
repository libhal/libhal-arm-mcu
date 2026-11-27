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

export module hal.arm_mcu.stm32_generic:quadrature_encoder;

import hal;
import hal.util;
import hal.arm_mcu.cortex_m;

import :registers;

namespace hal::stm32_generic {
/**
 * @brief Which way a quadrature counter wrapped, from its count just after
 *
 * The counter wraps from 65,535 to 0 counting up, and from 0 to 65,535
 * counting down. Read soon after the wrap, the count is near the end it
 * wrapped to. Using the count, not the direction bit, keeps a wrap right when
 * the shaft turns back right after it without crossing back.
 *
 * @param p_count - the counter, read after the wrap flag
 * @return i64 - +1 for a wrap counting up, -1 for one counting down
 */
export [[nodiscard]] constexpr i64 quadrature_wrap(u32 p_count)
{
  return (p_count < 0x8000U) ? 1 : -1;
}

/**
 * @brief Combine a quadrature counter's wraps and count into one signed count
 *
 * @param p_wraps - net wraps: up wraps minus down wraps
 * @param p_count - the 16-bit counter
 * @return i64 - counts since the counter started at 0
 */
export [[nodiscard]] constexpr i64 quadrature_position(i64 p_wraps, u32 p_count)
{
  return (p_wraps * 65'536) + static_cast<i64>(p_count);
}

/**
 * @brief A quadrature encoder count shared by the STM32 timers
 *
 * Runs the timer in encoder mode 3: the counter counts every edge of both
 * TI1 and TI2, four counts per encoder pulse, up or down by which input leads.
 * The 16-bit counter wraps, raising an update interrupt. The platform driver
 * calls `update()` from that interrupt, which counts the wrap, and `count()`
 * combines the wraps with the counter into a signed 64-bit count.
 *
 * This class is meant to only be used by platform libraries or drivers.
 */
export class quadrature_counter
{
public:
  /**
   * @brief Construct over a timer's registers
   *
   * Doesn't touch the hardware.
   *
   * @param p_address - the timer's base address
   */
  explicit quadrature_counter(void* p_address)
    : m_reg(timer_reg::from(p_address))
  {
  }

  /**
   * @brief Start counting from 0 in encoder mode 3
   *
   * Both inputs are filtered: an edge must hold for 8 timer clock cycles to
   * count, 111 ns at 72 MHz.
   */
  void start() noexcept
  {
    stop();
    m_wraps = 0;

    // Only raise an update interrupt when the counter wraps, not on the
    // update generated below.
    bit_modify(m_reg->control_register)
      .clear<control::center_aligned_mode>()
      .clear<control::one_pulse_mode>()
      .set<control::update_request_source>();
    // CC1S = CC2S = 0b01: IC1 on TI1 and IC2 on TI2, each with the filter
    m_reg->capture_compare_mode_register = bit_value<u32>(0)
                                             .insert<mode::cc1_select>(0b01U)
                                             .insert<mode::ic1_filter>(filter)
                                             .insert<mode::cc2_select>(0b01U)
                                             .insert<mode::ic2_filter>(filter)
                                             .get();
    // CC1P = CC2P = 0: count the inputs as they are, not inverted
    m_reg->cc_enable_register = 0;
    bit_modify(m_reg->peripheral_control_register)
      .insert<slave_mode_select>(encoder_mode_3);
    m_reg->prescale_register = 0;
    m_reg->auto_reload_register = 0xFFFF;
    // Load the prescaler, then start from 0
    m_reg->event_generator_register = update_generation.value<u32>();
    m_reg->counter_register = 0;
    clear_wrap_flag();
    bit_modify(m_reg->interrupt_enable_register).set<update_interrupt_enable>();
    bit_modify(m_reg->control_register).set<control::counter_enable>();
  }

  /// Stop counting. `count()` keeps returning the last count.
  void stop() noexcept
  {
    bit_modify(m_reg->control_register).clear<control::counter_enable>();
    bit_modify(m_reg->interrupt_enable_register)
      .clear<update_interrupt_enable>();
  }

  /**
   * @brief Handle the update interrupt: count a wrap
   *
   * Call from the timer's update interrupt. Does nothing if the interrupt
   * wasn't a wrap.
   */
  void update() noexcept
  {
    // Masked so a higher-priority interrupt calling count() can't see the
    // flag cleared before the wrap is counted.
    cortex_m::interrupt_mask_guard const mask;
    count_pending_wrap();
  }

  /**
   * @brief Counts since `start()`: four per encoder pulse, signed
   *
   * Safe to call from any context, including with interrupts masked, as long
   * as the shaft doesn't cross the counter's wrap point and back while they
   * are.
   *
   * @return i64 - counts since the counter started
   */
  [[nodiscard]] i64 count() noexcept
  {
    cortex_m::interrupt_mask_guard const mask;
    while (true) {
      auto const counter = m_reg->counter_register & 0xFFFFU;
      // A wrap after this read leaves the flag clear here, so the counter
      // read above comes before it, matching m_wraps.
      if (not count_pending_wrap()) {
        return quadrature_position(m_wraps, counter);
      }
      // A wrap came before or during the read: it is counted now, so read
      // the counter again.
    }
  }

private:
  /// Bit masks for TIMx_CR1
  struct control
  {
    static constexpr auto counter_enable = hal::bit_mask::from<0>();
    static constexpr auto update_request_source = hal::bit_mask::from<2>();
    static constexpr auto one_pulse_mode = hal::bit_mask::from<3>();
    static constexpr auto center_aligned_mode = hal::bit_mask::from<5, 6>();
  };
  /// Bit masks for TIMx_CCMR1 in input capture mode
  struct mode
  {
    static constexpr auto cc1_select = hal::bit_mask::from<0, 1>();
    static constexpr auto ic1_filter = hal::bit_mask::from<4, 7>();
    static constexpr auto cc2_select = hal::bit_mask::from<8, 9>();
    static constexpr auto ic2_filter = hal::bit_mask::from<12, 15>();
  };
  /// SMS in TIMx_SMCR
  static constexpr auto slave_mode_select = hal::bit_mask::from<0, 2>();
  /// SMS = 0b011: count both edges of both inputs
  static constexpr u32 encoder_mode_3 = 0b011;
  /// ICxF = 0b0011: sample at the timer clock, 8 samples must agree
  static constexpr u32 filter = 0b0011;
  /// UIE in TIMx_DIER
  static constexpr auto update_interrupt_enable = hal::bit_mask::from<0>();
  /// UIF in TIMx_SR
  static constexpr auto update_interrupt_flag = hal::bit_mask::from<0>();
  /// UG in TIMx_EGR
  static constexpr auto update_generation = hal::bit_mask::from<0>();

  /// Count a wrap if one is waiting. Call with interrupts masked.
  bool count_pending_wrap() noexcept
  {
    if (not bit_is_set<update_interrupt_flag>(m_reg->status_register)) {
      return false;
    }
    clear_wrap_flag();
    // Read after the flag, so the count is from after the wrap
    m_wraps = m_wraps + quadrature_wrap(m_reg->counter_register & 0xFFFFU);
    return true;
  }

  void clear_wrap_flag() noexcept
  {
    // The status flags are cleared by writing 0. Writing 1 leaves them as
    // they are, so only UIF clears.
    m_reg->status_register = ~update_interrupt_flag.value<u32>();
  }

  timer_reg* m_reg;
  i64 volatile m_wraps = 0;
};
}  // namespace hal::stm32_generic
