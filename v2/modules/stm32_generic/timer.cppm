// Copyright 2024 - 2025 Khalil Estell and the libhal contributors
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

#include <limits>

export module hal.arm_mcu.stm32_generic:timer;

import hal;
import hal.util;
import hal.arm_mcu.cortex_m;

namespace hal::stm32_generic {
/// stm32 general purpose/advanced timer register map, shared by the timer
/// helper driver.
struct timer_reg_t
{
  /// Offset: 0x00 Control Register (R/W)
  hal::u32 volatile control_register;
  /// Offset: 0x04 Control Register 2 (R/W)
  hal::u32 volatile control_register_2;
  /// Offset: 0x08 Peripheral Mode Control Register (R/W)
  hal::u32 volatile peripheral_control_register;
  /// Offset: 0x0C DMA/Interrupt enable register (R/W)
  hal::u32 volatile interrupt_enable_register;
  /// Offset: 0x10 Status Register register (R/W)
  hal::u32 volatile status_register;
  /// Offset: 0x14 Event Generator Register register (R/W)
  hal::u32 volatile event_generator_register;
  /// Offset: 0x18 Capture/Compare mode register (R/W)
  hal::u32 volatile capture_compare_mode_register;
  /// Offset: 0x1C Capture/Compare mode register (R/W)
  hal::u32 volatile capture_compare_mode_register_2;
  /// Offset: 0x20 Capture/Compare Enable register (R/W)
  hal::u32 volatile cc_enable_register;
  /// Offset: 0x24 Counter (R/W)
  hal::u32 volatile counter_register;
  /// Offset: 0x28 Prescalar (R/W)
  hal::u32 volatile prescale_register;
  /// Offset: 0x2C Auto Reload Register (R/W)
  hal::u32 volatile auto_reload_register;
  /// Offset: 0x30 Repetition Counter Register (R/W)
  hal::u32 volatile repetition_counter_register;
  /// Offset: 0x34 Capture Compare Register (R/W)
  hal::u32 volatile capture_compare_register;
  /// Offset: 0x38 Capture Compare Register (R/W)
  hal::u32 volatile capture_compare_register_2;
  // Offset: 0x3C Capture Compare Register (R/W)
  hal::u32 volatile capture_compare_register_3;
  // Offset: 0x40 Capture Compare Register (R/W)
  hal::u32 volatile capture_compare_register_4;
  /// Offset: 0x44 Break and dead-time register
  hal::u32 volatile break_and_deadtime_register;
  /// Offset: 0x48 DMA control register
  hal::u32 volatile dma_control_register;
  /// Offset: 0x4C DMA address for full transfer
  hal::u32 volatile dma_address_register;
};

[[nodiscard]] timer_reg_t* get_timer_reg(void* p_reg)
{
  return reinterpret_cast<timer_reg_t*>(p_reg);
}

void setup(timer_reg_t* p_reg)
{
  constexpr auto auto_reload_preload_enable = hal::bit_mask::from<7>();
  constexpr auto one_pulse_mode = hal::bit_mask::from<3>();
  constexpr auto update_request_source = hal::bit_mask::from<2>();
  constexpr auto update_interrupt_enable = hal::bit_mask::from<0>();

  bit_modify(p_reg->control_register)
    .set(auto_reload_preload_enable)
    .set(one_pulse_mode)
    .set(update_request_source);

  bit_modify(p_reg->interrupt_enable_register).set(update_interrupt_enable);
}

/**
 * @brief Implements shared timer setup and control logic common to all STM32
 * series.
 *
 * This class provides the common functionality for timer configuration and
 * control, abstracting the parts of the timer interface that remain
 * consistent across different STM32 series. It is intended to be used by
 * series-specific implementations, which handle MCU-specific configuration
 * details and pass any required parameters to these generic functions.
 */
export class timer final
{
public:
  /**
   * @brief Construct timer uninitialized
   *
   * The purpose of this is to send the settings through the initialize
   * function instead, because all the settings are series-specific.
   * Therefore in the series-specific implementations of the timer, the
   * address is deduced from the timer, as well as all the interrupt
   * configuration is done, then they are passed to initialize.
   *
   * It is unsafe to call any API of this class before calling the
   * `initialize()` API with the correct inputs. Once that API has been
   * called without failure, then the other APIs will become available.
   */
  timer() = default;

  /**
   * @brief Determine if the timer is currently running
   *
   * @return true - if a callback has been scheduled and has not been
   * invoked yet, false otherwise.
   */
  [[nodiscard]] bool is_running()
  {
    constexpr auto counter_enable = hal::bit_mask::from<0>();
    auto* reg = get_timer_reg(m_reg);

    return bit_extract<counter_enable>(reg->control_register);
  }

  /**
   * @brief Stops a scheduled event from happening.
   *
   * Does nothing if the timer is not currently running.
   *
   * Note that there must be sufficient time between the this call
   * finishing and the scheduled event's termination. If this call is too
   * close to when the schedule event expires, this function may not
   * complete before the timer interrupt is triggered.
   */
  void cancel()
  {
    constexpr auto counter_enable = hal::bit_mask::from<0>();
    auto* reg = get_timer_reg(m_reg);

    bit_modify(reg->control_register).clear(counter_enable);
  }

  /**
   * @brief Schedule an interrupt to occur for this timer
   *
   * If this is called and the timer has already scheduled an event (in
   * other words, `is_running()` returns true), then the previous scheduled
   * event will be canceled and the new scheduled event will be started.
   *
   * If the delay time result in a tick period of 0, then the timer will
   * execute after 1 tick period. For example, if the tick period is 1ms
   * and the requested time delay is 500us, then the event will be
   * scheduled for 1ms.
   *
   * If the tick period is 1ms and the requested time is 2.5ms then the
   * event will be scheduled after 2 tick periods or in 2ms.
   *
   * @param p_delay - the amount of time until the timer expires
   * @param p_timer_clock_frequency - the clock driving the timer
   * @throws hal::argument_out_of_domain - if p_delay cannot be achieved.
   */
  void schedule(hal::time_duration p_delay, u32 p_timer_clock_frequency)
  {
    cancel();

    constexpr auto counter_enable = hal::bit_mask::from<0>();
    constexpr auto update_generation = hal::bit_mask::from<0>();
    constexpr auto prescaler = hal::bit_mask::from<0, 15>();
    constexpr auto auto_reload_value = hal::bit_mask::from<0, 15>();
    constexpr auto counter = hal::bit_mask::from<0, 15>();

    auto* reg = get_timer_reg(m_reg);

    constexpr u32 scale_factor = decltype(p_delay)::period::den;
    constexpr u16 prescaler_max_value = std::numeric_limits<hal::u16>::max();
    constexpr u16 timer_max_ticks = std::numeric_limits<hal::u16>::max();
    constexpr u16 timer_reset_value = 0;

    u32 const time_per_tick_ns = scale_factor / p_timer_clock_frequency;
    // Check if CPU Frequency too fast
    if (time_per_tick_ns == 0) {
      throw hal::argument_out_of_domain(this);
    }
    // Use 64-bit to prevent immediate overflow. If still too big after
    // division, exception thrown.
    u64 const ticks_required = p_delay.count() / time_per_tick_ns;
    u64 const prescaler_value = ticks_required / timer_max_ticks;
    if (prescaler_value > prescaler_max_value) {
      throw hal::argument_out_of_domain(this);
    }

    u16 prescaler_ticks_required = 0;
    // When the delay amount is shorter than one clock cycle
    if (ticks_required == 0) {
      prescaler_ticks_required = 1;
    } else {
      u32 const prescaler_frequency =
        p_timer_clock_frequency / (static_cast<u16>(prescaler_value) + 1);
      u32 const prescaler_time_per_tick_ns =
        scale_factor / prescaler_frequency;
      prescaler_ticks_required =
        static_cast<u16>(p_delay.count() / prescaler_time_per_tick_ns);
    }

    bit_modify(reg->prescale_register)
      .insert<prescaler>(static_cast<u16>(prescaler_value));
    bit_modify(reg->auto_reload_register)
      .insert<auto_reload_value>(prescaler_ticks_required);
    bit_modify(reg->counter_register).insert<counter>(timer_reset_value);
    bit_modify(reg->event_generator_register).set(update_generation);
    bit_modify(reg->control_register).set(counter_enable);
  }

  /**
   * @brief Initialize the timer with the series-specific settings
   *
   * This is where the constructed uninitialized timer gets initialized. It
   * gets all the series-specific settings passed to it that it needs, and
   * it handles them appropriately.
   *
   * @param p_peripheral_address - the address of the chosen timer
   * peripheral
   * @param p_initialize_interrupts_function - the function needed to
   * initialize interrupts for the specific series stm32. For example, for
   * the stm32f1 series, the function passed would be
   * `hal::stm32f1::initialize_interrupts()`.
   * @param p_irq - the irq number for the chosen timer
   * @param p_handler - the platform specific interrupt handler to be
   * installed
   * @throws hal::device_or_resource_busy - if the timer interrupt vector is
   * already in use.
   */
  void initialize(void* p_peripheral_address,
                  void (*p_initialize_interrupts_function)(),
                  cortex_m::irq_t p_irq,
                  cortex_m::interrupt_pointer p_handler)
  {
    m_reg = p_peripheral_address;
    p_initialize_interrupts_function();
    if (hal::cortex_m::is_interrupt_enabled(p_irq)) {
      throw hal::device_or_resource_busy(this);
    }
    cortex_m::enable_interrupt(p_irq, p_handler);
    setup(get_timer_reg(m_reg));
  }

private:
  /// Stores the base address of the timer
  void* m_reg = nullptr;
};
}  // namespace hal::stm32_generic
