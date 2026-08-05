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

#include <cstdint>

export module hal.arm_mcu.cortex_m:systick_timer;

import hal;
import hal.util;

import :interrupt;

namespace hal::cortex_m {
/// Structure type to access the System Timer (SysTick).
struct systick_register_t
{
  /// Offset: 0x000 (R/W)  SysTick Control and Status Register
  std::uint32_t volatile control;
  /// Offset: 0x004 (R/W)  SysTick Reload Value Register
  std::uint32_t volatile reload;
  /// Offset: 0x008 (R/W)  SysTick Current Value Register
  /// NOTE: Setting this value to anything will zero it out. Setting this zero
  /// will NOT cause the SysTick interrupt to be fired.
  std::uint32_t volatile current_value;
  /// Offset: 0x00C (R/ )  SysTick Calibration Register
  std::uint32_t const volatile calib;
};

/// Namespace containing the bit_mask objects that are used to manipulate the
/// ARM Cortex Mx SysTick Timer.
namespace systick_control_register {
/// When set to 1, takes the contents of the reload counter, writes it to
/// the current_value register and begins counting down to zero. Setting
/// this to zero stops the counter. Restarting the counter will restart the
/// count.
constexpr auto enable_counter = hal::bit_mask::from<0>();

/// When SysTick timer's count goes from 1 to 0, if this bit is set, the
/// SysTick interrupt will fire.
constexpr auto enable_interrupt = hal::bit_mask::from<1>();

/// If set to 0, clock source is external, if set to 1, clock source follows
/// the processor clock.
constexpr auto clock_source = hal::bit_mask::from<2>();

/// Set to 1 when count falls from 1 to 0. This bit is cleared on the next
/// read of this register.
constexpr auto count_flag = hal::bit_mask::from<16>();
}  // namespace systick_control_register

/// The address of the sys_tick register
constexpr auto systick_address = static_cast<uptr>(0xE000'E010UL);

/// @return auto* - Address of the ARM Cortex SysTick peripheral
// NOLINTNEXTLINE(performance-no-int-to-ptr)
auto* sys_tick = reinterpret_cast<systick_register_t*>(systick_address);

void systick_start()
{
  hal::bit_modify(sys_tick->control)
    .set<systick_control_register::enable_counter>();
}

void systick_stop()
{
  hal::bit_modify(sys_tick->control)
    .clear<systick_control_register::enable_counter>();
}

/**
 * @brief SysTick driver for the ARM Cortex Mx series chips.
 *
 * Available in all ARM Cortex M series processors. Provides a generic and
 * simple timer for every platform using these processor.
 *
 */
export class systick_timer : public hal::timed_interrupt
{
public:
  /**
   * @brief Defines the set of clock sources for the SysTick timer
   *
   */
  enum class clock_source : std::uint8_t
  {
    /// Use an external clock source. What this source is depends on the
    /// architecture and configuration of the platform.
    external = 0,
    /// Use the clock given to the CPU
    processor = 1,
  };

  /**
   * @brief Construct a new systick_timer timer object
   *
   * PRECONDITION: Interrupt vector table must be initialized before creating
   * an instance of this object.
   *
   * @param p_frequency - the clock source's frequency
   * @param p_source - the source of the clock to the systick timer
   * @throws hal::operation_not_permitted - thrown when the precondition to
   * initialize the interrupt vector table before constructing this object.
   */
  systick_timer(hertz p_frequency,
                clock_source p_source = clock_source::processor)
    : m_frequency(p_frequency)
  {
    if (not interrupt_vector_table_initialized()) {
      throw hal::operation_not_permitted(this);
    }
    register_cpu_frequency(p_frequency, p_source);
  }

  /**
   * @brief Inform the driver of the operating frequency of the CPU in order
   * to generate the correct uptime.
   *
   * Use this when the CPU's operating frequency has changed and no longer
   * matches the frequency supplied to the constructor. Care should be taken
   * when expecting this function when there is the potentially other parts
   * of the system that depend on this counter's uptime to operate.
   *
   * This will clear any ongoing scheduled events as the timing will no
   * longer be valid.
   *
   * @param p_frequency - the clock source's frequency
   * @param p_source - the source of the clock to the systick timer
   */
  void register_cpu_frequency(hertz p_frequency,
                              clock_source p_source = clock_source::processor)
  {
    systick_stop();
    m_frequency = p_frequency;

    // Since reloads only occur when the current_value falls from 1 to 0,
    // setting this register directly to zero from any other number will
    // disable reloading of the register and will stop the timer.
    sys_tick->current_value = 0;

    auto control = hal::bit_value<std::uint32_t>(0);
    control.set<systick_control_register::enable_interrupt>();

    if (p_source == clock_source::processor) {
      control.set<systick_control_register::clock_source>();
    } else {
      control.clear<systick_control_register::clock_source>();
    }

    // Disable the counter if it was previously enabled.
    control.clear<systick_control_register::enable_counter>();

    sys_tick->control = control.get();
  }

  /**
   * @brief Destroy the system timer object
   *
   * Stop the timer and disable the interrupt service routine.
   */
  ~systick_timer() override
  {
    systick_stop();
    disable_interrupt(irq::systick);
    if (s_active == this) {
      s_active = nullptr;
    }
  }

private:
  static constexpr u32 maximum = 0x00FF'FFFF;

  static void isr()
  {
    if (s_active == nullptr) {
      systick_stop();
      return;
    }

    auto& active = *s_active;

    if (active.m_remaining_counts > maximum) {
      sys_tick->current_value = 0;
      sys_tick->reload = maximum;
      active.m_remaining_counts -= maximum;
      systick_start();
      return;
    } else if (active.m_remaining_counts > 0) {
      sys_tick->current_value = 0;
      // NOTE: This value is safe to cast without narrowing due to the if
      // statement before it that checks if the value is greater than maximum.
      sys_tick->reload = static_cast<std::uint32_t>(active.m_remaining_counts);
      active.m_remaining_counts = 0;
      systick_start();
      return;
    }

    // remaining counts is 0, time to execute callback

    if (active.m_callback.has_value()) {
      active.m_callback.value()->callback();
    }

    if (active.m_mode == timer_mode::one_shot) {
      systick_stop();
      active.m_callback.reset();
    }
  }

  bool driver_scheduled() override
  {
    return m_callback.has_value();
  }

  void driver_schedule(mem::optional_ptr<hal::timed_callback> const& p_callback,
                       hal::time_duration p_delay,
                       hal::timer_mode p_mode) override
  {
    systick_stop();

    if (not p_callback.has_value()) {
      m_callback = nullptr;
      return;
    }

    auto cycle_count = hal::cycles_per(m_frequency, p_delay);

    if (cycle_count <= 1) {
      cycle_count = 1;
    } else if (cycle_count > maximum) {
      m_remaining_counts = cycle_count - maximum;
      cycle_count = maximum;
    }

    m_callback = p_callback;
    m_mode = p_mode;
    s_active = this;

    // Enable interrupt service routine for SysTick and use this callback as
    // the handler
    enable_interrupt(irq::systick, &isr);

    sys_tick->current_value = 0;
    sys_tick->reload = static_cast<std::uint32_t>(cycle_count);

    // Starting the timer will restart the count
    systick_start();
  }

  inline static systick_timer* s_active = nullptr;

  hertz m_frequency{ 1 * mp_units::si::unit_symbols::MHz };
  mem::optional_ptr<hal::timed_callback> m_callback;
  timer_mode m_mode = timer_mode::one_shot;
  u64 m_remaining_counts = 0;
};
}  // namespace hal::cortex_m
