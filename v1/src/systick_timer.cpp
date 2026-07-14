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

#include <libhal-arm-mcu/systick_timer.hpp>

#include <cstdint>

#include <libhal-arm-mcu/interrupt.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/static_callable.hpp>
#include <libhal-util/units.hpp>
#include <libhal/error.hpp>
#include <libhal/functional.hpp>

#include "systick_timer_reg.hpp"

namespace hal::cortex_m {

void start()
{
  hal::bit_modify(sys_tick->control)
    .set<systick_control_register::enable_counter>();
}

void stop()
{
  hal::bit_modify(sys_tick->control)
    .clear<systick_control_register::enable_counter>();
}

systick_timer::systick_timer(hertz p_frequency, clock_source p_source)
  : m_frequency(p_frequency)
{
  if (not interrupt_vector_table_initialized()) {
    hal::safe_throw(hal::operation_not_permitted(this));
  }
  register_cpu_frequency(p_frequency, p_source);
}

void systick_timer::register_cpu_frequency(hertz p_frequency,
                                           clock_source p_source)
{
  stop();
  m_frequency = p_frequency;

  // Since reloads only occur when the current_value falls from 1 to 0,
  // setting this register directly to zero from any other number will disable
  // reloading of the register and will stop the timer.
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

systick_timer::~systick_timer()
{
  stop();
  disable_interrupt(event_number);
}

bool systick_timer::driver_is_running()
{
  auto running_bit = static_cast<bool>(
    hal::bit_extract<systick_control_register::enable_counter>(
      sys_tick->control));
  return running_bit;
}

void systick_timer::driver_cancel()
{
  // All that is needed is to stop the timer. When the timer is started again
  // via `schedule()`, the timer value will be reloaded/reset.
  stop();
}

void systick_timer::driver_schedule(hal::callback<void(void)> p_callback,
                                    hal::time_duration p_delay)
{
  constexpr std::int64_t maximum = 0x00FFFFFF;

  auto cycle_count = cycles_per(m_frequency, p_delay);
  if (cycle_count <= 1) {
    cycle_count = 1;
  } else if (cycle_count > maximum) {
    throw std::errc::invalid_argument;
  }

  // Stop the previously scheduled event
  stop();

  // Save the p_callback to the static_callable object's statically allocated
  // callback function. The lifetime of this object exists for the duration of
  // the program, so this will never become a dangling reference.
  auto handler = static_callable<systick_timer, 0, void(void)>(p_callback);

  // Enable interrupt service routine for SysTick and use this callback as the
  // handler
  enable_interrupt(event_number, handler.get_handler());

  sys_tick->current_value = 0;
  sys_tick->reload = static_cast<uint32_t>(cycle_count);

  // Starting the timer will restart the count
  start();
}
}  // namespace hal::cortex_m
