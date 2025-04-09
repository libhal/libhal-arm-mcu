// Copyright 2024 Khalil Estell
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

#pragma once

#include <libhal-arm-mcu/interrupt.hpp>
#include <libhal/functional.hpp>
#include <libhal/initializers.hpp>
#include <libhal/units.hpp>

namespace hal::stm32_generic {

class timer final
{
public:
  // potentially dont need??
  timer(hal::unsafe,
        void* p_peripheral_address,
        void (*initialize_interrupts_function)(),
        cortex_m::irq_t p_irq,
        cortex_m::interrupt_pointer p_handler);

  timer(hal::unsafe);

  /**
   * @brief Determine if the timer is currently running
   *
   * @return true - if a callback has been scheduled and has not been invoked
   * yet, false otherwise.
   */
  [[nodiscard]] bool is_running();

  /**
   * @brief Stops a scheduled event from happening.
   *
   * Does nothing if the timer is not currently running.
   *
   * Note that there must be sufficient time between the this call finishing and
   * the scheduled event's termination. If this call is too close to when the
   * schedule event expires, this function may not complete before the hardware
   * calls the callback.
   */
  void cancel();

  /**
   * @brief Schedule an interrupt to occur for this timer
   *
   * If this is called and the timer has already scheduled an event (in other
   * words, `is_running()` returns true), then the previous scheduled event will
   * be canceled and the new scheduled event will be started.
   *
   * If the delay time result in a tick period of 0, then the timer will execute
   * after 1 tick period. For example, if the tick period is 1ms and the
   * requested time delay is 500us, then the event will be scheduled for 1ms.
   *
   * If the tick period is 1ms and the requested time is 2.5ms then the event
   * will be scheduled after 2 tick periods or in 2ms.
   *
   * @param p_delay - the amount of time until the timer expires
   * @param p_timer_clock_frequency - the clock driving the timer
   * @throws hal::argument_out_of_domain - if p_interval is greater than what
   * can be cannot be achieved.
   */
  void schedule(hal::time_duration p_delay, u32 p_timer_clock_frequency);

  void initialize(hal::unsafe,
                  void* p_peripheral_address,
                  void (*initialize_interrupts_function)(),
                  cortex_m::irq_t p_irq,
                  cortex_m::interrupt_pointer p_handler);

private:
  void* m_reg = nullptr;
};
}  // namespace hal::stm32_generic
