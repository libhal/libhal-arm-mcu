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
#include <libhal-arm-mcu/dwt_counter.hpp>
#include <libhal-arm-mcu/startup.hpp>
#include <libhal-arm-mcu/system_control.hpp>

#include <libhal-arm-mcu/stm32f411/clock.hpp>
#include <libhal-arm-mcu/stm32f411/constants.hpp>
#include <libhal-arm-mcu/stm32f411/input_pin.hpp>
#include <libhal-arm-mcu/stm32f411/output_pin.hpp>
#include <libhal-arm-mcu/stm32f411/spi.hpp>

#include <libhal-soft/inert_drivers/inert_adc.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/units.hpp>
#include <resource_list.hpp>

resource_list initialize_platform()
{
  using namespace hal::literals;
  hal::stm32f411::pin a(hal::stm32f411::peripheral::gpio_a, 8);
  // hal::stm32f411::maximum_speed_using_internal_oscillator();

  auto cpu_frequency = hal::stm32f411::frequency(hal::stm32f411::peripheral::cpu);
  static hal::cortex_m::dwt_counter steady_clock(cpu_frequency);

  static hal::stm32f411::input_pin button(
    hal::stm32f411::peripheral::gpio_c,
    13,
    { .resistor = hal::pin_resistor::pull_up });
  static hal::stm32f411::output_pin led(hal::stm32f411::peripheral::gpio_a, 5);

  return {
    .reset = []() { hal::cortex_m::reset(); },
    .status_led = &led,
    .clock = &steady_clock,
    .input_pin = &button
  };
}