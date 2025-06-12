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

#include <libhal-arm-mcu/dwt_counter.hpp>
#include <libhal-arm-mcu/startup.hpp>
#include <libhal-arm-mcu/stm32f411/clock.hpp>
#include <libhal-arm-mcu/stm32f411/constants.hpp>
#include <libhal-arm-mcu/stm32f411/dma.hpp>
#include <libhal-arm-mcu/stm32f411/input_pin.hpp>
#include <libhal-arm-mcu/stm32f411/output_pin.hpp>
#include <libhal-arm-mcu/stm32f411/pin.hpp>
#include <libhal-arm-mcu/stm32f411/spi.hpp>
#include <libhal-arm-mcu/stm32f411/uart.hpp>
#include <libhal-arm-mcu/system_control.hpp>
#include <libhal/initializers.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/units.hpp>

#include <resource_list.hpp>

void hal::watchdog::start()
{
  throw hal::operation_not_supported(nullptr);
};
void hal::watchdog::reset()
{
  throw hal::operation_not_supported(nullptr);
}
void hal::watchdog::set_countdown_time(hal::time_duration p_wait_time)
{
  if (p_wait_time == p_wait_time) {  // so compiler dosen't freakout
    throw hal::operation_not_supported(nullptr);
  }
}
bool hal::watchdog::check_flag()
{
  throw hal::operation_not_supported(nullptr);
}
void hal::watchdog::clear_flag()
{
  throw hal::operation_not_supported(nullptr);
}

void initialize_platform(resource_list& p_resources)
{
  using namespace hal::literals;
  hal::stm32f411::maximum_speed_using_internal_oscillator();

  auto const cpu_frequency =
    hal::stm32f411::frequency(hal::stm32f411::peripheral::cpu);
  static hal::cortex_m::dwt_counter steady_clock(cpu_frequency / 8);

  static hal::stm32f411::input_pin button(
    hal::stm32f411::peripheral::gpio_c,
    13,
    { .resistor = hal::pin_resistor::pull_up });
  static hal::stm32f411::output_pin led(hal::stm32f411::peripheral::gpio_a, 5);

  static hal::stm32f411::spi spi(hal::runtime{}, 2, {});
  static hal::stm32f411::output_pin chip_select(
    hal::stm32f411::peripheral::gpio_b, 13);

  static hal::stm32f411::uart uart2(
    hal::port<2>, hal::buffer<128>, { .baud_rate = 115200 });
  p_resources.console = &uart2;
  p_resources.reset = []() { hal::cortex_m::reset(); };
  p_resources.status_led = &led;
  p_resources.clock = &steady_clock;
  p_resources.input_pin = &button;
  p_resources.spi = &spi;
  p_resources.spi_chip_select = &chip_select;
  return;
}
