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
#include <libhal-arm-mcu/stm32f1/can.hpp>
#include <libhal-arm-mcu/stm32f1/clock.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32f1/input_pin.hpp>
#include <libhal-arm-mcu/stm32f1/output_pin.hpp>
#include <libhal-arm-mcu/stm32f1/spi.hpp>
#include <libhal-arm-mcu/stm32f1/uart.hpp>
#include <libhal-arm-mcu/system_control.hpp>
#include <libhal-soft/bit_bang_i2c.hpp>
#include <libhal-soft/inert_drivers/inert_adc.hpp>

#include <libhal/output_pin.hpp>
#include <libhal/units.hpp>
#include <resource_list.hpp>

void initialize_platform(resource_list& p_resources)
{
  using namespace hal::literals;
  p_resources.reset = []() { hal::cortex_m::reset(); };

  // Set the MCU to the maximum clock speed
  hal::stm32f1::maximum_speed_using_internal_oscillator();

  auto cpu_frequency = hal::stm32f1::frequency(hal::stm32f1::peripheral::cpu);
  static hal::cortex_m::dwt_counter steady_clock(cpu_frequency);
  p_resources.clock = &steady_clock;

  static hal::stm32f1::uart uart1(hal::port<1>, hal::buffer<128>);
  p_resources.console = &uart1;

  static hal::stm32f1::can can({ .baud_rate = 1'000'000 },
                               hal::stm32f1::can_pins::pb9_pb8);
  p_resources.can = &can;

  static hal::stm32f1::output_pin led('C', 13);
  p_resources.status_led = &led;

  // pin G0 on the STM micromod is port B, pin 4
  static hal::stm32f1::input_pin input_pin('B', 4);
  p_resources.input_pin = &input_pin;

  static hal::stm32f1::output_pin sda_output_pin('B', 7);
  static hal::stm32f1::output_pin scl_output_pin('B', 6);

  sda_output_pin.configure({
    .resistor = hal::pin_resistor::pull_up,
    .open_drain = true,
  });
  scl_output_pin.configure({
    .resistor = hal::pin_resistor::pull_up,
    .open_drain = true,
  });
  static hal::bit_bang_i2c::pins bit_bang_pins{
    .sda = &sda_output_pin,
    .scl = &scl_output_pin,
  };
  static hal::bit_bang_i2c bit_bang_i2c(bit_bang_pins, steady_clock);
  p_resources.i2c = &bit_bang_i2c;

  static hal::stm32f1::spi spi1(hal::bus<1>,
                                {
                                  .clock_rate = 250.0_kHz,
                                  .clock_polarity = false,
                                  .clock_phase = false,
                                });
  p_resources.spi = &spi1;

  static hal::stm32f1::output_pin spi_chip_select('A', 4);
  p_resources.spi_chip_select = &spi_chip_select;
}
