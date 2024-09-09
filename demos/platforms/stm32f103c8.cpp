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
#include <libhal-soft/bit_bang_spi.hpp>
#include <libhal-soft/inert_drivers/inert_adc.hpp>

#include <libhal/output_pin.hpp>
#include <libhal/units.hpp>
#include <resource_list.hpp>

constexpr bool use_bit_bang_spi = false;

resource_list initialize_platform()
{
  using namespace hal::literals;

  // Set the MCU to the maximum clock speed
  hal::stm32f1::maximum_speed_using_internal_oscillator();

  auto cpu_frequency = hal::stm32f1::frequency(hal::stm32f1::peripheral::cpu);
  static hal::cortex_m::dwt_counter steady_clock(cpu_frequency);
  static hal::stm32f1::uart uart1(hal::port<1>, hal::buffer<128>);
  static hal::stm32f1::can can({ .baud_rate = 1'000'000 },
                               hal::stm32f1::can_pins::pb9_pb8);
  static hal::stm32f1::output_pin led('C', 13);

  // TODO: replace with actual ADC
  static hal::soft::inert_adc adc(0.5);

  // pin G0 on the STM micromod is port B, pin 4
  static hal::stm32f1::input_pin input_pin('B', 4);

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

  static hal::stm32f1::output_pin spi_chip_select('A', 4);
  static hal::stm32f1::output_pin sck('A', 5);
  static hal::stm32f1::output_pin copi('A', 6);
  static hal::stm32f1::input_pin cipo('A', 7);

  static hal::soft::bit_bang_spi::pins bit_bang_spi_pins{ .sck = &sck,
                                                          .copi = &copi,
                                                          .cipo = &cipo };

  static hal::spi::settings bit_bang_spi_settings{
    .clock_rate = 250.0_kHz,
    .clock_polarity = false,
    .clock_phase = false,
  };

  hal::spi* spi = nullptr;

  if constexpr (use_bit_bang_spi) {
    static hal::soft::bit_bang_spi bit_bang_spi(
      bit_bang_spi_pins, steady_clock, bit_bang_spi_settings);
    spi = &bit_bang_spi;
  } else {
    static hal::stm32f1::spi spi1(hal::bus<2>,
                                  {
                                    .clock_rate = 250.0_kHz,
                                    .clock_polarity = false,
                                    .clock_phase = false,
                                  });
    spi = &spi1;
  }

  return {
    .reset = []() { hal::cortex_m::reset(); },
    .console = &uart1,
    .status_led = &led,
    .clock = &steady_clock,
    .can = &can,
    .adc = &adc,
    .input_pin = &input_pin,
    .i2c = &bit_bang_i2c,
    .spi = spi,
    .spi_chip_select = &spi_chip_select,
  };
}
