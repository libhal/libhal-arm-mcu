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
#include <libhal-arm-mcu/lpc40/adc.hpp>
#include <libhal-arm-mcu/lpc40/can.hpp>
#include <libhal-arm-mcu/lpc40/constants.hpp>
#include <libhal-arm-mcu/lpc40/dma_spi.hpp>
#include <libhal-arm-mcu/lpc40/i2c.hpp>
#include <libhal-arm-mcu/lpc40/input_pin.hpp>
#include <libhal-arm-mcu/lpc40/interrupt_pin.hpp>
#include <libhal-arm-mcu/lpc40/output_pin.hpp>
#include <libhal-arm-mcu/lpc40/pwm.hpp>
#include <libhal-arm-mcu/lpc40/spi.hpp>
#include <libhal-arm-mcu/lpc40/stream_dac.hpp>
#include <libhal-arm-mcu/lpc40/uart.hpp>
#include <libhal-arm-mcu/startup.hpp>
#include <libhal-arm-mcu/system_control.hpp>
#include <libhal-lpc40/clock.hpp>

#include <resource_list.hpp>

resource_list initialize_platform()
{
  using namespace hal::literals;

  // Set the MCU to the maximum clock speed
  hal::lpc40::maximum(12.0_MHz);

  static hal::cortex_m::dwt_counter counter(
    hal::lpc40::get_frequency(hal::lpc40::peripheral::cpu));

  static std::array<hal::byte, 64> receive_buffer{};
  static hal::lpc40::uart uart0(0,
                                receive_buffer,
                                hal::serial::settings{
                                  .baud_rate = 115200,
                                });

  static hal::lpc40::can can(2,
                             hal::can::settings{
                               .baud_rate = 1.0_MHz,
                             });

  static hal::lpc40::output_pin led(1, 10);
  static hal::lpc40::adc adc4(hal::channel<4>);
  static hal::lpc40::input_pin input_pin(0, 29);
  static hal::lpc40::i2c i2c2(2);
  static hal::lpc40::interrupt_pin interrupt_pin(0, 29);
  static hal::lpc40::pwm pwm(1, 6);
  static hal::lpc40::dma_spi spi2(2);
  static hal::lpc40::output_pin chip_select(1, 8);
  static hal::lpc40::stream_dac_u8 stream_dac;

  return {
    .reset = []() { hal::cortex_m::reset(); },
    .console = &uart0,
    .status_led = &led,
    .clock = &counter,
    .can = &can,
    .adc = &adc4,
    .input_pin = &input_pin,
    .i2c = &i2c2,
    .interrupt_pin = &interrupt_pin,
    .pwm = &pwm,
    .spi = &spi2,
    .spi_chip_select = &chip_select,
    .stream_dac = &stream_dac,
  };
}
