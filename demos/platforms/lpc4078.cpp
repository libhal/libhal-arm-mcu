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
#include <libhal-arm-mcu/lpc40/adc.hpp>
#include <libhal-arm-mcu/lpc40/can.hpp>
#include <libhal-arm-mcu/lpc40/clock.hpp>
#include <libhal-arm-mcu/lpc40/constants.hpp>
#include <libhal-arm-mcu/lpc40/dac.hpp>
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

#include <libhal/error.hpp>
#include <resource_list.hpp>

void hal::watchdog::start()
{
  throw hal::operation_not_supported(nullptr);
};
void hal::watchdog::reset()
{
  throw hal::operation_not_supported(nullptr);
}
void hal::watchdog::set_countdown_time(
  [[maybe_unused]] hal::time_duration p_wait_time)
{
  throw hal::operation_not_supported(nullptr);
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

  p_resources.reset = []() { hal::cortex_m::reset(); };

  // Set the MCU to the maximum clock speed
  hal::lpc40::maximum(12.0_MHz);

  static hal::lpc40::output_pin led(1, 10);
  p_resources.status_led = &led;

  static hal::cortex_m::dwt_counter counter(
    hal::lpc40::get_frequency(hal::lpc40::peripheral::cpu));
  p_resources.clock = &counter;

  static std::array<hal::byte, 64> receive_buffer{};
  static hal::lpc40::uart uart0(0,
                                receive_buffer,
                                hal::serial::settings{
                                  .baud_rate = 115200,
                                });
  p_resources.console = &uart0;

  static hal::lpc40::can can(2,
                             hal::can::settings{
                               .baud_rate = 1.0_kHz,
                             });
  // p_resources.can = &can;

  static hal::lpc40::adc adc4(hal::channel<4>);
  p_resources.adc = &adc4;

  static hal::lpc40::input_pin input_pin(0, 29);
  p_resources.input_pin = &input_pin;

  static hal::lpc40::i2c i2c2(2);
  p_resources.i2c = &i2c2;

  static hal::lpc40::interrupt_pin interrupt_pin(0, 29);
  p_resources.interrupt_pin = &interrupt_pin;

  static hal::lpc40::pwm pwm(1, 6);
  p_resources.pwm = &pwm;

  static hal::lpc40::dma_spi spi2(2);
  p_resources.spi = &spi2;

  static hal::lpc40::output_pin chip_select(1, 8);
  p_resources.spi_chip_select = &chip_select;

  static hal::lpc40::stream_dac_u8 stream_dac;
  p_resources.stream_dac = &stream_dac;

  static hal::lpc40::dac dac;
  p_resources.dac = &dac;
}
