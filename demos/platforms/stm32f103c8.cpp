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
#include <libhal-util/bit_bang_i2c.hpp>
#include <libhal-util/bit_bang_spi.hpp>
#include <libhal-util/inert_drivers/inert_adc.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>

#include <resource_list.hpp>

constexpr bool use_bit_bang_spi = false;

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

  static hal::stm32f1::can_peripheral_manager can(
    1_MHz, hal::stm32f1::can_pins::pb9_pb8);

  static std::array<hal::can_message, 32> receive_buffer{};
  static auto can_transceiver = can.acquire_transceiver(receive_buffer);
  p_resources.can_transceiver = &can_transceiver;

  static auto can_bus_manager = can.acquire_bus_manager();
  p_resources.can_bus_manager = &can_bus_manager;

  static auto can_interrupt = can.acquire_interrupt();
  p_resources.can_interrupt = &can_interrupt;

  static auto id_filters_x4 = can.acquire_identifier_filter();
  id_filters_x4[0].allow(0x240);
  id_filters_x4[1].allow(0x250);
  id_filters_x4[2].allow(0x260);
  id_filters_x4[3].allow(0x270);
  static auto extended_id_filters_x2 = can.acquire_extended_identifier_filter();
  extended_id_filters_x2[0].allow(0x0111'0000);
  extended_id_filters_x2[1].allow(0x0111'1111);

  static auto mask_id_filters_x2 = can.acquire_mask_filter();
  // allow 0x222 to 0x0223
  mask_id_filters_x2[0].allow({ { .id = 0x222, .mask = 0x1F4 } });
  // allow 0x330 to 0x0223
  mask_id_filters_x2[1].allow({ { .id = 0x333, .mask = 0x1F4 } });

  static auto extended_mask_id_filter = can.acquire_extended_mask_filter();
  // allow 0x0222'0000 to 0x0222'0003
  extended_mask_id_filter.allow({ { .id = 0x0222'0000, .mask = 0x7FFF'FFF4 } });

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

  static hal::stm32f1::output_pin spi_chip_select('A', 4);
  p_resources.spi_chip_select = &spi_chip_select;
  static hal::stm32f1::output_pin sck('A', 5);
  static hal::stm32f1::output_pin copi('A', 6);
  static hal::stm32f1::input_pin cipo('A', 7);

  static hal::bit_bang_spi::pins bit_bang_spi_pins{ .sck = &sck,
                                                    .copi = &copi,
                                                    .cipo = &cipo };

  static hal::spi::settings bit_bang_spi_settings{
    .clock_rate = 250.0_kHz,
    .clock_polarity = false,
    .clock_phase = false,
  };

  hal::spi* spi = nullptr;

  if constexpr (use_bit_bang_spi) {
    static hal::bit_bang_spi bit_bang_spi(
      bit_bang_spi_pins, steady_clock, bit_bang_spi_settings);
    spi = &bit_bang_spi;
  } else {
    static hal::stm32f1::spi spi1(hal::bus<1>,
                                  {
                                    .clock_rate = 250.0_kHz,
                                    .clock_polarity = false,
                                    .clock_phase = false,
                                  });
    spi = &spi1;
  }
  p_resources.spi = spi;
}
