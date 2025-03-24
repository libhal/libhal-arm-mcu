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
#include <libhal-arm-mcu/stm32f1/adc.hpp>
#include <libhal-arm-mcu/stm32f1/can.hpp>
#include <libhal-arm-mcu/stm32f1/clock.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32f1/input_pin.hpp>
#include <libhal-arm-mcu/stm32f1/output_pin.hpp>
#include <libhal-arm-mcu/stm32f1/spi.hpp>
#include <libhal-arm-mcu/stm32f1/timer.hpp>
#include <libhal-arm-mcu/stm32f1/uart.hpp>
#include <libhal-arm-mcu/system_control.hpp>
#include <libhal-util/atomic_spin_lock.hpp>
#include <libhal-util/bit_bang_i2c.hpp>
#include <libhal-util/bit_bang_spi.hpp>
#include <libhal-util/inert_drivers/inert_adc.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/pwm.hpp>
#include <libhal/units.hpp>

#include <resource_list.hpp>

constexpr bool use_bit_bang_spi = false;
constexpr bool use_libhal_4_pwm = false;

void initialize_platform(resource_list& p_resources)
{
  using namespace hal::literals;
  p_resources.reset = []() { hal::cortex_m::reset(); };

  // Set the MCU to the maximum clock speed
  hal::stm32f1::maximum_speed_using_internal_oscillator();
  hal::stm32f1::release_jtag_pins();

  auto cpu_frequency = hal::stm32f1::frequency(hal::stm32f1::peripheral::cpu);
  static hal::cortex_m::dwt_counter steady_clock(cpu_frequency);
  p_resources.clock = &steady_clock;

  static hal::stm32f1::uart uart1(hal::port<1>, hal::buffer<128>);
  p_resources.console = &uart1;

  static hal::stm32f1::output_pin led('C', 13);
  p_resources.status_led = &led;

  static hal::stm32f1::input_pin input_pin('B', 4);
  p_resources.input_pin = &input_pin;

  static hal::atomic_spin_lock adc_lock;
  static hal::stm32f1::adc_peripheral_manager adc(
    hal::stm32f1::adc_peripheral_manager::adc_selection::adc1, adc_lock);
  static auto pb0 =
    adc.acquire_channel(hal::stm32f1::adc_peripheral_manager::pins::pb0);
  p_resources.adc = &pb0;

  static hal::stm32f1::output_pin sda_output_pin('B', 7);
  static hal::stm32f1::output_pin scl_output_pin('B', 6);
  static hal::bit_bang_i2c bit_bang_i2c(
    hal::bit_bang_i2c::pins{
      .sda = &sda_output_pin,
      .scl = &scl_output_pin,
    },
    steady_clock);
  p_resources.i2c = &bit_bang_i2c;

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

  hal::pwm16_channel* pwm_channel = nullptr;
  hal::pwm_group_manager* pwm_frequency = nullptr;

  if constexpr (use_libhal_4_pwm) {
    // Use old PWM
  } else {
    static hal::stm32f1::general_purpose_timer<hal::stm32f1::peripheral::timer2>
      timer;
    static auto timer_pwm_channel =
      timer.acquire_pwm16_channel(hal::stm32f1::timer2_pin::pa1);
    pwm_channel = &timer_pwm_channel;
    static auto timer1_pwm_frequency = timer.acquire_pwm_group_frequency();
    pwm_frequency = &timer1_pwm_frequency;
  }

  p_resources.pwm_channel = pwm_channel;
  p_resources.pwm_frequency = pwm_frequency;

  try {
    using namespace std::chrono_literals;
    static hal::stm32f1::can_peripheral_manager can(
      100_kHz, steady_clock, 1ms, hal::stm32f1::can_pins::pb9_pb8);

    // Self test allows the can transceiver to see its own messages as if they
    // were received on the bus. This also prevents messages from being received
    // from the bus. Set to `false` if you want to get actual CAN messages from
    // the bus and not the device's own messages.
    can.enable_self_test(true);

    static std::array<hal::can_message, 8> receive_buffer{};
    static auto can_transceiver = can.acquire_transceiver(receive_buffer);
    p_resources.can_transceiver = &can_transceiver;

    static auto can_bus_manager = can.acquire_bus_manager();
    p_resources.can_bus_manager = &can_bus_manager;

    static auto can_interrupt = can.acquire_interrupt();
    p_resources.can_interrupt = &can_interrupt;

    // Allow all messages
    static auto mask_id_filters_x2 = can.acquire_mask_filter();
    mask_id_filters_x2.filter[0].allow({ { .id = 0, .mask = 0 } });
  } catch (hal::timed_out&) {
    hal::print(
      uart1,
      "⚠️ CAN peripheral timeout error!\n"
      "- CAN disabled - check CANRX/CANTX connections to transceiver.\n"
      "- System will operate normally if CAN is NOT required.\n\n");
  }
}
