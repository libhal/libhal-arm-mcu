// Copyright 2026 Khalil Estell and the libhal contributors
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

module;

#include <array>

module arm_mcu_demos;

import hal;
import hal.arm_mcu.stm32f1;
import hal.arm_mcu.cortex_m;

namespace resources {
hal::static_ref<hal::steady_clock> clock()
{
  static hal::cortex_m::dwt_counter counter(
    hal::stm32f1::frequency(hal::stm32f1::peripheral::cpu));
  return counter;
}

hal::static_ref<hal::output_pin> status_led()
{
  static hal::stm32f1::output_pin led({ .port = 'C', .pin = 13 });
  return led;
}

hal::static_ref<hal::input_pin> input_pin()
{
  static hal::stm32f1::input_pin pin({ .port = 'B', .pin = 4 });
  return pin;
}

hal::static_ref<hal::serial> console()
{
  static hal::stm32f1::usart usart1(hal::stm32f1::peripheral::usart1);
  static std::array<hal::byte, 128> receive_buffer{};
  static auto serial = usart1.acquire_serial(receive_buffer);
  return serial;
}

hal::static_ref<hal::adc16> adc()
{
  static hal::no_op_lock single_context;
  static hal::stm32f1::adc adc1(
    hal::stm32f1::peripheral::adc1, *clock(), single_context);
  static auto channel = adc1.acquire_channel(hal::stm32f1::adc_pins::pb0);
  return channel;
}

hal::static_ref<hal::spi_channel> spi_channel()
{
  using namespace mp_units::si::unit_symbols;
  static hal::no_op_lock single_context;
  static hal::spi_bus_manager spi1(
    [] { return hal::stm32f1::spi(hal::stm32f1::peripheral::spi1); },
    single_context);
  static hal::stm32f1::output_pin chip_select({ .port = 'A', .pin = 4 });
  static auto channel =
    spi1.acquire_channel(chip_select, { .clock_rate = 1 * MHz });
  return channel;
}
}  // namespace resources

void initialize_platform()
{
  // std::set_terminate(resources::terminate_handler);
  hal::cortex_m::initialize_interrupts<hal::stm32f1::irq::max>();

  // Set the MCU to the maximum clock speed
#if 0
  hal::stm32f1::configure_clocks(hal::stm32f1::clock_tree{
    .high_speed_external = 8 * mp_units::si::unit_symbols::MHz,
    .pll = {
      .enable = true,
      .source = hal::stm32f1::pll_source::high_speed_external,
      .multiply = hal::stm32f1::pll_multiply::multiply_by_9,
      .usb = {
        .divider = hal::stm32f1::usb_divider::divide_by_1_point_5,
      }
    },
    .system_clock = hal::stm32f1::system_clock_select::pll,
    .ahb = {
      .divider = hal::stm32f1::ahb_divider::divide_by_1,
      .apb1 = {
        .divider = hal::stm32f1::apb_divider::divide_by_2,
      },
      .apb2 = {
        .divider = hal::stm32f1::apb_divider::divide_by_1,
        .adc = {
          .divider = hal::stm32f1::adc_divider::divide_by_6,
        }
      },
    },
  });
#endif

  hal::stm32f1::activate_mco_pa8(
    hal::stm32f1::mco_source::pll_clock_divided_by_2);

  hal::stm32f1::release_jtag_pins();

  // Blocking calls such as hal::sleep_for() sleep the CPU with WFI, woken by
  // SysTick. Constructed after the clocks are configured, since SysTick counts
  // CPU cycles. The waiter installs itself when constructed.
  static hal::cortex_m::systick_timer timer(
    hal::stm32f1::frequency(hal::stm32f1::peripheral::cpu));
  static hal::cortex_m::wfi_waiter waiter(timer);
}
