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

#include <memory_resource>

module arm_mcu_demos;

import hal;
import hal.arm_mcu.stm32f1;

namespace resources {
// using namespace hal::literals;
using st_peripheral = hal::stm32f1::peripheral;

std::array<hal::byte, 512> driver_memory{};
std::pmr::monotonic_buffer_resource resource(driver_memory.data(),
                                             driver_memory.size(),
                                             std::pmr::null_memory_resource());

hal::allocator driver_allocator()
{
  return &resource;
}

hal::ptr<hal::timed_interrupt> timer()
{
  auto const cpu_frequency = hal::stm32f1::frequency(st_peripheral::cpu);
  return hal::allocate<hal::cortex_m::systick_timer>(driver_allocator(),
                                                     cpu_frequency);
}

hal::ptr<hal::output_pin> status_led()
{
  return hal::stm32f1::output_pin::create(driver_allocator(),
                                          { .port = 'C', .pin = 13 });
}

hal::ptr<hal::input_pin> input_pin()
{
  return hal::stm32f1::input_pin::create(driver_allocator(),
                                         { .port = 'B', .pin = 4 });
}
}  // namespace resources

void initialize_platform()
{
  // std::set_terminate(resources::terminate_handler);
  hal::cortex_m::initialize_interrupts<hal::stm32f1::irq::max>();

  // Set the MCU to the maximum clock speed
#if 0
  hal::stm32f1::configure_clocks(hal::stm32f1::clock_tree{
    .high_speed_external = 8 * MHz,
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
}
