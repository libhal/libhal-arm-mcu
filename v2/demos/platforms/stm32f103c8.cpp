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
#include <coroutine>
#include <memory_resource>

module arm_mcu_demos;

import hal;
import hal.arm_mcu.stm32f1;
import hal.arm_mcu.cortex_m;

namespace resources {
// using namespace hal::literals;
using st_peripheral = hal::stm32f1::peripheral;

std::array<hal::byte, 2048> driver_memory{};
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

hal::ptr<hal::steady_clock> clock()
{
  static auto const cpu_frequency = hal::stm32f1::frequency(st_peripheral::cpu);
  static auto driver = hal::allocate<hal::cortex_m::dwt_counter>(
    driver_allocator(), cpu_frequency);
  return driver;
}

hal::ptr<hal::output_pin> status_led()
{
  static auto status_led_obj = hal::stm32f1::output_pin::create(
    driver_allocator(), { .port = 'C', .pin = 13 });
  return status_led_obj;
}

hal::ptr<hal::input_pin> input_pin()
{
  return hal::stm32f1::input_pin::create(driver_allocator(),
                                         { .port = 'B', .pin = 4 });
}

std::array<hal::byte, 128> uart_receive_buffer{};

hal::ptr<hal::serial> console()
{
  auto usart1 =
    hal::stm32f1::usart::create(driver_allocator(), st_peripheral::usart1);
  return usart1->acquire_serial(uart_receive_buffer, hal::serial::settings{});
}

hal::ptr<hal::adc16> adc()
{
  auto adc1 =
    hal::stm32f1::adc::create(driver_allocator(), st_peripheral::adc1);
  return adc1->acquire_channel(hal::stm32f1::adc_pins::pb0);
}

hal::opt_ptr<hal::stm32f1::can> can_manager;

hal::ptr<hal::stm32f1::can> can_peripheral()
{
  if (not can_manager) {
    can_manager = hal::stm32f1::can::create(
      driver_allocator(),
      100'000,
      { .pins = hal::stm32f1::can_pins::pb9_pb8, .enable_self_test = true });
  }
  return can_manager;
}

hal::ptr<hal::can_transceiver> can_transceiver()
{
  auto manager = can_peripheral();
  return manager->acquire_transceiver();
}

hal::ptr<hal::can_bus_manager> can_bus_manager()
{
  auto manager = can_peripheral();
  return manager->acquire_bus_manager();
}

hal::ptr<hal::can_id_filter> can_identifier_filter()
{
  auto manager = can_peripheral();
  auto filters = manager->acquire_identifier_filter();
  return filters[0];
}

hal::opt_ptr<hal::stm32f1::usb> usb_manager;

async::future<hal::ptr<hal::stm32f1::usb>> usb_peripheral(async::context& p_ctx)
{
  if (not usb_manager) {
    usb_manager = co_await hal::stm32f1::usb::create(p_ctx, driver_allocator());
  }
  co_return usb_manager;
}

async::future<hal::ptr<hal::usb::control_endpoint>> usb_control_endpoint(
  async::context& p_ctx)
{
  auto manager = co_await usb_peripheral(p_ctx);
  co_return manager->acquire_control_endpoint();
}

hal::opt_ptr<hal::usb::bulk_out_endpoint> bulk_out_ep1;
hal::opt_ptr<hal::usb::bulk_in_endpoint> bulk_in_ep1;

async::future<void> acquire_bulk_endpoint1(async::context& p_ctx)
{
  if (not bulk_out_ep1) {
    auto manager = co_await usb_peripheral(p_ctx);
    auto pair = manager->acquire_bulk_endpoint();
    bulk_out_ep1 = pair.out;
    bulk_in_ep1 = pair.in;
  }
  co_return;
}

async::future<hal::ptr<hal::usb::bulk_out_endpoint>> usb_bulk_out_endpoint1(
  async::context& p_ctx)
{
  co_await acquire_bulk_endpoint1(p_ctx);
  co_return bulk_out_ep1;
}

async::future<hal::ptr<hal::usb::bulk_in_endpoint>> usb_bulk_in_endpoint1(
  async::context& p_ctx)
{
  co_await acquire_bulk_endpoint1(p_ctx);
  co_return bulk_in_ep1;
}

hal::opt_ptr<hal::usb::interrupt_out_endpoint> interrupt_out_ep1;
hal::opt_ptr<hal::usb::interrupt_in_endpoint> interrupt_in_ep1;

async::future<void> acquire_interrupt_endpoint1(async::context& p_ctx)
{
  if (not interrupt_out_ep1) {
    auto manager = co_await usb_peripheral(p_ctx);
    auto pair = manager->acquire_interrupt_endpoint();
    interrupt_out_ep1 = pair.out;
    interrupt_in_ep1 = pair.in;
  }
  co_return;
}

async::future<hal::ptr<hal::usb::interrupt_out_endpoint>>
usb_interrupt_out_endpoint1(async::context& p_ctx)
{
  co_await acquire_interrupt_endpoint1(p_ctx);
  co_return interrupt_out_ep1;
}

async::future<hal::ptr<hal::usb::interrupt_in_endpoint>>
usb_interrupt_in_endpoint1(async::context& p_ctx)
{
  co_await acquire_interrupt_endpoint1(p_ctx);
  co_return interrupt_in_ep1;
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
}
