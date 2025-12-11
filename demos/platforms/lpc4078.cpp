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
#include <libhal-exceptions/control.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>

#include <resource_list.hpp>

namespace resources {
using namespace hal::literals;

std::pmr::polymorphic_allocator<> driver_allocator()
{
  static std::array<hal::byte, 1024> driver_memory{};
  static std::pmr::monotonic_buffer_resource resource(
    driver_memory.data(),
    driver_memory.size(),
    std::pmr::null_memory_resource());
  return &resource;
}

hal::v5::optional_ptr<hal::cortex_m::dwt_counter> clock_ptr;
hal::v5::strong_ptr<hal::steady_clock> clock()
{
  if (not clock_ptr) {
    clock_ptr = hal::v5::make_strong_ptr<hal::cortex_m::dwt_counter>(
      driver_allocator(),
      hal::lpc40::get_frequency(hal::lpc40::peripheral::cpu));
  }
  return clock_ptr;
}
hal::v5::strong_ptr<hal::adc> adc()
{
  return hal::v5::make_strong_ptr<hal::lpc40::adc>(
    resources::driver_allocator(), hal::channel<4>);
}
hal::v5::strong_ptr<hal::serial> console()
{
  static std::array<hal::byte, 64> uart0_buffer{};
  return hal::v5::make_strong_ptr<hal::lpc40::uart>(driver_allocator(),
                                                    0,
                                                    uart0_buffer,
                                                    hal::serial::settings{
                                                      .baud_rate = 115200,
                                                    });
}

hal::v5::optional_ptr<hal::lpc40::output_pin> led_ptr;
hal::v5::strong_ptr<hal::output_pin> status_led()
{
  if (not led_ptr) {
    led_ptr = hal::v5::make_strong_ptr<hal::lpc40::output_pin>(
      driver_allocator(), 1, 10);
  }
  return led_ptr;
}

hal::v5::strong_ptr<hal::i2c> i2c()
{
  return hal::v5::make_strong_ptr<hal::lpc40::i2c>(driver_allocator(),
                                                   2,
                                                   hal::i2c::settings{
                                                     .clock_rate = 100.0_kHz,
                                                   });
}

hal::v5::strong_ptr<hal::pwm> pwm()
{
  return hal::v5::make_strong_ptr<hal::lpc40::pwm>(
    resources::driver_allocator(), 1, 6);
}

hal::v5::strong_ptr<hal::can_transceiver> can_transceiver()
{
  throw hal::operation_not_supported(nullptr);
}

hal::v5::strong_ptr<hal::can_bus_manager> can_bus_manager()
{
  throw hal::operation_not_supported(nullptr);
}

hal::v5::strong_ptr<hal::can_identifier_filter> can_identifier_filter()
{
  throw hal::operation_not_supported(nullptr);
}

hal::v5::strong_ptr<hal::spi> spi()
{
  return hal::v5::make_strong_ptr<hal::lpc40::dma_spi>(driver_allocator(), 2);
}
hal::v5::strong_ptr<hal::output_pin> spi_chip_select()
{
  return hal::v5::make_strong_ptr<hal::lpc40::output_pin>(
    driver_allocator(), 1, 8);
}
hal::v5::strong_ptr<hal::stream_dac_u8> stream_dac()
{
  return hal::v5::make_strong_ptr<hal::lpc40::stream_dac_u8>(
    driver_allocator());
}
hal::v5::strong_ptr<hal::input_pin> input_pin()
{
  return hal::v5::make_strong_ptr<hal::lpc40::input_pin>(
    driver_allocator(), 0, 29);
}
hal::v5::strong_ptr<hal::interrupt_pin> interrupt_pin()
{
  return hal::v5::make_strong_ptr<hal::lpc40::interrupt_pin>(
    driver_allocator(), 0, 29);
}

hal::v5::strong_ptr<hal::timer> timed_interrupt()
{
  throw hal::operation_not_supported(nullptr);
}

hal::v5::strong_ptr<hal::pwm16_channel> pwm_channel()
{
  throw hal::operation_not_supported(nullptr);
}
hal::v5::strong_ptr<hal::pwm_group_manager> pwm_frequency()
{
  throw hal::operation_not_supported(nullptr);
}
hal::v5::strong_ptr<hal::dac> dac()
{
  return hal::v5::make_strong_ptr<hal::lpc40::dac>(driver_allocator());
}
class lpc4078_watchdog : public custom::watchdog
{
  void start() override
  {
    throw hal::operation_not_supported(nullptr);
  };
  void reset() override
  {
    throw hal::operation_not_supported(nullptr);
  }
  void set_countdown_time(hal::time_duration) override
  {
    throw hal::operation_not_supported(nullptr);
  }
  bool check_flag() override
  {
    throw hal::operation_not_supported(nullptr);
  }
  void clear_flag() override
  {
    throw hal::operation_not_supported(nullptr);
  }
};

hal::v5::strong_ptr<custom::watchdog> watchdog()
{
  return hal::v5::make_strong_ptr<lpc4078_watchdog>(driver_allocator());
}

[[noreturn]] void terminate_handler() noexcept
{
  if (not led_ptr && not clock_ptr) {
    // spin here until debugger is connected
    while (true) {
      continue;
    }
  }

  // Otherwise, blink the led in a pattern
  auto status_led = resources::status_led();
  auto clock = resources::clock();

  while (true) {
    using namespace std::chrono_literals;
    status_led->level(false);
    hal::delay(*clock, 100ms);
    status_led->level(true);
    hal::delay(*clock, 100ms);
    status_led->level(false);
    hal::delay(*clock, 100ms);
    status_led->level(true);
    hal::delay(*clock, 1000ms);
  }
}

hal::v5::strong_ptr<hal::v5::usb::control_endpoint> usb_control_endpoint()
{
  throw hal::operation_not_supported(nullptr);
}

hal::v5::strong_ptr<hal::v5::usb::interrupt_in_endpoint>
usb_interrupt_in_endpoint1()
{
  throw hal::operation_not_supported(nullptr);
}
hal::v5::strong_ptr<hal::v5::usb::interrupt_out_endpoint>
usb_interrupt_out_endpoint1()
{
  throw hal::operation_not_supported(nullptr);
}

hal::v5::strong_ptr<hal::v5::usb::interrupt_in_endpoint>
usb_interrupt_in_endpoint2()
{
  throw hal::operation_not_supported(nullptr);
}

hal::v5::strong_ptr<hal::v5::usb::interrupt_out_endpoint>
usb_interrupt_out_endpoint2()
{
  throw hal::operation_not_supported(nullptr);
}

hal::v5::strong_ptr<hal::v5::usb::bulk_in_endpoint> usb_bulk_in_endpoint1()
{
  throw hal::operation_not_supported(nullptr);
}

hal::v5::strong_ptr<hal::v5::usb::bulk_out_endpoint> usb_bulk_out_endpoint1()
{
  throw hal::operation_not_supported(nullptr);
}

hal::v5::strong_ptr<hal::v5::usb::bulk_in_endpoint> usb_bulk_in_endpoint2()
{
  throw hal::operation_not_supported(nullptr);
}

hal::v5::strong_ptr<hal::v5::usb::bulk_out_endpoint> usb_bulk_out_endpoint2()
{
  throw hal::operation_not_supported(nullptr);
}

}  // namespace resources

void initialize_platform()
{
  using namespace hal::literals;
  std::set_terminate(resources::terminate_handler);
  hal::lpc40::maximum(12.0_MHz);
}
