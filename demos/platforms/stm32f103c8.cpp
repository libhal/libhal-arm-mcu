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

#include <cassert>

#include <libhal-arm-mcu/dwt_counter.hpp>
#include <libhal-arm-mcu/startup.hpp>
#include <libhal-arm-mcu/stm32f1/adc.hpp>
#include <libhal-arm-mcu/stm32f1/can.hpp>
#include <libhal-arm-mcu/stm32f1/clock.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32f1/gpio.hpp>
#include <libhal-arm-mcu/stm32f1/input_pin.hpp>
#include <libhal-arm-mcu/stm32f1/output_pin.hpp>
#include <libhal-arm-mcu/stm32f1/spi.hpp>
#include <libhal-arm-mcu/stm32f1/timer.hpp>
#include <libhal-arm-mcu/stm32f1/uart.hpp>
#include <libhal-arm-mcu/stm32f1/usart.hpp>
#include <libhal-arm-mcu/system_control.hpp>
#include <libhal-util/atomic_spin_lock.hpp>
#include <libhal-util/bit_bang_i2c.hpp>
#include <libhal-util/bit_bang_spi.hpp>
#include <libhal-util/inert_drivers/inert_adc.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/lock.hpp>
#include <libhal/pwm.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>

#include <pointers.hpp>
#include <resource_list.hpp>
#include <type_traits>

constexpr bool use_bit_bang_spi = false;

hal::monotonic_allocator<2048> mem{};

void initialize_platform()
{
  // Set the MCU to the maximum clock speed
  hal::stm32f1::maximum_speed_using_internal_oscillator();
  hal::stm32f1::release_jtag_pins();
}

namespace resources {
using st_peripheral = hal::stm32f1::peripheral;

auto gpio_a()
{
  static auto port =
    hal::v5::make_strong_ptr<hal::stm32f1::gpio<st_peripheral::gpio_a>>(*mem);
  return port;
}

auto gpio_b()
{
  static auto port =
    hal::v5::make_strong_ptr<hal::stm32f1::gpio<st_peripheral::gpio_b>>(*mem);
  return port;
}

auto gpio_c()
{
  static auto port =
    hal::v5::make_strong_ptr<hal::stm32f1::gpio<st_peripheral::gpio_c>>(*mem);
  return port;
}

hal::v5::strong_ptr<hal::serial> console()
{
  if (not opt_console) {
    opt_console = hal::v5::make_strong_ptr<hal::stm32f1::uart>(
      *mem, hal::port<1>, hal::buffer<128>);
  }
  return opt_console.value();
}

hal::v5::strong_ptr<hal::zero_copy_serial> zero_copy_serial()
{
  auto usart2 =
    hal::v5::make_strong_ptr<hal::stm32f1::usart<st_peripheral::usart2>>(*mem);
  return hal::v5::make_strong_ptr(*mem,
                                  usart2->acquire_serial(hal::buffer<128>));
}

hal::v5::strong_ptr<hal::output_pin> status_led()
{
  if (not opt_status_led) {
    auto gpio = gpio_c();
    opt_status_led =
      hal::v5::make_strong_ptr(*mem, gpio->acquire_output_pin(13));
  }
  return opt_status_led.value();
}

hal::v5::strong_ptr<hal::steady_clock> uptime_clock()
{
  if (not opt_uptime_clock) {
    opt_uptime_clock = hal::v5::make_strong_ptr<hal::cortex_m::dwt_counter>(
      *mem, hal::stm32f1::frequency(hal::stm32f1::peripheral::cpu));
  }
  return opt_uptime_clock.value();
}

hal::v5::strong_ptr<hal::adc> adc()
{
  static auto adc_lock = hal::v5::make_strong_ptr<hal::atomic_spin_lock>(*mem);
  auto adc = hal::v5::make_strong_ptr<hal::stm32f1::adc<st_peripheral::adc1>>(
    *mem, &*adc_lock);
  return hal::v5::make_strong_ptr(
    *mem, adc->acquire_channel(hal::stm32f1::adc_pins::pb0));
}

hal::v5::strong_ptr<hal::input_pin> input_pin()
{
  return hal::v5::make_strong_ptr<hal::stm32f1::input_pin>(*mem, 'B', 4);
}

hal::v5::strong_ptr<hal::i2c> i2c()
{
  static auto i2c_sda =
    hal::v5::make_strong_ptr<hal::stm32f1::output_pin>(*mem, 'B', 7);
  static auto i2c_scl =
    hal::v5::make_strong_ptr<hal::stm32f1::output_pin>(*mem, 'B', 6);

  auto clock_ref = uptime_clock();

  return hal::v5::make_strong_ptr<hal::bit_bang_i2c>(
    *mem,
    hal::bit_bang_i2c::pins{
      // dangerous! Ensure object has static storage duration
      .sda = &(*i2c_sda),
      // dangerous! Ensure object has static storage duration
      .scl = &(*i2c_scl),
    },
    *clock_ref);
}

auto timer2()
{
  static auto timer = hal::v5::make_strong_ptr<
    hal::stm32f1::general_purpose_timer<hal::stm32f1::peripheral::timer2>>(
    *mem);
  return timer;
}

hal::v5::strong_ptr<hal::pwm16_channel> pwm_channel()
{
  auto timer = timer2();
  return hal::v5::make_strong_ptr(
    *mem, timer->acquire_pwm16_channel(hal::stm32f1::timer2_pin::pa1));
}

hal::v5::strong_ptr<hal::pwm_group_manager> pwm_frequency()
{
  auto timer = timer2();
  return hal::v5::make_strong_ptr(*mem, timer->acquire_pwm_group_frequency());
}

auto can_manager()
{
  using namespace hal::literals;
  using namespace std::chrono_literals;
  auto clock_ref = uptime_clock();
  static auto can =
    hal::v5::make_strong_ptr<hal::stm32f1::can_peripheral_manager>(
      *mem, 100_kHz, *clock_ref, 1ms, hal::stm32f1::can_pins::pb9_pb8);

  // Self test allows the can transceiver to see its own messages as if they
  // were received on the bus. This also prevents messages from being received
  // from the bus. Set to `false` if you want to get actual CAN messages from
  // the bus and not the device's own messages.
  can->enable_self_test(true);

  return can;
}

hal::v5::strong_ptr<hal::can_transceiver> can_transceiver()
{
  static std::array<hal::can_message, 8> receive_buffer{};
  auto can = can_manager();
  return hal::v5::make_strong_ptr(*mem,
                                  can->acquire_transceiver(receive_buffer));
}

hal::v5::strong_ptr<hal::can_mask_filter> can_mask_filter()
{
  // Does not need to be static because the aliasing constructor will extend the
  // lifetime of the object.
  auto can = can_manager();
  auto dual_mask_filter =
    hal::v5::make_strong_ptr(*mem, can->acquire_mask_filter());

  using dual_type = std::remove_reference_t<decltype(*dual_mask_filter)>;

  auto single_mask_filter =
    hal::v5::strong_ptr(dual_mask_filter, &dual_type::filter, 0);
  single_mask_filter->allow({ { .id = 0, .mask = 0 } });

  return single_mask_filter;
}

hal::v5::strong_ptr<hal::can_bus_manager> can_bus_manager()
{
  auto can = can_manager();
  return hal::v5::make_strong_ptr(*mem, can->acquire_bus_manager());
}

hal::v5::strong_ptr<hal::can_interrupt> can_interrupt()
{
  auto can = can_manager();
  return hal::v5::make_strong_ptr(*mem, can->acquire_interrupt());
}

hal::v5::strong_ptr<hal::spi> spi()
{
  using namespace hal::literals;

  if constexpr (use_bit_bang_spi) {
    static auto sck =
      hal::v5::make_strong_ptr<hal::stm32f1::output_pin>(*mem, 'A', 5);
    static auto copi =
      hal::v5::make_strong_ptr<hal::stm32f1::output_pin>(*mem, 'A', 6);
    static auto cipo =
      hal::v5::make_strong_ptr<hal::stm32f1::input_pin>(*mem, 'A', 7);

    return hal::v5::make_strong_ptr<hal::bit_bang_spi>(
      *mem,
      hal::bit_bang_spi::pins{
        // dangerous! Ensure object has static storage duration
        .sck = &*sck,
        // dangerous! Ensure object has static storage duration
        .copi = &*copi,
        // dangerous! Ensure object has static storage duration
        .cipo = &*cipo,
      },
      uptime_clock(),
      hal::spi::settings{
        .clock_rate = 250.0_kHz,
        .clock_polarity = false,
        .clock_phase = false,
      });
  } else {
    return hal::v5::make_strong_ptr<hal::stm32f1::spi>(
      *mem,
      hal::bus<1>,
      hal::spi::settings{
        .clock_rate = 250.0_kHz,
        .clock_polarity = false,
        .clock_phase = false,
      });
  }
}

hal::v5::strong_ptr<hal::output_pin> spi_chip_select()
{
  return hal::v5::make_strong_ptr<hal::stm32f1::output_pin>(*mem, 'A', 4);
}

hal::v5::strong_ptr<hal::stream_dac_u8> stream_dac()
{
  hal::safe_throw(hal::operation_not_supported(nullptr));
}

hal::v5::strong_ptr<hal::dac> dac()
{
  hal::safe_throw(hal::operation_not_supported(nullptr));
}

hal::v5::strong_ptr<hal::interrupt_pin> interrupt_pin()
{
  hal::safe_throw(hal::operation_not_supported(nullptr));
}

hal::v5::strong_ptr<hal::pwm> pwm()
{
  hal::safe_throw(hal::operation_not_supported(nullptr));
}
}  // namespace resources
