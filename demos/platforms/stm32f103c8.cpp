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

#include <resource_list.hpp>

#include "smart_ref.hpp"

constexpr bool use_bit_bang_spi = false;

void initialize_platform()
{
  // Set the MCU to the maximum clock speed
  hal::stm32f1::maximum_speed_using_internal_oscillator();
  hal::stm32f1::release_jtag_pins();
}

namespace resources {
using st_peripheral = hal::stm32f1::peripheral;

void reset_device()
{
  hal::cortex_m::reset();
}

auto& driver_memory()
{
  auto& driver_memory = static_arena(hal::buffer<1024>);
  return driver_memory;
}

auto mem()
{
  return hal::make_alloc_helper(driver_memory().allocator());
}

auto gpio_a()
{
  static auto port = mem().alloc<hal::stm32f1::gpio<st_peripheral::gpio_a>>();
  return port;
}

auto gpio_b()
{
  static auto port = mem().alloc<hal::stm32f1::gpio<st_peripheral::gpio_b>>();
  return port;
}

auto gpio_c()
{
  static auto port = mem().alloc<hal::stm32f1::gpio<st_peripheral::gpio_c>>();
  return port;
}

hal::smart_ref<hal::serial> console()
{
  return mem().alloc<hal::stm32f1::uart>(hal::port<1>, hal::buffer<128>);
}

hal::smart_ref<hal::zero_copy_serial> zero_copy_serial()
{
  auto usart2 = mem().alloc<hal::stm32f1::usart<st_peripheral::usart2>>();
  return mem().alloc(usart2->acquire_serial(hal::buffer<128>));
}

hal::smart_ref<hal::output_pin> status_led()
{
  auto gpio = gpio_c();
  return mem().alloc(gpio->acquire_output_pin(13));
}

auto static_dwt_counter()
{
  static auto steady_clock = mem().alloc<hal::cortex_m::dwt_counter>(
    hal::stm32f1::frequency(hal::stm32f1::peripheral::cpu));
  return steady_clock;
}

hal::smart_ref<hal::steady_clock> uptime_clock()
{
  return mem().alloc<hal::cortex_m::dwt_counter>(
    hal::stm32f1::frequency(hal::stm32f1::peripheral::cpu));
}

hal::smart_ref<hal::adc> adc()
{
  static auto adc_lock = mem().alloc<hal::atomic_spin_lock>();
  auto adc = mem().alloc<hal::stm32f1::adc<st_peripheral::adc1>>(&*adc_lock);
  return mem().alloc(adc->acquire_channel(hal::stm32f1::adc_pins::pb0));
}

hal::smart_ref<hal::input_pin> input_pin()
{
  return mem().alloc<hal::stm32f1::input_pin>('B', 4);
}

hal::smart_ref<hal::i2c> i2c()
{
  static auto i2c_sda = mem().alloc<hal::stm32f1::output_pin>('B', 7);
  static auto i2c_scl = mem().alloc<hal::stm32f1::output_pin>('B', 6);

  auto clock_ref = uptime_clock();

  return mem().alloc<hal::bit_bang_i2c>(
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
  static auto timer = mem()
                        .alloc<hal::stm32f1::general_purpose_timer<
                          hal::stm32f1::peripheral::timer2>>();
  return timer;
}

hal::smart_ref<hal::pwm16_channel> pwm_channel()
{
  auto timer = timer2();
  return mem().alloc(
    timer->acquire_pwm16_channel(hal::stm32f1::timer2_pin::pa1));
}

hal::smart_ref<hal::pwm_group_manager> pwm_frequency()
{
  auto timer = timer2();
  return mem().alloc(timer->acquire_pwm_group_frequency());
}

auto can_manager()
{
  using namespace hal::literals;
  using namespace std::chrono_literals;
  auto clock_ref = uptime_clock();
  static auto can = mem().alloc<hal::stm32f1::can_peripheral_manager>(
    100_kHz, *clock_ref, 1ms, hal::stm32f1::can_pins::pb9_pb8);

  // Self test allows the can transceiver to see its own messages as if they
  // were received on the bus. This also prevents messages from being received
  // from the bus. Set to `false` if you want to get actual CAN messages from
  // the bus and not the device's own messages.
  can->enable_self_test(true);

  return can;
}

hal::smart_ref<hal::can_transceiver> can_transceiver()
{
  static std::array<hal::can_message, 8> receive_buffer{};
  auto can = can_manager();
  return mem().alloc(can->acquire_transceiver(receive_buffer));
}

hal::smart_ref<hal::can_mask_filter> can_mask_filter()
{
  // Does not need to be static because the aliasing constructor will extend the
  // lifetime of the object.
  auto can = can_manager();
  auto dual_mask_filter = mem().alloc(can->acquire_mask_filter());
  dual_mask_filter->filter[0].allow({ { .id = 0, .mask = 0 } });
  return { dual_mask_filter, &dual_mask_filter->filter[0] };
}

hal::smart_ref<hal::can_bus_manager> can_bus_manager()
{
  auto can = can_manager();
  return mem().alloc(can->acquire_bus_manager());
}

hal::smart_ref<hal::can_interrupt> can_interrupt()
{
  auto can = can_manager();
  return mem().alloc(can->acquire_interrupt());
}

hal::smart_ref<hal::spi> spi()
{
  using namespace hal::literals;

  if constexpr (use_bit_bang_spi) {
    static auto sck = mem().alloc<hal::stm32f1::output_pin>('A', 5);
    static auto copi = mem().alloc<hal::stm32f1::output_pin>('A', 6);
    static auto cipo = mem().alloc<hal::stm32f1::input_pin>('A', 7);

    return mem().alloc<hal::bit_bang_spi>(
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
    return mem().alloc<hal::stm32f1::spi>(hal::bus<1>,
                                          hal::spi::settings{
                                            .clock_rate = 250.0_kHz,
                                            .clock_polarity = false,
                                            .clock_phase = false,
                                          });
  }
}

hal::smart_ref<hal::output_pin> spi_chip_select()
{
  return mem().alloc<hal::stm32f1::output_pin>('A', 4);
}

hal::smart_ref<hal::stream_dac_u8> stream_dac()
{
  hal::safe_throw(hal::operation_not_supported(nullptr));
}

hal::smart_ref<hal::dac> dac()
{
  hal::safe_throw(hal::operation_not_supported(nullptr));
}

hal::smart_ref<hal::interrupt_pin> interrupt_pin()
{
  hal::safe_throw(hal::operation_not_supported(nullptr));
}

hal::smart_ref<hal::pwm> pwm()
{
  hal::safe_throw(hal::operation_not_supported(nullptr));
}
}  // namespace resources

// Override global new operator
void* operator new(std::size_t)
{
  throw std::bad_alloc();
}

// Override global new[] operator
void* operator new[](std::size_t)
{
  throw std::bad_alloc();
}

void* operator new(unsigned int, std::align_val_t)
{
  throw std::bad_alloc();
}

// Override global delete operator
void operator delete(void*) noexcept
{
}

// Override global delete[] operator
void operator delete[](void*) noexcept
{
}

// Optional: Override sized delete operators (C++14 and later)
void operator delete(void*, std::size_t) noexcept
{
}

void operator delete[](void*, std::size_t) noexcept
{
}

void operator delete[](void*, std::align_val_t) noexcept
{
}

void operator delete(void*, std::align_val_t) noexcept
{
}
