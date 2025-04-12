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

#include <memory>
#include <memory_resource>

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
#include <libhal/pwm.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>

#include <resource_list.hpp>
#include <type_traits>

constexpr bool use_bit_bang_spi = false;
constexpr bool use_libhal_4_pwm = false;

// Create a polymorphic allocator using the monotonic buffer resource
std::array<hal::byte, 2048> driver_memory{};
std::pmr::monotonic_buffer_resource resource(driver_memory.data(),
                                             driver_memory.size(),
                                             std::pmr::null_memory_resource());
std::pmr::polymorphic_allocator<hal::byte> driver_allocator(&resource);

namespace hal {
template<typename T, typename... Args>
std::shared_ptr<T> alloc(Args... p_args)
{
  return std::allocate_shared<T>(driver_allocator, p_args...);
}

template<typename T, typename... Args>
std::shared_ptr<T> alloc(T&& p_object)
{
  return std::allocate_shared<T>(driver_allocator, std::move(p_object));
}
}  // namespace hal

void initialize_platform(resource_list& p_resources)
{
  driver_memory.fill(0U);
  using namespace hal::literals;
  p_resources.reset = []() { hal::cortex_m::reset(); };

  // Set the MCU to the maximum clock speed
  hal::stm32f1::maximum_speed_using_internal_oscillator();
  hal::stm32f1::release_jtag_pins();

  using st_peripheral = hal::stm32f1::peripheral;

  p_resources.clock = hal::alloc<hal::cortex_m::dwt_counter>(
    hal::stm32f1::frequency(hal::stm32f1::peripheral::cpu));

  p_resources.console =
    hal::alloc<hal::stm32f1::uart>(hal::port<1>, hal::buffer<128>);

  auto usart2 = hal::alloc<hal::stm32f1::usart<st_peripheral::usart2>>();
  // TODO(kammce): Acquire serial should take an allocator and return a
  // shared_ptr.
  {
    auto object = usart2->acquire_serial(hal::buffer<128>);
    p_resources.zero_copy_serial = hal::alloc(std::move(object));
  }

  // ===========================================================================
  // Setup GPIO
  // ===========================================================================

  auto gpio_a = hal::alloc<hal::stm32f1::gpio<st_peripheral::gpio_a>>();
  auto gpio_b = hal::alloc<hal::stm32f1::gpio<st_peripheral::gpio_b>>();
  auto gpio_c = hal::alloc<hal::stm32f1::gpio<st_peripheral::gpio_c>>();
  p_resources.status_led = hal::alloc(gpio_c->acquire_output_pin(13));

  p_resources.input_pin = hal::alloc<hal::stm32f1::input_pin>('B', 4);

#if 0
  auto adc_lock = hal::alloc<hal::atomic_spin_lock>();
  auto adc = hal::alloc<hal::stm32f1::adc<st_peripheral::adc1>>(adc_lock);
  {
    auto object = adc->acquire_channel(hal::stm32f1::adc_pins::pb0);
    p_resources.adc = hal::alloc(std::move(object));
  }

  auto sda_output_pin = hal::alloc<hal::stm32f1::output_pin>('B', 7);
  auto scl_output_pin = hal::alloc<hal::stm32f1::output_pin>('B', 6);

  p_resources.i2c = hal::alloc<hal::bit_bang_i2c>(
    hal::bit_bang_i2c::pins{
      .sda = sda_output_pin.get(),  // TODO(kammce): dangerous!
      .scl = scl_output_pin.get(),  // TODO(kammce): dangerous!
    },
    p_resources.clock);

  p_resources.spi_chip_select = hal::alloc<hal::stm32f1::output_pin>('A', 4);

  if constexpr (use_bit_bang_spi) {
    auto sck = hal::alloc<hal::stm32f1::output_pin>('A', 5);
    auto copi = hal::alloc<hal::stm32f1::output_pin>('A', 6);
    auto cipo = hal::alloc<hal::stm32f1::input_pin>('A', 7);

    p_resources.spi = hal::alloc<hal::bit_bang_spi>(
      hal::bit_bang_spi::pins{
        .sck = sck.get(),    // TODO(kammce): dangerous!
        .copi = copi.get(),  // TODO(kammce): dangerous!
        .cipo = cipo.get(),  // TODO(kammce): dangerous!
      },
      p_resources.clock,
      hal::spi::settings{
        .clock_rate = 250.0_kHz,
        .clock_polarity = false,
        .clock_phase = false,
      });
  } else {
    p_resources.spi = hal::alloc<hal::stm32f1::spi>(hal::bus<1>,
                                                    hal::spi::settings{
                                                      .clock_rate = 250.0_kHz,
                                                      .clock_polarity = false,
                                                      .clock_phase = false,
                                                    });
  }

  if constexpr (use_libhal_4_pwm) {
    // Use old PWM
  } else {
    auto timer = hal::alloc<
      hal::stm32f1::general_purpose_timer<hal::stm32f1::peripheral::timer2>>();
    p_resources.pwm_channel =
      hal::alloc(timer->acquire_pwm16_channel(hal::stm32f1::timer2_pin::pa1));
    p_resources.pwm_frequency =
      hal::alloc(timer->acquire_pwm_group_frequency());
  }

  try {
    using namespace std::chrono_literals;
    auto can = alloc<hal::stm32f1::can_peripheral_manager>(
      100_kHz, p_resources.clock, 1ms, hal::stm32f1::can_pins::pb9_pb8);

    // Self test allows the can transceiver to see its own messages as if they
    // were received on the bus. This also prevents messages from being received
    // from the bus. Set to `false` if you want to get actual CAN messages from
    // the bus and not the device's own messages.
    can->enable_self_test(true);

    auto receive_buffer = std::make_shared<std::array<hal::can_message, 8>>();
    p_resources.can_transceiver =
      hal::alloc(can->acquire_transceiver(*receive_buffer.get()));

    p_resources.can_bus_manager = hal::alloc(can->acquire_bus_manager());

    p_resources.can_interrupt = hal::alloc(can->acquire_interrupt());

    // Allow all messages
    {
      auto object = hal::alloc(can->acquire_mask_filter());
      std::shared_ptr<std::remove_reference_t<decltype(object->filter[0])>>
        filter(object, &object->filter[0]);
      filter->allow({ { .id = 0, .mask = 0 } });
      p_resources.can_mask_filter = filter;
    }
  } catch (hal::timed_out&) {
    hal::print(
      *p_resources.console,
      "⚠️ CAN peripheral timeout error!\n"
      "- CAN disabled - check CANRX/CANTX connections to transceiver.\n"
      "- System will operate normally if CAN is NOT required.\n\n");
  }
#endif
}
