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

#include <libhal/lock.hpp>
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

namespace hal {

template<usize buffer_size>
struct arena
{
  arena() = default;
  arena(arena&) = delete;
  arena& operator=(arena&) = delete;
  arena(arena&&) = default;
  arena& operator=(arena&&) = default;
  ~arena() = default;

  auto& allocator()
  {
    return m_allocator;
  }

  void release(hal::unsafe)
  {
    m_resource.release();
    m_driver_memory.fill(0);
  }

private:
  // Create a polymorphic allocator using the monotonic buffer resource
  std::array<hal::byte, buffer_size> m_driver_memory{};
  std::pmr::monotonic_buffer_resource m_resource{
    m_driver_memory.data(),
    m_driver_memory.size(),
    std::pmr::null_memory_resource()
  };
  std::pmr::polymorphic_allocator<hal::byte> m_allocator{ &m_resource };
};

template<class = decltype([]() {})>
auto& static_arena(buffer_param auto p_buffer_size)
{
  static arena<p_buffer_size()> allocator_object{};
  return allocator_object;
}

template<typename Allocator>
auto make_alloc_helper(Allocator& allocator)
{
  return [&allocator]<typename T, typename... Args>(Args&&... args) {
    return std::allocate_shared<T>(allocator, std::forward<Args>(args)...);
  };
}
}  // namespace hal

void initialize_platform(resource_list& p_resources)
{
  auto& driver_memory = static_arena(hal::buffer<1024>);
  driver_memory.release(hal::unsafe{});

  using namespace hal::literals;
  p_resources.reset = []() { hal::cortex_m::reset(); };

  // Set the MCU to the maximum clock speed
  hal::stm32f1::maximum_speed_using_internal_oscillator();
  hal::stm32f1::release_jtag_pins();

  auto alloc = hal::make_alloc_helper(driver_memory.allocator());

  using st_peripheral = hal::stm32f1::peripheral;
  p_resources.clock = alloc<hal::cortex_m::dwt_counter>(
    hal::stm32f1::frequency(hal::stm32f1::peripheral::cpu));

  p_resources.console = std::allocate_shared<hal::stm32f1::uart>(
    driver_memory.allocator(), hal::port<1>, hal::buffer<128>);

  auto usart2 =
    std::allocate_shared<hal::stm32f1::usart<st_peripheral::usart2>>(
      driver_memory.allocator());
  // TODO(kammce): Acquire serial should take an allocator and return a
  // shared_ptr.
#if 0
  p_resources.zero_copy_serial = std::allocate_shared(
    driver_memory.allocator(), usart2->acquire_serial(hal::buffer<128>));

  // ===========================================================================
  // Setup GPIO
  // ===========================================================================

  auto gpio_a =
    std::allocate_shared<hal::stm32f1::gpio<st_peripheral::gpio_a>>();
  auto gpio_b =
    std::allocate_shared<hal::stm32f1::gpio<st_peripheral::gpio_b>>();
  auto gpio_c =
    std::allocate_shared<hal::stm32f1::gpio<st_peripheral::gpio_c>>();
  p_resources.status_led = std::allocate_shared(gpio_c->acquire_output_pin(13));
  p_resources.input_pin = std::allocate_shared<hal::stm32f1::input_pin>('B', 4);

  auto adc_lock = std::allocate_shared<hal::atomic_spin_lock>();
  auto adc =
    std::allocate_shared<hal::stm32f1::adc<st_peripheral::adc1>>(adc_lock);
  p_resources.adc =
    std::allocate_shared(adc->acquire_channel(hal::stm32f1::adc_pins::pb0));

  static auto i2c_sda = std::allocate_shared<hal::stm32f1::output_pin>('B', 7);
  static auto i2c_scl = std::allocate_shared<hal::stm32f1::output_pin>('B', 6);

  p_resources.i2c = std::allocate_shared<hal::bit_bang_i2c>(
    hal::bit_bang_i2c::pins{
      // dangerous! Ensure object has static storage duration
      .sda = i2c_sda.get(),
      // dangerous! Ensure object has static storage duration
      .scl = i2c_scl.get(),
    },
    std::ref(*p_resources.clock));  // TODO(kammce): dangerous!

  p_resources.spi_chip_select =
    std::allocate_shared<hal::stm32f1::output_pin>('A', 4);

  if constexpr (use_bit_bang_spi) {
    static auto sck = std::allocate_shared<hal::stm32f1::output_pin>('A', 5);
    static auto copi = std::allocate_shared<hal::stm32f1::output_pin>('A', 6);
    static auto cipo = std::allocate_shared<hal::stm32f1::input_pin>('A', 7);

    p_resources.spi = std::allocate_shared<hal::bit_bang_spi>(
      hal::bit_bang_spi::pins{
        // dangerous! Ensure object has static storage duration
        .sck = sck.get(),
        // dangerous! Ensure object has static storage duration
        .copi = copi.get(),
        // dangerous! Ensure object has static storage duration
        .cipo = cipo.get(),
      },
      p_resources.clock,
      hal::spi::settings{
        .clock_rate = 250.0_kHz,
        .clock_polarity = false,
        .clock_phase = false,
      });
  } else {
    p_resources.spi =
      std::allocate_shared<hal::stm32f1::spi>(hal::bus<1>,
                                              hal::spi::settings{
                                                .clock_rate = 250.0_kHz,
                                                .clock_polarity = false,
                                                .clock_phase = false,
                                              });
  }

  if constexpr (use_libhal_4_pwm) {
    // Use old PWM
  } else {
    auto timer = std::allocate_shared<
      hal::stm32f1::general_purpose_timer<hal::stm32f1::peripheral::timer2>>();
    p_resources.pwm_channel = std::allocate_shared(
      timer->acquire_pwm16_channel(hal::stm32f1::timer2_pin::pa1));
    p_resources.pwm_frequency =
      std::allocate_shared(timer->acquire_pwm_group_frequency());
  }

  try {
    using namespace std::chrono_literals;
    auto can = alloc<hal::stm32f1::can_peripheral_manager>(
      100_kHz,
      std::ref(*p_resources.clock),
      1ms,
      hal::stm32f1::can_pins::pb9_pb8);

    // Self test allows the can transceiver to see its own messages as if they
    // were received on the bus. This also prevents messages from being received
    // from the bus. Set to `false` if you want to get actual CAN messages from
    // the bus and not the device's own messages.
    can->enable_self_test(true);

    static std::array<hal::can_message, 8> receive_buffer{};
    // dangerous! Ensure object has static storage duration
    p_resources.can_transceiver =
      std::allocate_shared(can->acquire_transceiver(receive_buffer));
    p_resources.can_bus_manager =
      std::allocate_shared(can->acquire_bus_manager());
    p_resources.can_interrupt = std::allocate_shared(can->acquire_interrupt());

    // Allow all messages
    {
      auto dual_mask_filter = std::allocate_shared(can->acquire_mask_filter());
      using can_mask_filter =
        std::remove_reference_t<decltype(dual_mask_filter->filter[0])>;
      std::shared_ptr<can_mask_filter> filter(dual_mask_filter,
                                              &dual_mask_filter->filter[0]);
      p_resources.can_mask_filter = filter;
      p_resources.can_mask_filter->allow({ { .id = 0, .mask = 0 } });
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
