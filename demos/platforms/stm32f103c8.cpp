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
#include <cstddef>
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
#include <libhal/lock.hpp>
#include <libhal/pwm.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>

#include <resource_list.hpp>

#include "smart_ref.hpp"

constexpr bool use_bit_bang_spi = false;
constexpr bool use_libhal_4_pwm = false;

namespace hal {

class monotonic_resource : public std::pmr::memory_resource
{
private:
  void* m_buffer = nullptr;
  size_t m_buffer_size = 0;
  size_t m_current_offset = 0;

  // Align the given size up to the specified alignment
  static size_t align_up(size_t size, size_t alignment)
  {
    return (size + alignment - 1) & ~(alignment - 1);
  }

public:
  // Constructor that takes a pre-allocated buffer and its size
  monotonic_resource(void* buffer, size_t buffer_size) noexcept
    : m_buffer(buffer)
    , m_buffer_size(buffer_size)
  {
  }

  // Reset the resource to its initial state
  void reset() noexcept
  {
    m_current_offset = 0;
  }

private:
  // Implement the required functions from std::pmr::memory_resource

  // NOLINTNEXTLINE
  void* do_allocate(size_t bytes, size_t alignment) override
  {
    // Align the current offset to satisfy the requested alignment
    size_t aligned_offset = align_up(m_current_offset, alignment);

    // Check if we have enough space
    if (aligned_offset + bytes > m_buffer_size) {
      throw std::bad_alloc();
    }

    // Update the current offset
    void* result = static_cast<char*>(m_buffer) + aligned_offset;
    m_current_offset = aligned_offset + bytes;

    return result;
  }

  void do_deallocate(void*, size_t, size_t) override
  {
    // In a monotonic resource, deallocate is a no-op
    // Memory is only reclaimed when reset() is called
  }

  [[nodiscard]] bool do_is_equal(
    std::pmr::memory_resource const& other) const noexcept override
  {
    // Resources are equal if they are the same object
    return this == &other;
  }
};

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
    m_driver_memory.fill(0);
  }

private:
  // Create a polymorphic allocator using the monotonic buffer resource
  std::array<hal::byte, buffer_size> m_driver_memory{};
  monotonic_resource m_resource{ m_driver_memory.data(),
                                 m_driver_memory.size() };
  std::pmr::polymorphic_allocator<hal::byte> m_allocator{ &m_resource };
};

template<class = decltype([]() {})>
auto& static_arena(buffer_param auto p_buffer_size)
{
  static arena<p_buffer_size()> allocator_object{};
  return allocator_object;
}

auto& driver_memory = static_arena(hal::buffer<1024>);

template<class allocator>
class allocator_helper
{
public:
  allocator_helper(allocator_helper&) = delete;
  allocator_helper& operator=(allocator_helper&) = delete;
  allocator_helper(allocator_helper&&) = delete;
  allocator_helper& operator=(allocator_helper&&) = delete;

  template<typename T, typename... Args>
  hal::smart_ref<T> alloc(Args&&... p_args)
  {
    return hal::make_shared_ref<T>(m_allocator, std::forward<Args>(p_args)...);
  }

  template<typename T>
  hal::smart_ref<T> alloc(T&& p_object)
  {
    return hal::make_shared_ref<T>(m_allocator, std::forward<T>(p_object));
  }

  auto inner_allocator()
  {
    return m_allocator;
  }

  auto& operator*() noexcept
  {
    return m_allocator;
  }

  auto* operator->() noexcept
  {
    return &m_allocator;
  }

private:
  template<typename Allocator>
  friend auto make_alloc_helper(Allocator& p_allocator);

  allocator_helper(allocator& p_allocator)
    : m_allocator(p_allocator)
  {
  }

  allocator& m_allocator;
};

template<typename Allocator>
auto make_alloc_helper(Allocator& p_allocator)
{
  return allocator_helper{ p_allocator };
}
}  // namespace hal

hal::u64 initial_uptime = 0;

void initialize_platform(resource_list& p_resources)
{
  hal::driver_memory.release(hal::unsafe{});

  using namespace hal::literals;
  p_resources.reset = []() { hal::cortex_m::reset(); };

  auto mem = hal::make_alloc_helper(hal::driver_memory.allocator());

  // Set the MCU to the maximum clock speed
  hal::stm32f1::maximum_speed_using_internal_oscillator();
  hal::stm32f1::release_jtag_pins();

  using st_peripheral = hal::stm32f1::peripheral;

  p_resources.clock = hal::make_shared_ref<hal::cortex_m::dwt_counter>(
    *mem, hal::stm32f1::frequency(hal::stm32f1::peripheral::cpu));

  p_resources.console =
    mem.alloc<hal::stm32f1::uart>(hal::port<1>, hal::buffer<128>);

  auto usart2 = mem.alloc<hal::stm32f1::usart<st_peripheral::usart2>>();
  // TODO(kammce): Acquire serial should take an allocator and return a
  // shared_ptr.

  p_resources.zero_copy_serial =
    mem.alloc(usart2->acquire_serial(hal::buffer<128>));

  // ===========================================================================
  // Setup GPIO
  // ===========================================================================

  auto gpio_a = mem.alloc<hal::stm32f1::gpio<st_peripheral::gpio_a>>();
  auto gpio_b = mem.alloc<hal::stm32f1::gpio<st_peripheral::gpio_b>>();
  auto gpio_c = mem.alloc<hal::stm32f1::gpio<st_peripheral::gpio_c>>();
  p_resources.status_led = mem.alloc(gpio_c->acquire_output_pin(13));
  p_resources.input_pin = mem.alloc<hal::stm32f1::input_pin>('B', 4);

  static auto adc_lock = mem.alloc<hal::atomic_spin_lock>();
  auto adc = mem.alloc<hal::stm32f1::adc<st_peripheral::adc1>>(&*adc_lock);
  p_resources.adc =
    mem.alloc(adc->acquire_channel(hal::stm32f1::adc_pins::pb0));

  static auto i2c_sda = mem.alloc<hal::stm32f1::output_pin>('B', 7);
  static auto i2c_scl = mem.alloc<hal::stm32f1::output_pin>('B', 6);

  p_resources.i2c = mem.alloc<hal::bit_bang_i2c>(
    hal::bit_bang_i2c::pins{
      // dangerous! Ensure object has static storage duration
      .sda = &(*i2c_sda),
      // dangerous! Ensure object has static storage duration
      .scl = &(*i2c_scl),
    },
    **p_resources.clock);  // TODO(kammce): dangerous!

  p_resources.spi_chip_select = mem.alloc<hal::stm32f1::output_pin>('A', 4);

  if constexpr (use_bit_bang_spi) {

    static auto sck = mem.alloc<hal::stm32f1::output_pin>('A', 5);
    static auto copi = mem.alloc<hal::stm32f1::output_pin>('A', 6);
    static auto cipo = mem.alloc<hal::stm32f1::input_pin>('A', 7);

    p_resources.spi = mem.alloc<hal::bit_bang_spi>(
      hal::bit_bang_spi::pins{
        // dangerous! Ensure object has static storage duration
        .sck = &*sck,
        // dangerous! Ensure object has static storage duration
        .copi = &*copi,
        // dangerous! Ensure object has static storage duration
        .cipo = &*cipo,
      },
      p_resources.clock,
      hal::spi::settings{
        .clock_rate = 250.0_kHz,
        .clock_polarity = false,
        .clock_phase = false,
      });
  } else {
    p_resources.spi = mem.alloc<hal::stm32f1::spi>(hal::bus<1>,
                                                   hal::spi::settings{
                                                     .clock_rate = 250.0_kHz,
                                                     .clock_polarity = false,
                                                     .clock_phase = false,
                                                   });
  }

  if constexpr (use_libhal_4_pwm) {
    // Use old PWM
  } else {
    auto timer = mem.alloc<
      hal::stm32f1::general_purpose_timer<hal::stm32f1::peripheral::timer2>>();
    p_resources.pwm_channel =
      mem.alloc(timer->acquire_pwm16_channel(hal::stm32f1::timer2_pin::pa1));
    p_resources.pwm_frequency = mem.alloc(timer->acquire_pwm_group_frequency());
  }

  try {
    using namespace std::chrono_literals;
    auto can = mem.alloc<hal::stm32f1::can_peripheral_manager>(
      100_kHz, **p_resources.clock, 1ms, hal::stm32f1::can_pins::pb9_pb8);

    // Self test allows the can transceiver to see its own messages as if they
    // were received on the bus. This also prevents messages from being received
    // from the bus. Set to `false` if you want to get actual CAN messages from
    // the bus and not the device's own messages.
    can->enable_self_test(true);

    static std::array<hal::can_message, 8> receive_buffer{};
    // dangerous! Ensure object has static storage duration
    p_resources.can_transceiver =
      mem.alloc(can->acquire_transceiver(receive_buffer));
    p_resources.can_bus_manager = mem.alloc(can->acquire_bus_manager());
    p_resources.can_interrupt = mem.alloc(can->acquire_interrupt());

    // Allow all messages
    {
      auto dual_mask_filter = mem.alloc(can->acquire_mask_filter());
      // std::shared_ptr<hal::can_mask_filter> filter(
      //   dual_mask_filter, &dual_mask_filter->filter[0]);
      // p_resources.can_mask_filter = filter;
      // p_resources.can_mask_filter->allow({ { .id = 0, .mask = 0 } });
    }
  } catch (hal::timed_out&) {
    hal::print(
      **p_resources.console,
      "⚠️ CAN peripheral timeout error!\n"
      "- CAN disabled - check CANRX/CANTX connections to transceiver.\n"
      "- System will operate normally if CAN is NOT required.\n\n");
  }
}
