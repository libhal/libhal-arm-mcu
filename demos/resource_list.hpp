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

#pragma once

#include <memory_resource>

#include <libhal/adc.hpp>
#include <libhal/can.hpp>
#include <libhal/dac.hpp>
#include <libhal/error.hpp>
#include <libhal/functional.hpp>
#include <libhal/i2c.hpp>
#include <libhal/initializers.hpp>
#include <libhal/input_pin.hpp>
#include <libhal/interrupt_pin.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/pwm.hpp>
#include <libhal/serial.hpp>
#include <libhal/spi.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/stream_dac.hpp>
#include <libhal/units.hpp>
#include <libhal/zero_copy_serial.hpp>

#include "smart_ref.hpp"

namespace hal {
template<class T>
using optional_smart_ref = std::optional<hal::smart_ref<T>>;

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
auto& static_arena(hal::buffer_param auto p_buffer_size)
{
  static arena<p_buffer_size()> allocator_object{};
  return allocator_object;
}

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

namespace resources {
void reset_device();
hal::smart_ref<hal::serial> console();
hal::smart_ref<hal::zero_copy_serial> zero_copy_serial();
hal::smart_ref<hal::output_pin> status_led();
hal::smart_ref<hal::steady_clock> uptime_clock();
hal::smart_ref<hal::can_transceiver> can_transceiver();
hal::smart_ref<hal::can_mask_filter> can_mask_filter();
hal::smart_ref<hal::can_bus_manager> can_bus_manager();
hal::smart_ref<hal::can_interrupt> can_interrupt();
hal::smart_ref<hal::adc> adc();
hal::smart_ref<hal::input_pin> input_pin();
hal::smart_ref<hal::i2c> i2c();
hal::smart_ref<hal::interrupt_pin> interrupt_pin();
hal::smart_ref<hal::pwm> pwm();
hal::smart_ref<hal::pwm16_channel> pwm_channel();
hal::smart_ref<hal::pwm_group_manager> pwm_frequency();
hal::smart_ref<hal::spi> spi();
hal::smart_ref<hal::output_pin> spi_chip_select();
hal::smart_ref<hal::stream_dac_u8> stream_dac();
hal::smart_ref<hal::dac> dac();
}  // namespace resources

// Each application file should have this function implemented
void application();

// Each platform file should have this function implemented
void initialize_platform();
