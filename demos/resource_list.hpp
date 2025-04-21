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

#include <memory>
#include <memory_resource>

#include <libhal/adc.hpp>
#include <libhal/can.hpp>
#include <libhal/dac.hpp>
#include <libhal/error.hpp>
#include <libhal/functional.hpp>
#include <libhal/i2c.hpp>
#include <libhal/input_pin.hpp>
#include <libhal/interrupt_pin.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/pwm.hpp>
#include <libhal/serial.hpp>
#include <libhal/spi.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/stream_dac.hpp>
#include <libhal/zero_copy_serial.hpp>

#include "intrusive_ptr.hpp"

namespace hal {

class ref_info
{
public:
  /// Takes a pointer to the object, cast it to the type it belongs to and
  /// destructs it and returns its size in bytes,.
  using destructor_t = usize(void const*);

  ref_info(std::pmr::polymorphic_allocator<hal::byte> p_allocator,
           destructor_t* p_destructor = nullptr)
    : m_allocator(p_allocator)
    , m_destructor(p_destructor)
  {
  }

protected:
  friend void intrusive_ptr_release(ref_info* p_info) noexcept;

  std::pmr::polymorphic_allocator<hal::byte> m_allocator;
  destructor_t* m_destructor = nullptr;
  std::atomic<int> m_count = 1;
};

constexpr auto ref_info_size = sizeof(ref_info);

template<class Derived>
class local_rc
  : public Derived
  , protected ref_info
{
public:
  template<typename... Args>
  explicit local_rc(std::pmr::polymorphic_allocator<hal::byte> p_alloc,
                    Args... p_args)
    : Derived(std::forward<Args>(p_args)...)
    , ref_info(
        p_alloc,
        +[](void const* p_object) {
          auto const* obj = static_cast<local_rc<Derived> const*>(p_object);
          obj->~local_rc<Derived>();
          return sizeof(local_rc<Derived>);
        })

  {
  }

  // Access to the wrapped object - might not be needed if inheritance is
  // sufficient
  [[nodiscard]] Derived* get()
  {
    return this;
  }
  [[nodiscard]] Derived const* get() const
  {
    return this;
  }

  // Add this template conversion operator to enable polymorphism
  template<class Base>
  operator boost::intrusive_ptr<local_rc<Base>>() const
  {
    // This will only compile if Derived is convertible to Base
    static_assert(std::is_convertible_v<Derived*, Base*>,
                  "Cannot convert local_rc<Derived> to local_rc<Base>");

    // Cast this to local_rc<Base>* and create a new intrusive_ptr
    local_rc<Base>* base_ptr = const_cast<local_rc<Derived>*>(this);
    // Increment reference count since we're creating a new intrusive_ptr
    intrusive_ptr_add_ref(this);
    return boost::intrusive_ptr<local_rc<Base>>(base_ptr);
  }

private:
  friend Derived;

  // Factory function - similar to allocate_shared
  template<typename T, typename... Args>
  friend boost::intrusive_ptr<local_rc<T>> allocate_intrusive(
    std::pmr::polymorphic_allocator<hal::byte> alloc,
    Args&&... args);

  template<typename Tp, typename... Args>
  friend constexpr auto construct_at(Tp*, Args&&...) noexcept;

  template<class Other>
  friend inline void intrusive_ptr_add_ref(local_rc<Other>* p) noexcept;
  template<class Other>
  friend inline void intrusive_ptr_release(local_rc<Other>* p) noexcept;
};

template<class Derived>
inline void intrusive_ptr_add_ref(local_rc<Derived>* rc) noexcept
{
  ++rc->m_count;
}

inline void intrusive_ptr_release(ref_info* p_info) noexcept
{
  if (p_info->m_count.fetch_sub(1) == 1) {
    // store this for later
    auto a = p_info->m_allocator;
    auto const size = p_info->m_destructor(p_info);
    a.deallocate_bytes(p_info, size);
  }
}

template<class Derived>
inline void intrusive_ptr_release(local_rc<Derived>* rc) noexcept
{
  intrusive_ptr_release(static_cast<ref_info*>(rc));
}

// Factory function - similar to allocate_shared
template<typename T, typename... Args>
boost::intrusive_ptr<local_rc<T>> allocate_intrusive(
  std::pmr::polymorphic_allocator<hal::byte> alloc,
  Args&&... args)
{
  // Allocate memory
  auto* ptr = alloc.allocate_object<local_rc<T>>();

  // Construct in-place
  std::construct_at(ptr, alloc, std::forward<Args...>(args)...);

  // Create and return the intrusive_ptr
  // Note: No need to call intrusive_ptr_add_ref as the initial count is 0
  return boost::intrusive_ptr<local_rc<T>>(ptr);
}

// =============================================================================
//  Extra stuff
// =============================================================================

inline int volatile global = 0;

class interface
{
public:
  virtual void foo() = 0;
  virtual ~interface() = default;

private:
  friend void intrusive_ptr_add_ref(interface* p_rc) noexcept;
  friend void intrusive_ptr_release(interface* p_rc) noexcept;

  std::atomic<int> m_atomic_count;
  std::pmr::polymorphic_allocator<hal::byte> m_allocator;
};

class impl : public interface
{
public:
  void foo() override
  {
    counter++;
  }

  ~impl() override
  {
    global = global + 1;
  }

  int counter = 0;
};
class impl2 : public interface
{
public:
  void foo() override
  {
    counter--;
  }

  ~impl2() override
  {
    global = global - 1;
  }

  int counter = 0;
};

inline void intrusive_ptr_add_ref(interface* p_rc) noexcept
{
  ++p_rc->m_atomic_count;
}

inline void intrusive_ptr_release(interface* p_rc) noexcept
{
  if (p_rc->m_atomic_count.fetch_sub(1) == 1) {
    // auto a = p_rc->m_allocator;
    p_rc->~interface();
  }
}
}  // namespace hal

struct resource_list
{
  hal::callback<void()> reset;
  // Each resource is made optional because some mcus do not support all
  // possible drivers in the resource list. If an application needs a driver it
  // will access them via `std::optional::value()` which will throw an exception
  // if the value, is not present. That exception will be caught in main and a
  // message will be printed if the `console` field has been set, then call
  // std::terminate.
  std::shared_ptr<hal::serial> console;
  std::shared_ptr<hal::zero_copy_serial> zero_copy_serial;
  std::shared_ptr<hal::output_pin> status_led;
  boost::intrusive_ptr<hal::local_rc<hal::steady_clock>> clock;
  boost::intrusive_ptr<hal::interface> interface;
  std::shared_ptr<hal::can_transceiver> can_transceiver;
  std::shared_ptr<hal::can_mask_filter> can_mask_filter;
  std::shared_ptr<hal::can_bus_manager> can_bus_manager;
  std::shared_ptr<hal::can_interrupt> can_interrupt;
  std::shared_ptr<hal::adc> adc;
  std::shared_ptr<hal::input_pin> input_pin;
  std::shared_ptr<hal::i2c> i2c;
  std::shared_ptr<hal::interrupt_pin> interrupt_pin;
  std::shared_ptr<hal::pwm> pwm;
  std::shared_ptr<hal::pwm16_channel> pwm_channel;
  std::shared_ptr<hal::pwm_group_manager> pwm_frequency;
  std::shared_ptr<hal::spi> spi;
  std::shared_ptr<hal::output_pin> spi_chip_select;
  std::shared_ptr<hal::stream_dac_u8> stream_dac;
  std::shared_ptr<hal::dac> dac;
};

namespace hal {
// class empty
}

template<class T>
T& resource_contract_assert(std::shared_ptr<T> p_object)
{
  if (not p_object) {
    hal::safe_throw(hal::unknown(nullptr));
  }

  return *p_object;
}

template<class T>
T& resource_contract_assert(boost::intrusive_ptr<T> p_object)
{
  return *p_object;
}

// Each application file should have this function implemented
[[gnu::noinline]]
void application(resource_list& p_map);

// Each platform file should have this function implemented
void initialize_platform(resource_list& p_resources);
