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

#include <libhal-arm-mcu/interrupt.hpp>
#include <libhal/input_pin.hpp>
#include <libhal/interrupt_pin.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/pointers.hpp>
#include <memory_resource>

#include "constants.hpp"
#include "pin.hpp"

namespace hal::stm32f1 {
/**
 * @brief Implementation of the GPIO port manager class
 *
 * This class has a private constructor and can only be used via its derived
 * class `gpio<peripheral>` class.
 *
 */
class gpio_manager : public hal::v5::enable_strong_from_this<gpio_manager>
{
public:
  template<peripheral select>
  friend class gpio;

  gpio_manager(gpio_manager&) = delete;
  gpio_manager& operator=(gpio_manager&) = delete;
  gpio_manager(gpio_manager&&) noexcept = default;
  gpio_manager& operator=(gpio_manager&&) noexcept = default;
  /**
   * @brief Destroy the gpio port manager object
   *
   * This actually does nothing as this driver cannot disable the GPIO port
   * peripherals if other pins are used within the application.
   */
  ~gpio_manager() = default;

  class input;
  class output;
  class interrupt;

  input acquire_input_pin(u8 p_pin, input_pin::settings const& p_settings = {});
  output acquire_output_pin(u8 p_pin,
                            output_pin::settings const& p_settings = {});
  interrupt acquire_interrupt_pin(
    u8 p_pin,
    interrupt_pin::settings const& p_settings = {});

  hal::v5::strong_ptr<input> input_pin(
    u8 p_pin,
    input_pin::settings const& p_settings = {});
  hal::v5::strong_ptr<output> output_pin(
    u8 p_pin,
    output_pin::settings const& p_settings = {});
  hal::v5::strong_ptr<interrupt> interrupt_pin(
    u8 p_pin,
    interrupt_pin::settings const& p_settings = {});

private:
  friend hal::v5::strong_ptr<hal::input_pin> acquire_input_pin(
    std::pmr::polymorphic_allocator<> p_allocator,
    hal::v5::strong_ptr<gpio_manager> p_manager,
    u8 p_pin,
    input_pin::settings const& p_settings);

  friend hal::v5::strong_ptr<hal::output_pin> acquire_output_pin(
    std::pmr::polymorphic_allocator<> p_allocator,
    hal::v5::strong_ptr<gpio_manager> p_manager,
    u8 p_pin,
    output_pin::settings const& p_settings);

  gpio_manager(peripheral p_select);
  peripheral m_port;
};

/**
 * @brief Gpio manager for the gpio ports A through to G.
 *
 * Use the acquire APIs in order to get input and output pins. If a pin is
 * already in use, the `hal::device_or_resource_busy` will be thrown.
 *
 * @tparam select - gpio peripheral port selection. Only peripheral::gpio_a to
 * peripheral::gpio_g.
 */
template<peripheral select>
class gpio final : public gpio_manager
{
public:
  static_assert(select == peripheral::gpio_a or /* line break */
                  select == peripheral::gpio_b or
                  select == peripheral::gpio_c or
                  select == peripheral::gpio_d or
                  select == peripheral::gpio_e or
                  select == peripheral::gpio_f or /* line break */
                  select == peripheral::gpio_g,
                "Only peripheral gpio_(a to g) is allowed for this class");
  gpio()
    : gpio_manager(select)
  {
  }
  ~gpio() = default;
};

class gpio_manager::input final : public hal::input_pin
{
public:
  template<peripheral port>
  input(gpio<port> const&, u8 p_pin, settings const& p_settings = {})
    : input(port, p_pin, p_settings)
  {
  }

private:
  friend class gpio_manager;

  friend hal::v5::strong_ptr<hal::input_pin> acquire_input_pin(
    std::pmr::polymorphic_allocator<> p_allocator,
    hal::v5::strong_ptr<gpio_manager> p_manager,
    u8 p_pin,
    input_pin::settings const& p_settings);

  input(peripheral p_port, u8 p_pin, settings const& p_settings);

  void driver_configure(settings const& p_settings) override;
  bool driver_level() override;

  pin_select m_pin;
};

class gpio_manager::output final : public hal::output_pin
{
public:
  template<peripheral port>
  output(gpio<port> const&, u8 p_pin, settings const& p_settings = {})
    : output(port, p_pin, p_settings)
  {
  }

private:
  friend class gpio_manager;

  friend hal::v5::strong_ptr<hal::output_pin> acquire_output_pin(
    std::pmr::polymorphic_allocator<> p_allocator,
    hal::v5::strong_ptr<gpio_manager> p_manager,
    u8 p_pin,
    output_pin::settings const& p_settings);

  output(peripheral p_port, u8 p_pin, settings const& p_settings);

  void driver_configure(settings const& p_settings) override;
  void driver_level(bool p_high) override;
  bool driver_level() override;

  pin_select m_pin;
};

class gpio_manager::interrupt final : public hal::interrupt_pin
{
public:
  template<peripheral port>
  interrupt(gpio<port> const&, u8 p_pin, settings const& p_settings = {})
    : interrupt(port, p_pin, p_settings)
  {
  }

private:
  friend class gpio_manager;

  interrupt(peripheral p_port, u8 p_pin, settings const& p_settings);

  void setup_interrupt();
  void driver_configure(settings const& p_settings) override;
  void driver_on_trigger(hal::callback<handler> p_callback) override;

  cortex_m::irq_t get_irq();

  pin_select m_pin;
};

/**
 * @brief Acquire an input pin from a gpio port
 *
 * Input pins returned from this API are not considered consumed by the manager.
 * Thus an exception representing an exhausted resource is not thrown. This
 * allows the caller to acquire the same pin multiple times if they wish,
 * without error. The only consequence is the generation of additional ref
 * counted control blocks for each instance. To conserve memory, it is better to
 * acquire the pin once and pass that around to multiple readers.
 *
 * @param p_allocator - allocator used to allocate the memory for the object
 * @param p_manager - the manager for the gpio port to acquire an input pin
 * from. For example, if you want pin A1, you will need the manager for port A
 * and pass the number `1` to parameter `p_pin`.
 * @param p_pin - The pin number to acquire from this port
 * @param p_settings - initial settings for the input pin.
 * @return hal::v5::strong_ptr<hal::input_pin> - input pin acquired from gpio
 * port.
 */
hal::v5::strong_ptr<hal::input_pin> acquire_input_pin(
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::v5::strong_ptr<gpio_manager> const& p_manager,
  u8 p_pin,
  input_pin::settings const& p_settings = {});

/**
 * @brief Acquire an output pin from a gpio port
 *
 * Output pins returned from this API are considered consumed by the manager.
 * This means that only one instance of an output pin from this gpio manager can
 * be acquired at a time. Attempting to create two instances of the same output
 * pin, having two writers to that peripheral, will result in the exception
 * `hal::device_or_resource_busy` being called with the instance value set to
 * the gpio_manager's address.
 *
 * @param p_allocator - allocator used to allocate the memory for the object
 * @param p_manager - the manager for the gpio port to acquire an output pin
 * from. For example, if you want pin A1, you will need the manager for port A
 * and pass the number `1` to parameter `p_pin`
 * @param p_pin - The pin number to acquire from this port
 * @param p_settings - initial settings for the output pin
 * @return hal::v5::strong_ptr<hal::output_pin> - output pin acquired from gpio
 * port.
 * @throw hal::device_or_resource_busy - if the output pin was already acquired
 * somewhere else in the program.
 */
hal::v5::strong_ptr<hal::output_pin> acquire_output_pin(
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::v5::strong_ptr<gpio_manager> const& p_manager,
  u8 p_pin,
  output_pin::settings const& p_settings = {});
}  // namespace hal::stm32f1

namespace hal {
using stm32f1::acquire_input_pin;
using stm32f1::acquire_output_pin;
}  // namespace hal
