// Copyright 2024 - 2025 Khalil Estelland the libhal contributors
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

#include <cstdint>

#include <libhal-arm-mcu/interrupt.hpp>
#include <libhal-arm-mcu/stm32_generic/i2c.hpp>
#include <libhal-arm-mcu/stm32f411/clock.hpp>
#include <libhal-arm-mcu/stm32f411/constants.hpp>
#include <libhal-arm-mcu/stm32f411/i2c.hpp>
#include <libhal-arm-mcu/stm32f411/interrupt.hpp>
#include <libhal-arm-mcu/stm32f411/output_pin.hpp>
#include <libhal-arm-mcu/stm32f411/pin.hpp>
#include <libhal-util/static_callable.hpp>
#include <libhal/i2c.hpp>
#include <libhal/units.hpp>

#include "libhal-arm-mcu/interrupt.hpp"
#include "power.hpp"

namespace hal::stm32f411 {
i2c_manager_impl::i2c_manager_impl(peripheral p_select)
  : m_port(p_select)
{
  power(m_port).on();
}

i2c_manager_impl::~i2c_manager_impl()
{
  power(m_port).off();
}
i2c_manager_impl::i2c i2c_manager_impl::acquire_i2c(
  hal::i2c::settings const& p_settings)
{
  return { *this, p_settings };
}

i2c_manager_impl::i2c::i2c(i2c_manager_impl& p_manager,
                           i2c::settings const& p_settings)
{
  auto const i2c1 = reinterpret_cast<void*>(0x4000'5400);
  auto const i2c2 = reinterpret_cast<void*>(0x4000'5800);
  auto const i2c3 = reinterpret_cast<void*>(0x4000'5C00);
  m_manager = &p_manager;
  switch (p_manager.m_port) {
    case peripheral::i2c1: {
      m_i2c = stm32_generic::i2c(i2c1);
      pin scl(peripheral::gpio_b, 6);
      pin sda(peripheral::gpio_b, 7);
      scl.function(pin::pin_function::alternate4)
        .open_drain(true)
        .resistor(pin_resistor::pull_up);
      sda.function(pin::pin_function::alternate4)
        .open_drain(true)
        .resistor(pin_resistor::pull_up);
    } break;
    case peripheral::i2c2: {
      m_i2c = stm32_generic::i2c(i2c2);
      pin scl(peripheral::gpio_b, 10);
      pin sda(peripheral::gpio_b, 11);
      scl.function(pin::pin_function::alternate4)
        .open_drain(true)
        .resistor(hal::pin_resistor::pull_up);
      sda.function(pin::pin_function::alternate4)
        .open_drain(true)
        .resistor(pin_resistor::pull_up);
    } break;
    case peripheral::i2c3: {
      m_i2c = stm32_generic::i2c(i2c3);
      pin scl(peripheral::gpio_a, 8);
      pin sda(peripheral::gpio_c, 9);
      scl.function(pin::pin_function::alternate4)
        .open_drain(true)
        .resistor(hal::pin_resistor::pull_up);
      sda.function(pin::pin_function::alternate4)
        .open_drain(true)
        .resistor(pin_resistor::pull_up);
    } break;
    default:
      hal::safe_throw(hal::operation_not_supported(this));
  }
  i2c::driver_configure(p_settings);
  stm32f411::initialize_interrupts();
  setup_interrupt();
};
void i2c_manager_impl::i2c::driver_transaction(
  hal::byte p_address,
  std::span<hal::byte const> p_data_out,
  std::span<hal::byte> p_data_in,
  hal::function_ref<hal::timeout_function> p_timeout)
{
  m_i2c.transaction(p_address, p_data_out, p_data_in, p_timeout);
}

i2c_manager_impl::i2c::~i2c()
{
  switch (m_manager->m_port) {
    case peripheral::i2c1:
      cortex_m::disable_interrupt(irq::i2c1_ev);
      cortex_m::disable_interrupt(irq::i2c1_er);
      break;
    case peripheral::i2c2:
      cortex_m::disable_interrupt(irq::i2c2_ev);
      cortex_m::disable_interrupt(irq::i2c2_er);
      break;
    case peripheral::i2c3:
      [[fallthrough]];
    default:
      cortex_m::disable_interrupt(irq::i2c3_ev);
      cortex_m::disable_interrupt(irq::i2c3_er);
      break;
  }
  power(m_manager->m_port).off();
}

void i2c_manager_impl::i2c::driver_configure(settings const& p_settings)
{
  m_i2c.configure(p_settings, frequency(m_manager->m_port));
}
void i2c_manager_impl::i2c::setup_interrupt()
{
  // Create a lambda to call the interrupt() method
  auto event_isr = [this]() { m_i2c.handle_i2c_event(); };
  auto error_isr = [this]() { m_i2c.handle_i2c_error(); };

  // A pointer to save the static_callable isr address to.
  cortex_m::interrupt_pointer event_handler;
  cortex_m::interrupt_pointer error_handler;

  switch (m_manager->m_port) {
    case peripheral::i2c1:
      event_handler =
        static_callable<i2c, 1, void(void)>(event_isr).get_handler();
      error_handler =
        static_callable<i2c, 2, void(void)>(error_isr).get_handler();
      // Enable interrupt service routine.
      cortex_m::enable_interrupt(irq::i2c1_ev, event_handler);
      cortex_m::enable_interrupt(irq::i2c1_er, error_handler);
      break;
    case peripheral::i2c2:
      event_handler =
        static_callable<i2c, 3, void(void)>(event_isr).get_handler();
      error_handler =
        static_callable<i2c, 4, void(void)>(error_isr).get_handler();
      // Enable interrupt service routine.
      cortex_m::enable_interrupt(irq::i2c2_ev, event_handler);
      cortex_m::enable_interrupt(irq::i2c2_er, error_handler);
      break;
    case peripheral::i2c3:
      [[fallthrough]];
    default:
      event_handler =
        static_callable<i2c, 5, void(void)>(event_isr).get_handler();
      error_handler =
        static_callable<i2c, 6, void(void)>(error_isr).get_handler();
      // Enable interrupt service routine.
      cortex_m::enable_interrupt(irq::i2c3_ev, event_handler);
      cortex_m::enable_interrupt(irq::i2c3_er, error_handler);
      break;
  }
}
}  // namespace hal::stm32f411
