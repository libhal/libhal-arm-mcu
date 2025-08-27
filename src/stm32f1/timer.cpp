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

#include <libhal-arm-mcu/interrupt.hpp>
#include <libhal-arm-mcu/stm32_generic/pwm.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32f1/interrupt.hpp>
#include <libhal-arm-mcu/stm32f1/pwm.hpp>
#include <libhal-arm-mcu/stm32f1/timer.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/enum.hpp>
#include <libhal-util/static_callable.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>
#include <libhal/rotation_sensor.hpp>
#include <libhal/units.hpp>
#include <memory_resource>

#include "../stm32_generic/timer.hpp"
#include "power.hpp"
#include "quadrature_encoder.hpp"

namespace hal::stm32f1 {

// Advanced timer
inline void* timer1 = reinterpret_cast<void*>(0x4001'2C00);
// General purpose timers 2 - 5
inline void* timer2 = reinterpret_cast<void*>(0x4000'0000);
inline void* timer3 = reinterpret_cast<void*>(0x4000'0400);
inline void* timer4 = reinterpret_cast<void*>(0x4000'0800);
inline void* timer5 = reinterpret_cast<void*>(0x4000'0C00);
// Advanced timer
inline void* timer8 = reinterpret_cast<void*>(0x4001'3400);
// General purpose timers 9 - 14
inline void* timer9 = reinterpret_cast<void*>(0x4001'4C00);
inline void* timer10 = reinterpret_cast<void*>(0x4001'5000);
inline void* timer11 = reinterpret_cast<void*>(0x4001'5400);
inline void* timer12 = reinterpret_cast<void*>(0x4000'1800);
inline void* timer13 = reinterpret_cast<void*>(0x4000'1C00);
inline void* timer14 = reinterpret_cast<void*>(0x4000'2000);

namespace {
void* peripheral_to_advanced_register(peripheral p_id)
{
  void* reg;
  if (p_id == peripheral::timer1) {
    reg = timer1;
  } else {
    reg = timer8;
  }
  return reg;
}

void* peripheral_to_general_register(peripheral p_id)
{
  void* reg;
  if (p_id == peripheral::timer2) {
    reg = timer2;
  } else if (p_id == peripheral::timer3) {
    reg = timer3;
  } else if (p_id == peripheral::timer4) {
    reg = timer4;
  } else if (p_id == peripheral::timer5) {
    reg = timer5;
  } else if (p_id == peripheral::timer9) {
    reg = timer9;
  } else if (p_id == peripheral::timer10) {
    reg = timer10;
  } else if (p_id == peripheral::timer11) {
    reg = timer11;
  } else if (p_id == peripheral::timer12) {
    reg = timer12;
  } else if (p_id == peripheral::timer13) {
    reg = timer13;
  } else {
    reg = timer14;
  }
  return reg;
}
}  // namespace

timer::timer(void* p_reg, timer_manager_data* p_manager_data_ptr)
  : m_timer(unsafe{})
  , m_manager_data_ptr(p_manager_data_ptr)
{
  // Captures the needed stm32f1 series interrupt data to be passed
  auto peripheral_interrupt_params = setup_interrupt();

  // Passes the stm32f1 series data to the generic stm32 timer object
  m_timer.initialize(unsafe{},
                     p_reg,
                     &stm32f1::initialize_interrupts,
                     peripheral_interrupt_params.irq,
                     peripheral_interrupt_params.handler);

  m_manager_data_ptr->m_usage = timer_manager_data::usage::callback_timer;
}

timer::~timer()
{
  m_manager_data_ptr->m_usage = timer_manager_data::usage::uninitialized;
  reset_peripheral(m_manager_data_ptr->m_id);
}

bool timer::driver_is_running()
{
  return m_timer.is_running();
}

void timer::driver_cancel()
{
  m_timer.cancel();
}

void timer::driver_schedule(hal::callback<void(void)> p_callback,
                            hal::time_duration p_delay)
{
  m_callback = p_callback;
  auto const timer_frequency = frequency(m_manager_data_ptr->m_id);
  m_timer.schedule(p_delay, static_cast<u32>(timer_frequency));
}

timer::interrupt_params timer::setup_interrupt()
{
  // Create a lambda to call the interrupt() method
  auto isr = [this]() { interrupt(); };

  // A pointer to save the static_callable isr address to
  interrupt_params peripheral_interrupt_params;

  // Determines IRQ and handler to use
  switch (m_manager_data_ptr->m_id) {
    case peripheral::timer1:
      peripheral_interrupt_params.irq =
        static_cast<cortex_m::irq_t>(irq::tim1_up);
      peripheral_interrupt_params.handler =
        static_callable<timer, 0, void(void)>(isr).get_handler();
      break;
    case peripheral::timer2:
      peripheral_interrupt_params.irq = static_cast<cortex_m::irq_t>(irq::tim2);
      peripheral_interrupt_params.handler =
        static_callable<timer, 1, void(void)>(isr).get_handler();
      break;
    case peripheral::timer3:
      peripheral_interrupt_params.irq = static_cast<cortex_m::irq_t>(irq::tim3);
      peripheral_interrupt_params.handler =
        static_callable<timer, 2, void(void)>(isr).get_handler();
      break;
    case peripheral::timer4:
      peripheral_interrupt_params.irq = static_cast<cortex_m::irq_t>(irq::tim4);
      peripheral_interrupt_params.handler =
        static_callable<timer, 3, void(void)>(isr).get_handler();
      break;
    case peripheral::timer5:
      peripheral_interrupt_params.irq = static_cast<cortex_m::irq_t>(irq::tim5);
      peripheral_interrupt_params.handler =
        static_callable<timer, 4, void(void)>(isr).get_handler();
      break;
    case peripheral::timer6:
      peripheral_interrupt_params.irq = static_cast<cortex_m::irq_t>(irq::tim6);
      peripheral_interrupt_params.handler =
        static_callable<timer, 5, void(void)>(isr).get_handler();
      break;
    case peripheral::timer7:
      peripheral_interrupt_params.irq = static_cast<cortex_m::irq_t>(irq::tim7);
      peripheral_interrupt_params.handler =
        static_callable<timer, 6, void(void)>(isr).get_handler();
      break;
    case peripheral::timer8:
      peripheral_interrupt_params.irq =
        static_cast<cortex_m::irq_t>(irq::tim8_up);
      peripheral_interrupt_params.handler =
        static_callable<timer, 7, void(void)>(isr).get_handler();
      break;
    case peripheral::timer9:
      peripheral_interrupt_params.irq =
        static_cast<cortex_m::irq_t>(irq::tim1_brk_tim9);
      peripheral_interrupt_params.handler =
        static_callable<timer, 8, void(void)>(isr).get_handler();
      break;
    case peripheral::timer10:  // uses timer 1's interrupt vector
      peripheral_interrupt_params.irq =
        static_cast<cortex_m::irq_t>(irq::tim1_up_tim10);
      peripheral_interrupt_params.handler =
        static_callable<timer, 9, void(void)>(isr).get_handler();
      break;
    case peripheral::timer11:
      peripheral_interrupt_params.irq =
        static_cast<cortex_m::irq_t>(irq::tim1_trg_com_tim11);
      peripheral_interrupt_params.handler =
        static_callable<timer, 10, void(void)>(isr).get_handler();
      break;
    case peripheral::timer12:
      peripheral_interrupt_params.irq =
        static_cast<cortex_m::irq_t>(irq::tim8_brk_tim12);
      peripheral_interrupt_params.handler =
        static_callable<timer, 11, void(void)>(isr).get_handler();
      break;
    case peripheral::timer13:  // uses timer 8's interrupt vector
      peripheral_interrupt_params.irq =
        static_cast<cortex_m::irq_t>(irq::tim8_up_tim13);
      peripheral_interrupt_params.handler =
        static_callable<timer, 12, void(void)>(isr).get_handler();
      break;
    case peripheral::timer14:
      [[fallthrough]];
    default:
      peripheral_interrupt_params.irq =
        static_cast<cortex_m::irq_t>(irq::tim8_trg_com_tim14);
      peripheral_interrupt_params.handler =
        static_callable<timer, 13, void(void)>(isr).get_handler();
      break;
  }
  return peripheral_interrupt_params;
}

void timer::handle_interrupt()
{
  void* reg = nullptr;
  if (m_manager_data_ptr->m_id == peripheral::timer1 ||
      m_manager_data_ptr->m_id == peripheral::timer8) {
    reg = peripheral_to_advanced_register(m_manager_data_ptr->m_id);
  } else {
    reg = peripheral_to_general_register(m_manager_data_ptr->m_id);
  }

  static auto timer_reg = stm32_generic::get_timer_reg(reg);

  static constexpr auto update_interrupt_flag = hal::bit_mask::from(0);
  bit_modify(timer_reg->status_register).clear(update_interrupt_flag);
}

void timer::interrupt()
{
  if (m_callback) {
    (*m_callback)();
    timer::handle_interrupt();
  }
}

advanced_timer_manager::advanced_timer_manager(peripheral p_id)
  : m_manager_data(p_id)
{
  power_on(m_manager_data.m_id);
}

general_purpose_timer_manager::general_purpose_timer_manager(peripheral p_id)
  : m_manager_data(p_id)
{
  power_on(m_manager_data.m_id);
}

advanced_timer_manager::~advanced_timer_manager()
{
  power_off(m_manager_data.m_id);
}

general_purpose_timer_manager::~general_purpose_timer_manager()
{
  power_off(m_manager_data.m_id);
}

hal::stm32f1::timer advanced_timer_manager::acquire_timer()
{
  if (m_manager_data.current_usage() !=
      timer_manager_data::usage::uninitialized) {
    safe_throw(hal::device_or_resource_busy(this));
  }

  return { peripheral_to_advanced_register(m_manager_data.m_id),
           &m_manager_data };
}

hal::stm32f1::timer general_purpose_timer_manager::acquire_timer()
{
  if (m_manager_data.current_usage() !=
      timer_manager_data::usage::uninitialized) {
    safe_throw(hal::device_or_resource_busy(this));
  }

  return { peripheral_to_general_register(m_manager_data.m_id),
           &m_manager_data };
}

hal::stm32f1::pwm_group_frequency
advanced_timer_manager::acquire_pwm_group_frequency()
{
  if (m_manager_data.current_usage() !=
        timer_manager_data::usage::uninitialized &&
      m_manager_data.current_usage() !=
        timer_manager_data::usage::pwm_generator) {
    safe_throw(hal::device_or_resource_busy(this));
  }

  return { peripheral_to_advanced_register(m_manager_data.m_id),
           &m_manager_data };
}

hal::stm32f1::pwm_group_frequency
general_purpose_timer_manager::acquire_pwm_group_frequency()
{
  if (m_manager_data.current_usage() !=
        timer_manager_data::usage::uninitialized &&
      m_manager_data.current_usage() !=
        timer_manager_data::usage::pwm_generator) {
    safe_throw(hal::device_or_resource_busy(this));
  }

  return { peripheral_to_general_register(m_manager_data.m_id),
           &m_manager_data };
}

hal::stm32f1::pwm16_channel advanced_timer_manager::acquire_pwm16_channel(
  timer_pins p_pin)
{
  if (m_manager_data.current_usage() !=
        timer_manager_data::usage::uninitialized &&
      m_manager_data.current_usage() !=
        timer_manager_data::usage::pwm_generator) {
    safe_throw(hal::device_or_resource_busy(this));
  }

  return { peripheral_to_advanced_register(m_manager_data.m_id),
           &m_manager_data,
           true,
           p_pin };
}

hal::stm32f1::pwm16_channel
general_purpose_timer_manager::acquire_pwm16_channel(timer_pins p_pin)
{
  if (m_manager_data.current_usage() !=
        timer_manager_data::usage::uninitialized &&
      m_manager_data.current_usage() !=
        timer_manager_data::usage::pwm_generator) {
    safe_throw(hal::device_or_resource_busy(this));
  }

  return { peripheral_to_general_register(m_manager_data.m_id),
           &m_manager_data,
           false,
           p_pin };
}

hal::stm32f1::pwm advanced_timer_manager::acquire_pwm(timer_pins p_pin)
{
  if (m_manager_data.current_usage() !=
        timer_manager_data::usage::uninitialized &&
      m_manager_data.current_usage() != timer_manager_data::usage::old_pwm) {
    safe_throw(hal::device_or_resource_busy(this));
  }

  return { peripheral_to_advanced_register(m_manager_data.m_id),
           &m_manager_data,
           true,
           p_pin };
}

hal::stm32f1::pwm general_purpose_timer_manager::acquire_pwm(timer_pins p_pin)
{
  if (m_manager_data.current_usage() !=
        timer_manager_data::usage::uninitialized &&
      m_manager_data.current_usage() != timer_manager_data::usage::old_pwm) {
    safe_throw(hal::device_or_resource_busy(this));
  }

  return { peripheral_to_general_register(m_manager_data.m_id),
           &m_manager_data,
           false,
           p_pin };
}

hal::v5::strong_ptr<hal::rotation_sensor>
general_purpose_timer_manager::acquire_quadrature_encoder(
  std::pmr::polymorphic_allocator<> p_allocator,
  timer_pins p_pin1,
  timer_pins p_pin2,
  u32 p_pulses_per_rotation)
{
  if (m_manager_data.current_usage() !=
      timer_manager_data::usage::uninitialized) {
    safe_throw(hal::device_or_resource_busy(this));
  }
  return hal::v5::make_strong_ptr<hal::stm32f1::quadrature_encoder>(
    p_allocator,
    p_pin1,
    p_pin2,
    m_manager_data.m_id,
    peripheral_to_general_register(m_manager_data.m_id),
    &m_manager_data,
    p_pulses_per_rotation);
}

hal::v5::strong_ptr<hal::rotation_sensor>
advanced_timer_manager::acquire_quadrature_encoder(
  std::pmr::polymorphic_allocator<> p_allocator,
  timer_pins p_pin1,
  timer_pins p_pin2,
  u32 p_pulses_per_rotation)
{
  if (m_manager_data.current_usage() !=
      timer_manager_data::usage::uninitialized) {
    safe_throw(hal::device_or_resource_busy(this));
  }
  return hal::v5::make_strong_ptr<hal::stm32f1::quadrature_encoder>(
    p_allocator,
    p_pin1,
    p_pin2,
    m_manager_data.m_id,
    peripheral_to_general_register(m_manager_data.m_id),
    &m_manager_data,
    p_pulses_per_rotation);
}

// Tell the compiler which instances to generate
template class advanced_timer<peripheral::timer1>;
template class advanced_timer<peripheral::timer8>;
template class general_purpose_timer<peripheral::timer2>;
template class general_purpose_timer<peripheral::timer3>;
template class general_purpose_timer<peripheral::timer4>;
template class general_purpose_timer<peripheral::timer5>;
template class general_purpose_timer<peripheral::timer9>;
template class general_purpose_timer<peripheral::timer10>;
template class general_purpose_timer<peripheral::timer11>;
template class general_purpose_timer<peripheral::timer12>;
template class general_purpose_timer<peripheral::timer13>;
template class general_purpose_timer<peripheral::timer14>;
}  // namespace hal::stm32f1
