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

#include <libhal-arm-mcu/stm32_generic/pwm.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32f1/pwm.hpp>
#include <libhal-arm-mcu/stm32f1/timer.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/enum.hpp>
#include <libhal/error.hpp>
#include <libhal/units.hpp>

#include "power.hpp"

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

template<peripheral select>
void* peripheral_to_general_register()
{
  void* reg;
  if constexpr (select == peripheral::timer2) {
    reg = timer2;
  } else if constexpr (select == peripheral::timer3) {
    reg = timer3;
  } else if constexpr (select == peripheral::timer4) {
    reg = timer4;
  } else if constexpr (select == peripheral::timer5) {
    reg = timer5;
  } else if constexpr (select == peripheral::timer9) {
    reg = timer9;
  } else if constexpr (select == peripheral::timer10) {
    reg = timer10;
  } else if constexpr (select == peripheral::timer11) {
    reg = timer11;
  } else if constexpr (select == peripheral::timer12) {
    reg = timer12;
  } else if constexpr (select == peripheral::timer13) {
    reg = timer13;
  } else {
    reg = timer14;
  }
  return reg;
}
}  // namespace

advanced_timer_manager::advanced_timer_manager(peripheral p_id)
  : m_id(p_id)
{
  power_on(m_id);
}

advanced_timer_manager::~advanced_timer_manager()
{
  power_off(m_id);
}

template<peripheral select>
general_purpose_timer<select>::general_purpose_timer()
{
  power_on(select);
}
template<peripheral select>
general_purpose_timer<select>::~general_purpose_timer()
{
  power_off(select);
}

hal::stm32f1::pwm advanced_timer_manager::acquire_pwm(timer_pins p_pin)
{
  return { peripheral_to_advanced_register(m_id), m_id, true, p_pin };
}

template<peripheral select>
hal::stm32f1::pwm general_purpose_timer<select>::acquire_pwm(pin_type p_pin)
{

  return { peripheral_to_general_register<select>(),
           select,
           false,
           static_cast<timer_pins>(p_pin) };
}

hal::stm32f1::pwm16_channel advanced_timer_manager::acquire_pwm16_channel(
  timer_pins p_pin)
{
  return { peripheral_to_advanced_register(m_id), m_id, true, p_pin };
}

template<peripheral select>
hal::stm32f1::pwm16_channel
general_purpose_timer<select>::acquire_pwm16_channel(pin_type p_pin)
{
  return { peripheral_to_general_register<select>(),
           select,
           false,
           static_cast<timer_pins>(p_pin) };
}

hal::stm32f1::pwm_group_frequency
advanced_timer_manager::acquire_pwm_group_frequency()
{
  return { peripheral_to_advanced_register(m_id), m_id };
}

template<peripheral select>
hal::stm32f1::pwm_group_frequency
general_purpose_timer<select>::acquire_pwm_group_frequency()
{
  return { peripheral_to_general_register<select>(), select };
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
