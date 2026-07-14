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

#include <libhal-util/bit.hpp>
#include <utility>

#include <libhal-arm-mcu/stm32_generic/pwm.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32f1/pin.hpp>
#include <libhal-arm-mcu/stm32f1/pwm.hpp>
#include <libhal-arm-mcu/stm32f1/timer.hpp>
#include <libhal/error.hpp>
#include <libhal/pwm.hpp>
#include <libhal/units.hpp>

#include "pin.hpp"
#include "power.hpp"

namespace {
struct pin_information
{
  hal::u8 channel;
  hal::stm32f1::pin_select pin_select;
};

constexpr pin_information determine_pin_info(
  hal::stm32f1::timer_pins p_pin,
  hal::stm32f1::peripheral p_peripheral)
{
  pin_information pin_info;
  switch (p_pin) {
    case hal::stm32f1::timer_pins::pa0:
      pin_info.pin_select.port = 'A';
      pin_info.pin_select.pin = 0;
      pin_info.channel = 1;
      break;
    case hal::stm32f1::timer_pins::pa1:
      pin_info.pin_select.port = 'A';
      pin_info.pin_select.pin = 1;
      pin_info.channel = 2;
      break;
    case hal::stm32f1::timer_pins::pa2:
      pin_info.pin_select.port = 'A';
      pin_info.pin_select.pin = 2;
      pin_info.channel =
        p_peripheral == hal::stm32f1::peripheral::timer9 ? 1 : 3;
      break;
    case hal::stm32f1::timer_pins::pa3:
      pin_info.pin_select.port = 'A';
      pin_info.pin_select.pin = 3;
      pin_info.channel =
        p_peripheral == hal::stm32f1::peripheral::timer9 ? 2 : 4;
      break;
    case hal::stm32f1::timer_pins::pa6:
      pin_info.pin_select.port = 'A';
      pin_info.pin_select.pin = 6;
      pin_info.channel = 1;
      break;
    case hal::stm32f1::timer_pins::pa7:
      pin_info.pin_select.port = 'A';
      pin_info.pin_select.pin = 7;
      pin_info.channel =
        p_peripheral == hal::stm32f1::peripheral::timer14 ? 1 : 2;
      break;
    case hal::stm32f1::timer_pins::pb0:
      pin_info.pin_select.port = 'B';
      pin_info.pin_select.pin = 0;
      pin_info.channel = 3;
      break;
    case hal::stm32f1::timer_pins::pb1:
      pin_info.pin_select.port = 'B';
      pin_info.pin_select.pin = 1;
      pin_info.channel = 4;
      break;
    case hal::stm32f1::timer_pins::pb6:
      pin_info.pin_select.port = 'B';
      pin_info.pin_select.pin = 6;
      pin_info.channel = 1;
      break;
    case hal::stm32f1::timer_pins::pb7:
      pin_info.pin_select.port = 'B';
      pin_info.pin_select.pin = 7;
      pin_info.channel = 2;
      break;
    case hal::stm32f1::timer_pins::pb8:
      pin_info.pin_select.port = 'B';
      pin_info.pin_select.pin = 8;
      pin_info.channel =
        p_peripheral == hal::stm32f1::peripheral::timer10 ? 1 : 3;
      break;
    case hal::stm32f1::timer_pins::pb9:
      pin_info.pin_select.port = 'B';
      pin_info.pin_select.pin = 9;
      pin_info.channel =
        p_peripheral == hal::stm32f1::peripheral::timer11 ? 1 : 4;
      break;
    case hal::stm32f1::timer_pins::pa8:
      pin_info.pin_select.port = 'A';
      pin_info.pin_select.pin = 8;
      pin_info.channel = 1;
      break;
    case hal::stm32f1::timer_pins::pa9:
      pin_info.pin_select.port = 'A';
      pin_info.pin_select.pin = 9;
      pin_info.channel = 2;
      break;
    case hal::stm32f1::timer_pins::pa10:
      pin_info.pin_select.port = 'A';
      pin_info.pin_select.pin = 10;
      pin_info.channel = 3;
      break;
    case hal::stm32f1::timer_pins::pa11:
      pin_info.pin_select.port = 'A';
      pin_info.pin_select.pin = 11;
      pin_info.channel = 4;
      break;
    case hal::stm32f1::timer_pins::pc6:
      pin_info.pin_select.port = 'C';
      pin_info.pin_select.pin = 6;
      pin_info.channel = 1;
      break;
    case hal::stm32f1::timer_pins::pc7:
      pin_info.pin_select.port = 'C';
      pin_info.pin_select.pin = 7;
      pin_info.channel = 2;
      break;
    case hal::stm32f1::timer_pins::pc8:
      pin_info.pin_select.port = 'C';
      pin_info.pin_select.pin = 8;
      pin_info.channel = 3;
      break;
    case hal::stm32f1::timer_pins::pc9:
      pin_info.pin_select.port = 'C';
      pin_info.pin_select.pin = 9;
      pin_info.channel = 4;
      break;
    case hal::stm32f1::timer_pins::pb14:
      pin_info.pin_select.port = 'B';
      pin_info.pin_select.pin = 14;
      pin_info.channel = 1;
      break;
    case hal::stm32f1::timer_pins::pb15:
      pin_info.pin_select.port = 'B';
      pin_info.pin_select.pin = 15;
      pin_info.channel = 2;
      break;
    default:
      std::unreachable();
  }
  return pin_info;
}
}  // namespace

namespace hal::stm32f1 {

pwm_group_frequency::pwm_group_frequency(void* p_reg,
                                         timer_manager_data* p_manager_data_ptr)
  : m_pwm_frequency(unsafe{}, p_reg)
  , m_manager_data_ptr(p_manager_data_ptr)
{
  m_manager_data_ptr->m_usage = timer_manager_data::usage::pwm_generator;
  m_manager_data_ptr->m_resource_count++;
}

pwm_group_frequency::pwm_group_frequency(pwm_group_frequency&& p_other) noexcept
  : m_pwm_frequency(std::move(p_other.m_pwm_frequency))
{
  m_manager_data_ptr = std::exchange(p_other.m_manager_data_ptr, nullptr);
}

pwm_group_frequency& pwm_group_frequency::operator=(
  pwm_group_frequency&& p_other) noexcept
{
  if (this == &p_other) {
    return *this;
  }

  m_pwm_frequency = std::move(p_other.m_pwm_frequency);
  m_manager_data_ptr = std::exchange(p_other.m_manager_data_ptr, nullptr);

  return *this;
}

pwm_group_frequency::~pwm_group_frequency()
{
  if (m_manager_data_ptr == nullptr) {
    return;
  }

  m_manager_data_ptr->m_resource_count--;

  if (m_manager_data_ptr->m_resource_count.load() == 0) {
    m_manager_data_ptr->m_usage = timer_manager_data::usage::uninitialized;
    reset_peripheral(m_manager_data_ptr->m_id);
  }
}

void pwm_group_frequency::driver_frequency(u32 p_frequency)
{
  return m_pwm_frequency.set_group_frequency({
    .pwm_frequency = p_frequency,
    .timer_clock_frequency =
      static_cast<u32>(stm32f1::frequency(m_manager_data_ptr->m_id)),
  });
}

pwm16_channel::pwm16_channel(void* p_reg,
                             timer_manager_data* p_manager_data_ptr,
                             bool p_is_advanced,
                             timer_pins p_pin)
  : m_pwm(unsafe{})
  , m_pin(p_pin)
  , m_manager_data_ptr(p_manager_data_ptr)
{
  pin_information pin_info =
    determine_pin_info(m_pin, m_manager_data_ptr->m_id);
  configure_pin(pin_info.pin_select, push_pull_alternative_output);

  // a generic pwm class requires a pin and a channel in order to use the right
  // registers, therefore we pass in the channel as an argument in the generic
  // pwm class's constructor
  m_pwm.initialize(unsafe{},
                   p_reg,
                   {
                     .channel = pin_info.channel,
                     .is_advanced = p_is_advanced,
                   });

  m_manager_data_ptr->m_usage = timer_manager_data::usage::pwm_generator;
  m_manager_data_ptr->m_resource_count++;
}
pwm16_channel::pwm16_channel(pwm16_channel&& p_other) noexcept
  : m_pwm(std::move(p_other.m_pwm))
{
  m_pin = p_other.m_pin;
  m_manager_data_ptr = std::exchange(p_other.m_manager_data_ptr, nullptr);
}

pwm16_channel& pwm16_channel::operator=(pwm16_channel&& p_other) noexcept
{
  if (this == &p_other) {
    return *this;
  }
  m_pwm = std::move(p_other.m_pwm);
  m_pin = p_other.m_pin;
  m_manager_data_ptr = std::exchange(p_other.m_manager_data_ptr, nullptr);
  return *this;
}
pwm16_channel::~pwm16_channel()
{
  if (m_manager_data_ptr == nullptr) {
    return;
  }

  pin_information pin_info =
    determine_pin_info(m_pin, m_manager_data_ptr->m_id);
  reset_pin(pin_info.pin_select);

  m_manager_data_ptr->m_resource_count--;

  if (m_manager_data_ptr->m_resource_count.load() == 0) {
    m_manager_data_ptr->m_usage = timer_manager_data::usage::uninitialized;
    reset_peripheral(m_manager_data_ptr->m_id);
  }
}

u32 pwm16_channel::driver_frequency()
{
  return m_pwm.frequency(
    static_cast<u32>(stm32f1::frequency(m_manager_data_ptr->m_id)));
}

void pwm16_channel::driver_duty_cycle(u16 p_duty_cycle)
{
  m_pwm.duty_cycle(p_duty_cycle);
}

pwm::pwm(void* p_reg,
         timer_manager_data* p_manager_data_ptr,
         bool p_is_advanced,
         stm32f1::timer_pins p_pin)
  : m_pwm(unsafe{})
  , m_pwm_frequency(unsafe{}, p_reg)
  , m_pin(p_pin)
  , m_manager_data_ptr(p_manager_data_ptr)
{
  pin_information pin_info =
    determine_pin_info(m_pin, m_manager_data_ptr->m_id);
  configure_pin(pin_info.pin_select, push_pull_alternative_output);

  // a generic pwm class requires a pin and a channel in order to use the right
  // registers, therefore we pass in the channel as an argument in the generic
  // pwm class's constructor
  m_pwm.initialize(unsafe{},
                   p_reg,
                   {
                     .channel = pin_info.channel,
                     .is_advanced = p_is_advanced,
                   });

  m_manager_data_ptr->m_usage = timer_manager_data::usage::old_pwm;
  m_manager_data_ptr->m_resource_count++;
}

pwm::pwm(pwm&& p_other) noexcept
  : m_pwm(std::move(p_other.m_pwm))
  , m_pwm_frequency(std::move(p_other.m_pwm_frequency))
  , m_pin(p_other.m_pin)
  , m_manager_data_ptr(p_other.m_manager_data_ptr)
{
  m_manager_data_ptr = std::exchange(p_other.m_manager_data_ptr, nullptr);
}

pwm& pwm::operator=(pwm&& p_other) noexcept
{
  if (this == &p_other) {
    return *this;
  }

  m_pwm = std::move(p_other.m_pwm);
  m_pwm_frequency = std::move(p_other.m_pwm_frequency);
  m_pin = p_other.m_pin;
  m_manager_data_ptr = std::exchange(p_other.m_manager_data_ptr, nullptr);

  return *this;
}

pwm::~pwm()
{
  if (m_manager_data_ptr == nullptr) {
    return;
  }
  pin_information pin_info =
    determine_pin_info(m_pin, m_manager_data_ptr->m_id);
  reset_pin(pin_info.pin_select);

  m_manager_data_ptr->m_resource_count--;

  if (m_manager_data_ptr->m_resource_count.load() == 0) {
    m_manager_data_ptr->m_usage = timer_manager_data::usage::uninitialized;
    reset_peripheral(m_manager_data_ptr->m_id);
  }
}

void pwm::driver_frequency(hertz p_frequency)
{
  m_pwm_frequency.set_group_frequency({
    .pwm_frequency = static_cast<u32>(p_frequency),
    .timer_clock_frequency =
      static_cast<u32>(stm32f1::frequency(m_manager_data_ptr->m_id)),
  });
}

void pwm::driver_duty_cycle(float p_duty_cycle)
{
  m_pwm.duty_cycle(p_duty_cycle);
}
}  // namespace hal::stm32f1
