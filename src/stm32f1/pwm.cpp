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

#include "interrupt_reg.hpp"
#include "pin.hpp"
#include "power.hpp"

namespace {
/**
 * @brief Static variable to track PWM availability.
 */
hal::u32 availability = 0;

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
                                         manager_data* p_manager_data_ptr)
  : m_pwm_frequency(unsafe{}, p_reg)
  , m_manager_data_ptr(p_manager_data_ptr)
{
}

pwm_group_frequency::~pwm_group_frequency()
{
  static constexpr u32 frequency_mask = ~(1 << 31);

  m_manager_data_ptr->m_resource_count.store(
    (m_manager_data_ptr->m_resource_count.load()) & frequency_mask);

  if (m_manager_data_ptr->m_resource_count.load() == 0) {
    m_manager_data_ptr->m_timer_usage =
      manager_data::timer_usage::uninitialized;
    power_off(m_manager_data_ptr->m_id);
    power_on(m_manager_data_ptr->m_id);
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
                             manager_data* p_manager_data_ptr,
                             bool p_is_advanced,
                             timer_pins p_pin)
  : m_pwm(unsafe{})
  , m_pin_num(hal::value(p_pin))
  , m_manager_data_ptr(p_manager_data_ptr)
{
  pin_information pin_info = determine_pin_info(p_pin, m_manager_data_ptr->m_id);
  configure_pin(pin_info.pin_select, push_pull_alternative_output);

  u32 const channel_mask = 1 << (pin_info.channel - 1);
  if ((m_manager_data_ptr->m_resource_count.load()) & channel_mask) {
    hal::safe_throw(hal::device_or_resource_busy(nullptr));
  }
  m_manager_data_ptr->m_timer_usage = manager_data::timer_usage::pwm_generator;
  m_manager_data_ptr->m_resource_count.store(
    (m_manager_data_ptr->m_resource_count.load()) | channel_mask);

  auto const pwm_pin_mask = bit_mask::from(m_pin_num);
  if (not hal::bit_extract(pwm_pin_mask, availability)) {
    bit_modify(availability).set(pwm_pin_mask);
  } else {
    hal::safe_throw(hal::device_or_resource_busy(nullptr));
  }

  // a generic pwm class requires a pin and a channel in order to use the right
  // registers, therefore we pass in the channel as an argument in the generic
  // pwm class's constructor
  m_pwm.initialize(unsafe{},
                   p_reg,
                   {
                     .channel = pin_info.channel,
                     .is_advanced = p_is_advanced,
                   });
}

pwm16_channel::~pwm16_channel()
{
  auto const pwm_pin_mask = bit_mask::from(m_pin_num);
  bit_modify(availability).clear(pwm_pin_mask);

  pin_information pin_info = determine_pin_info(static_cast<timer_pins>(m_pin_num), m_manager_data_ptr->m_id);
  u32 const channel_mask = ~(1 << (pin_info.channel - 1));

  m_manager_data_ptr->m_resource_count.store(
    (m_manager_data_ptr->m_resource_count.load()) & channel_mask);

  if (m_manager_data_ptr->m_resource_count.load() == 0) {
    m_manager_data_ptr->m_timer_usage =
      manager_data::timer_usage::uninitialized;
    power_off(m_manager_data_ptr->m_id);
    power_on(m_manager_data_ptr->m_id);
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
         manager_data* p_manager_data_ptr,
         bool p_is_advanced,
         stm32f1::timer_pins p_pin)
  : m_pwm(unsafe{})
  , m_pwm_frequency(unsafe{}, p_reg)
  , m_pin_num(hal::value(p_pin))
  , m_manager_data_ptr(p_manager_data_ptr)
{
  auto const pwm_pin_mask = bit_mask::from(m_pin_num);
  if (not hal::bit_extract(pwm_pin_mask, availability)) {
    bit_modify(availability).set(pwm_pin_mask);
  } else {
    hal::safe_throw(hal::device_or_resource_busy(nullptr));
  }

  pin_information pin_info = determine_pin_info(p_pin, m_manager_data_ptr->m_id);
  u32 const channel_mask = 1 << (pin_info.channel - 1);
  if ((m_manager_data_ptr->m_resource_count.load()) & channel_mask) {
    hal::safe_throw(hal::device_or_resource_busy(nullptr));
  }
  configure_pin(pin_info.pin_select, push_pull_alternative_output);

  m_manager_data_ptr->m_timer_usage = manager_data::timer_usage::old_pwm;
  m_manager_data_ptr->m_resource_count.store(
    (m_manager_data_ptr->m_resource_count.load()) | channel_mask);

  // a generic pwm class requires a pin and a channel in order to use the right
  // registers, therefore we pass in the channel as an argument in the generic
  // pwm class's constructor
  m_pwm.initialize(unsafe{},
                   p_reg,
                   {
                     .channel = pin_info.channel,
                     .is_advanced = p_is_advanced,
                   });
}

pwm::~pwm()
{
  auto const pwm_pin_mask = bit_mask::from(m_pin_num);
  bit_modify(availability).clear(pwm_pin_mask);

  pin_information pin_info = determine_pin_info(static_cast<timer_pins>(m_pin_num), m_manager_data_ptr->m_id);
  u32 const channel_mask = ~(1 << (pin_info.channel - 1));

  m_manager_data_ptr->m_resource_count.store(
    (m_manager_data_ptr->m_resource_count.load()) & channel_mask);

  if (m_manager_data_ptr->m_resource_count.load() == 0) {
    m_manager_data_ptr->m_timer_usage =
      manager_data::timer_usage::uninitialized;
    power_off(m_manager_data_ptr->m_id);
    power_on(m_manager_data_ptr->m_id);
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
