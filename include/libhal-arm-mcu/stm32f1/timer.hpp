// Copyright 2024 Khalil Estell
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

#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32_generic/pwm.hpp>

#include <libhal-util/bit.hpp>
#include <libhal/pwm.hpp>
#include <libhal/units.hpp>

#include "libhal-arm-mcu/stm32f1/clock.hpp"
#include "stm32f1/pin.hpp"
#include "stm32f1/power.hpp"

namespace hal::stm32f1 {
/**
 * @brief Default pins for the various timer peripherals. These pins can perform
 * any timer operation.
 */
// note to me: pwm availability needs to be handles here per advanced and
// general_purpose_timer classes.
enum class pins : u8
{
  pa0 = 0,
  pa1 = 1,
  pa2 = 2,
  pa3 = 3,
  pa6 = 4,
  pa7 = 5,
  pa8 = 6,
  pa9 = 7,
  pa10 = 8,
  pa11 = 9,
  pb0 = 10,
  pb1 = 11,
  pb6 = 12,
  pb7 = 13,
  pb8 = 14,
  pb9 = 15,
};

inline void* pwm_timer1 = reinterpret_cast<void*>(0x4001'2C00);  // TIM1 timer
inline void* pwm_timer2 = reinterpret_cast<void*>(0x4000'0000);
inline void* pwm_timer3 = reinterpret_cast<void*>(0x4000'0400);
inline void* pwm_timer4 = reinterpret_cast<void*>(0x4000'0800);

template<peripheral select>
class advanced_timer
{
public:
  /**
   */
  enum class timer1_pwm_pin : u8
  {
    pa8 = (u8)pins::pa8,
    pa9 = (u8)pins::pa9,
    pa10 = (u8)pins::pa10,
    pa11 = (u8)pins::pa11,
  };

  static_assert(select == peripheral::timer1,
                "Only timer 1 is allowed as advanced timers for this driver");

  /**
   * @brief This class takes in any Advanced timers such as Timers
   * 1 and 8. These timers can be used to do PWM
   * generation, as well as other advanced timer specific tasks
   */
  advanced_timer();

  /**
   * @brief Only one PWM channel is allowed. If someone tries to acquire it
   * again, the same channel won't be available until the destructor is
   * called.
   */
  hal::stm32_generic::pwm acquire_pwm(timer1_pwm_pin p_pin)
  {
    static_assert(select == peripheral::timer1,
                  "You can acquire only a timer1 or timer8 pin in this class");

    return acquire_pwm<select>(
      (pins)p_pin, pwm_timer1, stm32f1::frequency(peripheral::timer1));
  }
  peripheral m_peripheral_id;

private:
  hal::stm32_generic::pwm acquire_pwm(pins p_pin,
                                 void* p_reg,
                                 hertz current_timer_frequency)
  {
    // do all stuff pwm constructor stuff here.
    // find pwm register, do channel setup stuff, and then send to pwm as a
    // register.
    power_on(m_peripheral_id);
    int channel = 0;
    switch (p_pin) {
      case pins::pa8:
        configure_pin({ .port = 'A', .pin = 8 }, push_pull_alternative_output);
        channel = 1;
        break;
      case pins::pa9:
        configure_pin({ .port = 'A', .pin = 9 }, push_pull_alternative_output);
        channel = 2;
        break;
      case pins::pa10:
        configure_pin({ .port = 'A', .pin = 10 }, push_pull_alternative_output);
        channel = 3;
        break;
      case pins::pa11:
        configure_pin({ .port = 'A', .pin = 11 }, push_pull_alternative_output);
        channel = 4;
        break;
      default:
        break;  // will never go in here
    }
    return { p_reg, channel, current_timer_frequency };
  }
};

template<peripheral select>
class general_purpose_timer
{
public:
  /**
   * @brief This class takes in any general purpose timers such as Timers
   * 2,3,4,5,9,10,11,12,13,14 and 15. These timers can be used to do PWM
   * generation, as well as other timer specific tasks
   */
  enum class timer2_pwm_pin : u8
  {
    pa0 = (u8)pins::pa0,
    pa1 = (u8)pins::pa1,
    pa2 = (u8)pins::pa2,
    pa3 = (u8)pins::pa3,
  };

  enum class timer3_pwm_pin : u8
  {
    pa6 = (u8)pins::pa6,
    pa7 = (u8)pins::pa7,
    pb0 = (u8)pins::pb0,
    pb1 = (u8)pins::pb1,
  };

  enum class timer4_pwm_pin : u8
  {
    pb6 = (u8)pins::pb6,
    pb7 = (u8)pins::pb7,
    pb8 = (u8)pins::pb8,
    pb9 = (u8)pins::pb9,
  };
  static_assert(select == peripheral::timer2 or select == peripheral::timer3 or
                  select == peripheral::timer4,
                "Only timers 2, 3, or 4 are allowed as general purpose timers "
                "for this driver");

  general_purpose_timer();

  /**
   * @brief Only one PWM channel is allowed. If someone tries to acquire it
   * again, the same channel won't be available until the destructor is
   * called.
   */
  hal::stm32_generic::pwm acquire_pwm(timer2_pwm_pin p_pin)
  {
    static_assert(
      select == peripheral::timer2,
      "You can acquire a pin part of the selected timer peripheral");
    return acquire_pwm<select>(
      (pins)p_pin, pwm_timer2, stm32f1::frequency(peripheral::timer2));
  }

  hal::stm32_generic::pwm acquire_pwm(timer3_pwm_pin p_pin)
  {
    static_assert(
      select == peripheral::timer3,
      "You can acquire a pin part of the selected timer peripheral");
    return acquire_pwm(
      (pins)p_pin, pwm_timer3, stm32f1::frequency(peripheral::timer3));
  }

  hal::stm32_generic::pwm acquire_pwm(timer4_pwm_pin p_pin)
  {
    static_assert(
      select == peripheral::timer4,
      "You can acquire a pin part of the selected timer peripheral");
    return acquire_pwm<select>(
      (pins)p_pin, pwm_timer4, stm32f1::frequency(peripheral::timer4));
  }

private:
  hal::stm32_generic::pwm acquire_pwm(pins p_pin,
                                 stm32_generic::pwm_reg_t* p_reg,
                                 hertz current_timer_frequency)
  {
    // do all stuff pwm constructor stuff here.
    // find pwm register, do channel setup stuff, and then send to pwm as a
    // register.
    power_on(m_peripheral_id);
    int channel = 0;
    switch (p_pin) {
      case pins::pa0:
        configure_pin({ .port = 'A', .pin = 0 }, push_pull_alternative_output);
        channel = 1;
        break;
      case pins::pa1:
        configure_pin({ .port = 'A', .pin = 1 }, push_pull_alternative_output);
        channel = 2;
        break;
      case pins::pa2:
        configure_pin({ .port = 'A', .pin = 2 }, push_pull_alternative_output);
        channel = 3;
        break;
      case pins::pa3:
        configure_pin({ .port = 'A', .pin = 3 }, push_pull_alternative_output);
        channel = 4;
        break;
      case pins::pa6:
        configure_pin({ .port = 'A', .pin = 6 }, push_pull_alternative_output);
        channel = 1;
        break;
      case pins::pa7:
        configure_pin({ .port = 'A', .pin = 7 }, push_pull_alternative_output);
        channel = 2;
        break;
      case pins::pb0:
        configure_pin({ .port = 'B', .pin = 0 }, push_pull_alternative_output);
        channel = 3;
        break;
      case pins::pb1:
        configure_pin({ .port = 'B', .pin = 1 }, push_pull_alternative_output);
        channel = 4;
        break;
      case pins::pb6:
        configure_pin({ .port = 'B', .pin = 6 }, push_pull_alternative_output);
        channel = 1;
        break;
      case pins::pb7:
        configure_pin({ .port = 'B', .pin = 7 }, push_pull_alternative_output);
        channel = 2;
        break;
      case pins::pb8:
        configure_pin({ .port = 'B', .pin = 8 }, push_pull_alternative_output);
        channel = 3;
        break;
      case pins::pb9:
        configure_pin({ .port = 'B', .pin = 9 }, push_pull_alternative_output);
        channel = 4;
        break;
      default:
        break;  // will never go in here
    }
    return { p_reg, channel, current_timer_frequency };
  }
  peripheral m_peripheral_id;
};

}  // namespace hal::stm32f1
