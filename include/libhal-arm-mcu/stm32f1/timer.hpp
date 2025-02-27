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

#include <type_traits>

#include <libhal-arm-mcu/stm32f1/clock.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32f1/pwm_wrapper.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/enum.hpp>
#include <libhal/pwm.hpp>
#include <libhal/units.hpp>

namespace hal::stm32f1 {
/**
 * @brief Default pins for the various timer peripherals.
 *
 * These pins can perform any timer operation.
 */
enum class pins : u8
{
  pa0 = 0,
  pa1 = 1,
  pa2 = 2,
  pa3 = 3,
  pa6 = 4,
  pa7 = 5,
  pb0 = 6,
  pb1 = 7,
  pa8 = 8,
  pa9 = 9,
  pa10 = 10,
  pa11 = 11,
  pb6 = 12,
  pb7 = 13,
  pb8 = 14,
  pb9 = 15,
  pc6 = 16,
  pc7 = 17,
  pc8 = 18,
  pc9 = 19,
  pb14 = 20,
  pb15 = 21,
};

/**
 * @brief pins that are connected to the timer1 peripheral
 */
enum class timer1_pin : u8
{
  pa8 = hal::value(pins::pa8),
  pa9 = hal::value(pins::pa9),
  pa10 = hal::value(pins::pa10),
  pa11 = hal::value(pins::pa11),
};

/**
 * @brief pins that are connected to the timer 2 and 5 and 9 peripheral
 */
enum class timer2_pin : u8
{
  pa0 = hal::value(pins::pa0),
  pa1 = hal::value(pins::pa1),
  pa2 = hal::value(pins::pa2),
  pa3 = hal::value(pins::pa3),
};

/**
 * @brief pins that are connected to the timer3 peripheral
 */
enum class timer3_pin : u8
{
  pa6 = hal::value(pins::pa6),
  pa7 = hal::value(pins::pa7),
  pb0 = hal::value(pins::pb0),
  pb1 = hal::value(pins::pb1),
};

/**
 * @brief pins that are connected to the timer4 peripheral
 */
enum class timer4_pin : u8
{
  pb6 = hal::value(pins::pb6),
  pb7 = hal::value(pins::pb7),
  pb8 = hal::value(pins::pb8),
  pb9 = hal::value(pins::pb9),
};

/**
 * @brief pins that are connected to the timer 2, 5 and 9 peripheral
 */
enum class timer5_pin : u8
{
  pa0 = hal::value(pins::pa0),
  pa1 = hal::value(pins::pa1),
  pa2 = hal::value(pins::pa2),
  pa3 = hal::value(pins::pa3),
};

/**
 * @brief pins that are connected to the timer8 peripheral
 */
enum class timer8_pin : u8
{
  pc6 = hal::value(pins::pc6),
  pc7 = hal::value(pins::pc7),
  pc8 = hal::value(pins::pc8),
  pc9 = hal::value(pins::pc9),
};

/**
 * @brief pins that are connected to the timers 2, 5, 9 peripherals
 */
enum class timer9_pin : u8
{
  pa2 = hal::value(pins::pa2),
  pa3 = hal::value(pins::pa3),
};

/**
 * @brief pins that are connected to the timers 4 and 10 peripherals
 */
enum class timer10_pin : u8
{
  pb8 = hal::value(pins::pb8),
};

/**
 * @brief pins that are connected to the timers 4 and 11 peripherals
 */
enum class timer11_pin : u8  // won't work
{
  pb9 = hal::value(pins::pb9),
};

/**
 * @brief pins that are connected to the 12 peripheral
 */
enum class timer12_pin : u8
{
  pb14 = hal::value(pins::pb14),
  pb15 = hal::value(pins::pb15),
};

/**
 * @brief pins that are connected to the 13 and 3 peripheral
 */
enum class timer13_pin : u8
{
  pa6 = hal::value(pins::pa6),
};

/**
 * @brief pins that are connected to the 14 and 3 peripheral.
 *
 */
enum class timer14_pin : u8
{
  pa7 = hal::value(pins::pa7),
};

/**
 * @brief Gets the pwm timer type at compile time.
 */
template<peripheral id>
consteval auto get_pwm_timer_type()
{
  if constexpr (id == peripheral::timer1) {
    return std::type_identity<timer1_pin>();
  } else if constexpr (id == peripheral::timer2) {
    return std::type_identity<timer2_pin>();
  } else if constexpr (id == peripheral::timer3) {
    return std::type_identity<timer3_pin>();
  } else if constexpr (id == peripheral::timer4) {
    return std::type_identity<timer4_pin>();
  } else if constexpr (id == peripheral::timer5) {
    return std::type_identity<timer5_pin>();
  } else if constexpr (id == peripheral::timer8) {
    return std::type_identity<timer8_pin>();
  } else if constexpr (id == peripheral::timer9) {
    return std::type_identity<timer9_pin>();
  } else if constexpr (id == peripheral::timer10) {
    return std::type_identity<timer10_pin>();
  } else if constexpr (id == peripheral::timer11) {
    return std::type_identity<timer11_pin>();
  } else if constexpr (id == peripheral::timer12) {
    return std::type_identity<timer12_pin>();
  } else if constexpr (id == peripheral::timer13) {
    return std::type_identity<timer13_pin>();
  } else if constexpr (id == peripheral::timer14) {
    return std::type_identity<timer14_pin>();
  } else {
    return std::type_identity<void>();
  }
}

/**
 * @brief This template class takes can do any timer operation for timers 1
 * and 8.
 *
 * The peripheral ID is the template argument, in order to ensure
 * that the pins used correspond to the correct timer instantiation as well as
 * the correct coresponding pins at compile time.
 *
 * These timers can be used to do PWM generation, as well as other advanced
 * timer specific tasks
 */
template<peripheral select>
class advanced_timer
{
public:
  static_assert(
    select == peripheral::timer1 or select == peripheral::timer8,
    "Only timer 1 or 8 is allowed as advanced timers for this driver.");
  using pin_type = decltype(get_pwm_timer_type<select>())::type;

  advanced_timer(advanced_timer const& p_other) = delete;
  advanced_timer& operator=(advanced_timer const& p_other) = delete;
  advanced_timer(advanced_timer&& p_other) noexcept = delete;
  advanced_timer& operator=(advanced_timer&& p_other) noexcept = delete;

  advanced_timer();

  /**
   * @brief Acquire a PWM channel from this timer
   * @deprecated Use the `acquire_pwm16_channel` and
   * `acquire_pwm_group_frequency` functions instead. This function will be
   * removed in libhal 5.
   *
   * Only one PWM channel is allowed to exist per timer.
   * If a PWM channel object is destroyed, then another PWM channel can be
   * acquired from this timer.
   *
   * @throws hal::device_or_resource_busy - If the application attempts to
   * acquire a pwm channel while a pwm channel bound to this timer already
   * exists.
   */
  [[nodiscard]] hal::stm32f1::pwm acquire_pwm(pin_type p_pin);

  /**
   * @brief Acquire a PWM channel from this timer
   *
   * Only one PWM channel is allowed to exist per timer.
   * If a PWM channel object is destroyed, then another PWM channel can be
   * acquired from this timer.
   *
   * @throws hal::device_or_resource_busy - If the application attempts to
   * acquire a pwm channel while a pwm channel bound to this timer already
   * exists.
   */
  [[nodiscard]] hal::stm32f1::pwm16_channel acquire_pwm16_channel(
    pin_type p_pin);

  /**
   * @brief Acquire a PWM group frequency driver
   *
   */
  [[nodiscard]] hal::stm32f1::pwm_group_frequency acquire_pwm_group_frequency();
};
/**
 * @brief This template class takes can do any timer operation for timers 2
 * through 15 and excluding timers 6 and 7
 *
 * The peripheral ID is the template argument, in order to ensure that the pins
 * used correspond to the correct timer instantiation as well as the correct
 * corresponding pins at compile time.
 */
template<peripheral select>
class general_purpose_timer
{
public:
  static_assert(
    select == peripheral::timer2 or select == peripheral::timer3 or
      select == peripheral::timer4 or select == peripheral::timer5 or
      select == peripheral::timer9 or select == peripheral::timer10 or
      select == peripheral::timer11 or select == peripheral::timer12 or
      select == peripheral::timer13 or select == peripheral::timer14,
    "Only timers 2, 3, 4, 6, 9, 10, 11, 12, 13, and 14 are allowed as general "
    "purpose timers for this driver.");
  using pin_type = decltype(get_pwm_timer_type<select>())::type;

  general_purpose_timer(general_purpose_timer const& p_other) = delete;
  general_purpose_timer& operator=(general_purpose_timer const& p_other) =
    delete;
  general_purpose_timer(general_purpose_timer&& p_other) noexcept = delete;
  general_purpose_timer& operator=(general_purpose_timer&& p_other) noexcept =
    delete;
  general_purpose_timer();

  /**
   * @brief Acquire a PWM channel from this timer
   * @deprecated Use the `acquire_pwm16_channel` and
   * `acquire_pwm_group_frequency` functions instead. This function will be
   * removed in libhal 5.
   *
   * Only one PWM channel is allowed to exist per timer.
   * If a PWM channel object is destroyed, then another PWM channel can be
   * acquired from this timer.
   *
   * @throws hal::device_or_resource_busy - If the application attempts to
   * acquire a pwm channel while a pwm channel bound to this timer already
   * exists.
   */
  [[nodiscard]] hal::stm32f1::pwm acquire_pwm(pin_type p_pin);

  /**
   * @brief Acquire a PWM channel from this timer
   *
   * Only one PWM channel is allowed to exist per timer.
   * If a PWM channel object is destroyed, then another PWM channel can be
   * acquired from this timer.
   *
   * @throws hal::device_or_resource_busy - If the application attempts to
   * acquire a pwm channel while a pwm channel bound to this timer already
   * exists.
   */
  [[nodiscard]] hal::stm32f1::pwm16_channel acquire_pwm16_channel(
    pin_type p_pin);

  /**
   * @brief Acquire a PWM group frequency driver
   *
   */
  [[nodiscard]] hal::stm32f1::pwm_group_frequency acquire_pwm_group_frequency();
};

}  // namespace hal::stm32f1
