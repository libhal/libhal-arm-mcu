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

#include <libhal-arm-mcu/stm32f1/clock.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32f1/pwm_wrapper.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/enum.hpp>
#include <libhal/pwm.hpp>
#include <libhal/units.hpp>
#include <type_traits>

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
/** @brief pins that are connected to the timer1 peripheral
 */
enum class timer1_pin : u8
{
  pa8 = hal::value(pins::pa8),
  pa9 = hal::value(pins::pa9),
  pa10 = hal::value(pins::pa10),
  pa11 = hal::value(pins::pa11),
};

/** @brief pins that are connected to the timer2 peripheral
 */
enum class timer2_pin : u8
{
  pa0 = hal::value(pins::pa0),
  pa1 = hal::value(pins::pa1),
  pa2 = hal::value(pins::pa2),
  pa3 = hal::value(pins::pa3),
};
/** @brief pins that are connected to the timer3 peripheral
 */
enum class timer3_pin : u8
{
  pa6 = hal::value(pins::pa6),
  pa7 = hal::value(pins::pa7),
  pb0 = hal::value(pins::pb0),
  pb1 = hal::value(pins::pb1),
};
/** @brief pins that are connected to the timer4 peripheral
 */
enum class timer4_pin : u8
{
  pb6 = hal::value(pins::pb6),
  pb7 = hal::value(pins::pb7),
  pb8 = hal::value(pins::pb8),
  pb9 = hal::value(pins::pb9),
};

/** @brief Gets the pwm timer type at compile time.
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
  } else {
    return std::type_identity<void>();
  }
}
inline void* pwm_timer1 = reinterpret_cast<void*>(0x4001'2C00);
inline void* pwm_timer2 = reinterpret_cast<void*>(0x4000'0000);
inline void* pwm_timer3 = reinterpret_cast<void*>(0x4000'0400);
inline void* pwm_timer4 = reinterpret_cast<void*>(0x4000'0800);

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
  static_assert(select == peripheral::timer1,
                "Only timer 1 is allowed as advanced timers for this driver");
  using pin_type = decltype(get_pwm_timer_type<select>())::type;

  advanced_timer(advanced_timer const& p_other) = delete;
  advanced_timer& operator=(advanced_timer const& p_other) = delete;
  advanced_timer(advanced_timer&& p_other) noexcept = delete;
  advanced_timer& operator=(advanced_timer&& p_other) noexcept = delete;

  advanced_timer() = default;
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
  [[nodiscard]] hal::stm32f1::pwm_wrapper acquire_pwm(pin_type p_pin);

private:
  hal::stm32f1::pwm_wrapper acquire_pwm(pins p_pin,
                                        void* p_reg,
                                        hertz current_timer_frequency);
};
/**
 * @brief This template class takes can do any timer operation for timers 2
 * through 15.
 *
 * The peripheral ID is the template argument, in order to ensure that the pins
 * used correspond to the correct timer instantiation as well as the correct
 * coresponding pins at compile time.
 */
template<peripheral select>
class general_purpose_timer
{
public:
  static_assert(select == peripheral::timer2 or select == peripheral::timer3 or
                  select == peripheral::timer4,
                "Only timers 2, 3, or 4 are allowed as general purpose timers "
                "for this driver");
  using pin_type = decltype(get_pwm_timer_type<select>())::type;

  general_purpose_timer(general_purpose_timer const& p_other) = delete;
  general_purpose_timer& operator=(general_purpose_timer const& p_other) =
    delete;
  general_purpose_timer(general_purpose_timer&& p_other) noexcept = delete;
  general_purpose_timer& operator=(general_purpose_timer&& p_other) noexcept =
    delete;
  general_purpose_timer() = default;

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
  [[nodiscard]] hal::stm32f1::pwm_wrapper acquire_pwm(pin_type p_pin);

private:
  hal::stm32f1::pwm_wrapper acquire_pwm(pins p_pin,
                                        void* p_reg,
                                        hertz current_timer_frequency);
};

}  // namespace hal::stm32f1
