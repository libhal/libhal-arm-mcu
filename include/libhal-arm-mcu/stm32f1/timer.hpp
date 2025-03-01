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

#include <libhal-arm-mcu/stm32_generic/pwm.hpp>
#include <libhal-arm-mcu/stm32f1/clock.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
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
enum class timer_pins : u8
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
  pa8 = hal::value(timer_pins::pa8),
  pa9 = hal::value(timer_pins::pa9),
  pa10 = hal::value(timer_pins::pa10),
  pa11 = hal::value(timer_pins::pa11),
};

/**
 * @brief pins that are connected to the timer 2 and 5 and 9 peripheral
 */
enum class timer2_pin : u8
{
  pa0 = hal::value(timer_pins::pa0),
  pa1 = hal::value(timer_pins::pa1),
  pa2 = hal::value(timer_pins::pa2),
  pa3 = hal::value(timer_pins::pa3),
};

/**
 * @brief pins that are connected to the timer3 peripheral
 */
enum class timer3_pin : u8
{
  pa6 = hal::value(timer_pins::pa6),
  pa7 = hal::value(timer_pins::pa7),
  pb0 = hal::value(timer_pins::pb0),
  pb1 = hal::value(timer_pins::pb1),
};

/**
 * @brief pins that are connected to the timer4 peripheral
 */
enum class timer4_pin : u8
{
  pb6 = hal::value(timer_pins::pb6),
  pb7 = hal::value(timer_pins::pb7),
  pb8 = hal::value(timer_pins::pb8),
  pb9 = hal::value(timer_pins::pb9),
};

/**
 * @brief pins that are connected to the timer 2, 5 and 9 peripheral
 */
enum class timer5_pin : u8
{
  pa0 = hal::value(timer_pins::pa0),
  pa1 = hal::value(timer_pins::pa1),
  pa2 = hal::value(timer_pins::pa2),
  pa3 = hal::value(timer_pins::pa3),
};

/**
 * @brief pins that are connected to the timer8 peripheral
 */
enum class timer8_pin : u8
{
  pc6 = hal::value(timer_pins::pc6),
  pc7 = hal::value(timer_pins::pc7),
  pc8 = hal::value(timer_pins::pc8),
  pc9 = hal::value(timer_pins::pc9),
};

/**
 * @brief pins that are connected to the timers 2, 5, 9 peripherals
 */
enum class timer9_pin : u8
{
  pa2 = hal::value(timer_pins::pa2),
  pa3 = hal::value(timer_pins::pa3),
};

/**
 * @brief pins that are connected to the timers 4 and 10 peripherals
 */
enum class timer10_pin : u8
{
  pb8 = hal::value(timer_pins::pb8),
};

/**
 * @brief pins that are connected to the timers 4 and 11 peripherals
 */
enum class timer11_pin : u8  // won't work
{
  pb9 = hal::value(timer_pins::pb9),
};

/**
 * @brief pins that are connected to the 12 peripheral
 */
enum class timer12_pin : u8
{
  pb14 = hal::value(timer_pins::pb14),
  pb15 = hal::value(timer_pins::pb15),
};

/**
 * @brief pins that are connected to the 13 and 3 peripheral
 */
enum class timer13_pin : u8
{
  pa6 = hal::value(timer_pins::pa6),
};

/**
 * @brief pins that are connected to the 14 and 3 peripheral.
 *
 */
enum class timer14_pin : u8
{
  pa7 = hal::value(timer_pins::pa7),
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

class advanced_timer_manager;

template<hal::stm32f1::peripheral select>
class general_purpose_timer;

/**
 * @brief This class is a wrapper for the pwm class.
 *
 * It manages the pwm availability of the various different pins, and keeps
 * track of whether a pwm is available or not. It inherits hal::pwm because this
 * object is returned to the timer instance and can be used as an hal::pwm in
 * any application.
 */
class pwm16_channel : public hal::pwm16_channel
{
public:
  friend class hal::stm32f1::advanced_timer_manager;

  template<hal::stm32f1::peripheral select>
  friend class hal::stm32f1::general_purpose_timer;

  pwm16_channel(pwm16_channel const& p_other) = delete;
  pwm16_channel& operator=(pwm16_channel const& p_other) = delete;
  pwm16_channel(pwm16_channel&& p_other) noexcept = default;
  pwm16_channel& operator=(pwm16_channel&& p_other) noexcept = default;
  ~pwm16_channel() override;

private:
  /**
   * @brief The pwm constructor is private because the only way one should be
   * able to access pwm is through the timer class
   *
   * @param p_reg is a void pointer that points to the beginning of a timer
   * peripheral
   * @param p_select is the timer peripheral that this instance is using.
   * @param p_is_advanced whether the current peripheral is advanced or not
   * @param p_pins This number is used to update the an internal bitmap
   * containing which PWM pins have already been acquired as well as configure
   * the pins and their channels. On construction the bit indicated by this
   * value is checked to see if it is set. If it is set, an exception is thrown
   * indicating that this resource has already been acquired. Otherwise, this
   * driver sets that bit. On destruction that bit is cleared to allow another
   * driver to utilize that pin in the future.
   *
   * @throws device_or_resource_busy when a pwm is already being used on a pin.
   */
  pwm16_channel(void* p_reg,
                stm32f1::peripheral p_select,
                bool p_is_advanced,
                timer_pins p_pin);

  u32 driver_frequency() override;
  void driver_duty_cycle(u16 p_duty_cycle) override;

  hal::stm32_generic::pwm m_pwm;
  hal::u16 m_pin_num;
  stm32f1::peripheral m_select;
};

/**
 * @brief This class is a wrapper for the pwm class.
 *
 * It manages the pwm availability of the various different pins, and keeps
 * track of whether a pwm is available or not. It inherits hal::pwm because this
 * object is returned to the timer instance and can be used as an hal::pwm in
 * any application.
 */
class pwm_group_frequency : public hal::pwm_group_manager
{
public:
  friend class hal::stm32f1::advanced_timer_manager;

  template<hal::stm32f1::peripheral select>
  friend class hal::stm32f1::general_purpose_timer;

  pwm_group_frequency(pwm_group_frequency const& p_other) = delete;
  pwm_group_frequency& operator=(pwm_group_frequency const& p_other) = delete;
  pwm_group_frequency(pwm_group_frequency&& p_other) noexcept = default;
  pwm_group_frequency& operator=(pwm_group_frequency&& p_other) noexcept =
    default;
  ~pwm_group_frequency() override = default;

private:
  /**
   * @brief The pwm constructor is private because the only way one should be
   * able to access pwm is through the timer class
   *
   * @param p_reg is a void pointer that points to the beginning of a timer
   * peripheral
   * @param p_select is the timer peripheral that this instance is using.
   * @param p_is_advanced whether the current peripheral is advanced or not
   * @param p_pins This number is used to update the an internal bitmap
   * containing which PWM pins have already been acquired as well as configure
   * the pins and their channels. On construction the bit indicated by this
   * value is checked to see if it is set. If it is set, an exception is thrown
   * indicating that this resource has already been acquired. Otherwise, this
   * driver sets that bit. On destruction that bit is cleared to allow another
   * driver to utilize that pin in the future.
   *
   * @throws device_or_resource_busy when a pwm is already being used on a pin.
   */
  pwm_group_frequency(void* p_reg, stm32f1::peripheral p_select);

  void driver_frequency(u32 p_hertz) override;

  hal::stm32_generic::pwm_group_frequency m_pwm_frequency;
  stm32f1::peripheral m_select;
};

/**
 * @brief This class is a wrapper for the pwm class.
 * @deprecated Please use pwm16_channel and/or pwm_group_frequency. This class
 * implements the `hal::pwm` interface which will be deprecated in libhal 5.
 *
 * It manages the pwm availability of the various different pins, and keeps
 * track of whether a pwm is available or not. It inherits hal::pwm because this
 * object is returned to the timer instance and can be used as an hal::pwm in
 * any application.
 */
class pwm : public hal::pwm
{
public:
  friend class hal::stm32f1::advanced_timer_manager;

  template<hal::stm32f1::peripheral select>
  friend class hal::stm32f1::general_purpose_timer;

  pwm(pwm const& p_other) = delete;
  pwm& operator=(pwm const& p_other) = delete;
  pwm(pwm&& p_other) noexcept = default;
  pwm& operator=(pwm&& p_other) noexcept = default;
  ~pwm() override;

private:
  /**
   * @brief The pwm constructor is private because the only way one should be
   * able to access pwm is through the timer class
   *
   * @param p_reg is a void pointer that points to the beginning of a timer
   * peripheral
   * @param p_select is the timer peripheral that this instance is using.
   * @param p_is_advanced whether the current peripheral is advanced or not
   * @param p_pins This number is used to update the an internal bitmap
   * containing which PWM pins have already been acquired as well as configure
   * the pins and their channels. On construction the bit indicated by this
   * value is checked to see if it is set. If it is set, an exception is thrown
   * indicating that this resource has already been acquired. Otherwise, this
   * driver sets that bit. On destruction that bit is cleared to allow another
   * driver to utilize that pin in the future.
   *
   * @throws device_or_resource_busy when a pwm is already being used on a pin.
   */
  pwm(void* p_reg,
      stm32f1::peripheral p_select,
      bool p_is_advanced,
      stm32f1::timer_pins p_pin);

  void driver_frequency(hertz p_frequency) override;
  void driver_duty_cycle(float p_duty_cycle) override;

  hal::stm32_generic::pwm m_pwm;
  hal::stm32_generic::pwm_group_frequency m_pwm_frequency;
  hal::u16 m_pin_num;
  stm32f1::peripheral m_select;
};

/**
 * @brief This template class takes can do any timer operation for timers 1
 * and 8.
 *
 * The peripheral ID is the template argument, in order to ensure
 * that the pins used correspond to the correct timer instantiation as well as
 * the correct corresponding pins at compile time.
 *
 * These timers can be used to do PWM generation, as well as other advanced
 * timer specific tasks
 */
class advanced_timer_manager
{
public:
  advanced_timer_manager(advanced_timer_manager const& p_other) = delete;
  advanced_timer_manager& operator=(advanced_timer_manager const& p_other) =
    delete;
  advanced_timer_manager(advanced_timer_manager&& p_other) noexcept = delete;
  advanced_timer_manager& operator=(advanced_timer_manager&& p_other) noexcept =
    delete;

  ~advanced_timer_manager();

protected:
  advanced_timer_manager(peripheral p_id);

  [[nodiscard]] hal::stm32f1::pwm acquire_pwm(timer_pins p_pin);
  [[nodiscard]] hal::stm32f1::pwm16_channel acquire_pwm16_channel(
    timer_pins p_pin);
  [[nodiscard]] hal::stm32f1::pwm_group_frequency acquire_pwm_group_frequency();

  peripheral m_id;
};

template<peripheral select>
class advanced_timer final : public advanced_timer_manager
{
  static_assert(
    select == peripheral::timer1 or select == peripheral::timer8,
    "Only timer 1 or 8 is allowed as advanced timers for this driver.");
  using pin_type = decltype(get_pwm_timer_type<select>())::type;

  advanced_timer()
    : advanced_timer_manager(select)
  {
  }

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
  [[nodiscard]] hal::stm32f1::pwm acquire_pwm(pin_type p_pin)
  {
    return advanced_timer_manager::acquire_pwm(static_cast<timer_pins>(p_pin));
  }

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
    pin_type p_pin)
  {
    return advanced_timer_manager::acquire_pwm16_channel(
      static_cast<timer_pins>(p_pin));
  }
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
  ~general_purpose_timer();

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
  [[nodiscard]] hal::stm32f1::pwm_group_frequency acquire_pwm_group_frequency();
};

}  // namespace hal::stm32f1
