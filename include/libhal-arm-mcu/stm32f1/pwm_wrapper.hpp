#pragma once

#include <libhal-arm-mcu/stm32_generic/pwm.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal/pwm.hpp>
#include <libhal/units.hpp>

namespace hal::stm32f1 {
enum class pins : u8;

template<hal::stm32f1::peripheral select>
class advanced_timer;

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
  template<hal::stm32f1::peripheral select>
  friend class hal::stm32f1::advanced_timer;

  template<hal::stm32f1::peripheral select>
  friend class hal::stm32f1::general_purpose_timer;

  pwm16_channel(pwm16_channel const& p_other) = delete;
  pwm16_channel& operator=(pwm16_channel const& p_other) = delete;
  pwm16_channel(pwm16_channel&& p_other) noexcept = default;
  pwm16_channel& operator=(pwm16_channel&& p_other) noexcept = default;
  ~pwm16_channel();

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
                stm32f1::pins p_pin);

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
  template<hal::stm32f1::peripheral select>
  friend class hal::stm32f1::advanced_timer;

  template<hal::stm32f1::peripheral select>
  friend class hal::stm32f1::general_purpose_timer;

  pwm_group_frequency(pwm_group_frequency const& p_other) = delete;
  pwm_group_frequency& operator=(pwm_group_frequency const& p_other) = delete;
  pwm_group_frequency(pwm_group_frequency&& p_other) noexcept = default;
  pwm_group_frequency& operator=(pwm_group_frequency&& p_other) noexcept =
    default;
  ~pwm_group_frequency() = default;

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
  template<hal::stm32f1::peripheral select>
  friend class hal::stm32f1::advanced_timer;

  template<hal::stm32f1::peripheral select>
  friend class hal::stm32f1::general_purpose_timer;

  pwm(pwm const& p_other) = delete;
  pwm& operator=(pwm const& p_other) = delete;
  pwm(pwm&& p_other) noexcept = default;
  pwm& operator=(pwm&& p_other) noexcept = default;
  ~pwm();

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
      stm32f1::pins p_pin);

  void driver_frequency(hertz p_frequency) override;
  void driver_duty_cycle(float p_duty_cycle) override;

  hal::stm32_generic::pwm m_pwm;
  hal::stm32_generic::pwm_group_frequency m_pwm_frequency;
  hal::u16 m_pin_num;
  stm32f1::peripheral m_select;
};
}  // namespace hal::stm32f1
