#pragma once

#include "libhal-arm-mcu/stm32_generic/pwm.hpp"
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal/units.hpp>

namespace hal::stm32f1 {
template<hal::stm32f1::peripheral select>
class advanced_timer;

template<hal::stm32f1::peripheral select>
class general_purpose_timer;

/** @brief This class is a wrapper for the pwm class.
 *
 * It manages the pwm availability of the various different pins, and keeps
 * track of whether a pwm is available or not. It inherits hal::pwm because this
 * object is returned to the timer instance and can be used as an hal::pwm in
 * any application.
 */
class pwm_wrapper : public hal::pwm
{
public:
  template<hal::stm32f1::peripheral select>
  friend class hal::stm32f1::advanced_timer;

  template<hal::stm32f1::peripheral select>
  friend class hal::stm32f1::general_purpose_timer;

  pwm_wrapper(pwm_wrapper const& p_other) = delete;
  pwm_wrapper& operator=(pwm_wrapper const& p_other) = delete;
  pwm_wrapper(pwm_wrapper&& p_other) noexcept = delete;
  pwm_wrapper& operator=(pwm_wrapper&& p_other) noexcept = delete;

  ~pwm_wrapper();

private:
  /**
   * @brief Static variable to track PWM availability.
   */
  static hal::u16 m_availability;
  /**
   * @brief The pwm constructor is private because the only way one should be
   * able to access pwm is through the timer class
   *
   * @param pwm_pin is a void pointer that points to the beginning of a timer
   * peripheral
   * @param settings consist of channel number, frequency of the timer, and a
   * boolean to indicate whether the timer is advanced or not.
   *
   * @throws device_or_resource_busy when a pwm is already being used on it.
   */
  pwm_wrapper(void* p_reg,
              stm32_generic::pwm_settings settings,
              hal::u16 pin_num);

  void driver_frequency(hertz p_frequency) override;
  void driver_duty_cycle(float p_duty_cycle) override;

  hal::stm32_generic::pwm m_pwm;
  /**
   * @brief Pin number associated with the PWM.
   *
   * The same pin could be accessed from different timer classes. To prevent
   * conflicts when a PWM is acquired, we keep track of all PWMs that are
   * acquired at any given moment using m_availability.
   */
  hal::u16 m_pin_num;
};
}  // namespace hal::stm32f1
