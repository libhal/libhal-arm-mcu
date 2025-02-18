#pragma once

#include "libhal-arm-mcu/stm32_generic/pwm.hpp"
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal/units.hpp>
namespace hal::stm32f1 {
template<hal::stm32f1::peripheral select>
class advanced_timer;

template<hal::stm32f1::peripheral select>
class general_purpose_timer;

class pwm_wrapper : public hal::pwm
{
public:
  pwm_wrapper(pwm_wrapper const& p_other) = delete;
  pwm_wrapper& operator=(pwm_wrapper const& p_other) = delete;
  pwm_wrapper(pwm_wrapper&& p_other) noexcept = delete;
  pwm_wrapper& operator=(pwm_wrapper&& p_other) noexcept = delete;

  ~pwm_wrapper();
  template<hal::stm32f1::peripheral select>
  friend class hal::stm32f1::advanced_timer;

  template<hal::stm32f1::peripheral select>
  friend class hal::stm32f1::general_purpose_timer;

private:
  /**
   * @brief The pwm constructor is private because the only way one should be
   * able to access pwm is through the timer class
   */
  pwm_wrapper(void* p_reg,
              int p_channel,
              hertz p_clock_freq,
              bool is_advanced,
              hal::u16 pin_num);
  // frequency
  void driver_frequency(hertz p_frequency) override;
  // duty cycle
  void driver_duty_cycle(float p_duty_cycle) override;
  // destructor
  hal::stm32_generic::pwm m_pwm;
  /**
   * @brief The same pin could be accessed from different timer classes, and
   * to prevent when a PWM is acquired, we keep track of all pwms that are
   * acquired at any given moment
   */
  hal::u16 m_pin_num;
  static hal::u16 m_availability;
};
}  // namespace hal::stm32f1
