#pragma once

#include <libhal-util/bit.hpp>
#include <libhal/initializers.hpp>
#include <libhal/pwm.hpp>
#include <libhal/units.hpp>

namespace hal::stm32f1 {
class pwm_wrapper;
}  // namespace hal::stm32f1

namespace hal::stm32_generic {

struct pwm_settings
{
  /// Each Timer Peripheral has 2-4 channels, and this number allows the driver
  /// to use the correct output pin
  int channel;
  /// PWM depends on the frequency of the timer peripheral it uses.
  hertz frequency;
  /// Advanced timers work slightly different than general purpose ones.
  bool is_advanced;
};

/**
 * @brief This class should not be constructed directly.
 *
 * The user should instantiate a General Purpose or Advanced timer class first,
 * and then acquire a pwm pin through that class.
 */
class pwm final : public hal::pwm
{
public:
  friend class hal::stm32f1::pwm_wrapper;
  /**
   * @brief Construct generic stm32 pwm driver
   *
   * Care should be taken in constructing this outside of a platform specific
   * timer class. If both a platform specific timer class and this object exist,
   * care must be taken to ensure that the drivers do not conflict with each
   * others.
   *
   * @param p_reg is a void pointer that points to the beginning of a timer
   * peripheral
   * @param p_settings consist of channel number, frequency of the timer, and a
   * boolean to indicate whether the timer is advanced or not.
   */
  pwm(hal::unsafe, void* p_reg, pwm_settings p_settings);
  pwm(pwm const& p_other) = delete;
  pwm& operator=(pwm const& p_other) = delete;

  ~pwm() = default;

private:
  /**
   * @brief This constructor is constructed in pwm_wrapper.
   * The purpose of this is to send the settings through the initialize function
   * instead, because the channel calculation and pin configuration is done in
   * the contructor and the regular constructor cannot be initialized through
   * the initializer list.
   */
  pwm() = default;
  void driver_frequency(hertz p_frequency) override;
  void driver_duty_cycle(float p_duty_cycle) override;
  void initialize(void* p_reg, pwm_settings p_settings);

  void* m_reg;
  u32 volatile* m_compare_register_addr;
  hertz m_clock_freq;
};
}  // namespace hal::stm32_generic
