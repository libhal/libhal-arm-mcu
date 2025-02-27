#pragma once

#include <libhal-util/bit.hpp>
#include <libhal/initializers.hpp>
#include <libhal/pwm.hpp>
#include <libhal/units.hpp>
#include <limits>

namespace hal::stm32_generic {

struct pwm_channel_info
{
  /// Each Timer Peripheral has 2-4 channels, and this number allows the driver
  /// to use the correct output pin
  u8 channel;
  /// Advanced timers work slightly different than general purpose ones.
  bool is_advanced;
};

struct pwm_timer_frequency
{
  /// The target frequency all PWM channels within a timer will be set to
  u32 pwm_frequency;
  /// The timer's input clock frequency
  u32 timer_clock_frequency;
};

/**
 * @brief This class should not be constructed directly.
 *
 * The user should instantiate a General Purpose or Advanced timer class first,
 * and then acquire a pwm pin through that class.
 */
class pwm_group_frequency final
{
public:
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
   */
  pwm_group_frequency(hal::unsafe, void* p_reg);

  pwm_group_frequency(pwm_group_frequency const& p_other) = delete;
  pwm_group_frequency& operator=(pwm_group_frequency const& p_other) = delete;
  pwm_group_frequency(pwm_group_frequency&& p_other) = default;
  pwm_group_frequency& operator=(pwm_group_frequency&& p_other) = default;
  ~pwm_group_frequency() = default;

  /**
   * @brief Set the frequency for all PWM channels controlled by the timer
   * specified by the `p_reg` parameter passed at construction.
   *
   * @param p_timer_frequency - desired pwm frequency and input clock frequency
   */
  void set_group_frequency(pwm_timer_frequency p_timer_frequency);

private:
  void* m_reg = nullptr;
};

/**
 * @brief This class should not be constructed directly.
 *
 * The user should instantiate a General Purpose or Advanced timer class first,
 * and then acquire a pwm pin through that class.
 */
class pwm final
{
public:
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
  pwm(hal::unsafe, void* p_reg, pwm_channel_info p_settings);

  /**
   * @brief Construct pwm uninitialized
   *
   * The purpose of this is to send the settings through the initialize function
   * instead, because the channel calculation and pin configuration is done in
   * the constructor and the regular constructor cannot be initialized through
   * the initializer list.
   *
   * If this constructor is used, it is unsafe to call any API of this class
   * before calling the `initialize()` API with the correct inputs. Once that
   * API has been called without failure, then the other APIs will become
   * available.
   */
  pwm(hal::unsafe)
  {
  }

  pwm(pwm const& p_other) = delete;
  pwm& operator=(pwm const& p_other) = delete;
  pwm(pwm&& p_other) = default;
  pwm& operator=(pwm&& p_other) = default;
  ~pwm() = default;

  /**
   * @brief Initialize the driver if the `pwm(hal::unsafe)` was used
   *
   * This API must be called if the `pwm(hal::unsafe)` was used in order to
   * initialize this class.
   *
   * @param p_reg - address of the timer's peripheral
   * @param p_settings - pwm settings for the timer peripheral
   */
  void initialize(hal::unsafe, void* p_reg, pwm_channel_info p_settings);

  /**
   * @brief Acquire the operating frequency of the PWM channel
   *
   * @param p_timer_clock_frequency - the clock frequency driving the timer
   * peripheral.
   * @return u32 - the frequency of the PWM signal
   */
  u32 frequency(u32 p_timer_clock_frequency);

  /**
   * @brief Set pwm channel duty cycle using u16 value from 0x0000 to 0xFFFF
   *
   * @param p_duty_cycle - pwm duty cycle proportional value from 0x0000 to
   * 0xFFFF.
   */
  void duty_cycle(u16 p_duty_cycle);

  /**
   * @brief Set the duty cycle using a float
   *
   * @param p_duty_cycle - value from
   */
  inline void duty_cycle(float p_duty_cycle)
  {
    constexpr auto u16_max = std::numeric_limits<u16>::max();
    auto const clamped_duty_cycle = std::clamp(p_duty_cycle, 0.0f, 1.0f);
    auto const u16_value = static_cast<u16>(clamped_duty_cycle * u16_max);
    duty_cycle(u16_value);
  }

private:
  void* m_reg = nullptr;
  u32 volatile* m_compare_register_addr = nullptr;
};
}  // namespace hal::stm32_generic
