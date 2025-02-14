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
#include <libhal-util/bit.hpp>
#include <libhal/pwm.hpp>
#include <libhal/units.hpp>

namespace hal::stm32f1 {

class pwm final : public hal::pwm
{
public:
  /**
   * @brief Default pins for the various pwm channels
   */
  enum class pins : std::uint8_t
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

  pwm(pwm const& p_other) = delete;
  pwm& operator=(pwm const& p_other) = delete;
  pwm(pwm&& p_other) noexcept = delete;
  pwm& operator=(pwm&& p_other) noexcept = delete;
  /**
   * @brief when it is destroyed the corresponding
   * peripheral is powered off
   */
  ~pwm() noexcept;

private:
  /**
   * @brief The pwm constructor is private because the only way one should be
   * able to access pwm is through the timer class
   */
  pwm(pins pwm_pin);

  void driver_frequency(hertz p_frequency) override;
  void driver_duty_cycle(float p_duty_cycle) override;

  uint32_t volatile* m_compare_register_addr;
  pins m_pin;
  peripheral m_peripheral_id;
  friend class general_purpose_timer;
  friend class advanced_timer;
  /**
   * @brief The same pin could be accessed from different timer classes, and
   * to prevent when a PWM is acquired, we keep track of all pwms that are
   * acquired at any given moment
   */
  static hal::u16 availability;
};

class advanced_timer
{
public:
  /**
   * @brief This class takes in any Pin capable of performing PWM with timers
   * 1,8. These timers are more capable than general purpose timers.
   */
  advanced_timer(peripheral p_peripheral);

  /**
   * @brief Only one PWM channel is allowed. If someone tries to acquire it
   * again, the same channel won't be available until the destructor is
   * called.
   */
  pwm acquire_pwm(pwm::pins p_pin);
};

class general_purpose_timer
{
public:
  /**
   * @brief This class takes in any Pin capable of performing PWM with timers
   * 2,3,4. This timer can do things like input capture, PWM, as well as
   * encoder readings
   */
  general_purpose_timer(peripheral p_peripheral);

  /**
   * @brief Only one PWM channel is allowed. If someone tries to acquire it
   * again, the same channel won't be available until the destructor is
   * called.
   */
  pwm acquire_pwm(pwm::pins p_pin);
};

}  // namespace hal::stm32f1
