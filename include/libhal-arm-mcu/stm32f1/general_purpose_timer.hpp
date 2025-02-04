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

#include <bitset>
#include <cstdint>

#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-util/bit.hpp>
#include <libhal/pwm.hpp>

namespace hal::stm32f1 {
/** @brief This class takes in any Pin capable of performing PWM with timers
 * 1,2,3,4,8. The output of this pin will generate PWM at required frequency and
 * duty cycle.
 */
class general_purpose_timer
{
public:
  class pwm final : public hal::pwm
  {
  public:
    /**
     * @brief Default pins for the various pwm channels
     */
    enum class pwm_pins : std::uint8_t
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
    pwm(pwm_pins pwm_pin);

    pwm(pwm const& p_other) = delete;
    pwm& operator=(pwm const& p_other) = delete;
    pwm(pwm&& p_other) noexcept = delete;
    pwm& operator=(pwm&& p_other) noexcept = delete;
    ~pwm();  // when it is destroyed the corresponding peripheral is powered off

  private:
    void driver_frequency(hertz p_frequency) override;
    void driver_duty_cycle(float p_duty_cycle) override;

    uint32_t volatile* m_compare_register_addr;
    pwm_pins m_pin;
  };

  general_purpose_timer(peripheral p_peripheral);

  pwm acquire_pwm(
    pwm::pwm_pins p_pin);  // only one pwm channel is allowed, if someone tries
                           // to acquire it again, the same channel won't be
                           // available until destructor is called

  static std::bitset<16> pwm_availability;
};

}  // namespace hal::stm32f1
