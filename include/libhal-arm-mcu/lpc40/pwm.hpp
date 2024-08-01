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

#include <cstdint>

#include <libhal/pwm.hpp>

#include "constants.hpp"
#include "pin.hpp"

namespace hal::lpc40 {
/**
 * @brief pwm driver for the lpc40xx series of micro controllers
 *
 * This driver uses the dedicated PWM peripherals, PWM0, and PWM1 for generating
 * pwm signals. Other methods would include using timers and the MotorPWM
 * peripherals.
 *
 * NOTE: Channels within a PWM peripheral device are NOT independent. Meaning
 * that changing the frequency of one channel changes the frequency for all
 * channels within the peripheral block.
 *
 */
class pwm final : public hal::pwm
{
public:
  /// Channel specific information
  struct channel
  {
    /// peripheral id used to power on the pwm peripheral at creation
    peripheral peripheral_id;
    /// Pin to output pwm from
    pin pwm_pin;
    /// Channel index
    uint8_t index;
    /// Pin function code
    uint8_t pin_function;
  };

  /**
   * @brief Construct a new pwm object
   *
   * @param p_peripheral - Peripheral block, either 0 or 1
   * @param p_channel - PWM output channel within the peripheral block, from 1
   * to 6.
   */
  pwm(std::uint8_t p_peripheral, std::uint8_t p_channel);

  pwm(pwm const& p_other) = delete;
  pwm& operator=(pwm const& p_other) = delete;
  pwm(pwm&& p_other) noexcept = delete;
  pwm& operator=(pwm&& p_other) noexcept = delete;
  virtual ~pwm() = default;

private:
  void driver_frequency(hertz p_frequency) override;
  void driver_duty_cycle(float p_duty_cycle) override;

  channel m_channel;
};
}  // namespace hal::lpc40
