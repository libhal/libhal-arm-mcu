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

namespace hal::stm32f1 {
class pwm final : public hal::pwm
{
public:
  pwm(pwm_pins pwm_pin);

  pwm(pwm const& p_other) = delete;  // deletes the copy constructor
  pwm& operator=(pwm const& p_other) = delete;
  pwm(pwm&& p_other) noexcept = delete;
  pwm& operator=(pwm&& p_other) noexcept = delete;
  virtual ~pwm() = default;

private:
  void driver_frequency(hertz p_frequency) override;
  void driver_duty_cycle(float p_duty_cycle) override;

  pwm_pins m_pin;
  uint8_t m_channel;
  uint32_t volatile* m_compare_register_addr;
};
}  // namespace hal::stm32f1
