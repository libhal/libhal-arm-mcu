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

#include <libhal/adc.hpp>
#include <libhal/initializers.hpp>
#include <libhal/units.hpp>

namespace hal::stm32f1 {
/**
 * @brief Analog to digital converter
 *
 */
class adc final : public hal::adc
{
public:
  /**
   * @brief Defines the pins which can be used for analog input for the adc
   */
  enum class pins : hal::u8
  {
    pa0 = 0,
    pa1 = 1,
    pa2 = 2,
    pa3 = 3,
    pa4 = 4,
    pa5 = 5,
    pa6 = 6,
    pa7 = 7,
    pb0 = 8,
    pb1 = 9,
    pc0 = 10,
    pc1 = 11,
    pc2 = 12,
    pc3 = 13,
    pc4 = 14,
    pc5 = 15,
  };

  /**
   * @brief Construct an adc object based on the passed in pin. Note: Each adc
   * object is tied to one pin, so to add more pins you need to create more
   * objects.
   *
   * @param p_pin - Which pin to use. Note: The enum members in "pins" are the
   * only pins capable of analog input for the ADC.
   */
  adc(pins const& p_pin);

  adc(adc const& p_other) = delete;
  adc& operator=(adc const& p_other) = delete;
  adc(adc&& p_other) noexcept = delete;
  adc& operator=(adc&& p_other) noexcept = delete;
  virtual ~adc() = default;

private:
  float driver_read() override;

  pins m_pin;
};
}  // namespace hal::stm32f1
