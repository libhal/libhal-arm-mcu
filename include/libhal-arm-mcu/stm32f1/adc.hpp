// Copyright 2024 Khalil Estell
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LIC`ENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <cstdint>

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
  /// Channel specific information
  struct channel
  {
    /// ADC port
    std::uint8_t port;
    /// ADC pin
    std::uint8_t pin;
    /// Channel data index
    uint8_t index;
  };

  /**
   * @brief Get a predefined adc channel
   *
   * - ADC channel 0 is PA0
   * - ADC channel 1 is PA1
   * - ADC channel 2 is PA2
   * - ADC channel 3 is PA3
   * - ADC channel 4 is PA4
   * - ADC channel 5 is PA5
   * - ADC channel 6 is PA6
   * - ADC channel 7 is PA7
   * - ADC channel 8 is PB0
   * - ADC channel 9 is PB1
   * - ADC channel 10 is PC0
   * - ADC channel 11 is PC1
   * - ADC channel 12 is PC2
   * - ADC channel 13 is PC3
   * - ADC channel 14 is PC4
   * - ADC channel 15 is PC5
   *
   * @param p_channel - which adc channel to use
   */
  adc(hal::channel_param auto p_channel)
    : adc(get_predefined_channel_info(static_cast<std::uint8_t>(p_channel())))
  {
    static_assert(0 <= p_channel() && p_channel() <= 15,
                  "Available ADC channels are from 0 to 15");
  }

  /**
   * @brief Construct a custom adc object based on the passed in channel
   * information.
   *
   * @param p_channel - Which adc channel to use
   */
  adc(channel const& p_channel);

  adc(adc const& p_other) = delete;
  adc& operator=(adc const& p_other) = delete;
  adc(adc&& p_other) noexcept = delete;
  adc& operator=(adc&& p_other) noexcept = delete;
  virtual ~adc() = default;

private:
  channel get_predefined_channel_info(std::uint8_t p_channel);
  float driver_read() override;
};
}  // namespace hal::stm32f1
