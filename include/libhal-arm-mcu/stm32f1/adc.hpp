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
#include <libhal/adc.hpp>
#include <libhal/lock.hpp>
#include <libhal/units.hpp>

namespace hal::stm32f1 {

/**
 * @brief Manager for the stm32f1 series' onboard adc peripheral. Used to
 * construct and manage channel objects that are tied to an adc. This also
 * applies to the MCU variants that have more than one adc available.
 *
 */
class adc_peripheral_manager final
{
public:
  // Forward declaration.
  class channel;
  /**
   * @brief Defines the pins which can be used for analog input for adc1 and
   * adc2.
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
   * @brief Defines the available adc peripherals that CAN be onboard the MCU.
   * Note that the XL-density stm32f1 MCU's only have adc1.
   *
   */
  enum class adc_selection : hal::u8
  {
    adc1 = 0,
    adc2 = 1,
  };

  /**
   * @brief Construct a new adc peripheral manager object.
   *
   * @param p_adc_selection - The specified adc peripheral to use.
   * @param p_lock - An externally declared lock to use for thread safety when
   * trying to read from the adc's. This can be a basic_lock or any type that
   * derives from it.
   */
  adc_peripheral_manager(adc_selection p_adc_selection,
                         hal::basic_lock& p_lock);

  /**
   * @brief Creates and configures an adc channel under the calling adc
   * peripheral manager.
   *
   * @param p_pin - The pin to be used for the channels analog input.
   * @return channel - object that can be read for analog input .
   */
  channel acquire_channel(pins p_pin);

private:
  /**
   * @brief Takes an analog input reading.
   *
   * @param p_pin - The pin to read from.
   * @return float - The sampled adc value.
   */
  float read_channel(pins p_pin);

  /// The lock to be used for thread safety with adc reads.
  hal::basic_lock* m_lock;
  /// A pointer to track the location of the registers for the specified adc
  /// peripheral.
  void* adc_reg_location;
};

/**
 * @brief Creates channels to be used by the adc peripheral manager to read
 * certain pins' analog input.
 *
 */
class adc_peripheral_manager::channel : public hal::adc
{
private:
  /// Gives adc_peripheral_manager access to channel's private members.
  friend class adc_peripheral_manager;

  // Constructor for channel. Needs to configure the given pin
  /**
   * @brief Constructs a channel object.
   *
   * @param p_manager - The adc peripheral manager that will be managing this
   * channel resource.
   * @param p_pin - The pin that will be used for this channels analog input.
   */
  channel(adc_peripheral_manager& p_manager,
          adc_peripheral_manager::pins p_pin);

  /**
   * @brief Takes an analog input reading.
   *
   * @return float - The sampled adc value.
   */
  float driver_read() override;

  /// The adc peripheral manager that manages this channel.
  adc_peripheral_manager* m_manager;
  /// The pin that is used for this channel.
  adc_peripheral_manager::pins m_pin;
};

}  // namespace hal::stm32f1
