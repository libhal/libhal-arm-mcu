// Copyright 2024 - 2025 Khalil Estell and the libhal contributors
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

#include <span>

#include <libhal/initializers.hpp>
#include <libhal/spi.hpp>
#include <libhal/units.hpp>

namespace hal::stm32_generic {
/**
 * @brief A generic spi implementation for all stm32f series MCUs
 *
 * This class is meant to only be used by platform libraries or drivers aiming
 * to be platform libraries for stm32f devices. As an application develop,
 * prefer to use the platform specific drivers instead as they handle all of the
 * initialization for you.
 *
 */
class spi
{
public:
  /**
   * @brief Construct a new spi object
   *
   * Care must be taken when constructing this object. The constructor does not
   * and cannot power on the spi peripheral. Accessing the peripheral's
   * registers without it be powered on (or provided a clock), will result in a
   * memory fault as the peripheral cannot ACK the CPU when it attempts to
   * access the peripheral's registers. The power management system for each
   * peripheral across the stm32 series of MCUs is different, meaning its not
   * something that can efficiently be manged by this generic driver.
   *
   * After constructing this object, in order to use this driver the program
   * must:
   *
   * 1. Configure pins to be controlled by the spi peripheral pointed to by the
   *    `p_peripheral_address` input parameter.
   * 2. Power on the appropriate spi peripheral
   * 3. Execute the `configure()` function with the appropriate peripheral
   *    frequency.
   *
   * Performing these in this order will properly initialize the spi driver and
   * allow the usage of the `transfer()` API. Failing to execute these steps in
   * this order will result in UB.
   *
   * NOTE: why does this driver get away with breaking the rule about
   * construction means initialization? Because its constructor is marked unsafe
   * meaning care needs to be taken when using this.
   *
   * @param p_peripheral_address - starting address of the spi peripheral
   */
  spi(hal::unsafe, void* p_peripheral_address);

  spi(spi& p_other) = delete;
  spi& operator=(spi& p_other) = delete;
  spi(spi&& p_other) noexcept = delete;
  spi& operator=(spi&& p_other) noexcept = delete;
  ~spi();

  /**
   * @brief Configure the SPI peripheral
   *
   * Because each stm32f series MCU has its own unique clock tree, there is no
   * single, simple and space efficient may to determine the system's spi clock
   * speed. So the caller must supply the operating frequency of the spi
   * peripheral for this to work. This API is meant to be called by platform
   * specific spi drivers that have the necessary context and library apis to
   * retrieve the spi peripheral's clock rate.
   *
   * @param p_settings - spi settings
   * @param p_peripheral_clock_speed - peripheral's operating clock rate
   */
  void configure(hal::spi::settings const& p_settings,
                 hal::hertz p_peripheral_clock_speed);

  /**
   * @brief Perform a transfer operation as defined in `hal::spi::transfer`
   *
   * @param p_data_out - outgoing data
   * @param p_data_in - incoming data
   * @param p_filler - output filler bytes if the outgoing data runs out before
   * the incoming data.
   */
  void transfer(std::span<hal::byte const> p_data_out,
                std::span<hal::byte> p_data_in,
                hal::byte p_filler);

private:
  void* m_peripheral_address;
};
}  // namespace hal::stm32_generic
