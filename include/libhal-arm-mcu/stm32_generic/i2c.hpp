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

#include <cstdint>
#include <span>

#include <libhal/i2c.hpp>
#include <libhal/initializers.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/units.hpp>

namespace hal::stm32_generic {
/**
 * @brief A generic i2c implementation for all stm32 series MCUs
 *
 * This class is meant to only be used by platform libraries or drivers aiming
 * to be platform libraries for stm32 devices. As an application develop,
 * prefer to use the platform specific drivers instead as they handle all of
 * the initialization for you.
 *
 */
class i2c
{
public:
  /**
   * @brief Construct a new i2c object
   * Care must be taken when constructing this object. The constructor does not
   * and cannot power on the i2c peripheral. Accessing the peripheral's
   * registers without it be powered on (or provided a clock), will result in a
   * memory fault as the peripheral cannot ACK the CPU when it attempts to
   * access the peripheral's registers. The power management system for each
   * peripheral across the stm32 series of MCUs is different, meaning its not
   * something that can efficiently be manged by this generic driver.
   *
   * Things that needed to be done before construction:
   *
   * 1. Configure pins to be controlled by the i2c peripheral pointed to by the
   *    `p_peripheral_address` input parameter.
   * 2. Power on the appropriate i2c peripheral
   *
   * After construction, execute the `configure()` function with the appropriate
   * peripheral frequency and i2c settings.
   *
   * Performing these in this order will properly initialize the i2c driver and
   * allow the usage of the `transfer()` API. Failing to execute these steps in
   * this order will result in UB.
   *
   * NOTE: why does this driver get away with breaking the rule about
   * construction means initialization? Because its constructor is marked unsafe
   * meaning care needs to be taken when using this.
   *
   * @param p_i2c - i2c peripheral address
   */
  i2c(void* p_i2c);
  i2c();
  /**
   * @brief Perform a transfer operation as defined in `hal::i2c::transaction`
   *
   * @param p_address - target i2c slave address
   * @param p_data_out - outgoing data
   * @param p_data_in - incoming data
   * @param p_timeout - timeout function
   */
  void transaction(hal::byte p_address,
                   std::span<hal::byte const> p_data_out,
                   std::span<hal::byte> p_data_in,
                   hal::function_ref<hal::timeout_function> p_timeout);
  /**
   * @brief Configures i2c peripheral
   *
   * Because each stm32 series MCU has its own unique clock tree, there is no
   * single, simple and space efficient may to determine the system's i2c clock
   * speed. So the caller must supply the operating frequency of the i2c
   * peripheral for this to work. This API is meant to be called by platform
   * specific i2c drivers that have the necessary context and library apis to
   * retrieve the i2c peripheral's clock rate.
   *
   * @param p_settings - i2c settings
   * @param p_frequency - peripheral operating frequency
   */
  void configure(hal::i2c::settings const& p_settings, hertz p_frequency);
  /**
   * @brief Handles i2c event interupt when using the above configuration.
   * Insert this into the appropriate i2c peripheral event interupt.
   *
   */
  void handle_i2c_event() noexcept;
  /**
   * @brief Handles i2c error interupt when using the above configuration.
   * Insert this into the appropriate i2c peripheral error interupt.
   *
   */
  void handle_i2c_error() noexcept;
  /**
   * @brief Destroy the i2c object
   *
   */
  ~i2c();

private:
  enum class error_state : hal::u8
  {
    no_error = 0,
    no_such_device,
    io_error,
    arbitration_lost,
  };
  enum class transmission_state : hal::u8
  {
    transmitter,
    reciever,
    free,
  };
  void* m_i2c;
  std::span<hal::byte const> m_data_out;
  std::span<hal::byte> m_data_in;
  error_state m_status{};
  hal::byte m_address = hal::byte{ 0x00 };
  transmission_state m_state = transmission_state::free;
};
}  // namespace hal::stm32_generic
