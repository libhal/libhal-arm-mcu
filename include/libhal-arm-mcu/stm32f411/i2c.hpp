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

#include <libhal-arm-mcu/stm32_generic/i2c.hpp>
#include <libhal/i2c.hpp>
#include <libhal/initializers.hpp>

#include "constants.hpp"

namespace hal::stm32f411 {

/**
 * @brief Implementation of the i2c port manager class
 *
 * This class has a private constructor and can only be used via its derived
 * class `i2c<peripheral>` class.
 *
 */
class i2c_manager_impl
{
public:
  template<peripheral select>
  friend class i2c_manager;

  i2c_manager_impl(i2c_manager_impl&) = delete;
  i2c_manager_impl& operator=(i2c_manager_impl&) = delete;
  i2c_manager_impl(i2c_manager_impl&&) noexcept = default;
  i2c_manager_impl& operator=(i2c_manager_impl&&) noexcept = default;
  /**
   * @brief Destroy the i2c port manager object
   *
   */
  ~i2c_manager_impl();

  class i2c;

  i2c acquire_i2c(hal::i2c::settings const& p_settings = {});

private:
  i2c_manager_impl(peripheral p_select);
  peripheral m_port;
};

/**
 * @brief i2c driver for the stm32f411 series of microcontrollers
 *
 * The stm32f411 series i2c peripherals utilize a state machine and interrupts
 * to handle transmitting and receiving data. This driver does the same. A
 * `hal::io_waiter` may be passed to the constructor in order to control what
 * the driver does when its waiting for the i2c transaction to complete.
 */
template<peripheral select>
class i2c_manager final : public i2c_manager_impl
{
public:
  static_assert(select == peripheral::i2c1 or /* line break */
                  select == peripheral::i2c2 or select == peripheral::i2c3,
                "Only peripheral i2c(1 to 3) is allowed for this class");
  i2c_manager()
    : i2c_manager_impl(select)
  {
  }
};

class i2c_manager_impl::i2c final : public hal::i2c
{
public:
  /**
   * @brief Construct a new i2c object
   *
   * Will enable internal pull up resistors to make sure the signals don't drop
   * low. These pull ups should be taken into consideration but not be the only
   * pull ups on the line
   *
   * @param p_manager - i2c bus manager
   * @param p_settings - i2c setting
   * @throws hal::operation_not_supported - if the settings or if the bus number
   * @brief Construct a new i2c object
   *
   */
  i2c(i2c_manager_impl& p_manager, i2c::settings const& p_settings = {});
  i2c(i2c const& p_other) = delete;
  i2c& operator=(i2c const& p_other) = delete;
  i2c(i2c&& p_other) noexcept = delete;
  i2c& operator=(i2c&& p_other) noexcept = delete;
  ~i2c() override;

private:
  void driver_configure(settings const& p_settings) override;
  void driver_transaction(
    hal::byte p_address,
    std::span<hal::byte const> p_data_out,
    std::span<hal::byte> p_data_in,
    hal::function_ref<hal::timeout_function> p_timeout) override;
  void setup_interrupt();
  i2c_manager_impl* m_manager;
  stm32_generic::i2c m_i2c;
};
}  // namespace hal::stm32f411
