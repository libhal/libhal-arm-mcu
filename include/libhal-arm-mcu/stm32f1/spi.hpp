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

#include <libhal-arm-mcu/stm32_generic/spi.hpp>
#include <libhal/initializers.hpp>
#include <libhal/spi.hpp>

#include "constants.hpp"

namespace hal::stm32f1 {
class spi : public hal::spi
{
public:
  /**
   * @brief Construct a new spi driver
   *
   * @param p_bus - spi bus number from 1 to 3
   * @param p_settings - initial spi settings
   */
  spi(hal::bus_param auto p_bus, spi::settings const& p_settings = {})
    : spi(p_bus(), p_settings)
  {
    static_assert(1 <= p_bus() and p_bus() <= 3,
                  "stm32f1 only supports ports from 1 to 3");
  }

  spi(spi& p_other) = delete;
  spi& operator=(spi& p_other) = delete;
  spi(spi&& p_other) noexcept = delete;
  spi& operator=(spi&& p_other) noexcept = delete;
  ~spi() override;

private:
  spi(std::uint8_t p_bus_number, spi::settings const& p_settings = {});

  void driver_configure(settings const& p_settings) override;
  void driver_transfer(std::span<hal::byte const> p_data_out,
                       std::span<hal::byte> p_data_in,
                       hal::byte p_filler) override;

  peripheral m_peripheral_id;
  stm32_generic::spi m_spi_driver;
};
}  // namespace hal::stm32f1
