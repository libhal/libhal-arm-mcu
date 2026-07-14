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

#include <libhal/io_waiter.hpp>
#include <libhal/spi.hpp>
#include <libhal/units.hpp>

#include "constants.hpp"
#include "pin.hpp"

namespace hal::lpc40 {
class dma_spi final : public hal::spi
{
public:
  /// Information used to configure the spi bus
  struct bus_info
  {
    /// peripheral id used to power on the spi peripheral at creation
    peripheral peripheral_id;
    /// spi data pin
    pin clock;
    /// spi clock pin
    pin data_out;
    /// spi clock pin
    pin data_in;
    /// clock function code
    u8 clock_function;
    /// scl pin function code
    u8 data_out_function;
    /// scl pin function code
    u8 data_in_function;
  };

  /**
   * @brief Construct a new spi object
   *
   * @param p_bus - bus number to use
   * @param p_waiter - provides the implementation strategy for the time the CPU
   * waits until the transfer has completed.
   * @param p_settings - spi settings to achieve
   * @throws hal::operation_not_supported - if the p_bus is not 0, 1, or 2 or if
   * the spi settings could not be achieved.
   */
  dma_spi(u8 p_bus,
          hal::io_waiter& p_waiter = hal::polling_io_waiter(),
          spi::settings const& p_settings = {});
  /**
   * @brief Construct a new spi object using bus info directly
   *
   * @param p_bus - Full bus information
   */
  dma_spi(bus_info p_bus);

  dma_spi(dma_spi const& p_other) = delete;
  dma_spi& operator=(dma_spi const& p_other) = delete;
  dma_spi(dma_spi&& p_other) noexcept = delete;
  dma_spi& operator=(dma_spi&& p_other) noexcept = delete;
  ~dma_spi() override;

private:
  void driver_configure(settings const& p_settings) override;
  void driver_transfer(std::span<hal::byte const> p_data_out,
                       std::span<hal::byte> p_data_in,
                       hal::byte p_filler) override;

  hal::io_waiter* m_io_waiter;
  bus_info m_bus;
};
}  // namespace hal::lpc40
