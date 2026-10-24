// Copyright 2026 Khalil Estell and the libhal contributors
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

export module hal.arm_mcu.stm32f1:spi;

import hal;
import hal.util;
import hal.arm_mcu.stm32_generic;

import :constants;

namespace hal::stm32f1 {

/**
 * @brief STM32F1 SPI bus
 *
 * Owns one of the STM32F1's SPI hardware blocks and its clock, CIPO and COPI
 * pins, and keeps it powered for as long as it lives. Only `spi1`, `spi2`,
 * and `spi3` are supported.
 *
 * It has no chip select. To share the bus between devices, give it to a
 * `hal::spi_bus_manager`, which hands out a `hal::spi_channel` per device.
 * Built inside the manager from a factory, the bus can't be used behind the
 * manager's back:
 *
 * ```cpp
 * static hal::no_op_lock single_context;
 * static hal::spi_bus_manager spi1(
 *   [] { return hal::stm32f1::spi(hal::stm32f1::peripheral::spi1); },
 *   single_context);
 * static hal::stm32f1::output_pin flash_select({ .port = 'A', .pin = 4 });
 * static auto flash = spi1.acquire_channel(flash_select);
 * ```
 *
 * The peripheral's hardware chip select (NSS) is left unused.
 */
export class spi final : public hal::raw_spi_bus
{
public:
  /**
   * @brief Claim an SPI peripheral and its pins, power it on, and reset it
   *
   * Claims and configures the clock, CIPO and COPI pins: PA5, PA6, PA7 for
   * `spi1`, PB13, PB14, PB15 for `spi2`, and PB3, PB4, PB5 for `spi3`. Using
   * `spi3` releases those pins from JTAG.
   *
   * The peripheral is powered off and released, and its pins are reset and
   * released, when the bus is destroyed.
   *
   * @param p_id - which SPI peripheral to manage. Must be `spi1`, `spi2`, or
   * `spi3`.
   * @throws hal::operation_not_supported - if `p_id` is not `spi1`, `spi2`,
   * or `spi3`.
   * @throws hal::device_or_resource_busy - if another driver has claimed the
   * peripheral or one of its pins.
   */
  explicit spi(peripheral p_id);

  spi(spi const&) = delete;
  spi& operator=(spi const&) = delete;
  spi(spi&&) = delete;
  spi& operator=(spi&&) = delete;

  /// Power off the peripheral and release its pins
  ~spi();

private:
  void driver_configure(hal::spi_settings const& p_settings) override;
  hal::hertz driver_clock_rate(hal::spi_settings const& p_settings) override;
  void driver_transfer(mem::scatter_span<hal::byte const> p_data_out,
                       mem::scatter_span<hal::byte> p_data_in,
                       hal::byte p_filler) override;

  peripheral m_id;
  hal::stm32_generic::spi m_driver;
};
}  // namespace hal::stm32f1
