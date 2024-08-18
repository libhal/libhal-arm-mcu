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
#include <cstdint>

#include <libhal-arm-mcu/stm32f1/clock.hpp>
#include <libhal-arm-mcu/stm32f1/pin.hpp>
#include <libhal-arm-mcu/stm32f1/spi.hpp>
#include <libhal-util/bit.hpp>
#include <libhal/error.hpp>

#include "pin.hpp"
#include "power.hpp"

namespace hal::stm32f1 {
namespace {
inline void* peripheral_to_address(peripheral p_id)
{
  constexpr std::uintptr_t spi_reg1 = 0x4001'3000;
  constexpr std::uintptr_t spi_reg2 = 0x4000'3800;
  constexpr std::uintptr_t spi_reg3 = 0x4000'3C00;

  switch (p_id) {
    case peripheral::spi1:
      return reinterpret_cast<void*>(spi_reg1);
    case peripheral::spi2:
      return reinterpret_cast<void*>(spi_reg2);
    case peripheral::spi3:
      return reinterpret_cast<void*>(spi_reg3);
    default:
      hal::safe_throw(hal::operation_not_supported(nullptr));
  }
}

inline peripheral bus_number_to_peripheral(std::uint8_t p_bus_number)
{
  switch (p_bus_number) {
    case 1:
      return peripheral::spi1;
    case 2:
      return peripheral::spi2;
    case 3:
      return peripheral::spi3;
    default:
      hal::safe_throw(hal::operation_not_supported(nullptr));
  }
}
}  // namespace

spi::spi(std::uint8_t p_bus_number, spi::settings const& p_settings)
  : m_peripheral_id(bus_number_to_peripheral(p_bus_number))
  , m_spi_driver(hal::unsafe{}, peripheral_to_address(m_peripheral_id))
{
  // Datasheet: Chapter 4: Pin definition Table 9
  switch (m_peripheral_id) {
    case peripheral::spi1: {
      // clock
      configure_pin({ .port = 'A', .pin = 5 }, push_pull_alternative_output);
      // cipo
      configure_pin({ .port = 'A', .pin = 6 }, input_float);
      // copi
      configure_pin({ .port = 'A', .pin = 7 }, push_pull_alternative_output);
      hal::bit_modify(alternative_function_io->mapr)
        .clear<pin_remap::spi1_remap>();
      break;
    }
    case peripheral::spi2: {
      // clock
      configure_pin({ .port = 'B', .pin = 13 }, push_pull_alternative_output);
      // cipo
      configure_pin({ .port = 'B', .pin = 14 }, input_float);
      // copi
      configure_pin({ .port = 'B', .pin = 15 }, push_pull_alternative_output);
      break;
    }
    case peripheral::spi3: {
      // clock
      configure_pin({ .port = 'C', .pin = 10 }, push_pull_alternative_output);
      // cipo
      configure_pin({ .port = 'C', .pin = 11 }, input_float);
      // copi
      configure_pin({ .port = 'C', .pin = 12 }, push_pull_alternative_output);
      hal::bit_modify(alternative_function_io->mapr2)
        .clear<pin_remap2::spi3_remap>();
      break;
    }
    default:
      // "Supported spi busses are 1-5!";
      hal::safe_throw(hal::operation_not_supported(this));
  }

  power_on(m_peripheral_id);
  spi::driver_configure(p_settings);
}

spi::~spi()
{
  power_off(m_peripheral_id);
}

void spi::driver_configure(settings const& p_settings)
{
  m_spi_driver.configure(p_settings, frequency(m_peripheral_id));
}

void spi::driver_transfer(std::span<hal::byte const> p_data_out,
                          std::span<hal::byte> p_data_in,
                          hal::byte p_filler)
{
  m_spi_driver.transfer(p_data_out, p_data_in, p_filler);
}
}  // namespace hal::stm32f1
