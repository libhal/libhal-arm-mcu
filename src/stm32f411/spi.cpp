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

#include <libhal-arm-mcu/stm32f411/constants.hpp>
#include <libhal-arm-mcu/stm32f411/pin.hpp>
#include <libhal-arm-mcu/stm32f411/spi.hpp>
#include <libhal/error.hpp>

#include "power.hpp"

namespace hal::stm32f411 {
namespace {
void* peripheral_to_address(peripheral p_id)
{
  constexpr std::uintptr_t apb1_base = 0x4000'0000UL;
  constexpr std::uintptr_t apb2_base = 0x4001'0000UL;
  constexpr std::uintptr_t spi_reg1 = apb2_base + 0x3000;
  constexpr std::uintptr_t spi_reg2 = apb1_base + 0x3800;
  constexpr std::uintptr_t spi_reg3 = apb1_base + 0x3C00;
  constexpr std::uintptr_t spi_reg4 = apb2_base + 0x3400;
  constexpr std::uintptr_t spi_reg5 = apb2_base + 0x5000;

  switch (p_id) {
    case peripheral::spi1:
      // NOLINTNEXTLINE(performance-no-int-to-ptr)
      return reinterpret_cast<void*>(spi_reg1);
    case peripheral::spi2:
      // NOLINTNEXTLINE(performance-no-int-to-ptr)
      return reinterpret_cast<void*>(spi_reg2);
    case peripheral::spi3:
      // NOLINTNEXTLINE(performance-no-int-to-ptr)
      return reinterpret_cast<void*>(spi_reg3);
    case peripheral::spi4:
      // NOLINTNEXTLINE(performance-no-int-to-ptr)
      return reinterpret_cast<void*>(spi_reg4);
    case peripheral::spi5:
      // NOLINTNEXTLINE(performance-no-int-to-ptr)
      return reinterpret_cast<void*>(spi_reg5);
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
    case 4:
      return peripheral::spi4;
    case 5:
      return peripheral::spi5;
    default:
      hal::safe_throw(hal::operation_not_supported(nullptr));
  }
}
}  // namespace

spi::spi(hal::runtime,
         std::uint8_t p_bus_number,
         spi::settings const& p_settings)
  : m_peripheral_id(bus_number_to_peripheral(p_bus_number))
  , m_spi_driver(hal::unsafe{}, peripheral_to_address(m_peripheral_id))
{
  // Datasheet: Chapter 4: Pin definition Table 9
  switch (m_peripheral_id) {
    case peripheral::spi1: {
      pin clock(peripheral::gpio_a, 5);
      pin data_in(peripheral::gpio_a, 7);
      pin data_out(peripheral::gpio_a, 6);

      clock.function(pin::pin_function::alternate5)
        .open_drain(false)
        .resistor(pin_resistor::none);
      data_in.function(pin::pin_function::alternate5)
        .open_drain(false)
        .resistor(pin_resistor::none);
      data_out.function(pin::pin_function::alternate5)
        .open_drain(false)
        .resistor(pin_resistor::none);
      break;
    }
    case peripheral::spi2: {
      pin clock(peripheral::gpio_b, 10);
      pin data_in(peripheral::gpio_b, 14);
      pin data_out(peripheral::gpio_b, 15);

      clock.function(pin::pin_function::alternate5)
        .open_drain(false)
        .resistor(pin_resistor::none);
      data_in.function(pin::pin_function::alternate5)
        .open_drain(false)
        .resistor(pin_resistor::none);
      data_out.function(pin::pin_function::alternate5)
        .open_drain(false)
        .resistor(pin_resistor::none);
      break;
    }
    case peripheral::spi3: {
      pin clock(peripheral::gpio_b, 3);
      pin data_in(peripheral::gpio_b, 5);
      pin data_out(peripheral::gpio_b, 4);

      clock.function(pin::pin_function::alternate7)
        .open_drain(false)
        .resistor(pin_resistor::none);
      data_in.function(pin::pin_function::alternate7)
        .open_drain(false)
        .resistor(pin_resistor::none);
      data_out.function(pin::pin_function::alternate7)
        .open_drain(false)
        .resistor(pin_resistor::none);
      break;
    }
    case peripheral::spi4: {
      pin clock(peripheral::gpio_b, 0);
      pin data_in(peripheral::gpio_b, 8);
      pin data_out(peripheral::gpio_a, 12);

      clock.function(pin::pin_function::alternate6)
        .open_drain(false)
        .resistor(pin_resistor::none);
      data_in.function(pin::pin_function::alternate6)
        .open_drain(false)
        .resistor(pin_resistor::none);
      data_out.function(pin::pin_function::alternate6)
        .open_drain(false)
        .resistor(pin_resistor::none);
      break;
    }
    case peripheral::spi5: {
      pin clock(peripheral::gpio_b, 3);
      pin data_in(peripheral::gpio_b, 5);
      pin data_out(peripheral::gpio_b, 4);

      clock.function(pin::pin::pin_function::alternate6)
        .open_drain(false)
        .resistor(pin_resistor::none);
      data_in.function(pin::pin::pin_function::alternate6)
        .open_drain(false)
        .resistor(pin_resistor::none);
      data_out.function(pin::pin_function::alternate6)
        .open_drain(false)
        .resistor(pin_resistor::none);
      break;
    }
    default:
      // "Supported spi busses are 1-5!";
      hal::safe_throw(hal::operation_not_supported(this));
  }

  power(m_peripheral_id).on();
  spi::driver_configure(p_settings);
}

spi::~spi()
{
  power(m_peripheral_id).off();
}

void spi::driver_configure(settings const& p_settings)
{
  using namespace hal::literals;
  // TODO(#16): replace input clock with a get_frequency instruction
  m_spi_driver.configure(p_settings, 16.0_MHz);
}

void spi::driver_transfer(std::span<hal::byte const> p_data_out,
                          std::span<hal::byte> p_data_in,
                          hal::byte p_filler)
{
  m_spi_driver.transfer(p_data_out, p_data_in, p_filler);
}
}  // namespace hal::stm32f411
