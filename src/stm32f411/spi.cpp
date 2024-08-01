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

#include <bit>

#include <libhal-arm-mcu/stm32f411/constants.hpp>
#include <libhal-arm-mcu/stm32f411/pin.hpp>
#include <libhal-arm-mcu/stm32f411/spi.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/spi.hpp>
#include <libhal-util/static_callable.hpp>
#include <libhal/error.hpp>

#include "power.hpp"
#include "spi_reg.hpp"

namespace hal::stm32f411 {
namespace {
inline spi_reg_t* get_spi_reg(peripheral p_id)
{
  switch (p_id) {
    case peripheral::spi1:
      return stm32f411::spi_reg1;
    case peripheral::spi2:
      return stm32f411::spi_reg2;
    case peripheral::spi3:
      return stm32f411::spi_reg3;
    case peripheral::spi4:
      return stm32f411::spi_reg4;
    case peripheral::spi5:
      return stm32f411::spi_reg5;

    default:
      hal::safe_throw(hal::operation_not_supported(nullptr));
  }
}

inline bool busy(spi_reg_t* p_reg)
{
  return bit_extract<status_register::busy_flag>(p_reg->sr);
}
inline bool tx_empty(spi_reg_t* p_reg)
{
  return bit_extract<status_register::tx_buffer_empty>(p_reg->sr);
}
inline bool rx_not_empty(spi_reg_t* p_reg)
{
  return bit_extract<status_register::rx_buffer_not_empty>(p_reg->sr);
}
}  // namespace

spi::spi(hal::runtime,
         std::uint8_t p_bus_number,
         spi::settings const& p_settings)
{
  // Datasheet: Chapter 4: Pin definition Table 9
  switch (p_bus_number) {
    case 1: {
      m_peripheral_register = get_spi_reg(peripheral::spi1);
      m_peripheral_id = peripheral::spi1;
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
    case 2: {
      m_peripheral_register = get_spi_reg(peripheral::spi2);
      m_peripheral_id = peripheral::spi2;
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
    case 3: {
      m_peripheral_register = get_spi_reg(peripheral::spi3);
      m_peripheral_id = peripheral::spi3;
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
    case 4: {
      m_peripheral_register = get_spi_reg(peripheral::spi4);
      m_peripheral_id = peripheral::spi4;
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
    case 5: {
      m_peripheral_register = get_spi_reg(peripheral::spi5);
      m_peripheral_id = peripheral::spi5;
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
}  // namespace hal::lpc40

spi::~spi()
{
  power(m_peripheral_id).off();
}

void spi::driver_configure(settings const& p_settings)
{
  auto* reg = reinterpret_cast<spi_reg_t*>(m_peripheral_register);

  // Set SPI to master mode by clearing
  bit_modify(reg->cr1).set(control_register1::master_selection);

  // Setup operating frequency
  // TODO(#16): replace input clock with a get_frequency instruction
  using namespace hal::literals;
  auto const input_clock = 16.0_MHz;
  auto const clock_divider = input_clock / p_settings.clock_rate;
  auto prescaler = static_cast<std::uint16_t>(clock_divider);
  if (prescaler <= 1) {
    prescaler = 2;
  } else if (prescaler > 256) {
    hal::safe_throw(hal::operation_not_supported(this));
  }

  uint16_t baud_control = 15 - std::countl_zero(prescaler);
  if (std::has_single_bit(prescaler)) {
    baud_control--;
  }
  bit_modify(reg->cr1)
    .insert<control_register1::baud_rate_control>(baud_control)
    .insert<control_register1::clock_phase>(
      p_settings.data_valid_on_trailing_edge)
    .insert<control_register1::clock_polarity>(p_settings.clock_idles_high)
    .set<control_register1::bidirectional_output_enable>()
    .set<control_register1::software_slave_management>()
    .set<control_register1::internal_slave_select>()
    .clear<control_register1::frame_format>();

  bit_modify(reg->cr2)
    .set<control_register2::slave_select_output_enable>()
    .clear<control_register2::frame_format>();

  bit_modify(reg->cr1).set<control_register1::master_selection>();

  bit_modify(reg->cr1)
    .set<control_register1::enable>()
    .clear<control_register1::internal_slave_select>();
}

void spi::driver_transfer(std::span<hal::byte const> p_data_out,
                          std::span<hal::byte> p_data_in,
                          hal::byte p_filler)
{
  auto* reg = reinterpret_cast<spi_reg_t*>(m_peripheral_register);
  size_t max_length = std::max(p_data_in.size(), p_data_out.size());

  while (busy(reg)) {
    continue;
  }
  bit_modify(reg->cr1).set<control_register1::internal_slave_select>();
  for (size_t index = 0; index < max_length; index++) {
    hal::byte byte = 0;

    if (index < p_data_out.size()) {
      byte = p_data_out[index];
    } else {
      byte = p_filler;
    }

    while (!tx_empty(reg)) {
      continue;
    }

    reg->dr = byte;

    while (!rx_not_empty(reg)) {
      continue;
    }

    byte = static_cast<uint8_t>(reg->dr);
    if (index < p_data_in.size()) {
      p_data_in[index] = byte;
    }
  }
  bit_modify(reg->cr1).clear<control_register1::internal_slave_select>();
}
}  // namespace hal::stm32f411
