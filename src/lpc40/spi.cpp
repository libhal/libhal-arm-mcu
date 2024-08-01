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

#include <libhal-lpc40/spi.hpp>

#include <libhal-lpc40/clock.hpp>
#include <libhal-lpc40/constants.hpp>
#include <libhal-lpc40/interrupt.hpp>
#include <libhal-lpc40/power.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/spi.hpp>
#include <libhal-util/static_callable.hpp>

#include "spi_reg.hpp"

namespace hal::lpc40 {
namespace {
inline spi_reg_t* get_spi_reg(peripheral p_id)
{
  switch (p_id) {
    case peripheral::ssp0:
      return spi_reg0;
    case peripheral::ssp1:
      return spi_reg1;
    case peripheral::ssp2:
    default:
      return spi_reg2;
  }
}

inline bool busy(spi_reg_t* p_reg)
{
  return bit_extract<status_register::data_line_busy_bit>(p_reg->sr);
}
}  // namespace

spi::spi(std::uint8_t p_bus_number, const spi::settings& p_settings)
{
  // UM10562: Chapter 7: LPC408x/407x I/O configuration page 13
  if (p_bus_number == 0) {
    m_bus = {
      .peripheral_id = peripheral::ssp0,
      .clock = pin(0, 15),
      .data_out = pin(0, 18),
      .data_in = pin(0, 17),
      .clock_function = 0b010,
      .data_out_function = 0b010,
      .data_in_function = 0b010,
    };
  } else if (p_bus_number == 1) {
    m_bus = {
      .peripheral_id = peripheral::ssp1,
      .clock = pin(0, 7),
      .data_out = pin(0, 9),
      .data_in = pin(0, 8),
      .clock_function = 0b010,
      .data_out_function = 0b010,
      .data_in_function = 0b010,
    };
  } else if (p_bus_number == 2) {
    m_bus = {
      .peripheral_id = peripheral::ssp2,
      .clock = pin(1, 0),
      .data_out = pin(1, 1),
      .data_in = pin(1, 4),
      .clock_function = 0b100,
      .data_out_function = 0b100,
      .data_in_function = 0b100,
    };
  } else {
    // "Supported spi busses are 0, 1, and 2!";
    hal::safe_throw(hal::operation_not_supported(this));
  }

  spi::driver_configure(p_settings);
}  // namespace hal::lpc40

spi::~spi()
{
  power_off(m_bus.peripheral_id);
}

spi::spi(bus_info p_bus)
  : m_bus(p_bus)
{
}

void spi::driver_configure(const settings& p_settings)
{
  constexpr uint8_t spi_format_code = 0b00;

  auto* reg = get_spi_reg(m_bus.peripheral_id);

  // Power up peripheral
  power_on(m_bus.peripheral_id);

  // Set SSP frame format to SPI
  bit_modify(reg->cr0).insert<control_register0::frame_bit>(spi_format_code);

  // Set SPI to master mode by clearing
  bit_modify(reg->cr1).clear<control_register1::slave_mode_bit>();

  // Setup operating frequency
  const auto input_clock = get_frequency(m_bus.peripheral_id);
  const auto clock_divider = input_clock / p_settings.clock_rate;
  const auto prescaler = static_cast<std::uint16_t>(clock_divider);
  const auto prescaler_low = static_cast<std::uint8_t>(prescaler & 0xFF);
  const auto prescaler_high = static_cast<std::uint8_t>(prescaler >> 8);
  // Store lower half of prescalar in clock prescalar register
  reg->cpsr = prescaler_low;
  // Store upper 8 bit half of the prescalar in control register 0
  bit_modify(reg->cr0).insert<control_register0::divider_bit>(prescaler_high);

  // Set clock modes & bit size
  //
  // NOTE: In UM10562 page 611, you will see that DSS (Data Size Select) is
  // equal to the bit transfer minus 1. So we can add 3 to our DataSize enum
  // to get the appropriate transfer code.
  constexpr std::uint8_t size_code_8bit = 0b111;

  bit_modify(reg->cr0)
    .insert<control_register0::polarity_bit>(p_settings.clock_idles_high)
    .insert<control_register0::phase_bit>(
      p_settings.data_valid_on_trailing_edge)
    .insert<control_register0::data_bit>(size_code_8bit);

  // Initialize SSP pins
  m_bus.clock.function(m_bus.clock_function)
    .analog(false)
    .open_drain(false)
    .resistor(pin_resistor::none);
  m_bus.data_in.function(m_bus.data_in_function)
    .analog(false)
    .open_drain(false)
    .resistor(pin_resistor::none);
  m_bus.data_out.function(m_bus.data_out_function)
    .analog(false)
    .open_drain(false)
    .resistor(pin_resistor::none);

  // Enable SSP
  bit_modify(reg->cr1).set<control_register1::spi_enable>();
}

void spi::driver_transfer(std::span<const hal::byte> p_data_out,
                          std::span<hal::byte> p_data_in,
                          hal::byte p_filler)
{
  auto* reg = get_spi_reg(m_bus.peripheral_id);
  size_t max_length = std::max(p_data_in.size(), p_data_out.size());

  for (size_t index = 0; index < max_length; index++) {
    hal::byte byte = 0;

    if (index < p_data_out.size()) {
      byte = p_data_out[index];
    } else {
      byte = p_filler;
    }

    reg->dr = byte;

    while (busy(reg)) {
      continue;
    }

    byte = static_cast<uint8_t>(reg->dr);
    if (index < p_data_in.size()) {
      p_data_in[index] = byte;
    }
  }
}
}  // namespace hal::lpc40
