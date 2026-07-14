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

#include <cstdint>

#include <libhal-arm-mcu/lpc40/spi.hpp>

#include <libhal-arm-mcu/lpc40/clock.hpp>
#include <libhal-arm-mcu/lpc40/constants.hpp>
#include <libhal-arm-mcu/lpc40/power.hpp>
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

inline bool tx_fifo_full(spi_reg_t* p_reg)
{
  return not bit_extract<status_register::transmit_fifo_not_full>(p_reg->sr);
}
inline bool rx_fifo_not_empty(spi_reg_t* p_reg)
{
  return bit_extract<status_register::receive_fifo_not_empty>(p_reg->sr);
}
inline bool still_sending(spi_reg_t* p_reg)
{
  return bit_extract<status_register::data_line_busy_bit>(p_reg->sr);
}
}  // namespace

spi::spi(std::uint8_t p_bus_number, spi::settings const& p_settings)
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

void spi::driver_configure(settings const& p_settings)
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
  auto const input_clock = get_frequency(m_bus.peripheral_id);
  auto const clock_divider = input_clock / p_settings.clock_rate;
  auto const prescaler = static_cast<std::uint16_t>(clock_divider);
  auto const prescaler_low = static_cast<std::uint8_t>(prescaler & 0xFF);
  auto const prescaler_high = static_cast<std::uint8_t>(prescaler >> 8);
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

void spi::driver_transfer(std::span<hal::byte const> p_data_out,
                          std::span<hal::byte> p_data_in,
                          hal::byte p_filler)
{
  auto* reg = get_spi_reg(m_bus.peripheral_id);
  auto tx_interator = p_data_out.begin();
  auto rx_interator = p_data_in.begin();

  // iterate until both reach their end
  while (tx_interator != p_data_out.end() || rx_interator != p_data_in.end()) {
    if (rx_interator != p_data_in.end() && rx_fifo_not_empty(reg)) {
      *rx_interator++ = reg->dr;
    }
    if (tx_interator == p_data_out.end()) {
      reg->dr = p_filler;
    } else if (not tx_fifo_full(reg)) {
      reg->dr = *tx_interator++;
    }
  }

  // Wait for bus activity to cease before leaving the function
  while (still_sending(reg)) {
    continue;
  }
}
}  // namespace hal::lpc40
