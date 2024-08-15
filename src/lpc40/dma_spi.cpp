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

#include <libhal-arm-mcu/lpc40/clock.hpp>
#include <libhal-arm-mcu/lpc40/constants.hpp>
#include <libhal-arm-mcu/lpc40/dma.hpp>
#include <libhal-arm-mcu/lpc40/dma_spi.hpp>
#include <libhal-arm-mcu/lpc40/interrupt.hpp>
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

inline bool still_sending(spi_reg_t* p_reg)
{
  return bit_extract<status_register::data_line_busy_bit>(p_reg->sr);
}

struct dma_peripheral_pair
{
  dma_peripheral rx;
  dma_peripheral tx;
};

inline dma_peripheral_pair to_dma_peripheral_pair(dma_spi::bus_info& p_bus_info)
{
  switch (p_bus_info.peripheral_id) {
    case peripheral::ssp0:
      return {
        .rx = dma_peripheral::spi0_rx_and_timer1_match1,
        .tx = dma_peripheral::spi0_tx_and_timer1_match0,
      };
    case peripheral::ssp1:
      return {
        .rx = dma_peripheral::spi1_rx_and_timer2_match1,
        .tx = dma_peripheral::spi1_tx_and_timer2_match0,
      };
    case peripheral::ssp2:
      return {
        .rx = dma_peripheral::spi2_rx_and_i2s_channel_1,
        .tx = dma_peripheral::spi2_tx_and_i2s_channel_0,
      };
    default:
      hal::safe_throw(hal::operation_not_supported(&p_bus_info));
      break;
  }
}
}  // namespace

dma_spi::dma_spi(std::uint8_t p_bus_number,
                 hal::io_waiter& p_waiter,
                 dma_spi::settings const& p_settings)
  : m_io_waiter(&p_waiter)
  , m_bus{}
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

  dma_spi::driver_configure(p_settings);
}  // namespace hal::lpc40

dma_spi::~dma_spi()
{
  power_off(m_bus.peripheral_id);
}

dma_spi::dma_spi(bus_info p_bus)
  : m_bus(p_bus)
{
  if (p_bus.peripheral_id != peripheral::ssp0 &&
      p_bus.peripheral_id != peripheral::ssp1 &&
      p_bus.peripheral_id != peripheral::ssp2) {
    hal::safe_throw(hal::operation_not_supported(this));
  }
}

void dma_spi::driver_configure(settings const& p_settings)
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

  // Enable DMA
  bit_modify(reg->dmacr)
    .set<dma_register::receive_dma_enable>()
    .set<dma_register::transmit_dma_enable>();

  // Enable SSP
  bit_modify(reg->cr1).set<control_register1::spi_enable>();
}

void dma_spi::driver_transfer(std::span<hal::byte const> p_data_out,
                              std::span<hal::byte> p_data_in,
                              hal::byte p_filler)
{
  auto& reg = *get_spi_reg(m_bus.peripheral_id);

  auto const dma_peripheral_pair = to_dma_peripheral_pair(m_bus);
  auto const min = std::min(p_data_in.size(), p_data_out.size());

  dma receive_configuration = {
    .source = &reg.dr,
    .destination = p_data_in.data(),
    .length = min,
    .source_increment = 0,
    .destination_increment = 1,
    .transfer_type = dma_transfer_type::peripheral_to_memory,
    .source_transfer_width = dma_transfer_width::bit_8,
    .destination_transfer_width = dma_transfer_width::bit_8,
    .source_peripheral = dma_peripheral_pair.rx,
    .destination_peripheral = dma_peripheral::memory_or_timer0_match0,
    .source_burst_size = dma_burst_size::bytes_1,
    .destination_burst_size = dma_burst_size::bytes_1,
  };

  dma transmit_configuration = {
    .source = p_data_out.data(),
    .destination = &reg.dr,
    .length = min,
    .source_increment = 1,
    .destination_increment = 0,
    .transfer_type = dma_transfer_type ::memory_to_peripheral,
    .source_transfer_width = dma_transfer_width::bit_8,
    .destination_transfer_width = dma_transfer_width::bit_8,
    .source_peripheral = dma_peripheral::memory_or_timer0_match0,
    .destination_peripheral = dma_peripheral_pair.tx,
    .source_burst_size = dma_burst_size::bytes_1,
    .destination_burst_size = dma_burst_size::bytes_1,
  };

  bool rx_done = false;
  bool tx_done = false;

  auto receive_completion_handler = [this, &rx_done]() {
    rx_done = true;
    m_io_waiter->resume();
  };

  auto transmit_completion_handler = [this, &tx_done]() {
    tx_done = true;
    m_io_waiter->resume();
  };

  if (receive_configuration.length == 0) {
    rx_done = true;
  } else {
    hal::lpc40::setup_dma_transfer(receive_configuration,
                                   receive_completion_handler);
  }

  if (transmit_configuration.length == 0) {
    tx_done = true;
  } else {
    hal::lpc40::setup_dma_transfer(transmit_configuration,
                                   transmit_completion_handler);
  }

  while (not(rx_done && tx_done)) {
    m_io_waiter->wait();
  }

  rx_done = false;
  tx_done = false;

  if (p_data_in.size() == p_data_out.size()) {
    return;
  } else if (p_data_in.size() > p_data_out.size()) {
    // In this case, we have more bytes we want to read back than bytes we want
    // to transmit.
    p_data_in = p_data_in.subspan(min);
    receive_configuration.destination = p_data_in.data();
    receive_configuration.length = p_data_in.size();
    hal::lpc40::setup_dma_transfer(receive_configuration,
                                   receive_completion_handler);

    transmit_configuration.source = &p_filler;
    transmit_configuration.length = p_data_in.size();
    transmit_configuration.source_increment = 0;
    hal::lpc40::setup_dma_transfer(transmit_configuration,
                                   transmit_completion_handler);
  } else {
    // In this case, we have more bytes we want to transmit than we want to
    // read. So skip the receive dma transfer...
    rx_done = true;
    p_data_out = p_data_out.subspan(min);
    transmit_configuration.source = p_data_out.data();
    transmit_configuration.length = p_data_out.size();
    hal::lpc40::setup_dma_transfer(transmit_configuration,
                                   transmit_completion_handler);
  }

  while (not(rx_done && tx_done)) {
    m_io_waiter->wait();
  }

  // Stay in the loop until the bus is no longer active
  while (still_sending(&reg)) {
    m_io_waiter->wait();
  }
}
}  // namespace hal::lpc40
