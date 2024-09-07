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

#include <libhal-lpc40/can.hpp>

#include <libhal-armcortex/interrupt.hpp>
#include <libhal-lpc40/clock.hpp>
#include <libhal-lpc40/constants.hpp>
#include <libhal-lpc40/interrupt.hpp>
#include <libhal-lpc40/pin.hpp>
#include <libhal-lpc40/power.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/bit_limits.hpp>
#include <libhal-util/can.hpp>
#include <libhal-util/static_callable.hpp>

#include "can_reg.hpp"

namespace hal::lpc40 {
namespace {

can_reg_t* get_can_reg(peripheral p_id)
{
  switch (p_id) {
    case peripheral::can1:
      return can_reg1;
    case peripheral::can2:
    default:
      return can_reg2;
  }
}

/// Container for the LPC40xx CAN BUS registers
struct lpc_message
{
  /// TFI register contents
  uint32_t frame = 0;
  /// TID register contents
  uint32_t id = 0;
  /// TDA register contents
  uint32_t data_a = 0;
  /// TDB register contents
  uint32_t data_b = 0;
};

void enable_acceptance_filter() noexcept
{
  can_acceptance_filter->acceptance_filter =
    value(can_commands::accept_all_messages);
}

[[maybe_unused]] bool has_data(can_reg_t* p_reg) noexcept
{
  return bit_extract<can_global_status::receive_buffer>(p_reg->GSR);
}

can::message_t receive(can_reg_t* p_reg) noexcept
{
  static constexpr auto id_mask = bit_mask::from<0, 28>();
  can::message_t message;

  // Extract all of the information from the message frame
  auto frame = p_reg->RFS;
  auto remote_request = bit_extract<can_frame_info::remote_request>(frame);
  auto length = bit_extract<can_frame_info::length>(frame);

  message.is_remote_request = remote_request;
  message.length = static_cast<uint8_t>(length);

  // Get the frame ID
  message.id = bit_extract<id_mask>(p_reg->RID);

  // Pull the bytes from RDA into the payload array
  message.payload[0] = (p_reg->RDA >> (0 * 8)) & 0xFF;
  message.payload[1] = (p_reg->RDA >> (1 * 8)) & 0xFF;
  message.payload[2] = (p_reg->RDA >> (2 * 8)) & 0xFF;
  message.payload[3] = (p_reg->RDA >> (3 * 8)) & 0xFF;

  // Pull the bytes from RDB into the payload array
  message.payload[4] = (p_reg->RDB >> (0 * 8)) & 0xFF;
  message.payload[5] = (p_reg->RDB >> (1 * 8)) & 0xFF;
  message.payload[6] = (p_reg->RDB >> (2 * 8)) & 0xFF;
  message.payload[7] = (p_reg->RDB >> (3 * 8)) & 0xFF;

  // Release the RX buffer and allow another buffer to be read.
  p_reg->CMR = value(can_commands::release_rx_buffer);

  return message;
}

/// Convert message into the registers LPC40xx can bus registers.
///
/// @param message - message to convert.
can_lpc_message message_to_registers(can::message_t const& p_message)
{
  static constexpr auto highest_11_bit_number = 2048UL;
  can_lpc_message registers;

  uint32_t message_frame_info = 0;

  if (p_message.id < highest_11_bit_number) {
    message_frame_info =
      bit_value<decltype(message_frame_info)>(0)
        .insert<can_frame_info::length>(p_message.length)
        .insert<can_frame_info::remote_request>(p_message.is_remote_request)
        .insert<can_frame_info::format>(0U)
        .to<std::uint32_t>();
  } else {
    message_frame_info =
      bit_value<decltype(message_frame_info)>(0)
        .insert<can_frame_info::length>(p_message.length)
        .insert<can_frame_info::remote_request>(p_message.is_remote_request)
        .insert<can_frame_info::format>(1U)
        .to<std::uint32_t>();
  }

  uint32_t data_a = 0;
  data_a |= static_cast<std::uint32_t>(p_message.payload[0] << (0UL * 8));
  data_a |= static_cast<std::uint32_t>(p_message.payload[1] << (1UL * 8));
  data_a |= static_cast<std::uint32_t>(p_message.payload[2] << (2UL * 8));
  data_a |= static_cast<std::uint32_t>(p_message.payload[3] << (3UL * 8));

  uint32_t data_b = 0;
  data_b |= static_cast<std::uint32_t>(p_message.payload[4U] << (0UL * 8UL));
  data_b |= static_cast<std::uint32_t>(p_message.payload[5U] << (1UL * 8UL));
  data_b |= static_cast<std::uint32_t>(p_message.payload[6U] << (2UL * 8UL));
  data_b |= static_cast<std::uint32_t>(p_message.payload[7U] << (3UL * 8UL));

  registers.frame = message_frame_info;
  registers.id = p_message.id;
  registers.data_a = data_a;
  registers.data_b = data_b;

  return registers;
}
}  // namespace

void can::configure_baud_rate(can::port const& p_port,
                              can::settings const& p_settings)
{
  using namespace hal::literals;

  auto* reg = get_can_reg(p_port.id);
  auto const can_frequency = get_frequency(p_port.id);

  bool external_oscillator_used = using_external_oscillator();
  bool baud_rate_above_100khz = p_settings.baud_rate > 100.0_kHz;
  auto const valid_divider =
    hal::calculate_can_bus_divider(can_frequency, p_settings.baud_rate);

  if ((baud_rate_above_100khz && not external_oscillator_used) ||
      not valid_divider) {
    safe_throw(hal::operation_not_supported(this));
  }

  auto const dividers = valid_divider.value();
  auto const prescale = dividers.clock_divider - 1U;
  auto const sync_jump_width = dividers.synchronization_jump_width - 1U;

  auto phase_segment1 =
    (dividers.phase_segment1 + dividers.propagation_delay) - 1U;
  auto phase_segment2 = dividers.phase_segment2 - 1U;

  constexpr auto segment2_bit_limit =
    hal::bit_limits<can_bus_timing::time_segment2.width, std::uint32_t>::max();

  // Check if phase segment 2 does not fit
  if (phase_segment2 > segment2_bit_limit) {
    // Take the extra time quanta and add it to the phase 1 segment
    auto const phase_segment2_remainder = phase_segment2 - segment2_bit_limit;
    phase_segment1 += phase_segment2_remainder;
    // Cap phase segment 2 to the max available in the bit field
    phase_segment2 = segment2_bit_limit;
  }

  std::uint8_t enable_triple_sampling = 0;
  // The bus is sampled 3 times (recommended for low speeds, 100kHz is
  // considered HIGH).
  if (p_settings.baud_rate < 100.0_kHz) {
    enable_triple_sampling = 1U;
  }

  bit_modify(reg->BTR)
    .insert<can_bus_timing::sync_jump_width>(sync_jump_width)
    .insert<can_bus_timing::time_segment1>(phase_segment1)
    .insert<can_bus_timing::time_segment2>(phase_segment2)
    .insert<can_bus_timing::prescalar>(prescale)
    .insert<can_bus_timing::sampling>(enable_triple_sampling);
}

void can::setup(can::port const& p_port, can::settings const& p_settings)
{
  auto* reg = get_can_reg(p_port.id);

  initialize_interrupts();

  /// Power on CAN BUS peripheral
  power_on(p_port.id);

  /// Configure pins
  p_port.td.function(p_port.td_function_code);
  p_port.rd.function(p_port.rd_function_code);

  // Enable reset mode in order to write to CAN registers.
  bit_modify(reg->MOD).set<can_mode::reset>();

  configure_baud_rate(p_port, p_settings);
  enable_acceptance_filter();

  // Disable reset mode, enabling the device
  bit_modify(reg->MOD).clear<can_mode::reset>();
}

can::can(std::uint8_t p_port_number, can::settings const& p_settings)
{
  if (p_port_number == 1) {
    m_port = can::port{
      .td = pin(0, 1),
      .td_function_code = 1,
      .rd = pin(0, 0),
      .rd_function_code = 1,
      .id = peripheral::can1,
      .irq_number = irq::can,
    };
  } else if (p_port_number == 2) {
    m_port = can::port{
      .td = pin(2, 8),
      .td_function_code = 1,
      .rd = pin(2, 7),
      .rd_function_code = 1,
      .id = peripheral::can2,
      .irq_number = irq::can,
    };
  } else {
    safe_throw(hal::operation_not_supported(this));
  }

  setup(m_port, p_settings);
}

can::~can()
{
  auto* reg = get_can_reg(m_port.id);
  // Disable generating an interrupt request by this CAN peripheral, but leave
  // the interrupt enabled. We must NOT disable the interrupt via Arm's NVIC
  // as it could be used by the other CAN peripheral.
  bit_modify(reg->IER).clear<can_interrupts::received_message>();
}

/**
 * @brief Construct a new can object
 *
 * @param p_port - CAN port information
 */
can::can(port const& p_port, can::settings const& p_settings)
  : m_port(p_port)
{
  setup(p_port, p_settings);
}

void can::driver_configure(can::settings const& p_settings)
{
  return setup(m_port, p_settings);
}

void can::driver_send(message_t const& p_message)
{
  auto* reg = get_can_reg(m_port.id);
  auto can_message_registers = message_to_registers(p_message);

  // Wait for one of the buffers to be free so we can transmit a message
  // through it.
  bool sent = false;
  while (!sent) {
    auto const status_register = reg->SR;
    // Check if any buffer is available.
    if (bit_extract<can_buffer_status::bus_status>(status_register) ==
        can_buffer_status::bus_off) {
      throw std::errc::network_down;
    } else if (bit_extract<can_buffer_status::tx1_released>(status_register)) {
      reg->TFI1 = can_message_registers.frame;
      reg->TID1 = can_message_registers.id;
      reg->TDA1 = can_message_registers.data_a;
      reg->TDB1 = can_message_registers.data_b;
      reg->CMR = value(can_commands::send_tx_buffer1);
      sent = true;
    } else if (bit_extract<can_buffer_status::tx2_released>(status_register)) {
      reg->TFI2 = can_message_registers.frame;
      reg->TID2 = can_message_registers.id;
      reg->TDA2 = can_message_registers.data_a;
      reg->TDB2 = can_message_registers.data_b;
      reg->CMR = value(can_commands::send_tx_buffer2);
      sent = true;
    } else if (bit_extract<can_buffer_status::tx3_released>(status_register)) {
      reg->TFI3 = can_message_registers.frame;
      reg->TID3 = can_message_registers.id;
      reg->TDA3 = can_message_registers.data_a;
      reg->TDB3 = can_message_registers.data_b;
      reg->CMR = value(can_commands::send_tx_buffer3);
      sent = true;
    }
  }
}

void can::driver_bus_on()
{
  auto* reg = get_can_reg(m_port.id);
  // When the device is in "bus-off" mode, the mode::reset bit is set to '1'. To
  // re-enable the device, clear the reset bit.
  bit_modify(reg->MOD).clear<can_mode::reset>();
}

void can::driver_on_receive(hal::callback<can::handler> p_receive_handler)
{
  auto* reg = get_can_reg(m_port.id);
  // Save the handler
  m_receive_handler = p_receive_handler;

  // Create a lambda that passes this object's reference to the stored handler
  auto isr = [this, reg]() {
    auto message = receive(reg);
    m_receive_handler(message);
  };

  auto can_handler = static_callable<can, 0, void(void)>(isr).get_handler();
  cortex_m::enable_interrupt(irq::can, can_handler);

  bit_modify(reg->IER).set(can_interrupts::received_message);
}
}  // namespace hal::lpc40
