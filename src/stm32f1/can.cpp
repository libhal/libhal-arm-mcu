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

#include <bitset>
#include <cstdint>
#include <optional>

#include <libhal-arm-mcu/interrupt.hpp>
#include <libhal-arm-mcu/stm32f1/can.hpp>
#include <libhal-arm-mcu/stm32f1/clock.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32f1/interrupt.hpp>
#include <libhal-arm-mcu/stm32f1/pin.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/bit_limits.hpp>
#include <libhal-util/can.hpp>
#include <libhal-util/enum.hpp>
#include <libhal-util/static_callable.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <libhal/error.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>
#include <nonstd/scope.hpp>

#include "can_reg.hpp"
#include "pin.hpp"
#include "power.hpp"

// This is needed to allow backwards compatibility with previous versions of the
// can APIs.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

namespace hal::stm32f1 {
namespace {
/// Enable/Disable controller modes
///
/// @param mode - which mode to enable/disable
/// @param enable_mode - true if you want to enable the mode. False otherwise.
void set_master_mode(bit_mask p_mode, bool p_enable_mode)
{
  bit_modify(can1_reg->MCR).insert(p_mode, p_enable_mode);
}

bool get_master_status(bit_mask p_mode)
{
  return bit_extract(p_mode, can1_reg->MSR);
}

void enter_initialization()
{
  // Enter Initialization mode in order to write to CAN registers.
  set_master_mode(master_control::initialization_request, true);

  // Wait to enter Initialization mode
  while (not get_master_status(master_status::initialization_acknowledge)) {
    continue;
  }
}

void exit_initialization()
{
  // Leave Initialization mode
  set_master_mode(master_control::initialization_request, false);

  // Wait to leave initialization mode
  while (get_master_status(master_status::initialization_acknowledge)) {
    continue;
  }
}

void timed_exit_initialization(hal::steady_clock& p_clock,
                               hal::time_duration p_timeout_time)
{
  // Leave Initialization mode
  set_master_mode(master_control::initialization_request, false);

  auto const deadline = hal::future_deadline(p_clock, p_timeout_time);
  while (deadline > p_clock.uptime()) {
    if (not get_master_status(master_status::initialization_acknowledge)) {
      return;
    }
  }
  hal::safe_throw(hal::timed_out(nullptr));
}

void configure_baud_rate(hal::u32 p_baud_rate)
{
  auto const can_frequency = frequency(peripheral::can1);
  auto const valid_divider =
    calculate_can_bus_divider(can_frequency, static_cast<float>(p_baud_rate));

  if (not valid_divider) {
    hal::safe_throw(hal::operation_not_supported(nullptr));
  }

  auto const divisors = valid_divider.value();

  auto const prescale = divisors.clock_divider - 1U;
  auto const sync_jump_width = divisors.synchronization_jump_width - 1U;

  auto phase_segment1 =
    (divisors.phase_segment1 + divisors.propagation_delay) - 1U;
  auto phase_segment2 = divisors.phase_segment2 - 1U;

  constexpr auto segment2_bit_limit =
    hal::bit_limits<bus_timing::time_segment2.width, std::uint32_t>::max();

  // Check if phase segment 2 does not fit
  if (phase_segment2 > segment2_bit_limit) {
    // Take the extra time quanta and add it to the phase 1 segment
    auto const phase_segment2_remainder = phase_segment2 - segment2_bit_limit;
    phase_segment1 += phase_segment2_remainder;
    // Cap phase segment 2 to the max available in the bit field
    phase_segment2 = segment2_bit_limit;
  }

  bit_modify(can1_reg->BTR)
    .insert<bus_timing::prescalar>(prescale)
    .insert<bus_timing::time_segment1>(phase_segment1)
    .insert<bus_timing::time_segment2>(phase_segment2)
    .insert<bus_timing::sync_jump_width>(sync_jump_width)
    .clear<bus_timing::silent_mode>();
}

void set_filter_bank_mode(filter_bank_master_control p_mode)
{
  bit_modify(can1_reg->FMR)
    .insert<filter_master::initialization_mode>(hal::value(p_mode));
}

void set_filter_type(hal::u8 p_filter, filter_type p_filter_type)
{
  auto const filter_bit_mask = bit_mask::from(p_filter);
  bit_modify(can1_reg->FM1R).insert(filter_bit_mask, hal::value(p_filter_type));
}

void set_filter_scale(hal::u8 p_filter, filter_scale p_scale)
{
  auto const filter_bit_mask = bit_mask::from(p_filter);
  bit_modify(can1_reg->FS1R).insert(filter_bit_mask, hal::value(p_scale));
}

void set_filter_fifo_assignment(hal::u8 p_filter,
                                can_peripheral_manager::fifo_assignment p_fifo)
{
  auto const filter_bit_mask = bit_mask::from(p_filter);
  bit_modify(can1_reg->FFA1R).insert(filter_bit_mask, hal::value(p_fifo));
}

void set_filter_activation_state(hal::u8 p_filter, filter_activation p_state)
{
  auto const filter_bit_mask = bit_mask::from(p_filter);
  bit_modify(can1_reg->FA1R).insert(filter_bit_mask, hal::value(p_state));
}

void allow_all_acceptance_filter()
{
  // Activate filter initialization mode (Set bit)
  set_filter_bank_mode(filter_bank_master_control::initialization);

  // Deactivate filter 0 (Clear bit)
  set_filter_activation_state(0, filter_activation::not_active);

  // Configure filter 0 to single 32-bit scale configuration (Set bit)
  set_filter_scale(0, filter_scale::single_32_bit_scale);

  // Set ID and mask to 0, where a mask value of 0 means accept any message.
  can1_reg->filter_registers[0].FR1 = 0;
  can1_reg->filter_registers[0].FR2 = 0;

  // Set filter to mask mode
  set_filter_type(0, filter_type::mask);

  // Assign filter 0 to FIFO 0 (Clear bit)
  set_filter_fifo_assignment(0, can_peripheral_manager::fifo_assignment::fifo1);

  // Activate filter 0 (Set bit)
  set_filter_activation_state(0, filter_activation::active);

  // Deactivate filter initialization mode (clear bit)
  set_filter_bank_mode(filter_bank_master_control::active);
}

struct can_data_registers_t
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

/// Converts desired message to the CANx registers
can_data_registers_t convert_message_to_stm_can(
  hal::can::message_t const& message)
{
  can_data_registers_t registers;

  auto frame_info =
    bit_value(0U)
      .insert<frame_length_and_info::data_length_code>(message.length)
      .to<std::uint32_t>();

  uint32_t frame_id = 0;

  if (message.id >= (1UL << 11UL)) {
    frame_id =
      bit_value(0U)
        .insert<mailbox_identifier::transmit_mailbox_request>(true)
        .insert<mailbox_identifier::remote_request>(message.is_remote_request)
        .insert<mailbox_identifier::identifier_type>(
          value(mailbox_identifier::id_type::extended))
        .insert<mailbox_identifier::standard_identifier>(message.id)
        .to<std::uint32_t>();
  } else {
    frame_id =
      bit_value(0U)
        .insert<mailbox_identifier::transmit_mailbox_request>(true)
        .insert<mailbox_identifier::remote_request>(message.is_remote_request)
        .insert<mailbox_identifier::identifier_type>(
          value(mailbox_identifier::id_type::standard))
        .insert<mailbox_identifier::standard_identifier>(message.id)
        .to<std::uint32_t>();
  }

  uint32_t data_a = 0;
  data_a |= message.payload[0] << (0 * 8);
  data_a |= message.payload[1] << (1 * 8);
  data_a |= message.payload[2] << (2 * 8);
  data_a |= message.payload[3] << (3 * 8);

  uint32_t data_b = 0;
  data_b |= message.payload[4] << (0 * 8);
  data_b |= message.payload[5] << (1 * 8);
  data_b |= message.payload[6] << (2 * 8);
  data_b |= message.payload[7] << (3 * 8);

  registers.frame = frame_info;
  registers.id = frame_id;
  registers.data_a = data_a;
  registers.data_b = data_b;

  return registers;
}

/// Converts desired message to the CANx registers
can_data_registers_t convert_message_to_stm_can(hal::can_message const& message)
{
  can_data_registers_t registers;

  auto frame_info =
    bit_value(0U)
      .insert<frame_length_and_info::data_length_code>(message.length)
      .to<std::uint32_t>();

  uint32_t frame_id = 0;

  if (message.extended) {
    frame_id =
      bit_value(0U)
        .insert<mailbox_identifier::transmit_mailbox_request>(true)
        .insert<mailbox_identifier::remote_request>(message.remote_request)
        .set(mailbox_identifier::identifier_type)
        .insert<mailbox_identifier::extended_identifier>(message.id)
        .to<std::uint32_t>();
  } else {
    frame_id =
      bit_value(0U)
        .insert<mailbox_identifier::transmit_mailbox_request>(true)
        .insert<mailbox_identifier::remote_request>(message.remote_request)
        .clear<mailbox_identifier::identifier_type>()
        .insert<mailbox_identifier::standard_identifier>(message.id)
        .to<std::uint32_t>();
  }

  uint32_t data_a = 0;
  data_a |= message.payload[0] << (0 * 8);
  data_a |= message.payload[1] << (1 * 8);
  data_a |= message.payload[2] << (2 * 8);
  data_a |= message.payload[3] << (3 * 8);

  uint32_t data_b = 0;
  data_b |= message.payload[4] << (0 * 8);
  data_b |= message.payload[5] << (1 * 8);
  data_b |= message.payload[6] << (2 * 8);
  data_b |= message.payload[7] << (3 * 8);

  registers.frame = frame_info;
  registers.id = frame_id;
  registers.data_a = data_a;
  registers.data_b = data_b;

  return registers;
}

bool is_bus_off()
{
  // True = Bus is in sleep mode
  // False = Bus has left sleep mode.
  return bit_extract<master_status::sleep_acknowledge>(can1_reg->MCR);
}

void setup_can(hal::u32 p_baud_rate, can_pins p_pins, bool p_self_test = false)
{
  power_on(peripheral::can1);

  set_master_mode(master_control::sleep_mode_request, false);
  set_master_mode(master_control::no_automatic_retransmission, false);
  set_master_mode(master_control::automatic_bus_off_management, false);

  enter_initialization();

  // Ensure we have left initialization phase so the peripheral can operate
  // correctly. If an exception is thrown at any point, this will ensure that
  // the can peripheral is taken out of initialization.
  nonstd::scope_exit on_exit(&exit_initialization);

  configure_baud_rate(p_baud_rate);

  switch (p_pins) {
    case can_pins::pa11_pa12:
      configure_pin({ .port = 'A', .pin = 11 }, input_pull_up);
      configure_pin({ .port = 'A', .pin = 12 }, push_pull_alternative_output);
      break;
    case can_pins::pb9_pb8:
      configure_pin({ .port = 'B', .pin = 8 }, input_pull_up);
      configure_pin({ .port = 'B', .pin = 9 }, push_pull_alternative_output);
      break;
    case can_pins::pd0_pd1:
      configure_pin({ .port = 'D', .pin = 0 }, input_pull_up);
      configure_pin({ .port = 'D', .pin = 1 }, push_pull_alternative_output);
      break;
  }

  bit_modify(can1_reg->BTR).insert<bus_timing::loop_back_mode>(p_self_test);

  remap_pins(p_pins);
}

void setup_can(hal::u32 p_baud_rate,
               can_pins p_pins,
               can_peripheral_manager::self_test p_enable_self_test,
               hal::steady_clock& p_clock,
               hal::time_duration p_timeout_time)
{
  power_on(peripheral::can1);

  set_master_mode(master_control::sleep_mode_request, false);
  set_master_mode(master_control::no_automatic_retransmission, false);
  set_master_mode(master_control::automatic_bus_off_management, false);

  enter_initialization();

  configure_baud_rate(p_baud_rate);

  switch (p_pins) {
    case can_pins::pa11_pa12:
      configure_pin({ .port = 'A', .pin = 11 }, input_pull_up);
      configure_pin({ .port = 'A', .pin = 12 }, push_pull_alternative_output);
      break;
    case can_pins::pb9_pb8:
      configure_pin({ .port = 'B', .pin = 8 }, input_pull_up);
      configure_pin({ .port = 'B', .pin = 9 }, push_pull_alternative_output);
      break;
    case can_pins::pd0_pd1:
      configure_pin({ .port = 'D', .pin = 0 }, input_pull_up);
      configure_pin({ .port = 'D', .pin = 1 }, push_pull_alternative_output);
      break;
  }

  bit_modify(can1_reg->BTR)
    .insert<bus_timing::loop_back_mode>(hal::value(p_enable_self_test));

  remap_pins(p_pins);

  timed_exit_initialization(p_clock, p_timeout_time);
}

can::message_t read_receive_mailbox()
{
  can::message_t message{ .id = 0 };

  uint32_t fifo0_status = can1_reg->RF0R;
  uint32_t fifo1_status = can1_reg->RF1R;

  can_peripheral_manager::fifo_assignment fifo_select =
    can_peripheral_manager::fifo_assignment::fifo1;

  if (bit_extract<fifo_status::messages_pending>(fifo0_status)) {
    fifo_select = can_peripheral_manager::fifo_assignment::fifo1;
  } else if (bit_extract<fifo_status::messages_pending>(fifo1_status)) {
    fifo_select = can_peripheral_manager::fifo_assignment::fifo2;
  } else {
    // Error, tried to receive when there were no pending messages.
    return message;
  }

  uint32_t frame = can1_reg->fifo_mailbox[value(fifo_select)].RDTR;
  uint32_t id = can1_reg->fifo_mailbox[value(fifo_select)].RIR;

  // Extract all of the information from the message frame
  bool is_remote_request = bit_extract<mailbox_identifier::remote_request>(id);
  uint32_t length = bit_extract<frame_length_and_info::data_length_code>(frame);
  uint32_t format = bit_extract<mailbox_identifier::identifier_type>(id);

  message.is_remote_request = is_remote_request;
  message.length = static_cast<std::uint8_t>(length);

  // Get the frame ID
  if (format == value(mailbox_identifier::id_type::extended)) {
    message.id = bit_extract<mailbox_identifier::extended_identifier>(id);
  } else {
    message.id = bit_extract<mailbox_identifier::standard_identifier>(id);
  }

  auto low_read_data = can1_reg->fifo_mailbox[value(fifo_select)].RDLR;
  auto high_read_data = can1_reg->fifo_mailbox[value(fifo_select)].RDHR;

  // Pull the bytes from RDL into the payload array
  message.payload[0] = (low_read_data >> (0 * 8)) & 0xFF;
  message.payload[1] = (low_read_data >> (1 * 8)) & 0xFF;
  message.payload[2] = (low_read_data >> (2 * 8)) & 0xFF;
  message.payload[3] = (low_read_data >> (3 * 8)) & 0xFF;

  // Pull the bytes from RDH into the payload array
  message.payload[4] = (high_read_data >> (0 * 8)) & 0xFF;
  message.payload[5] = (high_read_data >> (1 * 8)) & 0xFF;
  message.payload[6] = (high_read_data >> (2 * 8)) & 0xFF;
  message.payload[7] = (high_read_data >> (3 * 8)) & 0xFF;

  // Release the RX buffer and allow another buffer to be read.
  if (fifo_select == can_peripheral_manager::fifo_assignment::fifo1) {
    bit_modify(can1_reg->RF0R).set<fifo_status::release_output_mailbox>();
  } else if (fifo_select == can_peripheral_manager::fifo_assignment::fifo2) {
    bit_modify(can1_reg->RF1R).set<fifo_status::release_output_mailbox>();
  }

  return message;
}

hal::callback<can::handler> can_receive_handler{};

void handler_can_interrupt()
{
  auto const message = read_receive_mailbox();
  // Why is this here? Because there was an stm32f103c8 chip that may have a
  // defect or was damaged in testing. That device was then able to set its
  // length to 9. The actual data in the data registers were garbage data. Even
  // if the device is damaged, its best to throw out those damaged frames then
  // attempt to pass them to a handler that may not able to manage them.
  if (message.length <= 8) {
    can_receive_handler(message);
  }
}

struct v2
{};

can_message read_receive_mailbox(v2)
{
  using fifo_assignment = can_peripheral_manager::fifo_assignment;

  can_message message{};

  uint32_t fifo0_status = can1_reg->RF0R;
  uint32_t fifo1_status = can1_reg->RF1R;

  fifo_assignment fifo_select = fifo_assignment::fifo1;

  if (bit_extract<fifo_status::messages_pending>(fifo0_status)) {
    fifo_select = fifo_assignment::fifo1;
  } else if (bit_extract<fifo_status::messages_pending>(fifo1_status)) {
    fifo_select = fifo_assignment::fifo2;
  } else {
    // Error, tried to receive when there were no pending messages.
    return message;
  }

  uint32_t frame = can1_reg->fifo_mailbox[value(fifo_select)].RDTR;
  uint32_t id = can1_reg->fifo_mailbox[value(fifo_select)].RIR;

  // Extract all of the information from the message frame
  bool const is_remote_request =
    bit_extract<mailbox_identifier::remote_request>(id);
  uint32_t const length =
    bit_extract<frame_length_and_info::data_length_code>(frame);
  uint32_t const format = bit_extract<mailbox_identifier::identifier_type>(id);
  bool const is_extended =
    format == value(mailbox_identifier::id_type::extended);

  message.remote_request = is_remote_request;
  message.length = static_cast<std::uint8_t>(length);
  message.extended = is_extended;

  // Get the frame ID
  if (is_extended) {
    message.id = bit_extract<mailbox_identifier::extended_identifier>(id);
  } else {
    message.id = bit_extract<mailbox_identifier::standard_identifier>(id);
  }

  auto low_read_data = can1_reg->fifo_mailbox[value(fifo_select)].RDLR;
  auto high_read_data = can1_reg->fifo_mailbox[value(fifo_select)].RDHR;

  // Pull the bytes from RDL into the payload array
  message.payload[0] = (low_read_data >> (0 * 8)) & 0xFF;
  message.payload[1] = (low_read_data >> (1 * 8)) & 0xFF;
  message.payload[2] = (low_read_data >> (2 * 8)) & 0xFF;
  message.payload[3] = (low_read_data >> (3 * 8)) & 0xFF;

  // Pull the bytes from RDH into the payload array
  message.payload[4] = (high_read_data >> (0 * 8)) & 0xFF;
  message.payload[5] = (high_read_data >> (1 * 8)) & 0xFF;
  message.payload[6] = (high_read_data >> (2 * 8)) & 0xFF;
  message.payload[7] = (high_read_data >> (3 * 8)) & 0xFF;

  // Release the RX buffer and allow another buffer to be read.
  if (fifo_select == can_peripheral_manager::fifo_assignment::fifo1) {
    bit_modify(can1_reg->RF0R).set<fifo_status::release_output_mailbox>();
  } else if (fifo_select == can_peripheral_manager::fifo_assignment::fifo2) {
    bit_modify(can1_reg->RF1R).set<fifo_status::release_output_mailbox>();
  }

  return message;
}

void bus_on()
{
  constexpr auto bus_off_mask = error_status_register::bus_off;
  bool const bus_off = bit_extract<bus_off_mask>(can1_reg->ESR);

  if (not bus_off) {
    return;  // nothing to do here, return
  }

  // RM0008 page 670 states that bus off can be recovered from by entering and
  // Request to enter initialization mode
  enter_initialization();

  // Leave Initialization mode
  exit_initialization();
}
}  // namespace

can::can(can::settings const& p_settings, can_pins p_pins)
{
  setup_can(static_cast<hal::u32>(p_settings.baud_rate), p_pins);
  allow_all_acceptance_filter();  // Default behavior in the original design
}

void can::enable_self_test(bool p_enable)
{
  enter_initialization();
  nonstd::scope_exit on_exit(&exit_initialization);

  if (p_enable) {
    bit_modify(can1_reg->BTR).set<bus_timing::loop_back_mode>();
  } else {
    bit_modify(can1_reg->BTR).clear<bus_timing::loop_back_mode>();
  }
}

can::~can()
{
  hal::cortex_m::disable_interrupt(irq::can1_rx0);
  power_off(peripheral::can1);
}

void can::driver_configure(can::settings const& p_settings)
{
  enter_initialization();
  nonstd::scope_exit on_exit(&exit_initialization);

  configure_baud_rate(static_cast<hal::u32>(p_settings.baud_rate));
  allow_all_acceptance_filter();
}

void can::driver_bus_on()
{
  hal::stm32f1::bus_on();
}

void can::driver_send(can::message_t const& p_message)
{
  if (is_bus_off()) {
    hal::safe_throw(hal::operation_not_permitted(this));
  }

  can_data_registers_t registers = convert_message_to_stm_can(p_message);
  std::optional<hal::u8> available_mailbox{};

  while (not available_mailbox) {
    hal::u32 const status_register = can1_reg->TSR;
    // Check if any buffer is available.
    if (bit_extract<transmit_status::transmit_mailbox0_empty>(
          status_register)) {
      available_mailbox = 0;
    } else if (bit_extract<transmit_status::transmit_mailbox1_empty>(
                 status_register)) {
      available_mailbox = 1;
    } else if (bit_extract<transmit_status::transmit_mailbox2_empty>(
                 status_register)) {
      available_mailbox = 2;
    }
  }

  // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
  auto& mailbox = can1_reg->transmit_mailbox[*available_mailbox];
  // The above is removed from lint because the while loop is our check that the
  // optional value has been set.

  bit_modify(mailbox.TDTR)
    .insert<frame_length_and_info::data_length_code>(p_message.length);
  mailbox.TDLR = registers.data_a;
  mailbox.TDHR = registers.data_b;
  mailbox.TIR = registers.id;
}

void can::driver_on_receive(hal::callback<handler> p_handler)
{
  initialize_interrupts();
  can_receive_handler = p_handler;

  // Enable interrupt service routine.
  cortex_m::enable_interrupt(irq::can1_rx0, handler_can_interrupt);
  cortex_m::enable_interrupt(irq::can1_rx1, handler_can_interrupt);

  bit_modify(can1_reg->IER)
    .set<interrupt_enable_register::fifo0_message_pending>();
  bit_modify(can1_reg->IER)
    .set<interrupt_enable_register::fifo1_message_pending>();
}
}  // namespace hal::stm32f1

// =============================================================================
//
// Split can driver code
//
// =============================================================================

namespace hal::stm32f1 {
namespace {
std::span<hal::can_message> can_receive_buffer{};
hal::u32 receive_count{};
hal::u32 current_baud_rate = 0;
can_interrupt::optional_receive_handler can_v2_receive_handler{};
can_bus_manager::optional_bus_off_handler can_v2_bus_off_handler{};
std::bitset<28> acquired_banks{};
can_peripheral_manager::disable_ids disable_id{};

void handler_circular_buffer_interrupt()
{
  auto const message = read_receive_mailbox(v2{});
  // Why is this here? Because there was an stm32f103c8 chip that may have a
  // defect or was damaged in testing. That device was then able to set its
  // length to 9. The actual data in the data registers were garbage data. Even
  // if the device is damaged, its best to throw out those damaged frames then
  // attempt to pass them to a handler that may not able to manage them.
  if (message.length > 8) {
    return;
  }

  if (can_v2_receive_handler) {
    using tag = hal::can_interrupt::on_receive_tag;
    (*can_v2_receive_handler)(tag{}, message);
  }

  if (not can_receive_buffer.empty()) {
    auto const write_index = receive_count++ % can_receive_buffer.size();
    can_receive_buffer[write_index] = message;
  }
}

void handler_status_change_interrupt()
{
  bool is_bus_off = bit_extract<error_status_register::bus_off>(can1_reg->ESR);
  if (is_bus_off && can_v2_bus_off_handler) {
    using tag = hal::can_bus_manager::bus_off_tag;
    (*can_v2_bus_off_handler)(tag{});
  }
}

hal::u16 standard_id_to_stm_filter(hal::u16 p_id)
{
  auto const reg = hal::bit_value()
                     .insert<standard_filter_bank::standard_id1>(p_id)
                     .clear(standard_filter_bank::rtr1)
                     .clear(standard_filter_bank::id_extension1)
                     .insert(standard_filter_bank::extended_id1, 0UL)
                     .to<hal::u16>();
  return reg;
}

hal::u32 extended_id_to_stm_filter(hal::u32 p_id)
{
  auto const reg = hal::bit_value()
                     .insert<extended_filter_bank::id>(p_id)
                     .clear(extended_filter_bank::rtr)
                     .clear(extended_filter_bank::id_extension)
                     .clear(extended_filter_bank::reserved)
                     .to<hal::u32>();
  return reg;
}

/**
 * @brief Scan through the acquired banks and return the index to an available
 * one.
 *
 * @return hal::u8 - returns the index of the filter banks that has been
 * acquired for the caller.
 * @throws hal::resource_unavailable_try_again - if no filter banks are
 * available
 */
hal::u8 available_filter()
{
  for (std::size_t i = 0; i < acquired_banks.size(); i++) {
    if (not acquired_banks.test(i)) {
      acquired_banks.set(i);
      return i;
    }
  }

  hal::safe_throw(hal::resource_unavailable_try_again(nullptr));
}
}  // namespace

can_peripheral_manager::can_peripheral_manager(hal::u32 p_baud_rate,
                                               can_pins p_pins,
                                               disable_ids p_disabled_ids,
                                               bool p_self_test)
{
  current_baud_rate = p_baud_rate;
  disable_id = p_disabled_ids;

  setup_can(p_baud_rate, p_pins, p_self_test);

  initialize_interrupts();

  // Setup interrupt service routines
  cortex_m::enable_interrupt(irq::can1_rx0, handler_circular_buffer_interrupt);
  cortex_m::enable_interrupt(irq::can1_rx1, handler_circular_buffer_interrupt);
  cortex_m::enable_interrupt(irq::can1_sce, handler_status_change_interrupt);

  bit_modify(can1_reg->IER)
    .set<interrupt_enable_register::fifo0_message_pending>();
  bit_modify(can1_reg->IER)
    .set<interrupt_enable_register::fifo1_message_pending>();
}

can_peripheral_manager::can_peripheral_manager(
  hal::u32 p_baud_rate,
  hal::steady_clock& p_clock,
  hal::time_duration p_timeout_time,
  can_pins p_pins,
  self_test p_enable_self_test,
  disable_ids p_disabled_ids)
{
  current_baud_rate = p_baud_rate;
  disable_id = p_disabled_ids;
  setup_can(p_baud_rate, p_pins, p_enable_self_test, p_clock, p_timeout_time);

  initialize_interrupts();

  // Setup interrupt service routines
  cortex_m::enable_interrupt(irq::can1_rx0, handler_circular_buffer_interrupt);
  cortex_m::enable_interrupt(irq::can1_rx1, handler_circular_buffer_interrupt);
  cortex_m::enable_interrupt(irq::can1_sce, handler_status_change_interrupt);

  bit_modify(can1_reg->IER)
    .set<interrupt_enable_register::fifo0_message_pending>();
  bit_modify(can1_reg->IER)
    .set<interrupt_enable_register::fifo1_message_pending>();
}

void can_peripheral_manager::enable_self_test(bool p_enable)
{
  enter_initialization();
  nonstd::scope_exit on_exit(&exit_initialization);

  if (p_enable) {
    bit_modify(can1_reg->BTR).set<bus_timing::loop_back_mode>();
  } else {
    bit_modify(can1_reg->BTR).clear<bus_timing::loop_back_mode>();
  }
}

can_peripheral_manager::transceiver::transceiver(
  std::span<can_message> p_receive_buffer)
{
  can_receive_buffer = p_receive_buffer;
  receive_count = 0;
}

u32 can_peripheral_manager::transceiver::driver_baud_rate()
{
  return current_baud_rate;
}

void can_peripheral_manager::transceiver::driver_send(
  can_message const& p_message)
{
  if (is_bus_off()) {
    hal::safe_throw(hal::operation_not_permitted(this));
  }

  can_data_registers_t const registers = convert_message_to_stm_can(p_message);
  std::optional<hal::u8> available_mailbox{};

  while (not available_mailbox) {
    hal::u32 const status_register = can1_reg->TSR;
    // Check if any buffer is available.
    if (bit_extract<transmit_status::transmit_mailbox0_empty>(
          status_register)) {
      available_mailbox = 0;
    } else if (bit_extract<transmit_status::transmit_mailbox1_empty>(
                 status_register)) {
      available_mailbox = 1;
    } else if (bit_extract<transmit_status::transmit_mailbox2_empty>(
                 status_register)) {
      available_mailbox = 2;
    }
  }

  // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
  auto& mailbox = can1_reg->transmit_mailbox[*available_mailbox];
  // The above is removed from lint because the while loop is our check that the
  // optional value has been set.

  bit_modify(mailbox.TDTR)
    .insert<frame_length_and_info::data_length_code>(p_message.length);
  mailbox.TDLR = registers.data_a;
  mailbox.TDHR = registers.data_b;
  mailbox.TIR = registers.id;
}

std::span<can_message const>
can_peripheral_manager::transceiver::driver_receive_buffer()
{
  return can_receive_buffer;
}

std::size_t can_peripheral_manager::transceiver::driver_receive_cursor()
{
  return receive_count % can_receive_buffer.size();
}

can_peripheral_manager::interrupt::interrupt()
{
  bit_modify(can1_reg->IER).set(interrupt_enable_register::bus_off);
}

void can_peripheral_manager::interrupt::driver_on_receive(
  optional_receive_handler p_callback)
{
  can_v2_receive_handler = p_callback;
}

void can_peripheral_manager::bus_manager::driver_baud_rate(hal::u32 p_hertz)
{
  enter_initialization();
  // Ensure we have left initialization phase so the peripheral can operate
  // correctly. If an exception is thrown at any point, this will ensure that
  // the can peripheral is taken out of initialization.
  nonstd::scope_exit on_exit(&exit_initialization);

  configure_baud_rate(p_hertz);
  current_baud_rate = p_hertz;
}

void can_peripheral_manager::bus_manager::driver_filter_mode(accept)
{
  // this does nothing for now. We should consider dropping this in favor of
  // always using a filter to manager message acceptance. A single mask filter
  // with its mask set to all ZEROs would do the trick.
}

void can_peripheral_manager::bus_manager::driver_on_bus_off(
  optional_bus_off_handler p_callback)
{
  can_v2_bus_off_handler = p_callback;
}

void can_peripheral_manager::bus_manager::driver_bus_on()
{
  bus_on();
}

can_peripheral_manager::transceiver can_peripheral_manager::acquire_transceiver(
  std::span<can_message> p_receive_buffer)
{
  return can_peripheral_manager::transceiver{ p_receive_buffer };
}

can_peripheral_manager::bus_manager
can_peripheral_manager::acquire_bus_manager()
{
  return can_peripheral_manager::bus_manager{};
}

can_peripheral_manager::interrupt can_peripheral_manager::acquire_interrupt()
{
  return can_peripheral_manager::interrupt{};
}

// =============================================================================
//
// Acquire Filters
//
// =============================================================================

can_peripheral_manager::identifier_filter_set
can_peripheral_manager::acquire_identifier_filter(fifo_assignment p_fifo)
{
  return identifier_filter_set{ available_filter(), p_fifo };
}

can_peripheral_manager::mask_filter_set
can_peripheral_manager::acquire_mask_filter(fifo_assignment p_fifo)
{
  return mask_filter_set{ available_filter(), p_fifo };
}

can_peripheral_manager::extended_identifier_filter_set
can_peripheral_manager::acquire_extended_identifier_filter(
  fifo_assignment p_fifo)
{
  return extended_identifier_filter_set{ available_filter(), p_fifo };
}

can_peripheral_manager::extended_mask_filter
can_peripheral_manager::acquire_extended_mask_filter(fifo_assignment p_fifo)
{
  return extended_mask_filter{ available_filter(), p_fifo };
}

// =============================================================================
//
// Filter Constructors
//
// =============================================================================

can_peripheral_manager::mask_filter::mask_filter(filter_resource p_resource)
  : m_resource(p_resource)
{
}

can_peripheral_manager::identifier_filter::identifier_filter(
  filter_resource p_resource)
  : m_resource(p_resource)
{
}

can_peripheral_manager::extended_mask_filter::extended_mask_filter(
  hal::u8 p_filter_index,
  fifo_assignment p_fifo)
  : m_filter_index(p_filter_index)
{
  set_filter_bank_mode(filter_bank_master_control::initialization);

  // On scope exit, whether via a return or an exception, invoke this.
  nonstd::scope_exit on_exit([p_filter_index]() {
    // Deactivate filter initialization mode (clear bit)
    set_filter_bank_mode(filter_bank_master_control::active);
    set_filter_activation_state(p_filter_index, filter_activation::active);
  });

  set_filter_activation_state(p_filter_index, filter_activation::not_active);
  set_filter_scale(p_filter_index, filter_scale::single_32_bit_scale);
  set_filter_type(p_filter_index, filter_type::mask);
  set_filter_fifo_assignment(p_filter_index, p_fifo);

  auto const id_reg = extended_id_to_stm_filter(disable_id.extended);
  auto const mask_reg = extended_id_to_stm_filter(0x1FFF'FFFF);
  can1_reg->filter_registers[p_filter_index].FR1 = id_reg;
  can1_reg->filter_registers[p_filter_index].FR2 = mask_reg;
}

can_peripheral_manager::extended_identifier_filter::extended_identifier_filter(
  filter_resource p_resource)
  : m_resource(p_resource)
{
}

can_peripheral_manager::mask_filter_set::mask_filter_set(hal::u8 p_filter_index,
                                                         fifo_assignment p_fifo)
  : filter{
    mask_filter{ { .filter_index = p_filter_index, .word_index = 0 } },
    mask_filter{ { .filter_index = p_filter_index, .word_index = 1 } },
  }
{
  // Required to change filter scale and type
  set_filter_bank_mode(filter_bank_master_control::initialization);

  // On scope exit, whether via a return or an exception, invoke this.
  nonstd::scope_exit on_exit([p_filter_index]() {
    set_filter_bank_mode(filter_bank_master_control::active);
    set_filter_activation_state(p_filter_index, filter_activation::active);
  });

  set_filter_activation_state(p_filter_index, filter_activation::not_active);
  set_filter_scale(p_filter_index, filter_scale::dual_16_bit_scale);
  set_filter_type(p_filter_index, filter_type::mask);
  set_filter_fifo_assignment(p_filter_index, p_fifo);

  auto const disable_id_reg = standard_id_to_stm_filter(disable_id.standard);
  auto const disable_mask_reg = standard_id_to_stm_filter(0x1FF);
  auto const disable_mask = (disable_mask_reg << 16) | disable_id_reg;

  can1_reg->filter_registers[p_filter_index].FR1 = disable_mask;
  can1_reg->filter_registers[p_filter_index].FR2 = disable_mask;
}

can_peripheral_manager::identifier_filter_set::identifier_filter_set(
  hal::u8 p_filter_index,
  fifo_assignment p_fifo)
  : filter{
    identifier_filter{ { .filter_index = p_filter_index, .word_index = 0 } },
    identifier_filter{ { .filter_index = p_filter_index, .word_index = 1 } },
    identifier_filter{ { .filter_index = p_filter_index, .word_index = 2 } },
    identifier_filter{ { .filter_index = p_filter_index, .word_index = 3 } },
  }
{
  // Required to change filter scale and type
  set_filter_bank_mode(filter_bank_master_control::initialization);

  // On scope exit, whether via a return or an exception, invoke this.
  nonstd::scope_exit on_exit([p_filter_index]() {
    set_filter_bank_mode(filter_bank_master_control::active);
    set_filter_activation_state(p_filter_index, filter_activation::active);
  });

  set_filter_activation_state(p_filter_index, filter_activation::not_active);
  set_filter_scale(p_filter_index, filter_scale::dual_16_bit_scale);
  set_filter_type(p_filter_index, filter_type::list);
  set_filter_fifo_assignment(p_filter_index, p_fifo);

  auto const disable_reg = standard_id_to_stm_filter(disable_id.standard);
  auto const disable_mask = (disable_reg << 16) | disable_reg;
  can1_reg->filter_registers[p_filter_index].FR1 = disable_mask;
  can1_reg->filter_registers[p_filter_index].FR2 = disable_mask;
}

can_peripheral_manager::extended_identifier_filter_set::
  extended_identifier_filter_set(hal::u8 p_filter_index, fifo_assignment p_fifo)
  : filter{
    can_peripheral_manager::extended_identifier_filter{
      { .filter_index = p_filter_index, .word_index = 0 } },
    can_peripheral_manager::extended_identifier_filter{
      { .filter_index = p_filter_index, .word_index = 1 } },
  }
{
  // Required to set filter scale and type
  set_filter_bank_mode(filter_bank_master_control::initialization);

  // On scope exit, whether via a return or an exception, invoke this.
  nonstd::scope_exit on_exit([p_filter_index]() {
    set_filter_bank_mode(filter_bank_master_control::active);
    set_filter_activation_state(p_filter_index, filter_activation::active);
  });

  set_filter_activation_state(p_filter_index, filter_activation::not_active);
  set_filter_scale(p_filter_index, filter_scale::single_32_bit_scale);
  set_filter_type(p_filter_index, filter_type::list);
  set_filter_fifo_assignment(p_filter_index, p_fifo);

  auto const disable_mask = extended_id_to_stm_filter(disable_id.extended);

  can1_reg->filter_registers[p_filter_index].FR1 = disable_mask;
  can1_reg->filter_registers[p_filter_index].FR2 = disable_mask;
}

// =============================================================================
//
// ::driver_allow()...
//
// =============================================================================

void can_peripheral_manager::identifier_filter::driver_allow(
  std::optional<u16> p_id)
{
  auto const id = p_id.value_or(disable_id.standard);

  set_filter_activation_state(m_resource.filter_index,
                              filter_activation::not_active);

  auto& filter = can1_reg->filter_registers[m_resource.filter_index];
  auto const reg = standard_id_to_stm_filter(id);

  switch (m_resource.word_index) {
    case 0:
      hal::bit_modify(filter.FR1).insert<standard_filter_bank::sub_bank1>(reg);
      break;
    case 1:
      hal::bit_modify(filter.FR1).insert<standard_filter_bank::sub_bank2>(reg);
      break;
    case 2:
      hal::bit_modify(filter.FR2).insert<standard_filter_bank::sub_bank1>(reg);
      break;
    case 3:
      hal::bit_modify(filter.FR2).insert<standard_filter_bank::sub_bank2>(reg);
      break;
    default:
      hal::safe_throw(hal::operation_not_supported(this));
      break;
  }

  set_filter_activation_state(m_resource.filter_index,
                              filter_activation::active);
}

void can_peripheral_manager::mask_filter::driver_allow(
  std::optional<pair> p_pair)
{
  auto const selected_pair = p_pair.value_or(pair{
    .id = disable_id.standard,
    .mask = 0x1FF,
  });

  auto& filter = can1_reg->filter_registers[m_resource.filter_index];

  set_filter_activation_state(m_resource.filter_index,
                              filter_activation::not_active);

  auto const id_reg = standard_id_to_stm_filter(selected_pair.id);
  auto const mask_reg = standard_id_to_stm_filter(selected_pair.mask);

  if (m_resource.word_index == 0) {
    hal::bit_modify(filter.FR1)
      .insert<standard_filter_bank::sub_bank1>(id_reg)
      .insert<standard_filter_bank::sub_bank2>(mask_reg);
  } else {
    hal::bit_modify(filter.FR2)
      .insert<standard_filter_bank::sub_bank1>(id_reg)
      .insert<standard_filter_bank::sub_bank2>(mask_reg);
  }

  set_filter_activation_state(m_resource.filter_index,
                              filter_activation::active);
}

void can_peripheral_manager::extended_identifier_filter::driver_allow(
  std::optional<u32> p_id)
{
  auto const id = p_id.value_or(disable_id.extended);
  auto const reg = extended_id_to_stm_filter(id);

  set_filter_activation_state(m_resource.filter_index,
                              filter_activation::not_active);

  auto& filter = can1_reg->filter_registers[m_resource.filter_index];

  if (m_resource.word_index == 0) {
    filter.FR1 = reg;
  } else {
    filter.FR2 = reg;
  }

  set_filter_activation_state(m_resource.filter_index,
                              filter_activation::active);
}

void can_peripheral_manager::extended_mask_filter::driver_allow(
  std::optional<pair> p_pair)
{
  auto const selected_pair = p_pair.value_or(pair{
    .id = disable_id.standard,
    .mask = 0x1FFF'FFFF,
  });

  auto const id_reg = extended_id_to_stm_filter(selected_pair.id);
  auto const mask_reg = extended_id_to_stm_filter(selected_pair.mask);

  auto& filter = can1_reg->filter_registers[m_filter_index];

  set_filter_activation_state(m_filter_index, filter_activation::not_active);
  filter.FR1 = id_reg;
  filter.FR2 = mask_reg;
  set_filter_activation_state(m_filter_index, filter_activation::active);
}

// =============================================================================
//
// Filter Destructors
//
// =============================================================================

can_peripheral_manager::~can_peripheral_manager()
{
  hal::cortex_m::disable_interrupt(irq::can1_rx0);
  hal::cortex_m::disable_interrupt(irq::can1_rx1);
  hal::cortex_m::disable_interrupt(irq::can1_sce);
  power_off(peripheral::can1);
}

can_peripheral_manager::transceiver::~transceiver()
{
  can_receive_buffer = {};
}

can_peripheral_manager::interrupt::~interrupt()
{
  can_v2_receive_handler = std::nullopt;
}

can_peripheral_manager::bus_manager::~bus_manager()
{
  can_v2_bus_off_handler = std::nullopt;
}

// NOLINTNEXTLINE(bugprone-exception-escape)
can_peripheral_manager::identifier_filter_set::~identifier_filter_set()
{
  // Free filter bank for use by a different identifier filter
  auto const index = filter[0].m_resource.filter_index;
  acquired_banks.reset(index);
  set_filter_activation_state(index, filter_activation::not_active);
}

// NOLINTNEXTLINE(bugprone-exception-escape)
can_peripheral_manager::mask_filter_set::~mask_filter_set()
{
  auto const index = filter[0].m_resource.filter_index;
  acquired_banks.reset(index);
  set_filter_activation_state(index, filter_activation::not_active);
}

can_peripheral_manager::extended_identifier_filter_set::
  // NOLINTNEXTLINE(bugprone-exception-escape)
  ~extended_identifier_filter_set()
{
  auto const index = filter[0].m_resource.filter_index;
  acquired_banks.reset(index);
  set_filter_activation_state(index, filter_activation::not_active);
}

// NOLINTNEXTLINE(bugprone-exception-escape)
can_peripheral_manager::extended_mask_filter::~extended_mask_filter()
{
  // Free filter bank for use by a different identifier filter
  acquired_banks.reset(m_filter_index);
  set_filter_activation_state(m_filter_index, filter_activation::not_active);
}
}  // namespace hal::stm32f1

#pragma GCC diagnostic pop
