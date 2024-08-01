#include <cstdint>

#include <libhal-armcortex/interrupt.hpp>
#include <libhal-stm32f1/can.hpp>
#include <libhal-stm32f1/constants.hpp>
#include <libhal-stm32f1/interrupt.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/bit_limits.hpp>
#include <libhal-util/can.hpp>
#include <libhal-util/enum.hpp>
#include <libhal-util/static_callable.hpp>
#include <libhal/error.hpp>
#include <nonstd/scope.hpp>

#include "can_reg.hpp"
#include "libhal-stm32f1/clock.hpp"
#include "libhal-stm32f1/constants.hpp"
#include "libhal-stm32f1/pin.hpp"
#include "pin.hpp"
#include "power.hpp"

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

void configure_baud_rate(stm32f1::can* p_can, can::settings const& p_settings)
{
  auto const can_frequency = frequency(peripheral::can1);
  auto const valid_divider =
    calculate_can_bus_divider(can_frequency, p_settings.baud_rate);

  if (not valid_divider) {
    hal::safe_throw(hal::operation_not_supported(p_can));
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
    .clear<bus_timing::silent_mode>()
    .clear<bus_timing::loop_back_mode>();
}

void set_filter_bank_mode(filter_bank_master_control p_mode)
{
  bit_modify(can1_reg->FMR)
    .insert<filter_master::initialization_mode>(hal::value(p_mode));
}

template<uint32_t filter>
void set_filter_type(filter_type p_filter_type)
{
  static_assert((filter >= 0 && filter <= 27),
                "There are only 28 filters available 0 - 27");
  constexpr auto filter_bit_mask = bit_mask::from(filter);
  bit_modify(can1_reg->FM1R)
    .template insert<filter_bit_mask>(hal::value(p_filter_type));
}

template<uint32_t filter>
void set_filter_scale(filter_scale p_scale)
{
  static_assert((filter >= 0 && filter <= 27),
                "There are only 28 filters available 0 - 27");
  constexpr auto filter_bit_mask = bit_mask::from(filter);
  bit_modify(can1_reg->FS1R)
    .template insert<filter_bit_mask>(hal::value(p_scale));
}

template<uint32_t filter>
void set_filter_fifo_assignment(fifo_assignment p_fifo)
{
  static_assert((filter >= 0 && filter <= 27),
                "There are only 28 filters available 0 - 27");
  constexpr auto filter_bit_mask = bit_mask::from(filter);
  bit_modify(can1_reg->FFA1R)
    .template insert<filter_bit_mask>(hal::value(p_fifo));
}

template<uint32_t filter>
void set_filter_activation_state(filter_activation p_state)
{
  static_assert((filter >= 0 && filter <= 27),
                "There are only 28 filters available 0 - 27");
  constexpr auto filter_bit_mask = bit_mask::from(filter);
  bit_modify(can1_reg->FA1R)
    .template insert<filter_bit_mask>(hal::value(p_state));
}

void enable_acceptance_filter()
{
  // Activate filter initialization mode (Set bit)
  set_filter_bank_mode(filter_bank_master_control::initialization);

  // Deactivate filter 0 (Clear bit)
  set_filter_activation_state<0>(filter_activation::not_active);

  // Configure filter 0 to single 32-bit scale configuration (Set bit)
  set_filter_scale<0>(filter_scale::single_32_bit_scale);

  // Clear filter 0 registers to accept all messages.
  can1_reg->sFilterRegister[0].FR1 = 0;
  can1_reg->sFilterRegister[0].FR2 = 0;

  // Set filter to mask mode
  set_filter_type<0>(filter_type::mask);

  // Assign filter 0 to FIFO 0 (Clear bit)
  set_filter_fifo_assignment<0>(fifo_assignment::fifo1);

  // Activate filter 0 (Set bit)
  set_filter_activation_state<0>(filter_activation::active);

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

bool is_bus_off()
{
  // True = Bus is in sleep mode
  // False = Bus has left sleep mode.
  return bit_extract<master_status::sleep_acknowledge>(can1_reg->MCR);
}

}  // namespace

can::message_t read_receive_mailbox()
{
  can::message_t message{ .id = 0 };

  uint32_t fifo0_status = can1_reg->RF0R;
  uint32_t fifo1_status = can1_reg->RF1R;

  fifo_assignment fifo_select = fifo_assignment::fifo_none;

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
  if (fifo_select == fifo_assignment::fifo1) {
    bit_modify(can1_reg->RF0R).set<fifo_status::release_output_mailbox>();
  } else if (fifo_select == fifo_assignment::fifo2) {
    bit_modify(can1_reg->RF1R).set<fifo_status::release_output_mailbox>();
  }

  return message;
}
can::can(can::settings const& p_settings, can_pins p_pins)
{
  power_on(peripheral::can1);

  set_master_mode(master_control::sleep_mode_request, false);
  set_master_mode(master_control::no_automatic_retransmission, false);
  set_master_mode(master_control::automatic_bus_off_management, false);

  can::driver_configure(p_settings);

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

  remap_pins(p_pins);

  // Ensure we have left initialization phase so the peripheral can operate
  // correctly.
  exit_initialization();
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

  configure_baud_rate(this, p_settings);
  enable_acceptance_filter();
}

void can::driver_bus_on()
{
  // RM0008 page 670 states that bus off can be recovered from by entering and
  // Request to enter initialization mode
  enter_initialization();

  // Leave Initialization mode
  exit_initialization();
}

void can::driver_send(can::message_t const& p_message)
{
  if (is_bus_off()) {
    hal::safe_throw(hal::operation_not_permitted(this));
  }

  can_data_registers_t registers = convert_message_to_stm_can(p_message);
  uint32_t status_register = can1_reg->TSR;

  // Check if any buffer is available.
  if (bit_extract<transmit_status::transmit_mailbox0_empty>(status_register)) {
    bit_modify(can1_reg->transmit_mailbox[0].TDTR)
      .insert<frame_length_and_info::data_length_code>(p_message.length);
    can1_reg->transmit_mailbox[0].TDLR = registers.data_a;
    can1_reg->transmit_mailbox[0].TDHR = registers.data_b;
    can1_reg->transmit_mailbox[0].TIR = registers.id;
    return;
  } else if (bit_extract<transmit_status::transmit_mailbox1_empty>(
               status_register)) {
    bit_modify(can1_reg->transmit_mailbox[1].TDTR)
      .insert<frame_length_and_info::data_length_code>(p_message.length);
    can1_reg->transmit_mailbox[1].TDLR = registers.data_a;
    can1_reg->transmit_mailbox[1].TDHR = registers.data_b;
    can1_reg->transmit_mailbox[1].TIR = registers.id;
    return;
  } else if (bit_extract<transmit_status::transmit_mailbox2_empty>(
               status_register)) {
    bit_modify(can1_reg->transmit_mailbox[2].TDTR)
      .insert<frame_length_and_info::data_length_code>(p_message.length);
    can1_reg->transmit_mailbox[2].TDLR = registers.data_a;
    can1_reg->transmit_mailbox[2].TDHR = registers.data_b;
    can1_reg->transmit_mailbox[2].TIR = registers.id;
    return;
  }

  hal::safe_throw(hal::resource_unavailable_try_again(this));
}

hal::callback<can::handler> can_receive_handler{};

void handler_interrupt()
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

void can::driver_on_receive(hal::callback<handler> p_handler)
{
  initialize_interrupts();
  can_receive_handler = p_handler;

  // Enable interrupt service routine.
  cortex_m::enable_interrupt(irq::can1_rx0, handler_interrupt);
  cortex_m::enable_interrupt(irq::can1_rx1, handler_interrupt);
  cortex_m::enable_interrupt(irq::can1_sce, handler_interrupt);

  bit_modify(can1_reg->IER)
    .set<interrupt_enable_register::fifo0_message_pending>();
  bit_modify(can1_reg->IER)
    .set<interrupt_enable_register::fifo1_message_pending>();
}
}  // namespace hal::stm32f1
