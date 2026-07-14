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

#pragma once

#include <array>

#include <libhal-util/bit.hpp>
#include <libhal/units.hpp>

namespace hal::stm32f1 {

struct can_tx_mailbox_t
{
  u32 volatile TIR;
  u32 volatile TDTR;
  u32 volatile TDLR;
  u32 volatile TDHR;
};

struct can_fifo_mailbox_t
{
  u32 volatile RIR;
  u32 volatile RDTR;
  u32 volatile RDLR;
  u32 volatile RDHR;
};

struct can_filter_register_t
{
  u32 volatile FR1;
  u32 volatile FR2;
};

/**
 * @brief Controller Area Network
 */

struct can_reg_t
{
  u32 volatile MCR;
  u32 volatile MSR;
  u32 volatile TSR;
  u32 volatile RF0R;
  u32 volatile RF1R;
  u32 volatile IER;
  u32 volatile ESR;
  u32 volatile BTR;
  std::array<u32, 88> reserved0;
  std::array<can_tx_mailbox_t, 3> transmit_mailbox;
  std::array<can_fifo_mailbox_t, 2> fifo_mailbox;
  std::array<u32, 12> reserved1;
  u32 volatile FMR;
  u32 volatile FM1R;
  u32 volatile reserved2;
  u32 volatile FS1R;
  u32 volatile reserved3;
  u32 volatile FFA1R;
  u32 volatile reserved4;
  u32 volatile FA1R;
  std::array<u32, 8> volatile reserved5;
  // Limited to only 14 on connectivity line devices
  std::array<can_filter_register_t, 28> filter_registers;
};

inline auto* can1_reg = reinterpret_cast<can_reg_t*>(0x4000'6400);

/// This struct holds bit timing values.
/// It is HW mapped to a 32-bit register: BTR (pg. 683).
struct bus_timing  // NOLINT
{
  /// Baud Rate Prescaler
  static constexpr auto prescalar = bit_mask::from<0, 9>();
  /// Time Segment 1
  static constexpr auto time_segment1 = bit_mask::from<16, 19>();
  /// Time Segment 2
  static constexpr auto time_segment2 = bit_mask::from<20, 22>();
  /// Resynchronization Jump Width
  static constexpr auto sync_jump_width = bit_mask::from<24, 25>();
  /// Loop back mode (debug)
  static constexpr auto loop_back_mode = bit_mask::from<30>();
  /// Silent Mode (debug)
  static constexpr auto silent_mode = bit_mask::from<31>();
};

/// This struct holds bit values for master control of CANx.
/// It is HW mapped to a 32-bit register: MCR (pg. 674).
struct master_control  // NOLINT
{
  /// Software sets this bit to request the CAN hardware to enter
  /// initialization mode.
  static constexpr auto initialization_request = bit_mask::from<0>();
  /// Software sets this bit to request the CAN hardware to enter sleep mode.
  static constexpr auto sleep_mode_request = bit_mask::from<1>();
  /// Set the transmission order when several mailboxes are pending at the
  /// same time.
  static constexpr auto transmit_fifo_priority = bit_mask::from<2>();
  /// Lock the FIFO from receiving new messages.
  static constexpr auto receive_fifo_locked = bit_mask::from<3>();
  /// Disable CAN hardware from retransmiting until successfully transmitted.
  static constexpr auto no_automatic_retransmission = bit_mask::from<4>();
  /// Controls the behavior of the CAN hardware on message reception during
  /// Sleep.
  static constexpr auto automatic_wakeup_mode = bit_mask::from<5>();
  /// Controls the behavior of the CAN hardware on leaving Bus-Off state.
  static constexpr auto automatic_bus_off_management = bit_mask::from<6>();
  /// Enable Time Triggered Communication mode.
  static constexpr auto time_triggered_comm_mode = bit_mask::from<7>();
  /// Force a master reset of the bxCan and go to Sleep mode.
  static constexpr auto can_master_reset = bit_mask::from<15>();
  /// Freeze CAN reception/transmission during debug.
  static constexpr auto debug_freeze = bit_mask::from<16>();
};

/// This struct holds bit assignments for the Master Status Register (MSR)
/// It is HW mapped to a 32-bit register: MSR (pg. 676).
struct master_status  // NOLINT
{
  /// Indicates to the software that the CAN hardware is now in initialization
  /// mode
  static constexpr auto initialization_acknowledge = bit_mask::from<0>();
  /// Indicates to the software that the CAN hardware is now in Sleep mode.
  static constexpr auto sleep_acknowledge = bit_mask::from<1>();
  /// Set by hardware when a bit of the ESR has been set
  static constexpr auto error_interrupt = bit_mask::from<2>();
  /// Set by hardware to signal that a SOF bit has been set.
  static constexpr auto wakeup_interrupt = bit_mask::from<3>();
  /// Set by hardware to signal that the bxCan has entered sleep.
  static constexpr auto sleep_acknowledge_interrupt = bit_mask::from<4>();
  /// Indicates if the CAN is a Transmitter
  static constexpr auto transmit_mode = bit_mask::from<8>();
  /// Indicates if the CAN is a receiver
  static constexpr auto receive_mode = bit_mask::from<9>();
  /// Holds the last value of Rx
  static constexpr auto last_sample_point = bit_mask::from<10>();
  /// Monitors the actual value of the CAN_Rx pin.
  static constexpr auto can_rx_signal = bit_mask::from<11>();
};

/// This struct holds CANx transmit status information.
/// It is HW mapped to a 32-bit register: TSR (pg. 677).
struct transmit_status  // NOLINT
{
  /// Mailbox 0 - Set by hardware when the last request (transmit or abort)
  /// has been completed
  static constexpr auto request_completed_mailbox0 = bit_mask::from<0>();
  /// Mailbox 0 - Hardware updates this bit after each transmission attempt
  static constexpr auto transmission_ok_mailbox0 = bit_mask::from<1>();
  /// Mailbox 0 - Set when the previous TX failed due to arbitration lost
  static constexpr auto arbitration_lost_mailbox0 = bit_mask::from<2>();
  /// Mailbox 0 - Set when the previous TX failed due to an error
  static constexpr auto transmission_error_mailbox0 = bit_mask::from<3>();
  /// Mailbox 0 - Set by software to abort the transmission for the mailbox
  static constexpr auto abort_request_mailbox0 = bit_mask::from<7>();
  /// Mailbox 1 - Set by hardware when the last request (transmit or abort)
  /// has been completed
  static constexpr auto request_completed_mailbox1 = bit_mask::from<8>();
  /// Mailbox 1 - Hardware updates this bit after each transmission attempt
  static constexpr auto transmission_ok_mailbox1 = bit_mask::from<9>();
  /// Mailbox 1 - Set when the previous TX failed due to arbitration lost
  static constexpr auto arbitration_lost_mailbox1 = bit_mask::from<10>();
  /// Mailbox 1 - Set when the previous TX failed due to an error
  static constexpr auto transmission_error_mailbox1 = bit_mask::from<11>();
  /// Mailbox 1 - Set by software to abort the transmission for the mailbox
  static constexpr auto abort_request_mailbox1 = bit_mask::from<15>();
  /// Mailbox 2 - Set by hardware when the last request (transmit or abort)
  /// has been completed
  static constexpr auto request_completed_mailbox2 = bit_mask::from<16>();
  /// Mailbox 2 - Hardware updates this bit after each transmission attempt
  static constexpr auto transmission_ok_mailbox2 = bit_mask::from<17>();
  /// Mailbox 2 - Set when the previous TX failed due to arbitration lost
  static constexpr auto arbitration_lost_mailbox2 = bit_mask::from<18>();
  /// Mailbox 2 - Set when the previous TX failed due to an error
  static constexpr auto transmission_error_mailbox2 = bit_mask::from<19>();
  /// Mailbox 2 - Set by software to abort the transmission for the mailbox
  static constexpr auto abort_request_mailbox2 = bit_mask::from<23>();
  /// Number of empty mailboxes
  static constexpr auto mailbox_code = bit_mask::from<24, 25>();
  /// Mailbox 0 - Set by hardware to indicate empty
  static constexpr auto transmit_mailbox0_empty = bit_mask::from<26>();
  /// Mailbox 1 - Set by hardware to indicate empty
  static constexpr auto transmit_mailbox1_empty = bit_mask::from<27>();
  /// Mailbox 2 - Set by hardware to indicate empty
  static constexpr auto transmit_mailbox2_empty = bit_mask::from<28>();
  /// Set by hardware when more than one mailbox is pending and mailbox 0 has
  /// lower priority
  static constexpr auto lowest_priority_flag_mailbox0 = bit_mask::from<29>();
  /// Set by hardware when more than one mailbox is pending and mailbox 1 has
  /// lower priority
  static constexpr auto lowest_priority_flag_mailbox1 = bit_mask::from<30>();
  /// Set by hardware when more than one mailbox is pending and mailbox 2 has
  /// lower priority
  static constexpr auto lowest_priority_flag_mailbox2 = bit_mask::from<31>();
};

/// This struct holds the bitmap for enabling CANx interrupts.
/// It is HW mapped to a 32-bit register: TSR (pg. 680).
struct interrupt_enable_register  // NOLINT
{
  /// Transmit mailbox empty interrupt enable
  static constexpr auto transmit_mailbox_empty = bit_mask::from<0>();
  /// FIFO 0 message pending interrupt enable
  static constexpr auto fifo0_message_pending = bit_mask::from<1>();
  /// FIFO 0 full interrupt enable
  static constexpr auto fifo0_full = bit_mask::from<2>();
  /// FIFO 0 overrun interrupt enable
  static constexpr auto fifo0_overrun = bit_mask::from<3>();
  /// FIFO 1 message pending interrupt enable
  static constexpr auto fifo1_message_pending = bit_mask::from<4>();
  /// FIFO 1 full interrupt enable
  static constexpr auto fifo1_full = bit_mask::from<5>();
  /// FIFO 1 overrun interrupt enable
  static constexpr auto fifo1_overrun = bit_mask::from<6>();
  /// Error warning interrupt enable
  static constexpr auto error_warning = bit_mask::from<8>();
  /// Error passive interrupt enable
  static constexpr auto error_passive = bit_mask::from<9>();
  /// Bus-off interrupt enable
  static constexpr auto bus_off = bit_mask::from<10>();
  /// Last error code interrupt enable
  static constexpr auto last_error_code = bit_mask::from<11>();
  /// Error interrupt enable
  static constexpr auto error_interrupt = bit_mask::from<15>();
  /// Wakeup interrupt enable
  static constexpr auto wakeup = bit_mask::from<16>();
  /// Sleep interrupt enable
  static constexpr auto sleep = bit_mask::from<17>();
};

/// Strut holding the masks for the error status register
struct error_status_register
{
  /// Set to 1 if the device has been put in to the bus off state
  static constexpr auto bus_off = bit_mask::from<2>();
};

/// This struct holds the bitmap for the mailbox identifier.
/// It is represents 32-bit register: CAN_TIxR(0 - 2) (pg. 685).
/// It is represents 32-bit register: CAN_RIxR(0 - 1) (pg. 688).
struct mailbox_identifier  // NOLINT
{
  enum class id_type : std::uint8_t
  {
    standard = 0,
    extended = 1,
  };

  /// Transmit
  static constexpr auto transmit_mailbox_request = bit_mask::from<0>();
  /// Receive/Transmit
  static constexpr auto remote_request = bit_mask::from<1>();
  /// Receive/Transmit
  static constexpr auto identifier_type = bit_mask::from<2>();
  /// Receive/Transmit
  static constexpr auto standard_identifier = bit_mask::from<21, 31>();
  /// Receive/Transmit
  static constexpr auto extended_identifier = bit_mask::from<3, 31>();
};

/// This struct holds the bitmap for data length control and time stamp.
/// It is represents 32-bit register: CAN_TDTxR(0 - 2) (pg. 686).
/// It is represents 32-bit register: CAN_RDTxR(0 - 1) (pg. 689).
struct frame_length_and_info  // NOLINT
{
  /// Receive/Transmit
  static constexpr auto data_length_code = bit_mask::from<0, 3>();
  /// Transmit
  static constexpr auto transmit_global_time = bit_mask::from<8>();
  /// Receive
  static constexpr auto filter_match_index = bit_mask::from<8, 15>();
  /// Receive/Transmit
  static constexpr auto message_time_stamp = bit_mask::from<16, 31>();
};

/// This struct holds the bitmap for the FIFOx Status
/// It is represents 32-bit register: CAN_RFxR(0 - 1) (pg. 680).
struct fifo_status  // NOLINT
{
  /// Indicates how many messages are pending in the receive FIFO
  static constexpr auto messages_pending = bit_mask::from<0, 1>();
  /// Set by hardware when three messages are stored in the FIFO.
  static constexpr auto is_fifo_full = bit_mask::from<3>();
  /// Set by hardware when a new message has been released and passed the
  /// filter while the FIFO is full.
  static constexpr auto is_fifo_overrun = bit_mask::from<4>();
  /// Release the output mailbox of the FIFO.
  static constexpr auto release_output_mailbox = bit_mask::from<5>();
};

/// This struct holds the bitmap for the filter master control.
/// It is represents 32-bit register: CAN_FMR (pg. 691).
struct filter_master
{
  /// Initialization mode for filter banks
  static constexpr auto initialization_mode = bit_mask::from<0>();
  /// Defines the start bank for CAN2
  static constexpr auto can2_start_bank = bit_mask::from<8, 13>();
};

/// This enumeration labels the initialization state of a filter.
/// Used with CAN Filter Master Register (CAN_FMR) (pg. 691).
enum class filter_bank_master_control : std::uint8_t
{
  /// Active filters state
  active = 0,
  /// Initialization state for the filter
  initialization = 1
};

/// This enumeration labels the mode of a filter
/// Used with CAN Filter Mode Register (CAN_FM1R) (pg. 692)
enum class filter_type : std::uint8_t
{
  /// Mask what bits in the identifier to accept
  mask = 0,
  /// List the identifier to accept
  list = 1
};

/// This enumeration labels the scale of a filter
/// Used with CAN Filter Scale Register (CAN_FS1R) (pg. 692)
enum class filter_scale : std::uint8_t
{
  /// Use two 16 bit identifiers
  dual_16_bit_scale = 0,
  /// Use one 32 bit identifier
  single_32_bit_scale = 1
};

/// This enumeration labels the activation state of a filter
/// Used with CAN Filter Activation Register (CAN_FFA1R) (pg. 693)
enum class filter_activation : std::uint8_t
{
  /// Disable filter
  not_active = 0,
  /// Enable fIlter
  active = 1
};

struct can_id
{
  static constexpr auto standard_id = hal::bit_mask::from(10, 0);
  static constexpr auto standard_id_part = hal::bit_mask::from(29, 0);
};

struct standard_filter_bank
{
  static constexpr auto standard_id1 = hal::bit_mask::from(5, 15);
  static constexpr auto rtr1 = hal::bit_mask::from(4);
  static constexpr auto id_extension1 = hal::bit_mask::from(3);
  static constexpr auto extended_id1 = hal::bit_mask::from(0, 2);

  static constexpr auto sub_bank1 = hal::bit_mask::from(0, 15);
  static constexpr auto sub_bank2 = hal::bit_mask::from(16, 31);
};
struct extended_filter_bank
{
  static constexpr auto id = hal::bit_mask::from(3, 31);
  static constexpr auto id_extension = hal::bit_mask::from(2);
  static constexpr auto rtr = hal::bit_mask::from(1);
  static constexpr auto reserved = hal::bit_mask::from(0);
};
}  // namespace hal::stm32f1
