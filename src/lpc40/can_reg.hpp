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

#pragma once

#include <cstdint>

#include <libhal-util/bit.hpp>

namespace hal::lpc40 {
struct can_acceptance_filter_ram_t
{
  /// Mask IDs
  std::uint32_t volatile mask[512];
};

struct can_acceptance_filter_t
{
  /// Offset: 0x00000000 - Acceptance Filter Register
  std::uint32_t volatile acceptance_filter;
  /// Offset: 0x00000004 - Standard Frame Individual Start Address Register
  std::uint32_t volatile SFF_sa;
  /// Offset: 0x00000008 - Standard Frame Group Start Address Register
  std::uint32_t volatile SFF_GRP_sa;
  /// Offset: 0x0000000C - Extended Frame Start Address Register
  std::uint32_t volatile EFF_sa;
  /// Offset: 0x00000010 - Extended Frame Group Start Address Register
  std::uint32_t volatile EFF_GRP_sa;
  /// Offset: 0x00000014 - End of AF Tables register
  std::uint32_t volatile ENDofTable;
  /// Offset: 0x00000018 - LUT Error Address register
  std::uint32_t const volatile LUTerrAd;
  /// Offset: 0x0000001C - LUT Error Register
  std::uint32_t const volatile LUTerr;
  /// Offset: 0x00000020 - CAN Central Transmit Status Register
  std::uint32_t volatile full_can_transmit_status;
  /// Offset: 0x00000024 - FullCAN Interrupt and Capture registers 0
  std::uint32_t volatile FCANIC0;
  /// Offset: 0x00000028 - FullCAN Interrupt and Capture registers 1
  std::uint32_t volatile FCANIC1;
};

struct can_central_reg_t
{
  std::uint32_t const volatile TxSR;
  std::uint32_t const volatile RxSR;
  std::uint32_t const volatile MSR;
};

struct can_reg_t
{
  /// Offset: 0x00000000 - Controls the operating mode of the CAN Controller
  std::uint32_t volatile MOD;
  /// Offset: 0x00000004 - Command bits that affect the state
  std::uint32_t volatile CMR;
  /// Offset: 0x00000008 - Global Controller Status and Error Counters
  std::uint32_t volatile GSR;
  /// Offset: 0x0000000C - Interrupt status, Arbitration Lost Capture, Error
  /// Code Capture
  std::uint32_t const volatile ICR;
  /// Offset: 0x00000010 - Interrupt Enable Register
  std::uint32_t volatile IER;
  /// Offset: 0x00000014 - Bus Timing Register
  std::uint32_t volatile BTR;
  /// Offset: 0x00000018 - Error Warning Limit
  std::uint32_t volatile EWL;
  /// Offset: 0x0000001C - Status Register
  std::uint32_t const volatile SR;
  /// Offset: 0x00000020 - Receive frame status
  std::uint32_t volatile RFS;
  /// Offset: 0x00000024 - Received Identifier
  std::uint32_t volatile RID;
  /// Offset: 0x00000028 - Received data bytes 1-4
  std::uint32_t volatile RDA;
  /// Offset: 0x0000002C - Received data bytes 5-8
  std::uint32_t volatile RDB;
  /// Offset: 0x00000030 - Transmit frame info (Tx Buffer 1)
  std::uint32_t volatile TFI1;
  /// Offset: 0x00000034 - Transmit Identifier (Tx Buffer 1)
  std::uint32_t volatile TID1;
  /// Offset: 0x00000038 - Transmit data bytes 1-4 (Tx Buffer 1)
  std::uint32_t volatile TDA1;
  /// Offset: 0x0000003C - Transmit data bytes 5-8 (Tx Buffer 1)
  std::uint32_t volatile TDB1;
  /// Offset: 0x00000040 - Transmit frame info (Tx Buffer 2)
  std::uint32_t volatile TFI2;
  /// Offset: 0x00000044 - Transmit Identifier (Tx Buffer 2)
  std::uint32_t volatile TID2;
  /// Offset: 0x00000048 - Transmit data bytes 1-4 (Tx Buffer 2)
  std::uint32_t volatile TDA2;
  /// Offset: 0x0000004C - Transmit data bytes 5-8 (Tx Buffer 2)
  std::uint32_t volatile TDB2;
  /// Offset: 0x00000050 - Transmit frame info (Tx Buffer 3)
  std::uint32_t volatile TFI3;
  /// Offset: 0x00000054 - Transmit Identifier (Tx Buffer 3)
  std::uint32_t volatile TID3;
  /// Offset: 0x00000058 - Transmit data bytes 1-4 (Tx Buffer 3)
  std::uint32_t volatile TDA3;
  /// Offset: 0x0000005C - Transmit data bytes 5-8 (Tx Buffer 3)
  std::uint32_t volatile TDB3;
};

/// Container for the LPC40xx CAN BUS registers
struct can_lpc_message
{
  /// TFI register contents
  std::uint32_t frame = 0;
  /// TID register contents
  std::uint32_t id = 0;
  /// TDA register contents
  std::uint32_t data_a = 0;
  /// TDB register contents
  std::uint32_t data_b = 0;
};  // namespace can_lpc_message

/// https://www.nxp.com/docs/en/user-guide/UM10562.pdf (pg. 554)
enum class can_commands : std::uint32_t
{
  release_rx_buffer = 0x04,
  send_tx_buffer1 = 0x21,
  send_tx_buffer2 = 0x41,
  send_tx_buffer3 = 0x81,
  self_reception_send_tx_buffer1 = 0x30,
  accept_all_messages = 0x02,
};

/// This struct holds bit timing values. It is used to configure the CAN bus
/// clock. It is HW mapped to a 32-bit register: BTR (pg. 562)
namespace can_bus_timing {
/// The peripheral bus clock is divided by this value
static constexpr auto prescalar = bit_mask::from<0, 9>();

/// Used to compensate for positive and negative edge phase errors
static constexpr auto sync_jump_width = bit_mask::from<14, 15>();
/// The delay from the nominal Sync point to the sample point is (this value
/// plus one) CAN clocks.
static constexpr auto time_segment1 = bit_mask::from<16, 19>();

/// The delay from the sample point to the next nominal sync point isCAN
/// clocks. The nominal CAN bit time is (this value plus the value in
/// time_segment1 plus 3) CAN clocks.
static constexpr auto time_segment2 = bit_mask::from<20, 22>();

/// How many times the bus is sampled; 0 == once, 1 == 3 times
static constexpr auto sampling = bit_mask::from<23>();
};  // namespace can_bus_timing

/// This struct holds interrupt flags and capture flag status. It is HW mapped
/// to a 16-bit register: ICR (pg. 557)
namespace can_interrupts {
// ICR - Interrupt and Capture Register
// NOTE: Bits 1-10 are cleared by the CAN controller
//       as soon as they are read.
//       Bits 16-23 & 24-31 are released by the CAN
//       controller as soon as they are read.

/// Assert interrupt when a new message has been received
static constexpr auto received_message = bit_mask::from<0>();

/// Assert interrupt when TX Buffer 1 has finished or aborted its
/// transmission.
static constexpr auto tx1_ready = bit_mask::from<1>();

/// Assert interrupt when bus status or error status is asserted.
static constexpr auto error_warning = bit_mask::from<2>();

/// Assert interrupt on data overrun occurs
static constexpr auto data_overrun = bit_mask::from<3>();

/// Assert interrupt when CAN controller is sleeping and was woken up from
/// bus activity.
static constexpr auto wakeup = bit_mask::from<4>();

/// Assert interrupt when the CAN Controller has reached the Error Passive
/// Status (error counter exceeds 127)
static constexpr auto error_passive = bit_mask::from<5>();

/// Assert interrupt when arbitration is lost
static constexpr auto arbitration_lost = bit_mask::from<6>();

/// Assert interrupt on bus error
static constexpr auto bus_error = bit_mask::from<7>();

/// Assert interrupt when any message has been successfully transmitted.
static constexpr auto identifier_ready = bit_mask::from<8>();

/// Assert interrupt when TX Buffer 2 has finished or aborted its
/// transmission.
static constexpr auto tx2_ready = bit_mask::from<9>();

/// Assert interrupt when TX Buffer 3 has finished or aborted its
/// transmission.
static constexpr auto tx3_ready = bit_mask::from<10>();

/// Error Code Capture status bits to be read during an interrupt
static constexpr auto error_code_location = bit_mask::from<16, 20>();
/// Indicates if the error occurred during transmission (0) or receiving (1)
static constexpr auto error_code_direction = bit_mask::from<21>();
/// The type of bus error that occurred such as bit error, stuff error, etc
static constexpr auto error_code_type = bit_mask::from<22, 23>();
/// Bit location of where arbitration was lost.
static constexpr auto arbitration_lost_loc = bit_mask::from<24, 31>();
};  // namespace can_interrupts

/// This struct holds CAN controller global status information.
/// It is a condensed version of the status register.
/// It is HW mapped to a 32-bit register: GSR (pg. 555)
namespace can_global_status {
/// If 1, receive buffer has at least 1 complete message stored
static constexpr auto receive_buffer = bit_mask::from<0>();

/// Bus status bit. If this is '1' then the bus is active, otherwise the bus
/// is bus off.
static constexpr auto bus_error = bit_mask::from<7>();
};  // namespace can_global_status

/// This struct holds CAN controller status information. It is HW mapped to a
/// 32-bit register: SR (pg. 564). Many of them are not here because they have
/// counter parts in GSR (global status register).
namespace can_buffer_status {
/// TX1 Buffer has been released
static constexpr auto tx1_released = bit_mask::from<2>();

/// TX2 Buffer has been released
static constexpr auto tx2_released = bit_mask::from<10>();

/// TX3 Buffer has been released
static constexpr auto tx3_released = bit_mask::from<18>();

/// Will be 0 if the device is Bus-On.
/// Will be 1 if the device is Bus-Off.
static constexpr auto bus_status = bit_mask::from<15>();
static constexpr std::uint32_t bus_on = 0;
static constexpr std::uint32_t bus_off = 1;
};  // namespace can_buffer_status

/// CAN BUS modes
namespace can_mode {
/// Reset CAN Controller, allows configuration registers to be modified.
static constexpr auto reset = bit_mask::from<0>();

/// Put device into Listen Only Mode, device will not acknowledge, messages.
static constexpr auto listen_only = bit_mask::from<1>();

/// Put device on self test mode.
static constexpr auto self_test = bit_mask::from<2>();

/// Enable transmit priority control. When enabled, allows a particular
static constexpr auto tx_priority = bit_mask::from<3>();

/// Put device to Sleep Mode.
static constexpr auto sleep_mode = bit_mask::from<4>();

/// Receive polarity mode. If 1 RD input is active high
static constexpr auto rx_polarity = bit_mask::from<5>();

/// Put CAN into test mode, which allows the TD pin to reflect its bits ot
/// the RD pin.
static constexpr auto test = bit_mask::from<7>();
};  // namespace can_mode

/// CAN Bus frame bit masks for the TFM and RFM registers
namespace can_frame_info {
/// The message priority bits (not used in this implementation)
static constexpr auto priority = bit_mask::from<0, 7>();

/// The length of the data
static constexpr auto length = bit_mask::from<16, 19>();

/// If set to 1, the message becomes a remote request message
static constexpr auto remote_request = bit_mask::from<30>();

/// If 0, the ID is 11-bits, if 1, the ID is 29-bits.
static constexpr auto format = bit_mask::from<31>();
};  // namespace can_frame_info

/// Pointer to the LPC CAN BUS acceptance filter peripheral in memory
inline auto* can_acceptance_filter =
  reinterpret_cast<can_acceptance_filter_t*>(0x4003'C000);
inline auto* can_reg1 = reinterpret_cast<can_reg_t*>(0x4004'4000);
inline auto* can_reg2 = reinterpret_cast<can_reg_t*>(0x4004'8000);
}  // namespace hal::lpc40
