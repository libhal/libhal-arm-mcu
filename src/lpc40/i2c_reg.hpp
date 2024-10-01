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

namespace hal::lpc40 {
struct i2c_reg_t
{
  /// Offset: 0x000 i2c control set register (r/w)
  std::uint32_t volatile control_set;
  /// offset: 0x004 i2c status register (r/ )
  std::uint32_t const volatile stat;
  /// offset: 0x008 i2c data register (r/w)
  std::uint32_t volatile dat;
  /// offset: 0x00c i2c peripheral address register 0 (r/w)
  std::uint32_t volatile address0;
  /// offset: 0x010 sch duty cycle register high half word (r/w)
  std::uint32_t volatile duty_cycle_high;
  /// offset: 0x014 scl duty cycle register low half word (r/w)
  std::uint32_t volatile duty_cycle_low;
  /// offset: 0x018 i2c control clear register ( /w)
  std::uint32_t volatile control_clear;
  /// offset: 0x01c monitor mode control register (r/w)
  std::uint32_t volatile monitor_mode_control;
  /// offset: 0x020 i2c peripheral address register 1 (r/w)
  std::uint32_t volatile address1;
  /// offset: 0x024 i2c peripheral address register 2 (r/w)
  std::uint32_t volatile address2;
  /// offset: 0x028 i2c peripheral address register 3 (r/w)
  std::uint32_t volatile address3;
  /// offset: 0x02c data buffer register ( /w)
  std::uint32_t const volatile data_buffer;
  /// offset: 0x030 i2c peripheral address mask register 0 (r/w)
  std::uint32_t volatile mask0;
  /// offset: 0x034 i2c peripheral address mask register 1 (r/w)
  std::uint32_t volatile mask1;
  /// offset: 0x038 i2c peripheral address mask register 2 (r/w)
  std::uint32_t volatile mask2;
  /// offset: 0x03c i2c peripheral address mask register 3 (r/w)
  std::uint32_t volatile mask3;
};

/// lpc40xx i2c peripheral control register flags
namespace i2c_control {
// AA
static constexpr auto assert_acknowledge = 1 << 2;
// SI
static constexpr auto interrupt = 1 << 3;
// STO
static constexpr auto stop = 1 << 4;
// STA
static constexpr auto start = 1 << 5;
// I2EN
static constexpr auto interface_enable = 1 << 6;
};  // namespace i2c_control

/// lpc40xx i2c peripheral state numbers
enum class i2c_host_state : std::uint32_t
{
  bus_error = 0x00,
  start_condition = 0x08,
  repeated_start = 0x10,
  peripheral_address_write_sent_received_ack = 0x18,
  peripheral_address_write_sent_received_nack = 0x20,
  transmitted_data_received_ack = 0x28,
  transmitted_data_received_nack = 0x30,
  arbitration_lost = 0x38,
  peripheral_address_read_sent_received_ack = 0x40,
  peripheral_address_read_sent_received_nack = 0x48,
  received_data_transmitted_ack = 0x50,
  received_data_transmitted_nack = 0x58,
  own_address_received = 0xA0,
  do_nothing = 0xF8
};

inline i2c_reg_t* i2c_reg0 = reinterpret_cast<i2c_reg_t*>(0x4001'C000);
inline i2c_reg_t* i2c_reg1 = reinterpret_cast<i2c_reg_t*>(0x4005'C000);
inline i2c_reg_t* i2c_reg2 = reinterpret_cast<i2c_reg_t*>(0x400A'0000);
}  // namespace hal::lpc40
