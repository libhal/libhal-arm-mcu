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
#include <cstdint>

#include <libhal-util/bit.hpp>

namespace hal::lpc40 {
/// adc register map
struct adc_reg_t
{
  /// Number of channels
  static constexpr size_t channel_length = 8;
  /// Offset: 0x000 A/D Control Register (R/W)
  uint32_t volatile control;
  /// Offset: 0x004 A/D Global Data Register (R/W)
  uint32_t volatile global_data;
  /// Offset: 0x008 Reserved 0
  std::array<uint32_t, 1> reserved0;
  /// Offset: 0x00C A/D Interrupt Enable Register (R/W)
  uint32_t volatile interrupt_enable;
  /// Offset: 0x010-0x02C A/D Channel 0..7 Data Register (R/W)
  std::array<uint32_t volatile, channel_length> data;
  /// Offset: 0x030 A/D Status Register (R/ )
  uint32_t const volatile stat;
  /// Offset: 0x034 A/D Trim Calibration (R/W)
  uint32_t volatile trim;
};

/// Namespace containing the bit_mask objects that are used to manipulate the
/// lpc40xx ADC Control register.
namespace adc_control_register {
/// In burst mode, sets the ADC channels to be automatically converted.
/// It bit position represents 1 channel with this 8 channel ADC.
/// In software mode, this should hold only a single 1 for the single
/// channel to be converted.
static constexpr auto channel_select = hal::bit_mask::from<0, 7>();

/// Sets the channel's clock divider. Potentially saving power if clock is
/// reduced further.
static constexpr auto clock_divider = hal::bit_mask::from<8, 15>();

/// Enable Burst Mode for the ADC. See BurstMode() method of this class to
/// learn more about what it is and how it works.
static constexpr auto burst_enable = hal::bit_mask::from<16>();

/// Power on the ADC
static constexpr auto power_enable = hal::bit_mask::from<21>();

/// In order to start a conversion a start code must be inserted into this
/// bit location.
static constexpr auto start_code = hal::bit_mask::from<24, 26>();

/// Not used in this driver, but allows the use of an external pins to
/// trigger a conversion. This flag indicates if rising or falling edges
/// trigger the conversion.
/// 1 = falling, 0 = rising.
static constexpr auto start_edge = hal::bit_mask::from<27>();
};  // namespace adc_control_register

/// Namespace containing the bit_mask objects that are used to manipulate the
/// lpc40xx ADC Global Data register.
namespace adc_data_register {
/// Result mask holds the latest result from the last ADC that was converted
static constexpr auto result = hal::bit_mask::from<4, 15>();

/// Converted channel mask indicates which channel was converted in the
/// latest conversion.
static constexpr auto converted_channel = hal::bit_mask::from<24, 26>();

/// Holds whether or not the ADC overran its conversion.
static constexpr auto overrun = hal::bit_mask::from<30>();

/// Indicates when the ADC conversion is complete.
static constexpr auto done = hal::bit_mask::from<31>();
};  // namespace adc_data_register

constexpr std::intptr_t lpc_apb0_base = 0x40000000UL;
constexpr std::intptr_t lpc_adc_addr = lpc_apb0_base + 0x34000;
// NOLINTNEXTLINE(performance-no-int-to-ptr)
inline auto* adc_reg = reinterpret_cast<adc_reg_t*>(lpc_adc_addr);
}  // namespace hal::lpc40
