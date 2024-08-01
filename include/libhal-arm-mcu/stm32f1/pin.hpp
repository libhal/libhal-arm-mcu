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

namespace hal::stm32f1 {
/**
 * @brief Make JTAG pins not associated with SWD available as IO
 *
 * The GPIO pins PB3, PB4, and PA15 are default initalized to be used for JTAG
 * purposes. If you are using SWD and want to use these pins as GPIO or as
 * other alternative functions, this function MUST be called.
 *
 */
void release_jtag_pins();

/**
 * @brief Main (Master) Clock Output options
 *
 * The following options are available to be sent to the output of the MCO pin
 */
enum class mco_source : std::uint8_t
{
  system_clock = 0b100,
  high_speed_internal = 0b101,
  high_speed_external = 0b110,
  pll_clock_divided_by_2 = 0b111,
};

/**
 * @brief Output a clock on the PA8 pin
 *
 * @param p_source - source clock to channel to the PA8 pin
 */
void activate_mco_pa8(mco_source p_source);

/**
 * @brief Remap pins for can bus peripheral
 *
 */
enum class can_pins : std::uint8_t
{
  pa11_pa12 = 0b00,
  pb9_pb8 = 0b10,
  pd0_pd1 = 0b11,
};
}  // namespace hal::stm32f1
