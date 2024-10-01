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

#include <array>
#include <cstddef>
#include <cstdint>

#include <libhal-arm-mcu/stm32f411/pin.hpp>
#include <libhal-util/bit.hpp>

namespace hal::stm32f411 {
/// gpio peripheral register map
struct gpio_config_t
{
  /// Offset: 0x000 pin mode (00 = Input, 01 = Output, 10 = Alternate Function
  /// mode, 11 Analog) (R/W)
  std::uint32_t volatile pin_mode;
  /// Offset: 0x004 output type(0 = output push-pull, 1 = output open-drain)
  std::uint32_t volatile output_type;
  /// Offset: 0x008 output speed(00 = Low speed, 01 = Medium speed, 10 = Fast
  /// speed, 11 = High speed)
  std::uint32_t volatile output_speed;
  /// Offset: 0x00C port pull-up/pull-down (00 = no pull-up/pull-down, 01 =
  /// pull-up, 10 pull-down)
  std::uint32_t volatile pull_up_pull_down;
  /// Offset: 0x010 port input data (RO)
  std::uint32_t volatile input_data;
  /// Offset: 0x014 port output data
  std::uint32_t volatile output_data;
  /// Offset: 0x018low port set (0 = no action, 1 = reset)
  std::uint16_t volatile set;
  /// Offset: 0x018high port reset (0 = no action, 1 = reset)
  std::uint16_t volatile reset;
  /// Offset: 0x01C config lock
  std::uint32_t volatile lock;
  /// Offset: 0x020 alternate function low (bits 0 - 7)
  std::uint32_t volatile alt_function_low;
  /// Offset: 0x024 alternate function high (bits 8 - 15)
  std::uint32_t volatile alt_function_high;
};

inline constexpr intptr_t ahb_base = 0x4002'0000UL;
static inline gpio_config_t* get_reg(hal::stm32f411::peripheral p_port)
{
  // STM has dedicated memory blocks where every 2^10 is a new
  return reinterpret_cast<gpio_config_t*>(ahb_base +
                                          (static_cast<int>(p_port) << 10));
}
}  // namespace hal::stm32f411
