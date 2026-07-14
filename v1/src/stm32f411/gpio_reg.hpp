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

#include <libhal-arm-mcu/stm32f411/pin.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/enum.hpp>
#include <libhal/units.hpp>

namespace hal::stm32f411 {
/// gpio peripheral register map
struct gpio_config_t
{
  /// Offset: 0x000 pin mode (00 = Input, 01 = Output, 10 = Alternate Function
  /// mode, 11 Analog) (R/W)
  u32 volatile pin_mode;
  /// Offset: 0x004 output type(0 = output push-pull, 1 = output open-drain)
  u32 volatile output_type;
  /// Offset: 0x008 output speed(00 = Low speed, 01 = Medium speed, 10 = Fast
  /// speed, 11 = High speed)
  u32 volatile output_speed;
  /// Offset: 0x00C port pull-up/pull-down (00 = no pull-up/pull-down, 01 =
  /// pull-up, 10 pull-down)
  u32 volatile pull_up_pull_down;
  /// Offset: 0x010 port input data (RO)
  u32 volatile input_data;
  /// Offset: 0x014 port output data
  u32 volatile output_data;
  /// Offset: 0x018low port set (0 = no action, 1 = reset)
  std::uint16_t volatile set;
  /// Offset: 0x018high port reset (0 = no action, 1 = reset)
  std::uint16_t volatile reset;
  /// Offset: 0x01C config lock
  u32 volatile lock;
  /// Offset: 0x020 alternate function low (bits 0 - 7)
  u32 volatile alt_function_low;
  /// Offset: 0x024 alternate function high (bits 8 - 15)
  u32 volatile alt_function_high;
};

inline constexpr intptr_t ahb_base = 0x4002'0000UL;

static inline gpio_config_t* get_gpio_reg(hal::stm32f411::peripheral p_port)
{
  // STM has dedicated memory blocks where every 2^10 is a new gpio register
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  return reinterpret_cast<gpio_config_t*>(ahb_base +
                                          (hal::value(p_port) << 10));
}
}  // namespace hal::stm32f411
