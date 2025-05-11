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
#include <libhal-util/units.hpp>

namespace hal::lpc40 {
/// gpio peripheral register map
struct lpc_gpio_t
{
  /// Offset: 0x000 Determine pin direction (0 == Input, 1 = Output) (R/W)
  std::uint32_t volatile direction;
  /// Offset: 0x004 - 0x00C
  std::array<std::uint32_t, 3> reserved0;
  /// Offset: 0x010 (R/W)
  std::uint32_t volatile mask;
  /// Offset: 0x014 Pin status and output control (R/W)
  std::uint32_t volatile pin;
  /// Offset: 0x018 Write 1 to this to set output pin as 1 (HIGH voltage) (R/W)
  std::uint32_t volatile set;
  /// Offset: 0x01C Write 1 to this to Set output pin to 0 (LOW voltage) (R/W)
  std::uint32_t volatile clear;
};

inline constexpr intptr_t ahb_base = 0x20080000UL;

// NOLINTBEGIN(performance-no-int-to-ptr)
inline std::array gpio_reg{
  reinterpret_cast<lpc_gpio_t*>(ahb_base + 0x18000),
  reinterpret_cast<lpc_gpio_t*>(ahb_base + 0x18020),
  reinterpret_cast<lpc_gpio_t*>(ahb_base + 0x18040),
  reinterpret_cast<lpc_gpio_t*>(ahb_base + 0x18060),
  reinterpret_cast<lpc_gpio_t*>(ahb_base + 0x18080),
  reinterpret_cast<lpc_gpio_t*>(ahb_base + 0x180a0),
};
// NOLINTEND(performance-no-int-to-ptr)

inline constexpr bit_mask pin_mask(std::uint8_t p_pin)
{
  return bit_mask{ .position = p_pin, .width = 1 };
}
}  // namespace hal::lpc40
