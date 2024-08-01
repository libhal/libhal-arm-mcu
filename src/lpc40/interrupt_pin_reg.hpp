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
#include <cstdint>

#include <libhal/interrupt_pin.hpp>

namespace hal::lpc40 {
/// Matrix of gpio interrupt service routine handlers 32 x 2. Matrix does not
/// need to be initialized at startup to work because the only entries that
/// will be accessed are the entries that have been setup via
/// attach_interrupt.
inline std::array<std::array<hal::callback<interrupt_pin::handler>, 32>, 2>
  interrupt_pin_handlers{};

/// interrupt register map
struct interrupt_pin_reg_t
{
  /// Offset: 0x080 GPIO overall Interrupt Status (RO)
  const volatile uint32_t status;
  /// Offset: 0x084 GPIO Interrupt Status for Rising edge for Port 0 (RO)
  const volatile uint32_t raising_status_port0;
  /// Offset: 0x088 GPIO Interrupt Status for Falling edge for Port 0 (RO)
  const volatile uint32_t falling_status_port0;
  /// Offset: 0x08C (WO)
  volatile uint32_t clear_interrupt_port0;
  /// Offset: 0x090 (R/W)
  volatile uint32_t enable_raising_port0;
  /// Offset: 0x094 (R/W)
  volatile uint32_t enable_falling_port0;
  /// Offset: 0x098 - 0x0A0
  std::array<uint32_t, 3> reserved0;
  /// Offset: 0x0A4 (RO)
  const volatile uint32_t raising_status_port2;
  /// Offset: 0x0A8 (RO)
  const volatile uint32_t falling_status_port2;
  /// Offset: 0x0AC (WO)
  volatile uint32_t clear_interrupt_port2;
  /// Offset: 0x0B0 (R/W)
  volatile uint32_t enable_raising_port2;
  /// Offset: 0x0B4 (R/W)
  volatile uint32_t enable_falling_port2;
};

inline interrupt_pin_reg_t* interrupt_pin_reg =
  reinterpret_cast<interrupt_pin_reg_t*>(0x4002'8080);
}  // namespace hal::lpc40