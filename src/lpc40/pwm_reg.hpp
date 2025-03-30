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

#include <cstdint>

namespace hal::lpc40 {
/**
 * @brief Register map for the lpc40xx PWM peripheral
 *
 */
struct pwm_reg_t
{
  /// Offset: 0x000 Interrupt Register (R/W)
  std::uint32_t volatile interrupt_register;
  /// Offset: 0x004 Timer Control Register (R/W)
  std::uint32_t volatile timer_control_register;
  /// Offset: 0x008 Timer Counter Register (R/W)
  std::uint32_t volatile timer_counter_register;
  /// Offset: 0x00C Prescale Register (R/W)
  std::uint32_t volatile prescale_register;
  /// Offset: 0x010 Prescale Counter Register (R/W)
  std::uint32_t volatile prescale_counter_register;
  /// Offset: 0x014 Match Control Register (R/W)
  std::uint32_t volatile match_control_register;
  /// Offset: 0x018 Match Register 0 (R/W)
  std::uint32_t volatile match_register_0;
  /// Offset: 0x01C Match Register 1 (R/W)
  std::uint32_t volatile match_register_1;
  /// Offset: 0x020 Match Register 2 (R/W)
  std::uint32_t volatile match_register_2;
  /// Offset: 0x024 Match Register 3 (R/W)
  std::uint32_t volatile match_register_3;
  /// Offset: 0x028 Capture Control Register (R/W)
  std::uint32_t volatile capture_control_register;
  /// Offset: 0x02C Capture Register 0 (R/ )
  std::uint32_t const volatile capture_register_0;
  /// Offset: 0x030 Capture Register 1 (R/ )
  std::uint32_t const volatile capture_register_1;
  /// Offset: 0x034 Capture Register 2 (R/ )
  std::uint32_t const volatile capture_register_2;
  /// Offset: 0x038 Capture Register 3 (R/ )
  std::uint32_t const volatile capture_register_3;
  std::uint32_t reserved0;
  /// Offset: 0x040 Match Register 4 (R/W)
  std::uint32_t volatile match_register_4;
  /// Offset: 0x044 Match Register 5 (R/W)
  std::uint32_t volatile match_register_5;
  /// Offset: 0x048 Match Register 6 (R/W)
  std::uint32_t volatile match_register_6;
  /// Offset: 0x04C PWM Control Register (R/W)
  std::uint32_t volatile pwm_control_register;
  /// Offset: 0x050 Load Enable Register (R/W)
  std::uint32_t volatile load_enable_register;
  std::uint32_t reserved1[7];
  /// Offset: 0x070 Counter Control Register (R/W)
  std::uint32_t volatile counter_control_register;
};

inline pwm_reg_t* pwm_reg0 = reinterpret_cast<pwm_reg_t*>(0x4001'4000);
inline pwm_reg_t* pwm_reg1 = reinterpret_cast<pwm_reg_t*>(0x4001'8000);

}  // namespace hal::lpc40
