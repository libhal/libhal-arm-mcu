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

#include <libhal-util/bit.hpp>

namespace hal::stm32f411 {
struct adc_cnfg_t
{
  /// Offset: 0x00 ADC status register
  std::uint32_t volatile sr;
  /// Offset: 0x04 ADC control register 1
  std::uint32_t volatile cr1;
  /// Offset: 0x08 ADC control register 2
  std::uint32_t volatile cr2;
  std::uint32_t volatile smpr1;
  std::uint32_t volatile smpr2;
  std::uint32_t volatile jofr1;
  std::uint32_t volatile jofr2;
  std::uint32_t volatile jofr3;
  std::uint32_t volatile jofr4;
  std::uint32_t volatile htr;
  std::uint32_t volatile ltr;
  std::uint32_t volatile sqr1;
  std::uint32_t volatile sqr2;
  std::uint32_t volatile sqr3;
  std::uint32_t volatile jsqr;
  std::uint32_t volatile jdr1;
  std::uint32_t volatile jdr2;
  std::uint32_t volatile jdr3;
  std::uint32_t volatile jdr4;
  std::uint32_t volatile dr;
};
struct adc_common_reg_t
{
  std::uint32_t reserved0;
  /// Offset: 0x04 ADC common control register
  std::uint32_t ccr;
};
struct adc_common_control_reg
{
  /// 00: PCLK2 divided by 2
  /// 01: PCLK2 divided by 4
  /// 10: PCLK2 divided by 6
  /// 11: PCLK2 divided by 8
  static constexpr auto prescaler = bit_mask::from<17, 16>();
  
  /// 0: Vbat channel disabled
  /// 1: Vbat channel enabled
  static constexpr auto vbat_enable = bit_mask::from<22>();
  
  /// 0: Temperature sensor and VREFINT channel disabled
  /// 1: Temperature sensor and VREFINT channel enabled
  static constexpr auto vrefint_enable = bit_mask::from<23>();
};
/**
 * @brief  The STM32F411 only has ADC1 available
 * @return adc_cnfg_t& - return the config registers for adc1
 *
 */
inline adc_cnfg_t* adc_cnfg =
  reinterpret_cast<adc_cnfg_t*>(0x40010000 + 0x2000);

inline adc_common_reg_t* adc_common =
  reinterpret_cast<adc_common_reg_t*>(0x40010000 + 0x2000 + 0x300);
}  // namespace hal::stm32f411
