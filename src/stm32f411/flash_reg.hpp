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
struct flash_config_t
{
  /// Offset 0x00: The Flash access control register is used to enable/disable
  /// the acceleration features and control the Flash memory access time
  /// according to CPU frequency.
  std::uint32_t volatile access_control_reg;
  /// Offset 0x04: The Flash key register is used to allow access to the Flash
  /// control register and so, to allow program and erase operations.
  std::uint32_t volatile key_reg;
  /// Offset 0x08: The Flash option key register is used to allow program and
  /// erase operations in the user configuration sector.
  std::uint32_t volatile option_key_reg;
  /// Offset 0x0C: The Flash status register gives information on ongoing
  /// program and erase operations.
  std::uint32_t volatile status_reg;
  /// Offset 0x10: The Flash control register is used to configure and start
  /// Flash memory operations.
  std::uint32_t volatile control_reg;
  /// Offset 0x14: The FLASH_OPTCR register is used to modify the user option
  /// bytes
  std::uint32_t volatile optional_control_reg;
};
struct flash_acess_control{
    static constexpr auto latency = bit_mask::from<3,0>();
    static constexpr auto prefetch_cache_en = bit_mask::from<8>();
    static constexpr auto instruction_cache_en = bit_mask::from<9>();
    static constexpr auto data_cache_en = bit_mask::from<10>();
    static constexpr auto instruction_cache_reset = bit_mask::from<11>();
    static constexpr auto data_cache_reset = bit_mask::from<12>();
    
};
inline flash_config_t* flash_config = reinterpret_cast<flash_config_t*>(0x4002'3C00);
}  // namespace hal::stm32f411