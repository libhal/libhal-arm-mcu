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

#include <limits>

#include <libhal/units.hpp>

#include "constants.hpp"

namespace hal::stm32f411 {
enum class dma : u8
{
  dma1 = static_cast<uint32_t>(peripheral::dma1),
  dma2 = static_cast<uint32_t>(peripheral::dma2),
};
/**
 * @brief Sets up the DMA memory to memory transfer mode
 *
 * @param p_dma Select DMA
 * @param p_source source span of bytes
 * @param p_destination destination span of bytes
 */
void set_dma_memory_transfer(dma p_dma,
                             std::span<byte const> const p_source,
                             std::span<byte> const p_destination);
/// Maximum length of a buffer that the stm32f411 series dma controller can
/// handle.
constexpr auto max_dma_length = std::numeric_limits<u16>::max();
}  // namespace hal::stm32f411
