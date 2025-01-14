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

#include <libhal/units.hpp>

#include "constants.hpp"

namespace hal::stm32f411 {
struct dma_channel_stream_t
{
  uint8_t stream;
  uint8_t channel;
};
/**
 * @brief Sets up the DMA memory to memory transfer mode
 *
 * @param p_dma Select DMA
 * @param p_stream_channel loops through the listed streams until an available
 * stream is found. Will spinlock until spare channel is found
 * @param p_source source address
 * @param p_destination destination address
 */
void set_dma_memory_transfer(peripheral p_dma,
                             std::span<dma_channel_stream_t> p_stream_channel,
                             std::span<hal::byte> p_source,
                             std::span<byte> p_destination);
/// Maximum length of a buffer that the stm32f411 series dma controller can
/// handle.
constexpr std::uint32_t max_dma_length = 65'535;
}  // namespace hal::stm32f411
