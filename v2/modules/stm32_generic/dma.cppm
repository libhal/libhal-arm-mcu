// Copyright 2026 Khalil Estell and the libhal contributors
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

module;

#include <concepts>
#include <cstddef>
#include <span>

export module hal.arm_mcu.stm32_generic:dma;

import hal;

namespace hal::stm32_generic {
/// The most bytes one DMA transfer can move. The transfer count register of
/// every STM32 DMA controller is 16 bits wide.
export constexpr std::size_t max_dma_transfer = 65'535;

/**
 * @brief What the generic drivers need from a platform's DMA channel
 *
 * Each STM32 series has its own DMA controller, so the platform library
 * supplies the channel and the generic drivers only start and wait on it.
 * A channel moves 8-bit values between a peripheral's data register and
 * memory, notifies a `hal::notifier` when a transfer ends, and reports how
 * many bytes are left (zero once it has finished) and whether the transfer
 * failed. See `hal::stm32f1::dma_channel` for the meaning of each function.
 *
 * @tparam T - the platform's DMA channel type
 */
export template<class T>
concept byte_dma_channel = requires(T& p_channel,
                                    hal::u32 volatile& p_register,
                                    std::span<hal::byte> p_buffer,
                                    std::span<hal::byte const> p_data,
                                    hal::byte& p_sink,
                                    hal::byte const& p_value,
                                    std::size_t p_count,
                                    hal::notifier& p_notifier) {
  p_channel.receive(p_register, p_buffer, p_notifier);
  p_channel.receive_discarding(p_register, p_sink, p_count, p_notifier);
  p_channel.transmit(p_register, p_data, p_notifier);
  p_channel.transmit_repeated(p_register, p_value, p_count, p_notifier);
  p_channel.stop();
  { p_channel.remaining() } -> std::convertible_to<std::size_t>;
  { p_channel.failed() } -> std::convertible_to<bool>;
};
}  // namespace hal::stm32_generic
