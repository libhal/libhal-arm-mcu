#pragma once

#include <cstdint>

namespace hal::stm32f1 {
/// Maximum length of a buffer that the stm32f1xxx series dma controller can
/// handle.
constexpr std::uint32_t max_dma_length = 65'535;
}  // namespace hal::stm32f1