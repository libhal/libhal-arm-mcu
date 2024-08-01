#pragma once

namespace hal::stm32f1 {
/**
 * @brief Initialize interrupts for the stm32f1 series processors
 *
 * Only initializes after the first call. Does nothing afterwards. Can be
 * called multiple times without issue.
 *
 */
void initialize_interrupts();
}  // namespace hal::stm32f1
