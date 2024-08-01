#pragma once

namespace hal::lpc40 {
/**
 * @brief Initialize interrupts for the lpc40 series processors
 *
 * Only initializes after the first call. Does nothing afterwards. Can be
 * called multiple times without issue.
 *
 */
void initialize_interrupts();
}  // namespace hal::lpc40
