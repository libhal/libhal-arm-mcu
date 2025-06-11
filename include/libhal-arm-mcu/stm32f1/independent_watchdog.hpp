#pragma once

#include <libhal/units.hpp>

namespace hal::stm32f1 {

/**
 * @brief start the watchdog countdown
 */
void start_independent_watchdog();
/**
 * @brief resets the watchdog countdown counter
 */
void reset_independent_watchdog_counter();
/**
 * @brief configures the watchdog countdown frequency and counter to specified
 * time
 *
 * the actual wait time may be 66%-133% of the specified time due to clock variance (sec. 7.2.5, pg. 96)
 */
void set_independent_watchdog_countdown_time(hal::time_duration wait_time);
/**
 * @brief checks if indpendet watchdog reset flag is set
 */
bool check_independent_watchdog_flag();
/**
 * @brief clears reset flags
 */
void clear_reset_flags();

}  // namespace hal::stm32f1
