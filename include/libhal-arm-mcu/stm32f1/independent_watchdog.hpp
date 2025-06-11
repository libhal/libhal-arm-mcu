#pragma once

#include <libhal/units.hpp>

namespace hal::stm32f1 {

// TODO: update documentation for if watch dog resets on start or if it just
// continues
/**
 * @brief start the watchdog countdown
 */
void start_independent_watchdog();
/**
 * @brief resets the watchdog countdown counter
 */
void restart_independent_watchdog();
/**
 * @brief stop and reset the watchdog countdown counter
 */
void stop_independent_watchdog();
/**
 * @brief configures the watchdog countdown frequency and counter to specified
 * time
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
