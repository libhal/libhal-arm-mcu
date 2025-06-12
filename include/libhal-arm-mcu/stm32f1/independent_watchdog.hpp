#pragma once
#include <libhal/units.hpp>

namespace hal::stm32f1 {

class independent_watchdog
{
public:
  /**
   * @brief start the watchdog countdown
   */
  void start();
  /**
   * @brief resets the watchdog countdown
   */
  void reset();
  /**
   * @brief configures the watchdog countdown frequency and counter to specified
   * time
   *
   * @param p_wait_time coundown time till the watchdog resets
   *
   * the actual wait time may be 66%-133% of the specified time due to clock
   * variance (sec. 7.2.5, pg. 96)
   */
  void set_countdown_time(hal::time_duration p_wait_time);

  /**
   * @brief checks if watchdog reset flag is set
   */
  virtual bool check_flag();
  /**
   * @brief clears reset flags
   */
  virtual void clear_flag();
};

}  // namespace hal::stm32f1
