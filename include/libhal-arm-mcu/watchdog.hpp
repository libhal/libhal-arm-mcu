#pragma once

#include <libhal/units.hpp>

namespace hal {

class watchdog
{
public:
  /**
   * @brief start the watchdog countdown
   */
  virtual void start() = 0;
  /**
   * @brief resets the watchdog countdown
   */
  virtual void reset() = 0;
  /**
   * @brief configures the watchdog to match countdown to specified time
   */
  virtual void set_countdown_time(hal::time_duration wait_time) = 0;
  /**
   * @brief checks if watchdog reset flag is set
   */
  virtual bool check_flag() = 0;
  /**
   * @brief clears reset flags
   */
  virtual void clear_flag() = 0;
};

}  // namespace hal
