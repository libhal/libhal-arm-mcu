#pragma once

#include <libhal-arm-mcu/watchdog.hpp>
#include <libhal/units.hpp>

namespace hal::stm32f1 {

class independent_watchdog : public hal::watchdog
{
public:
  void start() override;
  void reset() override;
  /**
   * @brief configures the watchdog countdown frequency and counter to specified
   * time
   *
   *
   * @param p_wait_time coundown time till the watchdog resets
   *
   * the actual wait time may be 66%-133% of the specified time due to clock
   * variance (sec. 7.2.5, pg. 96)
   */
  void set_countdown_time(hal::time_duration wait_time) override;
  
  bool check_flag() override;
  void clear_flag() override;
};

}  // namespace hal::stm32f1
