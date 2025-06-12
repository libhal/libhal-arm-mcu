#pragma once

#include <libhal/units.hpp>
#include <libhal-arm-mcu/watchdog.hpp>

namespace hal::stm32f1 {

class indepedent_watchdog : public hal::watchdog {
public:
    /**
    * @brief configures the watchdog countdown frequency and counter to specified
    * time
    *
    * the actual wait time may be 66%-133% of the specified time due to clock variance (sec. 7.2.5, pg. 96)
    */
    virtual void set_independent_watchdog_countdown_time(hal::time_duration wait_time);
};

}  // namespace hal::stm32f1
