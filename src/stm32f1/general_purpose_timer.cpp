#include <cmath>

#include "libhal-arm-mcu/stm32f1/constants.hpp"
#include <cstddef>
#include <libhal-arm-mcu/stm32f1/general_purpose_timer.hpp>
#include <libhal-util/bit.hpp>
#include <libhal/error.hpp>

#include "stm32f1/power.hpp"

namespace hal::stm32f1 {
std::bitset<16> general_purpose_timer::pwm_availability;

general_purpose_timer::general_purpose_timer(peripheral peripheral)
{
  pwm_availability = {};
  power_on(peripheral);
}
general_purpose_timer::pwm general_purpose_timer::acquire_pwm(
  general_purpose_timer::pwm::pwm_pins p_pin)
{
  if (not pwm_availability.test((int)p_pin)) {
    return pwm(p_pin);
  } else {
    hal::safe_throw(hal::resource_unavailable_try_again(nullptr));
  }
}
}  // namespace hal::stm32f1
