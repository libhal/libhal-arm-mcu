#include "libhal-arm-mcu/stm32f1/constants.hpp"
#include <libhal-arm-mcu/stm32f1/timer.hpp>
#include <libhal-util/bit.hpp>
#include <libhal/error.hpp>
#include <libhal/units.hpp>

#include "stm32f1/power.hpp"

namespace hal::stm32f1 {

general_purpose_timer::general_purpose_timer(peripheral peripheral)
{
  if (peripheral == peripheral::timer2 || peripheral == peripheral::timer3 ||
      peripheral == peripheral::timer4) {
    power_on(peripheral);
  } else {
    hal::safe_throw(hal::operation_not_supported(nullptr));
  }
}

advanced_timer::advanced_timer(peripheral peripheral)
{
  if (peripheral == peripheral::timer1) {
    power_on(peripheral);
  } else {
    hal::safe_throw(hal::operation_not_supported(nullptr));
  }
}
pwm general_purpose_timer::acquire_pwm(pwm::pins p_pin)
{
  // make sure that the p_pin is the corresponding register
  if (!hal::bit_extract(bit_mask{ .position = (hal::u16)p_pin, .width = 1 },
                        pwm::availability)) {
    return { p_pin };
  } else {
    hal::safe_throw(hal::resource_unavailable_try_again(nullptr));
  }
}

pwm advanced_timer::acquire_pwm(pwm::pins p_pin)
{
  // make sure that the p_pin is the corresponding register
  if (!hal::bit_extract(bit_mask{ .position = (hal::u16)p_pin, .width = 1 },
                        pwm::availability)) {
    return { p_pin };
  } else {
    hal::safe_throw(hal::resource_unavailable_try_again(nullptr));
  }
}
}  // namespace hal::stm32f1
