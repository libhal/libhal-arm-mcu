#include "libhal-arm-mcu/stm32f1/constants.hpp"
#include "pin.hpp"
#include <libhal-arm-mcu/stm32_generic/pwm.hpp>
#include <libhal-arm-mcu/stm32f1/timer.hpp>
#include <libhal-util/bit.hpp>
#include <libhal/error.hpp>
#include <libhal/units.hpp>

#include "power.hpp"
namespace hal::stm32f1 {
template<peripheral select>
hal::stm32f1::pwm_wrapper advanced_timer<select>::acquire_pwm(
  pins p_pin,
  void* p_reg,
  hertz current_timer_frequency)
{
  int channel = 0;
  switch (p_pin) {
    case pins::pa8:
      configure_pin({ .port = 'A', .pin = 8 }, push_pull_alternative_output);
      channel = 1;
      break;
    case pins::pa9:
      configure_pin({ .port = 'A', .pin = 9 }, push_pull_alternative_output);
      channel = 2;
      break;
    case pins::pa10:
      configure_pin({ .port = 'A', .pin = 10 }, push_pull_alternative_output);
      channel = 3;
      break;
    case pins::pa11:
      configure_pin({ .port = 'A', .pin = 11 }, push_pull_alternative_output);
      channel = 4;
      break;
    default:
      break;  // will never go in here
  }

  return { p_reg,
           { .channel = channel,
             .frequency = current_timer_frequency,
             .is_advanced = true },
           (hal::u16)p_pin };
}
template<>
hal::stm32f1::pwm_wrapper advanced_timer<peripheral::timer1>::acquire_pwm(
  pin_type p_pin)
{
  // static_assert(select == peripheral::timer1,
  //               "You can acquire only a timer1 or timer8 pin in this class");
  power_on(peripheral::timer1);

  return this->acquire_pwm(
    (pins)p_pin, pwm_timer1, stm32f1::frequency(peripheral::timer1));
}

template<peripheral select>
hal::stm32f1::pwm_wrapper general_purpose_timer<select>::acquire_pwm(
  pins p_pin,
  void* p_reg,
  hertz current_timer_frequency)
{
  // a generic pwm class requires a pin and a channel in order to use the right
  // registers, therefore we pass in the channel as an argumet in the generic
  // pwm class's constructor
  int channel = 0;
  switch (p_pin) {
    case pins::pa0:
      configure_pin({ .port = 'A', .pin = 0 }, push_pull_alternative_output);
      channel = 1;
      break;
    case pins::pa1:
      configure_pin({ .port = 'A', .pin = 1 }, push_pull_alternative_output);
      channel = 2;
      break;
    case pins::pa2:
      configure_pin({ .port = 'A', .pin = 2 }, push_pull_alternative_output);
      channel = 3;
      break;
    case pins::pa3:
      configure_pin({ .port = 'A', .pin = 3 }, push_pull_alternative_output);
      channel = 4;
      break;
    case pins::pa6:
      configure_pin({ .port = 'A', .pin = 6 }, push_pull_alternative_output);
      channel = 1;
      break;
    case pins::pa7:
      configure_pin({ .port = 'A', .pin = 7 }, push_pull_alternative_output);
      channel = 2;
      break;
    case pins::pb0:
      configure_pin({ .port = 'B', .pin = 0 }, push_pull_alternative_output);
      channel = 3;
      break;
    case pins::pb1:
      configure_pin({ .port = 'B', .pin = 1 }, push_pull_alternative_output);
      channel = 4;
      break;
    case pins::pb6:
      configure_pin({ .port = 'B', .pin = 6 }, push_pull_alternative_output);
      channel = 1;
      break;
    case pins::pb7:
      configure_pin({ .port = 'B', .pin = 7 }, push_pull_alternative_output);
      channel = 2;
      break;
    case pins::pb8:
      configure_pin({ .port = 'B', .pin = 8 }, push_pull_alternative_output);
      channel = 3;
      break;
    case pins::pb9:
      configure_pin({ .port = 'B', .pin = 9 }, push_pull_alternative_output);
      channel = 4;
      break;
    default:
      break;  // will never go in here
  }
  return { p_reg,
           { .channel = channel,
             .frequency = current_timer_frequency,
             .is_advanced = false },
           (hal::u16)p_pin };
}
template<>
hal::stm32f1::pwm_wrapper
general_purpose_timer<peripheral::timer2>::acquire_pwm(pin_type p_pin)
{
  power_on(peripheral::timer2);

  return this->acquire_pwm(
    (pins)p_pin, pwm_timer2, stm32f1::frequency(peripheral::timer2));
};
template<>
hal::stm32f1::pwm_wrapper
general_purpose_timer<peripheral::timer3>::acquire_pwm(pin_type p_pin)
{
  power_on(peripheral::timer3);

  return this->acquire_pwm(
    (pins)p_pin, pwm_timer3, stm32f1::frequency(peripheral::timer3));
};
template<>
hal::stm32f1::pwm_wrapper
general_purpose_timer<peripheral::timer4>::acquire_pwm(pin_type p_pin)
{
  power_on(peripheral::timer4);

  return this->acquire_pwm(
    (pins)p_pin, pwm_timer4, stm32f1::frequency(peripheral::timer4));
};
// default peripheral_map constructor
template<peripheral>
struct peripheral_map
{
  using pin = void;
};

// telling the compiler which instances to compile
template class advanced_timer<peripheral::timer1>;
template class general_purpose_timer<peripheral::timer2>;
template class general_purpose_timer<peripheral::timer3>;
template class general_purpose_timer<peripheral::timer4>;

}  // namespace hal::stm32f1
