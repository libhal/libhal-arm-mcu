#include <libhal-arm-mcu/stm32_generic/pwm.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32f1/pwm_wrapper.hpp>
#include <libhal-arm-mcu/stm32f1/timer.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/enum.hpp>
#include <libhal/error.hpp>
#include <libhal/units.hpp>

#include "power.hpp"

namespace hal::stm32f1 {
// Advanced timer
inline void* pwm_timer1 = reinterpret_cast<void*>(0x4001'2C00);
// General purpose timers 2 - 5
inline void* pwm_timer2 = reinterpret_cast<void*>(0x4000'0000);
inline void* pwm_timer3 = reinterpret_cast<void*>(0x4000'0400);
inline void* pwm_timer4 = reinterpret_cast<void*>(0x4000'0800);
inline void* pwm_timer5 = reinterpret_cast<void*>(0x4000'0C00);
// Advanced timer
inline void* pwm_timer8 = reinterpret_cast<void*>(0x4001'3400);
// General purpose timers 9 - 14
inline void* pwm_timer9 = reinterpret_cast<void*>(0x4001'4C00);
inline void* pwm_timer10 = reinterpret_cast<void*>(0x4001'5000);
inline void* pwm_timer11 = reinterpret_cast<void*>(0x4001'5400);
inline void* pwm_timer12 = reinterpret_cast<void*>(0x4000'1800);
inline void* pwm_timer13 = reinterpret_cast<void*>(0x4000'1C00);
inline void* pwm_timer14 = reinterpret_cast<void*>(0x4000'2000);

namespace {
template<peripheral select>
void* peripheral_to_advanced_register()
{
  void* reg;
  if constexpr (select == peripheral::timer1) {
    reg = pwm_timer1;
  } else {
    reg = pwm_timer8;
  }
  return reg;
}

template<peripheral select>
void* peripheral_to_general_register()
{
  void* reg;
  if constexpr (select == peripheral::timer2) {
    reg = pwm_timer2;
  } else if constexpr (select == peripheral::timer3) {
    reg = pwm_timer3;
  } else if constexpr (select == peripheral::timer4) {
    reg = pwm_timer4;
  } else if constexpr (select == peripheral::timer5) {
    reg = pwm_timer5;
  } else if constexpr (select == peripheral::timer9) {
    reg = pwm_timer9;
  } else if constexpr (select == peripheral::timer10) {
    reg = pwm_timer10;
  } else if constexpr (select == peripheral::timer11) {
    reg = pwm_timer11;
  } else if constexpr (select == peripheral::timer12) {
    reg = pwm_timer12;
  } else if constexpr (select == peripheral::timer13) {
    reg = pwm_timer13;
  } else {
    reg = pwm_timer14;
  }
  return reg;
}
}  // namespace

template<peripheral select>
advanced_timer<select>::advanced_timer()
{
  power_on(select);
}

template<peripheral select>
general_purpose_timer<select>::general_purpose_timer()
{
  power_on(select);
}

template<peripheral select>
hal::stm32f1::pwm advanced_timer<select>::acquire_pwm(pin_type p_pin)
{
  return { peripheral_to_advanced_register<select>(),
           select,
           true,
           static_cast<pins>(p_pin) };
}

template<peripheral select>
hal::stm32f1::pwm general_purpose_timer<select>::acquire_pwm(pin_type p_pin)
{

  return { peripheral_to_general_register<select>(),
           select,
           false,
           static_cast<pins>(p_pin) };
}

template<peripheral select>
hal::stm32f1::pwm16_channel advanced_timer<select>::acquire_pwm16_channel(
  pin_type p_pin)
{
  return { peripheral_to_advanced_register<select>(),
           select,
           true,
           static_cast<pins>(p_pin) };
}

template<peripheral select>
hal::stm32f1::pwm16_channel
general_purpose_timer<select>::acquire_pwm16_channel(pin_type p_pin)
{
  return { peripheral_to_general_register<select>(),
           select,
           false,
           static_cast<pins>(p_pin) };
}

template<peripheral select>
hal::stm32f1::pwm_group_frequency
advanced_timer<select>::acquire_pwm_group_frequency()
{
  return { peripheral_to_advanced_register<select>(), select };
}

template<peripheral select>
hal::stm32f1::pwm_group_frequency
general_purpose_timer<select>::acquire_pwm_group_frequency()
{
  return { peripheral_to_general_register<select>(), select };
}

// Tell the compiler which instances to generate
template class advanced_timer<peripheral::timer1>;
template class advanced_timer<peripheral::timer8>;
template class general_purpose_timer<peripheral::timer2>;
template class general_purpose_timer<peripheral::timer3>;
template class general_purpose_timer<peripheral::timer4>;
template class general_purpose_timer<peripheral::timer5>;
template class general_purpose_timer<peripheral::timer9>;
template class general_purpose_timer<peripheral::timer10>;
template class general_purpose_timer<peripheral::timer11>;
template class general_purpose_timer<peripheral::timer12>;
template class general_purpose_timer<peripheral::timer13>;
template class general_purpose_timer<peripheral::timer14>;
}  // namespace hal::stm32f1
