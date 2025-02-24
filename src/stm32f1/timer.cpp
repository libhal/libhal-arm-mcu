#include <libhal-arm-mcu/stm32_generic/pwm.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32f1/timer.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/enum.hpp>
#include <libhal/error.hpp>
#include <libhal/units.hpp>

namespace hal::stm32f1 {
inline void* pwm_timer1 =
  reinterpret_cast<void*>(0x4001'2C00);  // advanced timer
inline void* pwm_timer2 = reinterpret_cast<void*>(0x4000'0000);
inline void* pwm_timer3 = reinterpret_cast<void*>(0x4000'0400);
inline void* pwm_timer4 = reinterpret_cast<void*>(0x4000'0800);
inline void* pwm_timer5 = reinterpret_cast<void*>(0x4000'0C00);
inline void* pwm_timer8 =
  reinterpret_cast<void*>(0x4001'3400);  // advanced timer
inline void* pwm_timer9 = reinterpret_cast<void*>(0x4001'4C00);
inline void* pwm_timer10 = reinterpret_cast<void*>(0x4001'5000);
inline void* pwm_timer11 = reinterpret_cast<void*>(0x4001'5400);
inline void* pwm_timer12 = reinterpret_cast<void*>(0x4000'1800);
inline void* pwm_timer13 = reinterpret_cast<void*>(0x4000'1C00);
inline void* pwm_timer14 = reinterpret_cast<void*>(0x4000'2000);

template<peripheral select>
hal::stm32f1::pwm_wrapper advanced_timer<select>::acquire_pwm(pin_type p_pin)
{
  void* p_reg;
  if constexpr (select == peripheral::timer1) {
    p_reg = pwm_timer1;
  } else {
    p_reg = pwm_timer8;
  }
  return { p_reg, select, true, static_cast<pins>(p_pin) };
}

template<peripheral select>
hal::stm32f1::pwm_wrapper general_purpose_timer<select>::acquire_pwm(
  pin_type p_pin)
{
  void* p_reg;
  if constexpr (select == peripheral::timer2) {
    p_reg = pwm_timer2;
  } else if constexpr (select == peripheral::timer3) {
    p_reg = pwm_timer3;
  } else if constexpr (select == peripheral::timer4) {
    p_reg = pwm_timer4;
  } else if constexpr (select == peripheral::timer5) {
    p_reg = pwm_timer5;
  } else if constexpr (select == peripheral::timer9) {
    p_reg = pwm_timer9;
  } else if constexpr (select == peripheral::timer10) {
    p_reg = pwm_timer10;
  } else if constexpr (select == peripheral::timer11) {
    p_reg = pwm_timer11;
  } else if constexpr (select == peripheral::timer12) {
    p_reg = pwm_timer12;
  } else if constexpr (select == peripheral::timer13) {
    p_reg = pwm_timer13;
  } else {
    p_reg = pwm_timer14;
  }
  return { p_reg, select, false, static_cast<pins>(p_pin) };
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
