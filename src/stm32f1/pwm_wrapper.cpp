#include "libhal-arm-mcu/stm32f1/pwm_wrapper.hpp"
#include <libhal/error.hpp>
#include <libhal/units.hpp>

namespace hal::stm32f1 {
hal::u16 pwm_wrapper::m_availability;

pwm_wrapper::pwm_wrapper(void* p_reg,
                         stm32_generic::pwm_settings settings,
                         hal::u16 pin_num)
  : m_pwm(p_reg, settings)
  , m_pin_num(pin_num)
{
  auto const pwm_pin_mask = bit_mask{ .position = m_pin_num, .width = 1 };
  if (not hal::bit_extract(bit_mask::from(m_pin_num), m_availability)) {
    bit_modify(m_availability)
      .set(pwm_pin_mask);  // need to handle pwm pin availability somewhere
  } else {
    hal::safe_throw(hal::device_or_resource_busy(nullptr));
  }
}
void pwm_wrapper::driver_frequency(hertz p_frequency)
{
  m_pwm.driver_frequency(p_frequency);
}
void pwm_wrapper::driver_duty_cycle(float p_duty_cycle)
{
  m_pwm.driver_duty_cycle(p_duty_cycle);
}
pwm_wrapper::~pwm_wrapper()
{
  auto const pwm_pin_mask = bit_mask{ .position = m_pin_num, .width = 1 };
  bit_modify(m_availability).clear(pwm_pin_mask);
}
}  // namespace hal::stm32f1
