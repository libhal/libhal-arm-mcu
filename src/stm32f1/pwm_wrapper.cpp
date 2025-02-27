#include <utility>

#include <libhal-arm-mcu/stm32_generic/pwm.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32f1/pwm_wrapper.hpp>
#include <libhal-arm-mcu/stm32f1/timer.hpp>
#include <libhal/error.hpp>
#include <libhal/pwm.hpp>
#include <libhal/units.hpp>

#include "pin.hpp"

namespace {
/**
 * @brief Static variable to track PWM availability.
 */
hal::u32 availability;
}  // namespace

namespace hal::stm32f1 {
pwm::pwm(void* p_reg,
         stm32f1::peripheral p_select,
         bool p_is_advanced,
         stm32f1::pins p_pin)
  : m_pwm(unsafe{})
  , m_pwm_frequency(unsafe{}, p_reg)
  , m_pin_num(hal::value(p_pin))
  , m_select(p_select)
{
  // a generic pwm class requires a pin and a channel in order to use the right
  // registers, therefore we pass in the channel as an argument in the generic
  // pwm class's constructor
  u8 channel = 0;
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
      channel = p_select == peripheral::timer9 ? 1 : 3;
      break;
    case pins::pa3:
      configure_pin({ .port = 'A', .pin = 3 }, push_pull_alternative_output);
      channel = p_select == peripheral::timer9 ? 2 : 4;
      break;
    case pins::pa6:
      configure_pin({ .port = 'A', .pin = 6 }, push_pull_alternative_output);
      channel = 1;
      break;
    case pins::pa7:
      configure_pin({ .port = 'A', .pin = 7 }, push_pull_alternative_output);
      channel = p_select == peripheral::timer14 ? 1 : 2;
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
      channel = p_select == peripheral::timer10 ? 1 : 3;
      break;
    case pins::pb9:
      configure_pin({ .port = 'B', .pin = 9 }, push_pull_alternative_output);
      channel = p_select == peripheral::timer11 ? 1 : 4;
      break;
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
    case pins::pc6:
      configure_pin({ .port = 'C', .pin = 6 }, push_pull_alternative_output);
      channel = 1;
      break;
    case pins::pc7:
      configure_pin({ .port = 'C', .pin = 7 }, push_pull_alternative_output);
      channel = 2;
      break;
    case pins::pc8:
      configure_pin({ .port = 'C', .pin = 8 }, push_pull_alternative_output);
      channel = 3;
      break;
    case pins::pc9:
      configure_pin({ .port = 'C', .pin = 9 }, push_pull_alternative_output);
      channel = 4;
      break;
    case pins::pb14:
      configure_pin({ .port = 'B', .pin = 14 }, push_pull_alternative_output);
      channel = 1;
      break;
    case pins::pb15:
      configure_pin({ .port = 'B', .pin = 15 }, push_pull_alternative_output);
      channel = 2;
      break;
    default:
      std::unreachable();
  }

  m_pwm.initialize(unsafe{},
                   p_reg,
                   {
                     .channel = channel,
                     .is_advanced = p_is_advanced,
                   });

  auto const pwm_pin_mask = bit_mask::from(m_pin_num);
  if (not hal::bit_extract(pwm_pin_mask, availability)) {
    bit_modify(availability).set(pwm_pin_mask);
  } else {
    hal::safe_throw(hal::device_or_resource_busy(nullptr));
  }
}

void pwm::driver_frequency(hertz p_frequency)
{
  m_pwm_frequency.set_group_frequency({
    .pwm_frequency = static_cast<u32>(p_frequency),
    .timer_clock_frequency = static_cast<u32>(stm32f1::frequency(m_select)),
  });
}

void pwm::driver_duty_cycle(float p_duty_cycle)
{
  m_pwm.duty_cycle(p_duty_cycle);
}

pwm::~pwm()
{
  auto const pwm_pin_mask = bit_mask::from(m_pin_num);
  bit_modify(availability).clear(pwm_pin_mask);
}

pwm16_channel::pwm16_channel(void* p_reg,
                             stm32f1::peripheral p_select,
                             bool p_is_advanced,
                             stm32f1::pins p_pin)
  : m_pwm(unsafe{})
  , m_pin_num(hal::value(p_pin))
  , m_select(p_select)
{
  // a generic pwm class requires a pin and a channel in order to use the right
  // registers, therefore we pass in the channel as an argument in the generic
  // pwm class's constructor
  u8 channel = 0;
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
      channel = p_select == peripheral::timer9 ? 1 : 3;
      break;
    case pins::pa3:
      configure_pin({ .port = 'A', .pin = 3 }, push_pull_alternative_output);
      channel = p_select == peripheral::timer9 ? 2 : 4;
      break;
    case pins::pa6:
      configure_pin({ .port = 'A', .pin = 6 }, push_pull_alternative_output);
      channel = 1;
      break;
    case pins::pa7:
      configure_pin({ .port = 'A', .pin = 7 }, push_pull_alternative_output);
      channel = p_select == peripheral::timer14 ? 1 : 2;
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
      channel = p_select == peripheral::timer10 ? 1 : 3;
      break;
    case pins::pb9:
      configure_pin({ .port = 'B', .pin = 9 }, push_pull_alternative_output);
      channel = p_select == peripheral::timer11 ? 1 : 4;
      break;
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
    case pins::pc6:
      configure_pin({ .port = 'C', .pin = 6 }, push_pull_alternative_output);
      channel = 1;
      break;
    case pins::pc7:
      configure_pin({ .port = 'C', .pin = 7 }, push_pull_alternative_output);
      channel = 2;
      break;
    case pins::pc8:
      configure_pin({ .port = 'C', .pin = 8 }, push_pull_alternative_output);
      channel = 3;
      break;
    case pins::pc9:
      configure_pin({ .port = 'C', .pin = 9 }, push_pull_alternative_output);
      channel = 4;
      break;
    case pins::pb14:
      configure_pin({ .port = 'B', .pin = 14 }, push_pull_alternative_output);
      channel = 1;
      break;
    case pins::pb15:
      configure_pin({ .port = 'B', .pin = 15 }, push_pull_alternative_output);
      channel = 2;
      break;
    default:
      std::unreachable();
  }

  m_pwm.initialize(unsafe{},
                   p_reg,
                   {
                     .channel = channel,
                     .is_advanced = p_is_advanced,
                   });

  auto const pwm_pin_mask = bit_mask::from(m_pin_num);
  if (not hal::bit_extract(pwm_pin_mask, availability)) {
    bit_modify(availability).set(pwm_pin_mask);
  } else {
    hal::safe_throw(hal::device_or_resource_busy(nullptr));
  }
}

u32 pwm16_channel::driver_frequency()
{
  return m_pwm.frequency(static_cast<u32>(stm32f1::frequency(m_select)));
}

void pwm16_channel::driver_duty_cycle(u16 p_duty_cycle)
{
  m_pwm.duty_cycle(p_duty_cycle);
}

pwm16_channel::~pwm16_channel()
{
  auto const pwm_pin_mask = bit_mask::from(m_pin_num);
  bit_modify(availability).clear(pwm_pin_mask);
}

pwm_group_frequency::pwm_group_frequency(void* p_reg,
                                         stm32f1::peripheral p_select)
  : m_pwm_frequency(unsafe{}, p_reg)
  , m_select(p_select)
{
}

void pwm_group_frequency::driver_frequency(u32 p_frequency)
{
  return m_pwm_frequency.set_group_frequency({
    .pwm_frequency = p_frequency,
    .timer_clock_frequency = static_cast<u32>(stm32f1::frequency(m_select)),
  });
}
}  // namespace hal::stm32f1
