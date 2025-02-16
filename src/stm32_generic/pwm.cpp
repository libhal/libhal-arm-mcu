#include <cmath>

#include <libhal-arm-mcu/stm32f1/clock.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32f1/pin.hpp>
#include <libhal-arm-mcu/stm32f1/timer.hpp>
#include <libhal-util/bit.hpp>
#include <libhal/error.hpp>

#include "pin.hpp"
#include "power.hpp"

namespace hal::stm32_generic {

hal::u16 pwm::availability;

namespace {
[[nodiscard]] pwm_reg_t* get_pwm_reg(peripheral p_id)
{
  switch (p_id) {
    case (peripheral::timer1):
      return pwm_timer1;
    case (peripheral::timer2):
      return pwm_timer2;
    case (peripheral::timer3):
      return pwm_timer3;
    case (peripheral::timer4):
      return pwm_timer4;
    default:
      hal::safe_throw(hal::operation_not_supported(nullptr));
  }
}
[[nodiscard]] peripheral get_peripheral_id(pwm::pins p_pin) noexcept
{
  if (p_pin == pwm::pins::pa8 || p_pin == pwm::pins::pa9 ||
      p_pin == pwm::pins::pa10 || p_pin == pwm::pins::pa11) {
    return peripheral::timer1;
  } else if (p_pin == pwm::pins::pa0 || p_pin == pwm::pins::pa1 ||
             p_pin == pwm::pins::pa2 || p_pin == pwm::pins::pa3) {
    return peripheral::timer2;
  } else if (p_pin == pwm::pins::pa6 || p_pin == pwm::pins::pa7 ||
             p_pin == pwm::pins::pb0 || p_pin == pwm::pins::pb1) {
    return peripheral::timer3;
  } else if (p_pin == pwm::pins::pb6 || p_pin == pwm::pins::pb7 ||
             p_pin == pwm::pins::pb8 || p_pin == pwm::pins::pb9) {
    return peripheral::timer4;
  }
  hal::safe_throw(hal::operation_not_supported(nullptr));
}

[[nodiscard]] float get_duty_cycle(peripheral peripheral_id,
                                   uint32_t volatile* compare_register)
{
  // ccr / arr
  pwm_reg_t* const p_reg = get_pwm_reg(peripheral_id);

  static constexpr auto first_nonreserved_half = bit_mask::from<0, 15>();

  std::uint16_t compare_value =
    hal::bit_extract<first_nonreserved_half>(*compare_register);

  std::uint16_t arr_value =
    hal::bit_extract<first_nonreserved_half>(p_reg->auto_reload_register);

  return (static_cast<float>(compare_value) / static_cast<float>(arr_value));
}
void setup_channel(pwm_reg_t* p_reg, uint8_t p_channel, peripheral p_timer)
{

  uint8_t const start_pos = (p_channel - 1) * 4;

  auto const cc_enable = bit_mask::from(start_pos);
  auto const cc_polarity = bit_mask::from(start_pos + 1);
  static constexpr auto main_output_enable = bit_mask::from<15>();
  static constexpr auto ossr = bit_mask::from<11>();

  bit_modify(p_reg->cc_enable_register).set(cc_enable);
  bit_modify(p_reg->cc_enable_register).clear(cc_polarity);

  if (p_reg == pwm_timer1 || p_reg == pwm_timer8) {
    bit_modify(p_reg->break_and_deadtime_register)
      .clear(ossr)
      .set(main_output_enable);  // complementary channel stuff
  }

  return;
}
void setup(pwm::pins p_pin, peripheral peripheral_id)
{

  static constexpr auto clock_division = bit_mask::from<8, 9>();
  static constexpr auto edge_aligned_mode = bit_mask::from<5, 6>();
  static constexpr auto direction = bit_mask::from<4>();

  static constexpr auto output_compare_odd = bit_mask::from<4, 6>();
  static constexpr auto output_compare_even = bit_mask::from<12, 14>();
  static constexpr auto channel_output_select_odd = bit_mask::from<0, 1>();
  static constexpr auto channel_output_select_even = bit_mask::from<8, 9>();

  static constexpr auto counter_enable = bit_mask::from<0>();
  static constexpr auto auto_reload_preload_enable = bit_mask::from<7>();
  static constexpr auto odd_channel_preload_enable = bit_mask::from<3>();
  static constexpr auto even_channel_preload_enable = bit_mask::from<11>();
  static constexpr auto ug_bit = bit_mask::from<0>();

  // The PWM_MODE 1 makes it such that output will be high when Counter < CCR
  auto const pwm_mode_1 = 0b110U;

  auto const set_output = 0b00U;
  pwm_reg_t* const reg = get_pwm_reg(peripheral_id);

  bit_modify(reg->control_register)
    .insert<clock_division>(0b00U)
    .insert<edge_aligned_mode>(0b00U)
    .clear(direction);

  if (p_pin == pwm::pins::pa8 || p_pin == pwm::pins::pa0 ||
      p_pin == pwm::pins::pa6 || p_pin == pwm::pins::pb6) {  // channel 1 for
                                                             // timer1,2,3,4
    // Preload enable must be done for corresponding OCxPE
    bit_modify(reg->capture_compare_mode_register)
      .insert<output_compare_odd>(pwm_mode_1)
      .insert<channel_output_select_odd>(set_output)
      .set(odd_channel_preload_enable);

    setup_channel(reg, 1, peripheral_id);
  } else if (p_pin == pwm::pins::pa9 || p_pin == pwm::pins::pa1 ||
             p_pin == pwm::pins::pa7 || p_pin == pwm::pins::pb7) {  // channel 2
    bit_modify(reg->capture_compare_mode_register)
      .insert<output_compare_even>(pwm_mode_1)
      .insert<channel_output_select_even>(set_output)
      .set(even_channel_preload_enable);
    setup_channel(reg, 2, peripheral_id);

  } else if (p_pin == pwm::pins::pa10 || p_pin == pwm::pins::pa2 ||
             p_pin == pwm::pins::pb0 || p_pin == pwm::pins::pb8) {  // channel 3
    bit_modify(reg->capture_compare_mode_register_2)
      .insert<output_compare_odd>(pwm_mode_1)
      .insert<channel_output_select_odd>(set_output)
      .set(odd_channel_preload_enable);
    setup_channel(reg, 3, peripheral_id);

  } else if (p_pin == pwm::pins::pa11 || p_pin == pwm::pins::pa3 ||
             p_pin == pwm::pins::pb1 || p_pin == pwm::pins::pb9) {  // channel 4
    bit_modify(reg->capture_compare_mode_register_2)
      .insert<output_compare_even>(pwm_mode_1)
      .insert<channel_output_select_even>(set_output)
      .set(even_channel_preload_enable);

    setup_channel(reg, 4, peripheral_id);
  }
  bit_modify(reg->event_generator_register).set(ug_bit);
  bit_modify(reg->control_register).set(counter_enable);
  bit_modify(reg->control_register).set(auto_reload_preload_enable);

  // If the desired frequency is really low, and 16 bits are not enough
  // to represent that, the prescalar value can be increased. Example:
  // if prescalar = 2, 2 clock ticks would occur before incrementing
  // the counter by 1.
  reg->prescale_register = 0x0U;

  // The counter increments every clock cycle, and compares its value
  // to the CCR to check whether the output should be high or low
  reg->counter_register = 0x0U;

  // The ARR register is the top, once the counter reaches the ARR,
  // it starts counting again. ARR can be increased on decreased
  // depending on frequency.
  reg->auto_reload_register = 0xFFFF;
}

}  // namespace

pwm::pwm(pwm::pins p_pin)
  : m_pin{ p_pin }
{
  // config pin to alternate function push - pull
  m_peripheral_id = get_peripheral_id(p_pin);
  auto const p_reg = get_pwm_reg(m_peripheral_id);
  auto const pwm_pin_mask = bit_mask{ .position = (hal::u16)p_pin, .width = 1 };
  bit_modify(availability).set(pwm_pin_mask);
  power_on(m_peripheral_id);

  switch (p_pin) {
    case pwm::pins::pa10:
      configure_pin({ .port = 'A', .pin = 10 }, push_pull_alternative_output);
      m_compare_register_addr = &p_reg->capture_compare_register_3;
      break;
    case pwm::pins::pa11:
      configure_pin({ .port = 'A', .pin = 11 }, push_pull_alternative_output);
      m_compare_register_addr = &p_reg->capture_compare_register_4;
      break;
    case pwm::pins::pa8:
      configure_pin({ .port = 'A', .pin = 8 }, push_pull_alternative_output);
      m_compare_register_addr = &p_reg->capture_compare_register;
      break;
    case pwm::pins::pa9:
      configure_pin({ .port = 'A', .pin = 9 }, push_pull_alternative_output);
      m_compare_register_addr = &p_reg->capture_compare_register_2;
      break;
    case pwm::pins::pa0:
      configure_pin({ .port = 'A', .pin = 0 }, push_pull_alternative_output);
      m_compare_register_addr = &p_reg->capture_compare_register;
      break;
    case pwm::pins::pa1:
      configure_pin({ .port = 'A', .pin = 1 }, push_pull_alternative_output);
      m_compare_register_addr = &p_reg->capture_compare_register_2;
      break;
    case pwm::pins::pa2:
      configure_pin({ .port = 'A', .pin = 2 }, push_pull_alternative_output);
      m_compare_register_addr = &p_reg->capture_compare_register_3;
      break;
    case pwm::pins::pa3:
      configure_pin({ .port = 'A', .pin = 3 }, push_pull_alternative_output);
      m_compare_register_addr = &p_reg->capture_compare_register_4;
      break;
    case pwm::pins::pa6:
      configure_pin({ .port = 'A', .pin = 6 }, push_pull_alternative_output);
      m_compare_register_addr = &p_reg->capture_compare_register;
      break;
    case pwm::pins::pa7:
      configure_pin({ .port = 'A', .pin = 7 }, push_pull_alternative_output);
      m_compare_register_addr = &p_reg->capture_compare_register_2;
      break;
    case pwm::pins::pb0:
      configure_pin({ .port = 'B', .pin = 0 }, push_pull_alternative_output);
      m_compare_register_addr = &p_reg->capture_compare_register_3;
      break;
    case pwm::pins::pb1:
      configure_pin({ .port = 'B', .pin = 1 }, push_pull_alternative_output);
      m_compare_register_addr = &p_reg->capture_compare_register_4;
      break;
    case pwm::pins::pb6:
      configure_pin({ .port = 'B', .pin = 6 }, push_pull_alternative_output);
      m_compare_register_addr = &p_reg->capture_compare_register;
      break;
    case pwm::pins::pb7:
      configure_pin({ .port = 'B', .pin = 7 }, push_pull_alternative_output);
      m_compare_register_addr = &p_reg->capture_compare_register_2;
      break;
    case pwm::pins::pb8:
      configure_pin({ .port = 'B', .pin = 8 }, push_pull_alternative_output);
      m_compare_register_addr = &p_reg->capture_compare_register_3;
      break;
    case pwm::pins::pb9:
      configure_pin({ .port = 'B', .pin = 9 }, push_pull_alternative_output);
      m_compare_register_addr = &p_reg->capture_compare_register_4;
      break;
    default:
      safe_throw(hal::operation_not_supported(this));
  }
  setup(p_pin, m_peripheral_id);
}
void pwm::driver_frequency(hertz p_frequency)
{
  pwm_reg_t* reg = get_pwm_reg(m_peripheral_id);

  auto const current_clock = hal::stm32f1::frequency(m_peripheral_id);

  auto const previous_duty_cycle =
    get_duty_cycle(m_peripheral_id, m_compare_register_addr);

  if (p_frequency >= current_clock) {
    safe_throw(hal::operation_not_supported(this));
  }

  auto const possible_prescaler_value = static_cast<float>(
    current_clock / (p_frequency * std::numeric_limits<u16>::max()));

  std::uint16_t prescale = 0;
  std::uint16_t autoreload = 0xFFFF;

  if (possible_prescaler_value > 1) {
    // If the frequency is too low, the prescalar will be greater, which will
    // take more time to reach the ARR value
    prescale = std::floor(possible_prescaler_value);
  } else {
    // The frequency is too high, so the ARR value needs to be reduced.
    autoreload = static_cast<std::uint16_t>(current_clock / p_frequency);
  }
  reg->auto_reload_register = autoreload;
  reg->prescale_register = prescale;

  // Update duty cycle according to new frequency
  driver_duty_cycle(previous_duty_cycle);
}
void pwm::driver_duty_cycle(float p_duty_cycle)
{
  pwm_reg_t* reg = get_pwm_reg(m_peripheral_id);
  // the output changes from high to low when the counter > ccr, therefore, we
  // simply make the CCR equal to the required duty cycle fraction of the ARR
  // value.
  auto desired_ccr_value = static_cast<std::uint16_t>(
    static_cast<float>(reg->auto_reload_register) * p_duty_cycle);

  *m_compare_register_addr = desired_ccr_value;
}

pwm::~pwm() noexcept
{
  auto const pwm_pin_mask = bit_mask{ .position = (hal::u16)m_pin, .width = 1 };
  bit_modify(availability).clear(pwm_pin_mask);
  power_off(m_peripheral_id);
}
}  // namespace hal::stm32f1
