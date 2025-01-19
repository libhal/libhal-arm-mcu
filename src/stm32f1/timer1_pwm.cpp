#include <cmath>
#include <libhal-arm-mcu/stm32f1/clock.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32f1/pin.hpp>
#include <libhal-arm-mcu/stm32f1/timer1_pwm.hpp>
#include <libhal-util/bit.hpp>
#include <libhal/error.hpp>

#include "pin.hpp"
#include "power.hpp"

namespace hal::stm32f1 {
namespace {
[[nodiscard]] pwm_reg_t* get_pwm_reg(peripheral p_id)
{
  if (p_id == peripheral::timer1) {
    return pwm_timer1;
  }
  hal::safe_throw(hal::operation_not_supported(nullptr));
}
[[nodiscard]] peripheral get_peripheral_id(timer1_pwm::pwm_pins p_pin)
{
  if (p_pin == timer1_pwm::pwm_pins::pa8 ||
      p_pin == timer1_pwm::pwm_pins::pa9 ||
      p_pin == timer1_pwm::pwm_pins::pa10 ||
      p_pin == timer1_pwm::pwm_pins::pa11) {

    return peripheral::timer1;
  } else {
    hal::safe_throw(hal::operation_not_supported(nullptr));
  }
}

[[nodiscard]] float get_duty_cycle(timer1_pwm::pwm_pins p_pin,
                                   uint32_t volatile* compare_register)
{
  // ccr / arr
  auto const peripheral_id = get_peripheral_id(p_pin);
  pwm_reg_t* const p_reg = get_pwm_reg(peripheral_id);

  static constexpr auto first_nonreserved_half = bit_mask::from<0, 15>();

  std::uint16_t compare_value =
    hal::bit_extract<first_nonreserved_half>(*compare_register);

  std::uint16_t arr_value =
    hal::bit_extract<first_nonreserved_half>(p_reg->auto_reload_register);

  return (static_cast<float>(compare_value) / static_cast<float>(arr_value));
}
void setup_channel(pwm_reg_t* p_reg, uint8_t p_channel)
{

  // static constexpr auto main_output_enable = bit_mask::from<15>();
  // static constexpr auto ossr = bit_mask::from<11>();
  // OCx polarity is software programmable using the CCxP bit in the TIMx_CCER
  // register. It
  // can be programmed as active high or active low. OCx output is enabled by
  // the CCxE bit in the TIMx_CCER register. Refer to the TIMx_CCERx register
  // description for more details.
  uint8_t start_pos = (p_channel - 1) * 4;
  auto const cc_enable_value = 0b10U;
  static auto ccer_polarity_enable_mask =
    bit_mask::from(start_pos, start_pos + 1);

  bit_modify(p_reg->cc_enable_register)
    .insert(ccer_polarity_enable_mask, cc_enable_value);
  // bit_modify(p_reg->cc_enable_register).set(ccer_polarity_enable_mask);

  // bit_modify(p_reg->break_and_deadtime_register)
  //   .clear(ossr)
  //   .set(main_output_enable);
}
void setup(timer1_pwm::pwm_pins p_pin)
{

  static constexpr auto clock_division = bit_mask::from<8, 9>();
  static constexpr auto edge_aligned_mode = bit_mask::from<5, 6>();
  static constexpr auto direction = bit_mask::from<4>();

  static constexpr auto output_compare_odd = bit_mask::from<4, 6>();
  static constexpr auto output_compare_even = bit_mask::from<12, 14>();
  static constexpr auto counter_enable = bit_mask::from<0>();
  static constexpr auto auto_reload_preload_enable = bit_mask::from<7>();
  static constexpr auto odd_channel_preload_enable = bit_mask::from<3>();
  static constexpr auto even_channel_preload_enable = bit_mask::from<11>();
  auto const pwm_mode_1 = 0b110U;
  auto const peripheral_id = get_peripheral_id(p_pin);
  pwm_reg_t* const reg = get_pwm_reg(peripheral_id);

  bit_modify(reg->control_register)
    .insert<clock_division>(0b00U)
    .insert<edge_aligned_mode>(0b00U)
    .clear(direction);

  if (p_pin == timer1_pwm::pwm_pins::pa8) {  // channel 1
    // Preload enable must be done for corresponding OCxPE
    bit_modify(reg->capture_compare_mode_register)
      .insert<output_compare_odd>(pwm_mode_1)
      .set(odd_channel_preload_enable);
    setup_channel(reg, 1);
  } else if (p_pin == timer1_pwm::pwm_pins::pa9) {  // channel 2
    bit_modify(reg->capture_compare_mode_register)
      .insert<output_compare_even>(pwm_mode_1)
      .set(even_channel_preload_enable);
    setup_channel(reg, 2);

  } else if (p_pin == timer1_pwm::pwm_pins::pa10) {  // channel 3
    bit_modify(reg->capture_compare_mode_register_2)
      .insert<output_compare_odd>(pwm_mode_1)
      .set(odd_channel_preload_enable);
    setup_channel(reg, 3);

  } else if (p_pin == timer1_pwm::pwm_pins::pa11) {  // channel 4
    bit_modify(reg->capture_compare_mode_register_2)
      .insert<output_compare_even>(pwm_mode_1)
      .set(even_channel_preload_enable);
    setup_channel(reg, 4);
  }

  bit_modify(reg->control_register).set(counter_enable);
  bit_modify(reg->control_register).set(auto_reload_preload_enable);
  reg->prescale_register = 0x0U;

  // Set counter value to 0 because it upcounts.
  reg->counter_register = 0x0U;

  reg->auto_reload_register = 0xFFFF;
}

}  // namespace

timer1_pwm::timer1_pwm(timer1_pwm::pwm_pins p_pin)
  : m_pin{ p_pin }
{
  // config pin to alternate function push - pull
  auto const p_id = get_peripheral_id(p_pin);
  auto const p_reg = get_pwm_reg(p_id);
  power_on(p_id);
  // power_on(peripheral::afio);
  switch (p_pin) {
    case timer1_pwm::pwm_pins::pa10:
      configure_pin({ .port = 'A', .pin = 10 }, push_pull_alternative_output);
      m_compare_register_addr = &p_reg->capture_compare_register_3;
      m_channel = 3;
      break;
    case timer1_pwm::pwm_pins::pa11:
      configure_pin({ .port = 'A', .pin = 11 }, push_pull_alternative_output);
      m_compare_register_addr = &p_reg->capture_compare_register_4;
      m_channel = 4;
      break;
    case timer1_pwm::pwm_pins::pa8:
      configure_pin({ .port = 'A', .pin = 8 }, push_pull_alternative_output);
      m_compare_register_addr = &p_reg->capture_compare_register;
      m_channel = 1;
      break;
    case timer1_pwm::pwm_pins::pa9:
      configure_pin({ .port = 'A', .pin = 9 }, push_pull_alternative_output);
      m_compare_register_addr = &p_reg->capture_compare_register_2;
      m_channel = 2;
      break;
    default:
      safe_throw(hal::operation_not_supported(this));
  }
  setup(p_pin);
}
void timer1_pwm::driver_frequency(hertz p_frequency)
{
  auto const peripheral_id = get_peripheral_id(m_pin);
  pwm_reg_t* reg = get_pwm_reg(peripheral_id);

  auto const current_clock = hal::stm32f1::frequency(peripheral_id);

  auto const previous_duty_cycle =
    get_duty_cycle(m_pin, m_compare_register_addr);

  if (p_frequency >= current_clock) {
    safe_throw(hal::operation_not_supported(this));
  }

  auto const possible_prescaler_value =
    static_cast<float>(current_clock / (p_frequency * std::pow(2, 16)));

  std::uint16_t prescale = 0;
  std::uint16_t autoreload = 0xFFFF;

  if (possible_prescaler_value > 1) {
    prescale = std::floor(possible_prescaler_value);
  } else {
    // we have to reduce the ARR value.
    autoreload = static_cast<std::uint16_t>(current_clock / p_frequency);
  }
  reg->auto_reload_register = autoreload;
  reg->prescale_register = prescale;

  // Update duty cycle according to new frequency
  driver_duty_cycle(previous_duty_cycle);
}
void timer1_pwm::driver_duty_cycle(float p_duty_cycle)
{
  auto const peripheral_id = get_peripheral_id(m_pin);
  pwm_reg_t* reg = get_pwm_reg(peripheral_id);

  // std::uint16_t desired_ccr_value =
  //   (static_cast<std::uint16_t>(reg->auto_reload_register) *
  //    static_cast<std::uint16_t>(p_duty_cycle));
  std::uint16_t desired_ccr_value =
    static_cast<std::uint16_t>(reg->auto_reload_register * p_duty_cycle);

  *m_compare_register_addr = desired_ccr_value;
}
}  // namespace hal::stm32f1
