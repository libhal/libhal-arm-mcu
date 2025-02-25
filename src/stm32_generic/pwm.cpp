#include <cmath>

#include <limits>
#include <utility>

#include <libhal-arm-mcu/stm32_generic/pwm.hpp>
#include <libhal-arm-mcu/stm32f1/clock.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32f1/pin.hpp>
#include <libhal-arm-mcu/stm32f1/timer.hpp>
#include <libhal-util/bit.hpp>
#include <libhal/error.hpp>

namespace hal::stm32_generic {
struct timer_reg_t
{
  /// Offset: 0x00 Control Register (R/W)
  u32 volatile control_register;  // sets up timers
  /// Offset: 0x04 Control Register 2 (R/W)
  u32 volatile control_register_2;
  /// Offset: 0x08 Peripheral Mode Control Register (R/W)
  u32 volatile peripheral_control_register;
  /// Offset: 0x0C DMA/Interrupt enable register (R/W)
  u32 volatile interuupt_enable_register;
  /// Offset: 0x10 Status Register register (R/W)
  u32 volatile status_register;
  /// Offset: 0x14 Event Generator Register register (R/W)
  u32 volatile event_generator_register;
  /// Offset: 0x18 Capture/Compare mode register (R/W)
  u32 volatile capture_compare_mode_register;  // set up modes for
                                               // channel
  /// Offset: 0x1C Capture/Compare mode register (R/W)
  u32 volatile capture_compare_mode_register_2;
  /// Offset: 0x20 Capture/Compare Enable register (R/W)
  u32 volatile cc_enable_register;
  /// Offset: 0x24 Counter (R/W)
  u32 volatile counter_register;
  /// Offset: 0x28 Prescalar (R/W)
  u32 volatile prescale_register;
  /// Offset: 0x2C Auto Reload Register (R/W)
  u32 volatile auto_reload_register;  // affects frequency
  /// Offset: 0x30 Repetition Counter Register (R/W)
  u32 volatile repetition_counter_register;
  /// Offset: 0x34 Capture Compare Register (R/W)
  u32 volatile capture_compare_register;  // affects duty cycles
  /// Offset: 0x38 Capture Compare Register (R/W)
  u32 volatile capture_compare_register_2;
  // Offset: 0x3C Capture Compare Register (R/W)
  u32 volatile capture_compare_register_3;
  // Offset: 0x40 Capture Compare Register (R/W)
  u32 volatile capture_compare_register_4;
  /// Offset: 0x44 Break and dead-time register
  u32 volatile break_and_deadtime_register;
  /// Offset: 0x48 DMA control register
  u32 volatile dma_control_register;
  /// Offset: 0x4C DMA address for full transfer
  u32 volatile dma_address_register;
};
namespace {
[[nodiscard]] timer_reg_t* get_timer_reg(void* p_reg)
{
  return reinterpret_cast<timer_reg_t*>(p_reg);
}

void setup_channel(timer_reg_t* p_reg, u8 p_channel, bool p_is_advanced)
{
  static constexpr auto main_output_enable = bit_mask::from<15>();
  static constexpr auto ossr = bit_mask::from<11>();

  u8 const start_pos = (p_channel - 1) * 4;
  auto const cc_enable = bit_mask::from(start_pos);
  auto const cc_polarity = bit_mask::from(start_pos + 1);

  bit_modify(p_reg->cc_enable_register).set(cc_enable);
  bit_modify(p_reg->cc_enable_register).clear(cc_polarity);

  if (p_is_advanced) {
    bit_modify(p_reg->break_and_deadtime_register)
      .clear(ossr)
      .set(main_output_enable);  // complementary channel stuff
  }
}

u32 volatile* setup(timer_reg_t* p_reg, int p_channel, bool p_is_advanced)
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
  constexpr auto pwm_mode_1 = 0b110U;
  constexpr auto set_output = 0b00U;

  bit_modify(p_reg->control_register)
    .insert<clock_division>(0b00U)
    .insert<edge_aligned_mode>(0b00U)
    .clear(direction);
  u32 volatile* compare_register = nullptr;
  switch (p_channel) {
    case 1:
      // Preload enable must be done for corresponding OCxPE
      bit_modify(p_reg->capture_compare_mode_register)
        .insert<output_compare_odd>(pwm_mode_1)
        .insert<channel_output_select_odd>(set_output)
        .set(odd_channel_preload_enable);
      compare_register = &p_reg->capture_compare_register;
      break;

    case 2:
      bit_modify(p_reg->capture_compare_mode_register)
        .insert<output_compare_even>(pwm_mode_1)
        .insert<channel_output_select_even>(set_output)
        .set(even_channel_preload_enable);
      compare_register = &p_reg->capture_compare_register_2;
      break;

    case 3:
      bit_modify(p_reg->capture_compare_mode_register_2)
        .insert<output_compare_odd>(pwm_mode_1)
        .insert<channel_output_select_odd>(set_output)
        .set(odd_channel_preload_enable);
      compare_register = &p_reg->capture_compare_register_3;
      break;

    case 4:
      bit_modify(p_reg->capture_compare_mode_register_2)
        .insert<output_compare_even>(pwm_mode_1)
        .insert<channel_output_select_even>(set_output)
        .set(even_channel_preload_enable);
      compare_register = &p_reg->capture_compare_register_4;
      break;
    default:
      std::unreachable();
  }

  setup_channel(p_reg, p_channel, p_is_advanced);
  bit_modify(p_reg->event_generator_register).set(ug_bit);
  bit_modify(p_reg->control_register).set(counter_enable);
  bit_modify(p_reg->control_register).set(auto_reload_preload_enable);

  // If the desired frequency is really low, and 16 bits are not enough
  // to represent that, the prescalar value can be increased. Example:
  // if prescalar = 2, 2 clock ticks would occur before incrementing
  // the counter by 1.
  p_reg->prescale_register = 0x0U;

  // The counter increments every clock cycle, and compares its value
  // to the CCR to check whether the output should be high or low
  p_reg->counter_register = 0x0U;

  // The ARR register is the top, once the counter reaches the ARR,
  // it starts counting again. ARR can be increased on decreased
  // depending on frequency.
  p_reg->auto_reload_register = 0xFFFF;

  return compare_register;
}

u32 volatile* common_setup(pwm_channel_info p_settings, timer_reg_t* p_reg)
{
  if (p_settings.channel <= 4) {
    u32 volatile* p_compare_register_addr =
      setup(p_reg, p_settings.channel, p_settings.is_advanced);
    return p_compare_register_addr;
  } else {
    hal::safe_throw(hal::operation_not_supported(nullptr));
  }
}
}  // namespace

pwm::pwm(unsafe, void* p_reg, pwm_channel_info p_settings)
  : m_reg(p_reg)
{
  timer_reg_t* reg = get_timer_reg(m_reg);
  m_compare_register_addr = common_setup(p_settings, reg);
}

void pwm::initialize(unsafe, void* p_reg, pwm_channel_info p_settings)
{
  m_reg = p_reg;
  timer_reg_t* reg = get_timer_reg(m_reg);
  m_compare_register_addr = common_setup(p_settings, reg);
}

u32 pwm::frequency(u32 p_input_clock_frequency)
{
  timer_reg_t* reg = get_timer_reg(m_reg);

  auto const prescaled_clock = p_input_clock_frequency / reg->prescale_register;
  auto const final_frequency = prescaled_clock / reg->auto_reload_register;

  return final_frequency;
}

void pwm::duty_cycle(u16 p_duty_cycle)
{
  // the output changes from high to low when the counter > ccr, therefore, we
  // simply make the CCR equal to the required duty cycle fraction of the ARR
  // value.
  timer_reg_t* reg = get_timer_reg(m_reg);

  auto const reload_value = static_cast<u16>(reg->auto_reload_register);
  auto const upscaled_value = reload_value * p_duty_cycle;
  auto const normalized_ccr_value =
    upscaled_value / std::numeric_limits<u16>::max();

  *m_compare_register_addr = normalized_ccr_value;
}

pwm_group_frequency::pwm_group_frequency(hal::unsafe, void* p_reg)
  : m_reg(p_reg)
{
}

void pwm_group_frequency::set_group_frequency(pwm_timer_frequency p_frequency)
{
  timer_reg_t* reg = get_timer_reg(m_reg);

  // Calculate new frequency
  auto const [frequency, clock_frequency] = p_frequency;
  if (frequency >= clock_frequency) {
    safe_throw(hal::operation_not_supported(this));
  }

  auto const possible_prescaler_value =
    (clock_frequency / (frequency * std::numeric_limits<u16>::max()));

  u16 prescale = 0;
  u16 auto_reload = 0xFFFF;

  if (possible_prescaler_value > 0) {
    // If the frequency is low enough, the prescale can be increased, which
    // will take more time to reach the ARR value
    prescale = possible_prescaler_value;
  } else {
    // The frequency is too high, so the ARR value needs to be reduced.
    auto_reload = static_cast<u16>(clock_frequency / frequency);
  }

  // ===========================================================================
  // Updating all PWM duty cycles
  // ===========================================================================

  // NOTE: The capture_compare_register only contain 16-bits of information so
  // we can legally cast them to u16 and lose no information.
  auto const capture1 = static_cast<u16>(reg->capture_compare_register);
  auto const capture2 = static_cast<u16>(reg->capture_compare_register_2);
  auto const capture3 = static_cast<u16>(reg->capture_compare_register_3);
  auto const capture4 = static_cast<u16>(reg->capture_compare_register_4);
  auto const previous_auto_reload = reg->auto_reload_register;

  // Duty_new = (Duty_prev / AutoReload_prev) * AutoReload_new;
  //
  // Can also be written as:
  //
  // Duty_new = (Duty_prev * AutoReload_new) / AutoReload_prev;
  //
  // We choose the 2nd option because we can perform the operations with
  // integers without loss of precision.

  auto const new_capture1_upscaled = static_cast<u32>(capture1 * auto_reload);
  auto const new_capture2_upscaled = static_cast<u32>(capture2 * auto_reload);
  auto const new_capture3_upscaled = static_cast<u32>(capture3 * auto_reload);
  auto const new_capture4_upscaled = static_cast<u32>(capture4 * auto_reload);

  auto const new_capture1 = (new_capture1_upscaled / previous_auto_reload);
  auto const new_capture2 = (new_capture2_upscaled / previous_auto_reload);
  auto const new_capture3 = (new_capture3_upscaled / previous_auto_reload);
  auto const new_capture4 = (new_capture4_upscaled / previous_auto_reload);

  reg->capture_compare_register = new_capture1;
  reg->capture_compare_register_2 = new_capture2;
  reg->capture_compare_register_3 = new_capture3;
  reg->capture_compare_register_4 = new_capture4;

  // Set the prescale & auto reload register
  reg->prescale_register = prescale;
  reg->auto_reload_register = auto_reload;
}
}  // namespace hal::stm32_generic
