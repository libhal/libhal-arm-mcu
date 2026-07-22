// Copyright 2024 - 2025 Khalil Estell and the libhal contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

module;

#include <algorithm>
#include <limits>
#include <utility>

export module hal.arm_mcu.stm32_generic:pwm;

import hal;
import hal.util;

import :registers;

namespace hal::stm32_generic {

/**
 * @brief Each Timer Peripheral has 2-4 channels/is advanced info
 *
 */
export struct pwm_channel_info
{
  /// Each Timer Peripheral has 2-4 channels, and this number allows the
  /// driver to use the correct output pin
  u8 channel;
  /// Advanced timers work slightly different than general purpose ones.
  bool is_advanced;
};

/// The target and input clock frequency for a pwm timer group
export struct pwm_timer_frequency
{
  /// The target frequency all PWM channels within a timer will be set to
  u32 pwm_frequency;
  /// The timer's input clock frequency
  u32 timer_clock_frequency;
};

void setup_channel(timer_reg* p_reg, pwm_channel_info p_settings)
{
  constexpr auto main_output_enable = bit_mask::from<15>();
  constexpr auto ossr = bit_mask::from<11>();

  u8 const start_pos = (p_settings.channel - 1) * 4;
  auto const cc_enable = bit_mask::from(start_pos);
  auto const cc_polarity = bit_mask::from(start_pos + 1);

  bit_modify(p_reg->cc_enable_register).set(cc_enable);
  bit_modify(p_reg->cc_enable_register).clear(cc_polarity);

  if (p_settings.is_advanced) {
    bit_modify(p_reg->break_and_deadtime_register)
      .clear(ossr)
      .set(main_output_enable);  // complementary channel stuff
  }
}

u32 volatile* setup_timer_channel(timer_reg* p_reg, pwm_channel_info p_settings)
{
  constexpr auto clock_division = bit_mask::from<8, 9>();
  constexpr auto edge_aligned_mode = bit_mask::from<5, 6>();
  constexpr auto direction = bit_mask::from<4>();

  constexpr auto output_compare_odd = bit_mask::from<4, 6>();
  constexpr auto output_compare_even = bit_mask::from<12, 14>();
  constexpr auto channel_output_select_odd = bit_mask::from<0, 1>();
  constexpr auto channel_output_select_even = bit_mask::from<8, 9>();

  constexpr auto counter_enable = bit_mask::from<0>();
  constexpr auto auto_reload_preload_enable = bit_mask::from<7>();
  constexpr auto odd_channel_preload_enable = bit_mask::from<3>();
  constexpr auto even_channel_preload_enable = bit_mask::from<11>();
  constexpr auto ug_bit = bit_mask::from<0>();

  // The PWM_MODE 1 makes it such that output will be high when Counter < CCR
  constexpr auto pwm_mode_1 = 0b110U;
  constexpr auto set_output = 0b00U;

  bit_modify(p_reg->control_register)
    .insert<clock_division>(0b00U)
    .insert<edge_aligned_mode>(0b00U)
    .clear(direction);
  u32 volatile* compare_register = nullptr;
  switch (p_settings.channel) {
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

  setup_channel(p_reg, p_settings);
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

u32 volatile* common_setup(pwm_channel_info p_settings, timer_reg* p_reg)
{
  if (p_settings.channel > 4) {
    throw hal::operation_not_supported(nullptr);
  }
  return setup_timer_channel(p_reg, p_settings);
}

/**
 * @brief This class should not be constructed directly.
 *
 * The user should instantiate a General Purpose or Advanced timer class
 * first, and then acquire a pwm pin through that class.
 */
export class pwm final
{
public:
  /**
   * @brief Construct generic stm32 pwm driver
   *
   * Care should be taken in constructing this outside of a platform specific
   * timer class. If both a platform specific timer class and this object
   * exist, care must be taken to ensure that the drivers do not conflict
   * with each others.
   *
   * @param p_reg is a void pointer that points to the beginning of a timer
   * peripheral
   * @param p_settings consist of channel number, frequency of the timer,
   * and a boolean to indicate whether the timer is advanced or not.
   */
  pwm(void* p_reg, pwm_channel_info p_settings)
    : m_reg(p_reg)
  {
    m_compare_register_addr = common_setup(p_settings, timer_reg::from(m_reg));
  }

  /**
   * @brief Construct pwm uninitialized
   *
   * The purpose of this is to send the settings through the initialize
   * function instead, because the channel calculation and pin configuration
   * is done in the constructor and the regular constructor cannot be
   * initialized through the initializer list.
   *
   * It is unsafe to call any API of this class before calling the
   * `initialize()` API with the correct inputs. Once that API has been
   * called without failure, then the other APIs will become available.
   */
  pwm() = default;

  pwm(pwm const& p_other) = delete;
  pwm& operator=(pwm const& p_other) = delete;
  pwm(pwm&& p_other) = default;
  pwm& operator=(pwm&& p_other) = default;
  ~pwm() = default;

  /**
   * @brief Initialize the driver if the default `pwm()` was used
   *
   * This API must be called if the default `pwm()` was used in order to
   * initialize this class.
   *
   * @param p_reg - address of the timer's peripheral
   * @param p_settings - pwm settings for the timer peripheral
   */
  void initialize(void* p_reg, pwm_channel_info p_settings)
  {
    m_reg = p_reg;
    m_compare_register_addr = common_setup(p_settings, timer_reg::from(m_reg));
  }

  /**
   * @brief Acquire the operating frequency of the PWM channel
   *
   * @param p_timer_clock_frequency - the clock frequency driving the timer
   * peripheral.
   * @return u32 - the frequency of the PWM signal
   */
  u32 frequency(u32 p_timer_clock_frequency)
  {
    auto* reg = timer_reg::from(m_reg);

    // See page 419 in RM0008.pdf to find this equation:
    //
    //   Bits 15:0 PSC[15:0]: Prescaler value
    //   The counter clock frequency CK_CNT is equal to fCK_PSC / (PSC[15:0]
    //   + 1)
    //
    auto const prescale_value = reg->prescale_register + 1;
    auto const prescaled_clock = p_timer_clock_frequency / prescale_value;
    auto const final_frequency = prescaled_clock / reg->auto_reload_register;

    return final_frequency;
  }

  /**
   * @brief Set pwm channel duty cycle using u16 value from 0x0000 to 0xFFFF
   *
   * @param p_duty_cycle - pwm duty cycle proportional value from 0x0000 to
   * 0xFFFF.
   */
  void duty_cycle(u16 p_duty_cycle)
  {
    // the output changes from high to low when the counter > ccr, therefore,
    // we simply make the CCR equal to the required duty cycle fraction of
    // the ARR value.
    auto* reg = timer_reg::from(m_reg);

    auto const reload_value = static_cast<u16>(reg->auto_reload_register);
    auto const upscaled_value = reload_value * p_duty_cycle;
    auto const normalized_ccr_value =
      upscaled_value / std::numeric_limits<u16>::max();

    *m_compare_register_addr = normalized_ccr_value;
  }

  /**
   * @brief Set the duty cycle using a float
   *
   * @param p_duty_cycle - value from
   */
  void duty_cycle(float p_duty_cycle)
  {
    constexpr auto u16_max = std::numeric_limits<u16>::max();
    auto const clamped_duty_cycle = std::clamp(p_duty_cycle, 0.0f, 1.0f);
    auto const u16_value = static_cast<u16>(clamped_duty_cycle * u16_max);
    duty_cycle(u16_value);
  }

private:
  void* m_reg = nullptr;
  u32 volatile* m_compare_register_addr = nullptr;
};

/**
 * @brief This class should not be constructed directly.
 *
 * The user should instantiate a General Purpose or Advanced timer class
 * first, and then acquire a pwm pin through that class.
 */
export class pwm_group_frequency final
{
public:
  /**
   * @brief Construct generic stm32 pwm driver
   *
   * Care should be taken in constructing this outside of a platform specific
   * timer class. If both a platform specific timer class and this object
   * exist, care must be taken to ensure that the drivers do not conflict
   * with each others.
   *
   * @param p_reg is a void pointer that points to the beginning of a timer
   * peripheral
   */
  pwm_group_frequency(void* p_reg)
    : m_reg(p_reg)
  {
  }

  pwm_group_frequency(pwm_group_frequency const& p_other) = delete;
  pwm_group_frequency& operator=(pwm_group_frequency const& p_other) = delete;
  pwm_group_frequency(pwm_group_frequency&& p_other) = default;
  pwm_group_frequency& operator=(pwm_group_frequency&& p_other) = default;
  ~pwm_group_frequency() = default;

  /**
   * @brief Set the frequency for all PWM channels controlled by the timer
   * specified by the `p_reg` parameter passed at construction.
   *
   * @param p_timer_frequency - desired pwm frequency and input clock
   * frequency
   */
  void set_group_frequency(pwm_timer_frequency p_timer_frequency)
  {
    auto* reg = timer_reg::from(m_reg);

    // Calculate new frequency
    auto const [frequency, clock_frequency] = p_timer_frequency;
    if (frequency >= clock_frequency) {
      throw hal::operation_not_supported(this);
    }

    auto const possible_prescaler_value =
      (clock_frequency / (frequency * std::numeric_limits<u16>::max()));

    u16 prescale = 0;
    u16 auto_reload = 0xFFFF;

    if (possible_prescaler_value > 0) {
      // If the frequency is low enough, the prescale can be increased,
      // which will take more time to reach the ARR value
      prescale = static_cast<u16>(possible_prescaler_value);
    } else {
      // The frequency is too high, so the ARR value needs to be reduced.
      auto_reload = static_cast<u16>(clock_frequency / frequency);
    }

    // =========================================================================
    // Updating all PWM duty cycles
    // =========================================================================

    // NOTE: The capture_compare_register only contain 16-bits of
    // information so we can legally cast them to u16 and lose no
    // information.
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

private:
  void* m_reg = nullptr;
};
}  // namespace hal::stm32_generic
