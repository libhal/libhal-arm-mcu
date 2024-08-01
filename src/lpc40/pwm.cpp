// Copyright 2024 Khalil Estell
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

#include <libhal-lpc40/pwm.hpp>

#include <libhal-lpc40/clock.hpp>
#include <libhal-lpc40/constants.hpp>
#include <libhal-lpc40/pin.hpp>
#include <libhal-lpc40/power.hpp>

#include "pwm_reg.hpp"

namespace hal::lpc40 {
namespace {
[[nodiscard]] pwm_reg_t* get_pwm_reg(peripheral p_id)
{
  if (p_id == peripheral::pwm0) {
    return pwm_reg0;
  }

  return pwm_reg1;
}

[[nodiscard]] volatile uint32_t& get_match_registers(pwm_reg_t* p_reg,
                                                     uint8_t p_match)
{
  switch (p_match) {
    case 1:
      return p_reg->match_register_1;
    case 2:
      return p_reg->match_register_2;
    case 3:
      return p_reg->match_register_3;
    case 4:
      return p_reg->match_register_4;
    case 5:
      return p_reg->match_register_5;
    case 6:
      return p_reg->match_register_6;
    case 0:
    default:
      return p_reg->match_register_0;
  }
}

[[nodiscard]] float get_duty_cycle(pwm::channel p_channel, pwm_reg_t* p_reg)
{
  auto period = static_cast<float>(get_match_registers(p_reg, p_channel.index));
  auto max_period = static_cast<float>(get_match_registers(p_reg, 0));
  return period / max_period;
}

[[nodiscard]] uint32_t calculate_duty_cycle(pwm_reg_t* p_reg, float p_percent)
{
  auto pwm_period = static_cast<float>(get_match_registers(p_reg, 0));
  return static_cast<uint32_t>(p_percent * pwm_period);
}

void enable(pwm_reg_t* p_reg, bool p_enable)
{
  // When set to a 1, enables the TC (total count) register and begins
  // counting.
  static constexpr auto counter_enable = bit_mask::from<0>();

  // When set to a 1, will reset the total count register.
  static constexpr auto counter_reset = bit_mask::from<1>();

  // Enables PWM mode. Without setting the match registers cannot operate
  // with the timer.
  static constexpr auto pwm_enable = bit_mask::from<3>();

  if (p_enable) {
    // Reset the Timer Counter
    bit_modify(p_reg->timer_control_register).set(counter_reset);
    // Clear reset and allow timer to count
    bit_modify(p_reg->timer_control_register).clear(counter_reset);
    // Enable PWM output
    bit_modify(p_reg->timer_control_register).set(pwm_enable);
    // Enable counting
    bit_modify(p_reg->timer_control_register).set(counter_enable);
  } else {
    // Disable PWM output
    bit_modify(p_reg->timer_control_register).clear(pwm_enable);
  }
}

void setup(pwm::channel& p_channel)
{
  /// Controls the counting mode of the PWM peripheral. When set to 0, counts
  /// using the internal prescale counter which is driven by the peripheral
  /// clock. Other modes involve use of an external clock source.
  static constexpr auto mode = bit_mask::from<0, 1>();

  /// If set to a 1, tells the PWM hardware to reset the PWM total count
  /// register to be reset to 0 when it is equal to the match register 0.
  static constexpr auto pwm0_reset = bit_mask::from<1>();

  power_on(p_channel.peripheral_id);

  pwm_reg_t* reg = get_pwm_reg(p_channel.peripheral_id);

  // Set pre-scalar to 1 so the input frequency to the PWM peripheral is
  // equal to the peripheral clock frequency.
  reg->prescale_register = 0;

  // Set to 0 to cause the timer counter to increment after each peripheral
  // clock tick.
  reg->prescale_counter_register = 0;

  bit_modify(reg->counter_control_register).insert<mode>(0x0U);
  bit_modify(reg->match_control_register).set<pwm0_reset>();

  // Enable this pwm channel
  bit_modify(reg->pwm_control_register)
    .set(bit_mask::from(8U + p_channel.index));

  p_channel.pwm_pin.function(p_channel.pin_function);

  // Set duty cycle to zero
  get_match_registers(reg, p_channel.index) = 0;

  // Enable the PWM output channel
  enable(reg, true);
}
}  // namespace

pwm::pwm(std::uint8_t p_peripheral,  // NOLINT
         std::uint8_t p_channel)
  : m_channel{}
{
  bool valid_driver = p_peripheral <= 1 && p_channel <= 6;
  if (not valid_driver) {
    // "LPC40 series microcontrollers only have PWM0 and PWM1."
    safe_throw(hal::operation_not_supported(this));
  }

  m_channel.index = p_channel;

  if (p_peripheral == 0) {
    m_channel.peripheral_id = peripheral::pwm0;
    m_channel.pwm_pin = pin(3, 16 + (p_channel - 1));
    m_channel.pin_function = 0b010;
  } else if (p_peripheral == 1) {
    m_channel.peripheral_id = peripheral::pwm1;
    m_channel.pwm_pin = pin(2, 0 + (p_channel - 1));
    m_channel.pin_function = 0b001;
  }

  setup(m_channel);
}

void pwm::driver_frequency(hertz p_frequency)
{
  pwm_reg_t* reg = get_pwm_reg(m_channel.peripheral_id);

  const auto input_clock = get_frequency(m_channel.peripheral_id);

  if (p_frequency >= input_clock) {
    safe_throw(hal::operation_not_supported(this));
  }

  // Get the current duty cycle so we can match it to the updated frequency.
  float previous_duty_cycle = get_duty_cycle(m_channel, reg);

  // In order to avoid PWM glitches, the PWM must be disabled while updating
  // the MR0 register. Doing this will reset all counters to 0 and allow us to
  // update MR0.
  enable(reg, false);

  // Set frequency by setting match register 0 (the reset register) to the
  // counts required to reach the desired frequency.
  const auto new_frequency = input_clock / p_frequency;

  get_match_registers(reg, 0) = static_cast<std::uint32_t>(new_frequency);

  // Re-enable PWM which will also reset all of the counters which allow
  // setting the duty cycle of other PWM channels.
  enable(reg, true);

  // Update the duty cycle based on the previous percentage
  duty_cycle(previous_duty_cycle);
}

void pwm::driver_duty_cycle(float p_duty_cycle)
{
  pwm_reg_t* reg = get_pwm_reg(m_channel.peripheral_id);

  // Set match register for this channel
  get_match_registers(reg, m_channel.index) =
    calculate_duty_cycle(reg, p_duty_cycle);

  // Setup pwm peripheral to update the duty cycle on the next reset cycle after
  // a match_register_0 match occurs. This way the PWM duty cycle does ont
  // change instantaneously.
  bit_modify(reg->load_enable_register).set(bit_mask::from(m_channel.index));
}
}  // namespace hal::lpc40
