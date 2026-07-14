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

#include <utility>

export module hal.arm_mcu.stm32_generic:quadrature_encoder;

import hal;
import hal.util;

namespace hal::stm32_generic {
/// stm32 general purpose/advanced timer register map, shared by the
/// quadrature encoder helper driver.
struct timer_reg_t
{
  /// Offset: 0x00 Control Register (R/W)
  hal::u32 volatile control_register;
  /// Offset: 0x04 Control Register 2 (R/W)
  hal::u32 volatile control_register_2;
  /// Offset: 0x08 Peripheral Mode Control Register (R/W)
  hal::u32 volatile peripheral_control_register;
  /// Offset: 0x0C DMA/Interrupt enable register (R/W)
  hal::u32 volatile interrupt_enable_register;
  /// Offset: 0x10 Status Register register (R/W)
  hal::u32 volatile status_register;
  /// Offset: 0x14 Event Generator Register register (R/W)
  hal::u32 volatile event_generator_register;
  /// Offset: 0x18 Capture/Compare mode register (R/W)
  hal::u32 volatile capture_compare_mode_register;
  /// Offset: 0x1C Capture/Compare mode register (R/W)
  hal::u32 volatile capture_compare_mode_register_2;
  /// Offset: 0x20 Capture/Compare Enable register (R/W)
  hal::u32 volatile cc_enable_register;
  /// Offset: 0x24 Counter (R/W)
  hal::u32 volatile counter_register;
  /// Offset: 0x28 Prescalar (R/W)
  hal::u32 volatile prescale_register;
  /// Offset: 0x2C Auto Reload Register (R/W)
  hal::u32 volatile auto_reload_register;
  /// Offset: 0x30 Repetition Counter Register (R/W)
  hal::u32 volatile repetition_counter_register;
  /// Offset: 0x34 Capture Compare Register (R/W)
  hal::u32 volatile capture_compare_register;
  /// Offset: 0x38 Capture Compare Register (R/W)
  hal::u32 volatile capture_compare_register_2;
  // Offset: 0x3C Capture Compare Register (R/W)
  hal::u32 volatile capture_compare_register_3;
  // Offset: 0x40 Capture Compare Register (R/W)
  hal::u32 volatile capture_compare_register_4;
  /// Offset: 0x44 Break and dead-time register
  hal::u32 volatile break_and_deadtime_register;
  /// Offset: 0x48 DMA control register
  hal::u32 volatile dma_control_register;
  /// Offset: 0x4C DMA address for full transfer
  hal::u32 volatile dma_address_register;
};

[[nodiscard]] timer_reg_t* get_timer_reg(void* p_reg)
{
  return reinterpret_cast<timer_reg_t*>(p_reg);
}

void setup_channel(int p_channel, timer_reg_t* p_reg)
{
  constexpr auto odd_channel_input_mode = bit_mask::from<0, 1>();
  constexpr auto odd_channel_filter_select = bit_mask::from<4, 7>();

  constexpr auto even_channel_input_mode = bit_mask::from<8, 9>();
  constexpr auto even_channel_filter_select = bit_mask::from<12, 15>();

  constexpr auto input_select = 0b01U;
  constexpr auto input_capture_filter = 0b0000U;

  // Select the TI1 and TI2 polarity by programming the CC1P and CC2P bits in
  // the TIMx_CCER register. When needed, the user can program the input
  // filter as well. find out which pin is what channel
  switch (p_channel) {
    case 1:
      bit_modify(p_reg->capture_compare_mode_register)
        .insert<odd_channel_input_mode>(input_select)
        .insert<odd_channel_filter_select>(input_capture_filter);
      break;
    case 2:
      bit_modify(p_reg->capture_compare_mode_register)
        .insert<even_channel_input_mode>(input_select)
        .insert<even_channel_filter_select>(input_capture_filter);
      break;
    default:
      std::unreachable();
  }
}

void setup_enable_register(int p_channel, timer_reg_t* p_reg)
{
  // The polarity bit is the second bit in a set of 4 bits per channel.
  // Therefore if it is channel 1-> bit 1, channel 2-> bit->5. Application
  // Note Pg: 353
  auto const polarity_start_pos = ((p_channel - 1) * 4) + 1;

  // Similar to polarity bits, capture_enable bits are the first bit in a
  // set of 4 bits per channel. Application Note Pg: 353.
  auto const input_capture_enable = ((p_channel - 1) * 4);

  auto const input_start_pos = bit_mask::from(input_capture_enable);
  auto const polarity_inverted = bit_mask::from(polarity_start_pos);

  bit_modify(p_reg->cc_enable_register).clear(polarity_inverted);
  bit_modify(p_reg->cc_enable_register).set(input_start_pos);
}

/**
 * @brief These are the 2 channels from the same timer that a user must use.
 *
 * They must be channel 1 and channel 2 of any timer. If channel 1 and
 * channel 2 are flipped, the order of these parameters are important
 * otherwise the counter will work in the opposite direction.
 */
export struct encoder_channels
{
  u8 channel_a;
  u8 channel_b;
};

/**
 * @brief This class should not be constructed directly.
 *
 * The user should instantiate a platform specific timer class and acquire a
 * quadrature encoder through that class.
 */
export class quadrature_encoder final : public hal::rotation_sensor
{
public:
  /**
   * @brief Construct generic stm32 quadrature encoder driver
   *
   * @param p_channels - the two timer channels used for quadrature decoding
   * @param p_reg - address of the timer peripheral
   * @param p_pulses_per_rotation - number of encoder pulses per full
   * rotation
   */
  quadrature_encoder(encoder_channels p_channels,
                     void* p_reg,
                     u32 p_pulses_per_rotation)
  {
    initialize(p_channels, p_reg, p_pulses_per_rotation);
  }

  /**
   * @brief Construct quadrature encoder uninitialized
   *
   * It is unsafe to call any API of this class before calling the
   * `initialize()` API with the correct inputs. Once that API has been
   * called without failure, then the other APIs will become available.
   */
  quadrature_encoder() = default;

  /**
   * @brief Initialize the driver if the default `quadrature_encoder()` was
   * used
   *
   * @param p_channels - the two timer channels used for quadrature decoding
   * @param p_reg - address of the timer peripheral
   * @param p_pulses_per_rotation - number of encoder pulses per full
   * rotation
   */
  void initialize(encoder_channels p_channels,
                  void* p_reg,
                  u32 p_pulses_per_rotation)
  {
    m_reg = p_reg;
    m_pulses_per_rotation = static_cast<float>(p_pulses_per_rotation);

    auto* timer_register = get_timer_reg(m_reg);
    constexpr auto set_encoder_mode = bit_mask::from<0, 2>();
    // encoder counts up/down on only TI2FP1 level
    constexpr auto encoder_mode_3 = 0b010U;

    setup_channel(p_channels.channel_a, timer_register);
    setup_channel(p_channels.channel_b, timer_register);
    setup_enable_register(p_channels.channel_a, timer_register);
    setup_enable_register(p_channels.channel_b, timer_register);
    timer_register->auto_reload_register = 0xFFFF;  // Set max counter value
    timer_register->counter_register = 0x8000;       // Start at middle value
    bit_modify(timer_register->peripheral_control_register)
      .insert<set_encoder_mode>(encoder_mode_3);
    constexpr auto counter_enable = bit_mask::from<0>();
    bit_modify(timer_register->control_register).set(counter_enable);
  }

private:
  async::future<revolutions> driver_read(async::context&) override
  {
    auto* timer_register = get_timer_reg(m_reg);
    // difference from start pos
    i32 const diff_pulses =
      static_cast<u16>(timer_register->counter_register) - 0x8000;

    // pulses / pulses_per_rotation = revolutions.
    auto const revolution_count =
      static_cast<float>(diff_pulses) / m_pulses_per_rotation;

    return revolution_count * mp_units::angular::revolution;
  }

  void* m_reg = nullptr;
  float m_pulses_per_rotation = 1.0f;
};
}  // namespace hal::stm32_generic
