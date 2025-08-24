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

#include <libhal/initializers.hpp>
#include <utility>

#include <libhal-arm-mcu/stm32_generic/quadrature_encoder.hpp>
#include <libhal-util/bit.hpp>

#include "timer.hpp"

namespace hal::stm32_generic {

void setup_channel(int channel, timer_reg_t* p_reg)
{

  constexpr auto odd_channel_input_mode = bit_mask::from<0, 1>();
  constexpr auto odd_channel_filter_select = bit_mask::from<4, 7>();

  constexpr auto even_channel_input_mode = bit_mask::from<8, 9>();
  constexpr auto even_channel_filter_select = bit_mask::from<12, 15>();

  constexpr auto input_select = 0b01U;
  constexpr auto input_capture_filter = 0b0000U;

  // Select the TI1 and TI2 polarity by programming the CC1P and CC2P bits in
  // the TIMx_CCER
  // register. When needed, the user can program the input filter as well.
  // find out which pin is what channel
  switch (channel) {
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

void setup_enable_register(int channel, timer_reg_t* p_reg)
{
  // The polarity pits is the second bit in a set of 4 bits per channel.
  // Therefore if it is channel 1-> bit 1, channel 2-> bit->5. Application Note
  // Pg: 353
  auto const polarity_start_pos = ((channel - 1) * 4) + 1;

  // Similar to polarity bits, capture_enable bits are the first bit in a set of
  // 4 bits per channel. Application Note Pg: 353.
  auto const input_capture_enable = ((channel - 1) * 4);

  auto const input_start_pos = bit_mask::from(input_capture_enable);
  auto const polarity_inverted = bit_mask::from(polarity_start_pos);

  bit_modify(p_reg->cc_enable_register).clear(polarity_inverted);
  bit_modify(p_reg->cc_enable_register).set(input_start_pos);
}

quadrature_encoder::quadrature_encoder(hal::unsafe,
                                       encoder_channels channels,
                                       void* p_reg)
{
  initialize(hal::unsafe{}, channels, p_reg);
}
quadrature_encoder::quadrature_encoder(hal::unsafe)
{
}
void quadrature_encoder::initialize(unsafe,
                                    encoder_channels channels,
                                    void* p_reg)
{
  m_reg = p_reg;

  timer_reg_t* timer_register = get_timer_reg(m_reg);
  constexpr auto set_encoder_mode = bit_mask::from<0, 2>();
  // encoder counts up/down on both TI1FP1 and TI2FP1 level
  constexpr auto encoder_mode_3 = 0b011U;

  setup_channel(channels.channel_a, timer_register);
  setup_channel(channels.channel_b, timer_register);
  setup_enable_register(channels.channel_a, timer_register);
  setup_enable_register(channels.channel_b, timer_register);
  timer_register->auto_reload_register = 0xFFFF;  // Set max counter value
  timer_register->counter_register = 0x8000;      // Start at middle value
  bit_modify(timer_register->peripheral_control_register)
    .insert<set_encoder_mode>(encoder_mode_3);
  constexpr auto counter_enable = bit_mask::from<0>();
  bit_modify(timer_register->control_register).set(counter_enable);
}
quadrature_encoder::read_t quadrature_encoder::driver_read()
{
  read_t reading;
  timer_reg_t* timer_register = get_timer_reg(m_reg);

  reading.angle = static_cast<float>(timer_register->counter_register);
  return reading;
}
}  // namespace hal::stm32_generic
