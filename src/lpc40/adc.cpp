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

#include <libhal-lpc40/adc.hpp>

#include <libhal-lpc40/clock.hpp>
#include <libhal-lpc40/constants.hpp>
#include <libhal-lpc40/pin.hpp>
#include <libhal-lpc40/power.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/bit_limits.hpp>

#include "adc_reg.hpp"

namespace hal::lpc40 {

namespace {
void setup(adc::channel const& p_channel)
{
  using namespace hal::literals;

  if (p_channel.clock_rate > 1.0_MHz) {
    throw std::errc::invalid_argument;
  }

  if (p_channel.index >= adc_reg_t::channel_length) {
    throw std::errc::invalid_argument;
  }

  power_on(peripheral::adc);

  // For proper operation, analog pins must be set to floating.
  p_channel.adc_pin.function(p_channel.pin_function)
    .resistor(hal::pin_resistor::none)
    .open_drain(false)
    .analog(true);

  auto const clock_frequency = get_frequency(peripheral::adc);
  auto const clock_divider = clock_frequency / p_channel.clock_rate;
  auto const clock_divider_int = static_cast<std::uint32_t>(clock_divider);

  // Activate burst mode (continuous sampling), power on ADC and set clock
  // divider.
  hal::bit_modify(adc_reg->control)
    .set<adc_control_register::burst_enable>()
    .set<adc_control_register::power_enable>()
    .insert<adc_control_register::clock_divider>(clock_divider_int);

  // Enable channel. Must be done in a separate write to memory than power on
  // and burst enable.
  hal::bit_modify(adc_reg->control)
    .set(bit_mask{ .position = p_channel.index, .width = 1 });
}
}  // namespace

adc::adc(channel const& p_channel)
  : m_sample(&adc_reg->data[p_channel.index])
{
  setup(p_channel);
}

adc::channel adc::get_predefined_channel_info(std::uint8_t p_channel)
{
  enum adc_function : uint8_t
  {
    pin_0123 = 0b001,
    pin_4567 = 0b011
  };
  constexpr std::array channels{
    adc::channel{
      .adc_pin = pin(0, 23),
      .index = 0,
      .pin_function = adc_function::pin_0123,
    },
    adc::channel{
      .adc_pin = pin(0, 24),
      .index = 1,
      .pin_function = adc_function::pin_0123,
    },
    adc::channel{
      .adc_pin = pin(0, 25),
      .index = 2,
      .pin_function = adc_function::pin_0123,
    },
    adc::channel{
      .adc_pin = pin(0, 26),
      .index = 3,
      .pin_function = adc_function::pin_0123,
    },
    adc::channel{
      .adc_pin = pin(1, 30),
      .index = 4,
      .pin_function = adc_function::pin_4567,
    },
    adc::channel{
      .adc_pin = pin(1, 31),
      .index = 5,
      .pin_function = adc_function::pin_4567,
    },
    adc::channel{
      .adc_pin = pin(0, 12),
      .index = 6,
      .pin_function = adc_function::pin_4567,
    },
    adc::channel{
      .adc_pin = pin(0, 13),
      .index = 7,
      .pin_function = adc_function::pin_4567,
    },
  };

  return channels[p_channel];
}

float adc::driver_read()
{
  constexpr auto full_scale_max = bit_limits<12, size_t>::max();
  constexpr auto full_scale_float = static_cast<float>(full_scale_max);
  // Read sample from peripheral memory
  auto sample_integer = hal::bit_extract<adc_data_register::result>(*m_sample);
  auto sample = static_cast<float>(sample_integer);
  return sample / full_scale_float;
}
}  // namespace hal::lpc40
