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

#include <libhal-stm32f1/adc.hpp>
#include <libhal-stm32f1/clock.hpp>
#include <libhal-stm32f1/constants.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/bit_limits.hpp>

#include "adc_reg.hpp"
#include "pin.hpp"
#include "power.hpp"

namespace hal::stm32f1 {

namespace {
void setup(adc::channel const& p_channel)
{
  auto adc_frequency = frequency(hal::stm32f1::peripheral::adc1);
  if (adc_frequency > 14.0_MHz) {
    throw std::errc::invalid_argument;
  }

  if (p_channel.index >= adc_reg_t::regular_channel_length) {
    throw std::errc::invalid_argument;
  }

  // Power on adc clock
  power_on(peripheral::adc1);

  // Set specified channel to analog input mode
  configure_pin({ .port = p_channel.port, .pin = p_channel.pin }, input_analog);

  // Power on adc
  hal::bit_modify(adc_reg->control_2)
    .set<adc_control_register_2::ad_converter_on>();

  // Set the specified channel to be sampled
  hal::bit_modify(adc_reg->regular_sequence_3)
    .insert<adc_regular_sequence_register_3::first_conversion>(p_channel.index);

  // Start adc calibration
  hal::bit_modify(adc_reg->control_2)
    .set<adc_control_register_2::ad_calibration>();

  // Wait for calibration to complete
  while (bit_extract<adc_control_register_2::ad_calibration>(
           adc_reg->control_2) == 1) {
  }
}
}  // namespace

adc::adc(channel const& p_channel)
{
  setup(p_channel);
}

adc::channel adc::get_predefined_channel_info(std::uint8_t p_channel)
{
  constexpr std::array channels{
    adc::channel{
      .port = 'A',
      .pin = 0,
      .index = 0,
    },
    adc::channel{
      .port = 'A',
      .pin = 1,
      .index = 1,
    },
    adc::channel{
      .port = 'A',
      .pin = 2,
      .index = 2,
    },
    adc::channel{
      .port = 'A',
      .pin = 3,
      .index = 3,
    },
    adc::channel{
      .port = 'A',
      .pin = 4,
      .index = 4,
    },
    adc::channel{
      .port = 'A',
      .pin = 5,
      .index = 5,
    },
    adc::channel{
      .port = 'A',
      .pin = 6,
      .index = 6,
    },
    adc::channel{
      .port = 'A',
      .pin = 7,
      .index = 7,
    },
    adc::channel{
      .port = 'B',
      .pin = 0,
      .index = 8,
    },
    adc::channel{
      .port = 'B',
      .pin = 1,
      .index = 9,
    },
    adc::channel{
      .port = 'C',
      .pin = 0,
      .index = 10,
    },
    adc::channel{
      .port = 'C',
      .pin = 1,
      .index = 11,
    },
    adc::channel{
      .port = 'C',
      .pin = 2,
      .index = 12,
    },
    adc::channel{
      .port = 'C',
      .pin = 3,
      .index = 13,
    },
    adc::channel{
      .port = 'C',
      .pin = 4,
      .index = 14,
    },
    adc::channel{
      .port = 'C',
      .pin = 5,
      .index = 15,
    },
  };
  return channels[p_channel];
}

float adc::driver_read()
{
  // Start adc conversion
  hal::bit_modify(adc_reg->control_2)
    .set<adc_control_register_2::ad_converter_on>();

  // Wait for conversion to complete
  while (bit_extract<adc_status_register::end_of_conversion>(adc_reg->status) ==
         0) {
  }

  constexpr auto full_scale_max = bit_limits<12, size_t>::max();
  constexpr auto full_scale_float = static_cast<float>(full_scale_max);
  // Read sample from peripheral memory
  auto sample_integer =
    hal::bit_extract<adc_regular_data_register::regular_data>(
      adc_reg->regular_data);
  auto sample = static_cast<float>(sample_integer);
  return sample / full_scale_float;
}

}  // namespace hal::stm32f1
