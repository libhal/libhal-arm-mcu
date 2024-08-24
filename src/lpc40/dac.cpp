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

#include <libhal-lpc40/dac.hpp>
#include <libhal-lpc40/pin.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/bit_limits.hpp>

#include "dac_reg.hpp"

namespace hal::lpc40 {

dac::dac()
{
  pin dac_pin(0, 26);
  dac_pin.analog(true);
  dac_pin.dac(true);
}

void dac::driver_write(float p_percentage)
{
  auto const bits_to_modify = static_cast<std::uint32_t>(
    p_percentage *
    1023);  // getting the 10 most significant bits to set the value
  hal::bit_modify(dac_reg->conversion_register.whole)
    .insert<dac_converter_register::value>(bits_to_modify);
}
}  // namespace hal::lpc40
