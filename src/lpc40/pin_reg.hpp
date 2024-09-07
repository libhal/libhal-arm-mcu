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

#pragma once

#include <cstdint>
#include <cstring>

#include <libhal-util/bit.hpp>

namespace hal::lpc40 {
/// Pin map table for maping pins and ports to registers.
struct pin_map_t
{
  /// Register matrix that maps against the 6 ports and the 32 pins per port
  std::uint32_t volatile matrix[6][32];
};

/// The address of the IO connect peripheral
static constexpr intptr_t io_connect_address = 0x40000000UL + 0x2C000;

// Source: "UM10562 LPC408x/407x User manual" table 83 page 132
/// bit_mask for setting pin mux function code.
static constexpr auto pin_function = hal::bit_mask::from<0, 2>();

/// bit_mask for setting resistor mode of pin.
static constexpr auto pin_resistor = hal::bit_mask::from<3, 4>();

/// bit_mask for setting pin hysteresis mode.
static constexpr auto pin_hysteresis = hal::bit_mask::from<5>();

/// bit_mask for setting inputs as active low or active high. This will behave
/// badly if the pin is set to a mode that is an output or set to analog. See
/// user manual for more details.
static constexpr auto pin_input_invert = hal::bit_mask::from<6>();

/// bit_mask for setting a pin to analog mode.
static constexpr auto pin_analog_digital_mode = hal::bit_mask::from<7>();

/// bit_mask for enabling/disabling digital filter. This can be used to
/// ignore/reject noise, reflections, or signal bounce (from something like a
/// switch).
static constexpr auto pin_digital_filter = hal::bit_mask::from<8>();

/// bit_mask to enable/disable high speed I2C mode
static constexpr auto pin_i2c_highspeed = hal::bit_mask::from<8>();

/// bit_mask to change the slew rate of signal transitions for outputs for a
/// pin.
static constexpr auto pin_slew = hal::bit_mask::from<9>();

/// bit_mask to enable I2C high current drain. This can allow for even faster
/// I2C communications, as well as allow for more devices on the bus.
static constexpr auto pin_i2c_high_current = hal::bit_mask::from<9>();

/// bit_mask to enable/disable open drain mode.
static constexpr auto pin_open_drain = hal::bit_mask::from<10>();

/// bit_mask for enabling/disabling digital to analog pin mode.
static constexpr auto pin_dac_enable = hal::bit_mask::from<16>();

/// @return pin_map_t* -  Return the address of the pin map peripheral
inline pin_map_t* pin_map = reinterpret_cast<pin_map_t*>(io_connect_address);
}  // namespace hal::lpc40
