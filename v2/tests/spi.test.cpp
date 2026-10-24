// Copyright 2026 Khalil Estell and the libhal contributors
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

// achievable_clock_rate() is pure arithmetic, so it runs on the host. It
// never throws: requests outside the bus's range get the nearest rate.

#include <boost/ut.hpp>

import hal;
import hal.arm_mcu;

namespace {
// Boost.UT can't print quantities, so tests compare plain hertz values.
auto hz(hal::hertz p_rate)
{
  return p_rate.numerical_value_in(hal::hertz::unit);
}

void spi_test()
{
  using namespace boost::ut;
  using namespace mp_units::si::unit_symbols;
  using hal::stm32_generic::achievable_clock_rate;

  "picks the fastest prescaler that doesn't exceed the request"_test = []() {
    // Setup
    auto const peripheral_clock = 8 * MHz;

    // Exercise + Verify
    expect(that % hz(2 * MHz) ==
           hz(achievable_clock_rate(peripheral_clock, 2 * MHz)));
    expect(that % hz(500 * kHz) ==
           hz(achievable_clock_rate(peripheral_clock, 500 * kHz)));
  };

  "never runs faster than the request"_test = []() {
    // Setup
    auto const peripheral_clock = 8 * MHz;

    // Exercise
    // 8 MHz / 3 MHz rounds down to 2, which would give 4 MHz.
    auto const rate = achievable_clock_rate(peripheral_clock, 3 * MHz);

    // Verify
    expect(that % hz(2 * MHz) == hz(rate));
  };

  "a request above the maximum gets the maximum"_test = []() {
    // Setup
    auto const peripheral_clock = 72 * MHz;

    // Exercise
    auto const rate = achievable_clock_rate(peripheral_clock, 100 * MHz);

    // Verify
    expect(that % hz(36 * MHz) == hz(rate));
  };

  "the slowest prescaler is 256"_test = []() {
    // Setup
    auto const peripheral_clock = 8 * MHz;

    // Exercise
    auto const rate = achievable_clock_rate(peripheral_clock, 31'250 * Hz);

    // Verify
    expect(that % hz(31'250 * Hz) == hz(rate));
  };

  "a request below the minimum gets the slowest rate"_test = []() {
    // Setup
    auto const peripheral_clock = 72 * MHz;

    // Exercise + Verify
    // 72 MHz / 256 = 281.25 kHz is the slowest the bus can go.
    expect(that % hz(281'250 * Hz) ==
           hz(achievable_clock_rate(peripheral_clock, 1 * kHz)));
    // 72 MHz / 1.098 kHz = 65,573. Truncated to 16 bits that was 37, which
    // the old range check passed, running the bus at 1.125 MHz.
    expect(that % hz(281'250 * Hz) ==
           hz(achievable_clock_rate(peripheral_clock, 1'098 * Hz)));
  };

  "a zero clock rate gets the slowest rate"_test = []() {
    // Exercise + Verify
    expect(that % hz(31'250 * Hz) ==
           hz(achievable_clock_rate(8 * MHz, 0 * Hz)));
  };
}
}  // namespace

int main()
{
  spi_test();
}
