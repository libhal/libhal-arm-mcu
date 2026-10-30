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

// compute_i2c_timing() is pure arithmetic, so it runs on the host.

#include <boost/ut.hpp>

import hal;
import hal.arm_mcu;

namespace {
void i2c_test()
{
  using namespace boost::ut;
  using namespace mp_units::si::unit_symbols;
  using hal::stm32_generic::compute_i2c_timing;

  "standard mode divides the clock into equal high and low halves"_test = []() {
    // Exercise
    // 36 MHz / (2 * 100 kHz) = 180
    auto const timing = compute_i2c_timing(100 * kHz, 36 * MHz);

    // Verify
    expect(that % 36 == timing.frequency);
    expect(not timing.fast_mode);
    expect(that % 180 == timing.ccr);
    // 1000 ns at 36 MHz is 36 cycles, plus one.
    expect(that % 37 == timing.rise_time);
  };

  "the stall bound polls for at least 25 ms"_test = []() {
    // Exercise
    // 25 ms at 36 MHz is 900,000 cycles, and a poll takes at least two.
    auto const timing = compute_i2c_timing(100 * kHz, 36 * MHz);
    // 25 ms at 8 MHz is 200,000 cycles.
    auto const slow_timing = compute_i2c_timing(100 * kHz, 8 * MHz);

    // Verify
    expect(that % 450'000 == timing.stall_polls);
    expect(that % 100'000 == slow_timing.stall_polls);
  };

  "fast mode uses a 1:2 high:low duty cycle"_test = []() {
    // Exercise
    // 36 MHz / (3 * 400 kHz) = 30
    auto const timing = compute_i2c_timing(400 * kHz, 36 * MHz);

    // Verify
    expect(timing.fast_mode);
    expect(that % 30 == timing.ccr);
    // 300 ns at 36 MHz is 10.8 cycles, rounded down, plus one.
    expect(that % 11 == timing.rise_time);
  };

  "the divider rounds up, so the bus never runs faster than asked"_test = []() {
    // Exercise
    // 8 MHz / (2 * 90 kHz) = 44.4, which would run at 90.9 kHz.
    auto const timing = compute_i2c_timing(90 * kHz, 8 * MHz);

    // Verify
    expect(that % 45 == timing.ccr);
  };

  "a rate above fast mode runs at 400 kHz"_test = []() {
    // Exercise
    auto const timing = compute_i2c_timing(1 * MHz, 36 * MHz);

    // Verify
    expect(timing.fast_mode);
    expect(that % 30 == timing.ccr);
  };

  "unachievable settings are rejected"_test = []() {
    // Exercise + Verify
    // Zero has no divider.
    expect(throws<hal::operation_not_supported>(
      []() { static_cast<void>(compute_i2c_timing(0 * Hz, 36 * MHz)); }));
    // 36 MHz / (2 * 4 kHz) = 4500, above the largest divider of 4095.
    expect(throws<hal::operation_not_supported>(
      []() { static_cast<void>(compute_i2c_timing(4 * kHz, 36 * MHz)); }));
    // Fast mode needs an input clock of at least 4 MHz.
    expect(throws<hal::operation_not_supported>(
      []() { static_cast<void>(compute_i2c_timing(400 * kHz, 3 * MHz)); }));
    // The peripheral needs an input clock of at least 2 MHz.
    expect(throws<hal::operation_not_supported>(
      []() { static_cast<void>(compute_i2c_timing(100 * kHz, 1 * MHz)); }));
  };
}
}  // namespace

int main()
{
  i2c_test();
}
