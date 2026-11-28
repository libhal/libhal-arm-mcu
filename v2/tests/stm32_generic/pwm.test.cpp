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

// pwm_periods() and pwm_compare() are pure arithmetic, so they run on the
// host.

#include <cstdint>

#include <boost/ut.hpp>

import hal;
import hal.arm_mcu;

namespace {
void pwm_test()
{
  using namespace boost::ut;
  using hal::stm32_generic::pwm_compare;
  using hal::stm32_generic::pwm_periods;

  "20 kHz at 72 MHz needs no prescaler"_test = []() {
    // Exercise
    auto const period =
      pwm_periods({ .clock_hz = 72'000'000, .frequency_hz = 20'000 });

    // Verify
    expect(that % 1U == period.prescaler);
    expect(that % 3'600U == period.reload);
  };

  "a slow frequency picks the smallest prescaler that fits"_test = []() {
    // Exercise
    // 50 Hz: 1,440,000 cycles. 1,440,000 / 65,535 = 21.97, so 22, and
    // 1,440,000 / 22 rounds to 65,455.
    auto const servo =
      pwm_periods({ .clock_hz = 72'000'000, .frequency_hz = 50 });

    // Verify
    expect(that % 22U == servo.prescaler);
    expect(that % 65'455U == servo.reload);
  };

  "the reload never reaches 65,536"_test = []() {
    // Exercise
    // 72,000,000 / 1,098 = 65,574 cycles: over 65,535, so prescaler 2
    auto const period =
      pwm_periods({ .clock_hz = 72'000'000, .frequency_hz = 1'098 });

    // Verify
    expect(that % 2U == period.prescaler);
    expect(period.reload <= 65'535U);
  };

  "too fast runs at half the clock, and 0 Hz at the lowest"_test = []() {
    // Exercise
    auto const fast =
      pwm_periods({ .clock_hz = 72'000'000, .frequency_hz = 72'000'000 });
    auto const stopped =
      pwm_periods({ .clock_hz = 72'000'000, .frequency_hz = 0 });

    // Verify
    expect(that % 1U == fast.prescaler);
    expect(that % 2U == fast.reload);
    expect(that % 65'536U == stopped.prescaler);
    expect(that % 65'535U == stopped.reload);
  };

  "duty cycles map onto the period's steps"_test = []() {
    // Exercise + Verify
    expect(that % 0U == pwm_compare(0, 3'600));
    expect(that % 1'800U == pwm_compare(32'768, 3'600));
    // 100% is the whole period: the compare value is past ARR
    expect(that % 3'600U == pwm_compare(65'535, 3'600));
    expect(that % 65'535U == pwm_compare(65'535, 65'535));
  };
}
}  // namespace

int main()
{
  pwm_test();
}
