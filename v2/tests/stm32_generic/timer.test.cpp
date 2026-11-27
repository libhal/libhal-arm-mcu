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

// timer_periods() is pure arithmetic, so it runs on the host.

#include <array>
#include <cstdint>

#include <boost/ut.hpp>

import hal;
import hal.arm_mcu;

namespace {
void timer_test()
{
  using namespace boost::ut;
  using hal::stm32_generic::timer_periods;
  constexpr std::uint64_t max_period = 65'536ULL * 65'536ULL;

  "a short delay counts at full resolution"_test = []() {
    // Exercise
    auto const period = timer_periods(1'000);

    // Verify
    expect(that % 1U == period.counts);
    expect(that % 1U == period.prescaler);
    expect(that % 1'000U == period.reload);
  };

  "a delay under 2 cycles runs as 2"_test = []() {
    // Exercise + Verify
    for (std::uint64_t const cycles : { 0ULL, 1ULL }) {
      auto const period = timer_periods(cycles);
      expect(that % 1U == period.counts);
      expect(that % 1U == period.prescaler);
      expect(that % 2U == period.reload);
    }
  };

  "the prescaler starts dividing past the 16-bit counter"_test = []() {
    // Exercise
    auto const full = timer_periods(65'536);
    auto const over = timer_periods(65'537);

    // Verify
    expect(that % 1U == full.prescaler);
    expect(that % 65'536U == full.reload);
    expect(that % 2U == over.prescaler);
    expect(that % 32'769U == over.reload);
  };

  "one second at 72 MHz picks the smallest prescaler"_test = []() {
    // Exercise
    auto const period = timer_periods(72'000'000);

    // Verify
    // 72,000,000 / 65,536 = 1,098.6, so 1,099. 72,000,000 / 1,099 rounds up
    // to 65,515: 985 cycles (13.7 us) long.
    expect(that % 1U == period.counts);
    expect(that % 1'099U == period.prescaler);
    expect(that % 65'515U == period.reload);
  };

  "the longest single period uses both 16-bit dividers fully"_test = []() {
    // Exercise
    auto const period = timer_periods(max_period);

    // Verify
    expect(that % 1U == period.counts);
    expect(that % 65'536U == period.prescaler);
    expect(that % 65'536U == period.reload);
  };

  "one cycle past the longest period splits into two equal periods"_test =
    []() {
      // Exercise
      auto const period = timer_periods(max_period + 1);

      // Verify
      // Each period is 2^31 + 1 cycles: 32,769 * 65,535 rounds that up.
      expect(that % 2U == period.counts);
      expect(that % 32'769U == period.prescaler);
      expect(that % 65'535U == period.reload);
    };

  "long delays never end early and run less than a tick per period long"_test =
    []() {
      // Setup
      // 61 s, 2 minutes and 1 hour at 72 MHz, and a value with no common
      // factors
      constexpr std::array<std::uint64_t, 4> delays{
        61ULL * 72'000'000ULL,
        120ULL * 72'000'000ULL,
        3'600ULL * 72'000'000ULL,
        1'234'567'890'123ULL,
      };

      for (auto const cycles : delays) {
        // Exercise
        auto const period = timer_periods(cycles);

        // Verify
        auto const counted = period.counts * period.prescaler * period.reload;
        auto const min_counts = (cycles + (max_period - 1)) / max_period;
        expect(that % min_counts == period.counts);
        expect(counted >= cycles);
        expect((counted - cycles) < (period.counts * period.prescaler));
        expect((period.prescaler >= 1U) and (period.prescaler <= 65'536U));
        expect((period.reload >= 2U) and (period.reload <= 65'536U));
        // Each period is over half the longest one, so the interrupt has time
        expect((period.prescaler * std::uint64_t{ period.reload }) >
               (max_period / 2));
      }
    };
}

void counter_test()
{
  using namespace boost::ut;
  using hal::stm32_generic::counter_prescaler;
  using hal::stm32_generic::counter_ticks;
  using hal::stm32_generic::pending_wrap;

  "picks the finest ticks under the interrupt rate"_test = []() {
    // Exercise + Verify
    // 72 MHz / (20 * 65,536) = 54.9, so 55: 1.309 MHz, 19.98 wraps a second
    expect(that % 55U == counter_prescaler({ .clock_hz = 72'000'000,
                                             .max_interrupt_rate_hz = 20 }));
    expect(that % 74U == counter_prescaler({ .clock_hz = 72'000'000,
                                             .max_interrupt_rate_hz = 15 }));
    expect(that % 1'099U == counter_prescaler({ .clock_hz = 72'000'000,
                                                .max_interrupt_rate_hz = 1 }));
  };

  "a rate the full clock already meets doesn't divide it"_test = []() {
    // Exercise + Verify
    // 72 MHz wraps 1,098.6 times a second
    expect(that % 1U == counter_prescaler({ .clock_hz = 72'000'000,
                                            .max_interrupt_rate_hz = 1'099 }));
    expect(that % 1U ==
           counter_prescaler(
             { .clock_hz = 72'000'000, .max_interrupt_rate_hz = 1'000'000 }));
  };

  "a rate too low to reach gets the fewest wraps"_test = []() {
    // Exercise + Verify
    // 72 MHz / 65,536 / 65,536 = 0.017 wraps a second at the most
    expect(that % 65'536U == counter_prescaler({ .clock_hz = 72'000'000,
                                                 .max_interrupt_rate_hz = 0 }));
  };

  "the wraps become the upper bits"_test = []() {
    // Exercise
    auto const ticks = counter_ticks(
      { .wraps = 3, .count = 0x1234, .pending = pending_wrap::none });

    // Verify
    expect(that % 0x3'1234ULL == ticks);
  };

  "a wrap flagged before the read counts"_test = []() {
    // Exercise
    // Even late in the next period: interrupts were masked for a while
    auto const early = counter_ticks(
      { .wraps = 3, .count = 0x0002, .pending = pending_wrap::before_read });
    auto const late = counter_ticks(
      { .wraps = 3, .count = 0xF000, .pending = pending_wrap::before_read });

    // Verify
    expect(that % 0x4'0002ULL == early);
    expect(that % 0x4'F000ULL == late);
  };

  "a wrap flagged during the read counts only for a count after it"_test =
    []() {
      // Exercise
      auto const after_wrap = counter_ticks(
        { .wraps = 3, .count = 0x0001, .pending = pending_wrap::during_read });
      auto const before_wrap = counter_ticks(
        { .wraps = 3, .count = 0xFFFF, .pending = pending_wrap::during_read });

      // Verify
      expect(that % 0x4'0001ULL == after_wrap);
      expect(that % 0x3'FFFFULL == before_wrap);
    };
}
}  // namespace

int main()
{
  timer_test();
  counter_test();
}
