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

// Compares the two timer-based steady clocks with the platform's steady clock
// once a second. The timer clocks keep their own time from their interrupts,
// so they stay right however rarely uptime() is called.

module;

#include <chrono>
#include <cinttypes>
#include <cstdint>

module arm_mcu_demos;

import hal;
import hal.util;

namespace {
/// Microseconds in `p_ticks` of a clock running at `p_hz`
std::uint64_t microseconds(std::uint64_t p_ticks, std::uint64_t p_hz)
{
  return (p_ticks * 1'000'000U) / p_hz;
}
}  // namespace

void application()
{
  using namespace std::chrono_literals;

  auto console = resources::console();
  auto reference = resources::clock();
  auto timer_clock = resources::timer_clock();
  auto wide = resources::wide_timer_clock();

  auto const timer_hz =
    timer_clock->frequency().numerical_value_in(hal::hertz::unit);
  auto const wide_hz =
    wide->frequency().numerical_value_in(hal::hertz::unit);
  auto const reference_hz =
    reference->frequency().numerical_value_in(hal::hertz::unit);
  hal::print<96>(*console,
                 "Timer clock: %" PRIu32 " Hz ticks, wide: %" PRIu32
                 " Hz ticks\n",
                 static_cast<std::uint32_t>(timer_hz),
                 static_cast<std::uint32_t>(wide_hz));

  auto const timer_start = timer_clock->uptime();
  auto const wide_start = wide->uptime();
  auto const reference_start = reference->uptime();

  while (true) {
    hal::sleep_for(1s);
    auto const timer_us =
      microseconds(timer_clock->uptime() - timer_start, timer_hz);
    auto const wide_us =
      microseconds(wide->uptime() - wide_start, wide_hz);
    auto const reference_us =
      microseconds(reference->uptime() - reference_start, reference_hz);
    hal::print<128>(*console,
                    "timer clock %10" PRIu32 " us, wide %10" PRIu32
                    " us, reference %10" PRIu32 " us\n",
                    static_cast<std::uint32_t>(timer_us),
                    static_cast<std::uint32_t>(wide_us),
                    static_cast<std::uint32_t>(reference_us));
  }
}
