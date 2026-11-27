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

// Times one-shot delays against the steady clock, then blinks the status LED
// from a periodic timer's callback and counts its expirations.

module;

#include <array>
#include <chrono>
#include <cinttypes>
#include <cstdint>

module arm_mcu_demos;

import hal;
import hal.util;

namespace {
class blink_counter final : public hal::timed_callback
{
public:
  explicit blink_counter(hal::static_ref<hal::output_pin> p_led)
    : m_led(p_led)
  {
  }

  void callback() noexcept override
  {
    m_count = m_count + 1U;
    m_led->level(not m_led->level());
  }

  [[nodiscard]] std::uint32_t count() const
  {
    return m_count;
  }

private:
  hal::static_ref<hal::output_pin> m_led;
  std::uint32_t volatile m_count = 0;
};
}  // namespace

void application()
{
  using namespace std::chrono_literals;

  auto console = resources::console();
  auto clock = resources::clock();
  auto timer = resources::timer();
  auto const clock_hz = clock->frequency().numerical_value_in(hal::hertz::unit);

  hal::write(*console, "Timer demo\n");

  // Poll scheduled() to see each one-shot delay end, and measure it.
  constexpr std::array<hal::time_duration, 4> delays{ 100us, 1ms, 10ms, 100ms };
  for (auto const delay : delays) {
    auto const start = clock->uptime();
    timer->schedule(delay);
    while (timer->scheduled()) {
      continue;
    }
    auto const ticks = clock->uptime() - start;
    auto const took_us = (ticks * 1'000'000U) / clock_hz;
    hal::print<64>(*console,
                   "one-shot %6" PRIu32 " us: took %6" PRIu32 " us\n",
                   static_cast<std::uint32_t>(delay.count() / 1'000),
                   static_cast<std::uint32_t>(took_us));
  }

  // Blink from the callback: it runs in the timer's interrupt.
  blink_counter blinker(resources::status_led());
  auto const subscription = timer->on_expire(blinker);
  timer->schedule(250ms, hal::timer_mode::periodic);

  while (true) {
    hal::sleep_for(1s);
    hal::print<32>(*console, "expirations: %" PRIu32 "\n", blinker.count());
  }
}
