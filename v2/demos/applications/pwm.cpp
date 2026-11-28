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

// Steps a PWM channel through 0% to 100% duty at a few frequencies. Watch the
// pin with a scope or an LED: at 50 Hz an LED visibly flickers, and at 1 kHz
// and above its brightness follows the duty cycle.

module;

#include <array>
#include <chrono>
#include <cinttypes>
#include <cstdint>

module arm_mcu_demos;

import hal;
import hal.util;

void application()
{
  using namespace std::chrono_literals;

  auto console = resources::console();
  auto group = resources::pwm_group();
  auto channel = resources::pwm_channel();

  hal::write(*console, "PWM demo\n");

  constexpr std::array<std::uint32_t, 3> frequencies{ 20'000, 1'000, 50 };
  while (true) {
    for (auto const hertz : frequencies) {
      group->frequency(hertz * mp_units::si::hertz);
      auto const actual =
        channel->frequency().numerical_value_in(mp_units::si::hertz);
      hal::print<64>(*console,
                     "frequency: asked %" PRIu32 " Hz, got %" PRIu32 " Hz\n",
                     hertz,
                     static_cast<std::uint32_t>(actual));

      for (std::uint32_t percent = 0; percent <= 100; percent += 25) {
        channel->duty_cycle(static_cast<std::uint16_t>(
          (percent * std::uint32_t{ 65'535 }) / 100U));
        hal::print<32>(*console, "  duty %3" PRIu32 "%%\n", percent);
        hal::sleep_for(1s);
      }
    }
  }
}
