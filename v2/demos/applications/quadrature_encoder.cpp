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

// Prints a quadrature encoder's angle whenever it changes. Turn the shaft
// both ways: the angle goes up one way and down the other, and keeps counting
// past any number of turns.

module;

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
  auto encoder = resources::rotation_sensor();

  hal::write(*console, "Quadrature encoder demo\n");

  // Printed in thousandths of a turn: the console has no float formatting.
  std::int32_t last = -1;
  while (true) {
    auto const turns =
      encoder->read().numerical_value_in(mp_units::angular::revolution);
    auto const milliturns = static_cast<std::int32_t>(turns * 1'000.0f);
    if (milliturns != last) {
      hal::print<48>(*console, "angle: %" PRIi32 " / 1000 turns\n", milliturns);
      last = milliturns;
    }
    hal::sleep_for(50ms);
  }
}
