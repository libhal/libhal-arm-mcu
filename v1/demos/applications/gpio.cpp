// Copyright 2024 - 2025 Khalil Estell and the libhal contributors
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

#include <libhal-util/steady_clock.hpp>

#include <resource_list.hpp>

void application()
{
  auto clock = resources::clock();
  auto led = resources::status_led();
  auto button = resources::input_pin();

  while (true) {
    // Checking level for the lpc40xx drivers NEVER generates an error so this
    // is fine.
    if (button->level()) {
      using namespace std::chrono_literals;
      led->level(false);
      hal::delay(*clock, 200ms);
      led->level(true);
      hal::delay(*clock, 200ms);
    }
  }
}
