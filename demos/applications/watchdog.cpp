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

#include <cmath>

#include <array>

#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <libhal/timeout.hpp>
#include <libhal/units.hpp>

#include <resource_list.hpp>

void application()
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto console = resources::console();
  auto watchdog = resources::watchdog();
  constexpr auto wait_time = 5s;

  if (watchdog->check_flag()) {
    hal::print(*console, "Reset by watchdog\n");
    watchdog->clear_flag();
  } else {
    hal::print(*console, "Non-watchdog reset\n");
  }

  try {
    watchdog->set_countdown_time(wait_time);
    watchdog->start();
  } catch (hal::operation_not_supported e) {
    hal::print(*console, "invalid time\n");
  }

  hal::print(*console, "Type somehting into the console to pet the watchdog\n");
  while (true) {
    std::array<hal::byte, 64> read_buffer;
    auto read = console->read(read_buffer).data;
    if (!read.empty()) {
      hal::print(*console, "woof!\n");
      watchdog->reset();
    }
  }
}
