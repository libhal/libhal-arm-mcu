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

#include <array>
#include <cmath>
#include <libhal-arm-mcu/stm32f1/independent_watchdog.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include <libhal/can.hpp>
#include <libhal/timeout.hpp>
#include <libhal/units.hpp>
#include <resource_list.hpp>

void application(resource_list& p_map)
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto& console = *p_map.console.value();
  hal::time_duration const wait_time = 5s;

  if (hal::stm32f1::check_independent_watchdog_flag()) {
    hal::print(console, "Reset by watchdog\n");
    hal::stm32f1::clear_reset_flags();
  } else {
    hal::print(console, "Non-watchdog reset\n");
  }

  try {
    hal::stm32f1::set_independent_watchdog_countdown_time(wait_time);
    hal::stm32f1::start_independent_watchdog();
  } catch (hal::operation_not_supported e) {
    hal::print(console, "invalid time\n");
  }

  hal::print(console, "Type somehting into the console to pet the watchdog\n");
  while (true) {
    std::array<hal::byte, 64> read_buffer;
    auto read = console.read(read_buffer).data;
    if (!read.empty()) {
      hal::print(console, "woof!\n");
      hal::stm32f1::reset_independent_watchdog_counter();
    }
  }
}
