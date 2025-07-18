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

#include <cinttypes>

#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include <resource_list.hpp>

void application()
{
  auto clock = resources::clock();
  auto console = resources::console();
  auto adc = resources::adc();

  hal::print(*console, "ADC Application Starting...\n");

  while (true) {
    using namespace std::chrono_literals;
    auto percent = adc->read();
    // Get current uptime
    auto uptime = clock->uptime();
    hal::print<128>(*console,
                    "%" PRId32 "%%: %" PRIu32 "ns\n",
                    static_cast<std::int32_t>(percent * 100),
                    static_cast<std::uint32_t>(uptime));
    hal::delay(*clock, 100ms);
  }
}
