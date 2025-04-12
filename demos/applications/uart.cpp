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

#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/timeout.hpp>

#include <resource_list.hpp>

void application(resource_list& p_map)
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  resource_contract_assert(p_map.clock);
  resource_contract_assert(p_map.console);

  auto& clock = *p_map.clock;
  auto& console = *p_map.console;

  while (true) {
    using namespace std::chrono_literals;
    using namespace std::string_view_literals;

    std::string_view message = "Hello, World!\n";
    hal::print(console, message);
    // Echo anything received
    std::array<hal::byte, 64> read_buffer;
    console.write(console.read(read_buffer).data);

    hal::delay(clock, 1s);
  }
}
