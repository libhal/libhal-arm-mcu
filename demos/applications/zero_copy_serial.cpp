// Copyright 2024 Khalil Estell
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

  auto& clock = *p_map.clock.value();
  auto& console = *p_map.zero_copy_serial.value();

  auto previous_cursor = console.receive_cursor();

  while (true) {
    using namespace std::chrono_literals;
    using namespace std::string_view_literals;

    std::string_view message = "Hello, World!\n";
    hal::print(console, message);

    auto const new_cursor = console.receive_cursor();
    hal::print<64>(console, "cursor = %zu\n", new_cursor);

    hal::usize byte_count = 0;
    if (new_cursor < previous_cursor) {
      byte_count = console.receive_buffer().size() - new_cursor;
      // Echo anything received
      console.write(
        console.receive_buffer().subspan(previous_cursor, byte_count));
      previous_cursor = 0;
    } else {
      byte_count = new_cursor - previous_cursor;
      // Echo anything received
      console.write(
        console.receive_buffer().subspan(previous_cursor, byte_count));
      previous_cursor = new_cursor;
    }

    hal::delay(clock, 1s);
  }
}
