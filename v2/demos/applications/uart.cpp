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

module;

#include <algorithm>
#include <array>
#include <chrono>
#include <span>
#include <string_view>

module arm_mcu_demos;

import hal;
import hal.util;

void application()
{
  auto console = resources::console();

  auto last_count = console->receive_count();

  while (true) {
    using namespace std::chrono_literals;
    using namespace std::string_view_literals;

    hal::write(*console, "Hello, World!\n"sv);

    // Echo back the newest bytes received since the last iteration. The
    // count keeps growing, so unsigned subtraction gives the bytes received
    // in between, even when the count wraps.
    auto const receive_buffer = console->receive_buffer();
    auto const count = console->receive_count();
    auto const new_byte_count = count - last_count;
    if (new_byte_count > receive_buffer.size()) {
      hal::write(*console, "Receive buffer overrun, bytes were lost\n"sv);
    }

    std::array<hal::byte, 64> echo_buffer{};
    auto const echoed =
      std::min<hal::usize>(new_byte_count, echo_buffer.size());
    // receive_buffer is a circular_span, so indexes past the end wrap around.
    auto const first = count - echoed;
    for (hal::usize i = 0; i < echoed; i++) {
      echo_buffer[i] = receive_buffer[first + i];
    }
    last_count = count;

    if (echoed > 0) {
      hal::write(*console,
                 { std::span<hal::byte const>(echo_buffer.data(), echoed) });
    }

    hal::sleep_for(1s);
  }
}
