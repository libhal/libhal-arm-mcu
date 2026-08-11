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
#include <coroutine>
#include <string_view>

module arm_mcu_demos;

import hal;
import hal.util;

hal::task application(async::context& p_ctx)
{
  auto console = resources::console();

  auto last_cursor = console->receive_cursor();

  while (true) {
    using namespace std::chrono_literals;
    using namespace std::string_view_literals;

    co_await hal::write(p_ctx, *console, "Hello, World!\n"sv);

    // Echo back anything received since the last iteration.
    auto const receive_buffer = console->receive_buffer();
    auto const cursor = console->receive_cursor();
    auto const new_byte_count =
      std::min<hal::usize>(cursor - last_cursor, receive_buffer.size());

    std::array<hal::byte, 64> echo_buffer{};
    auto const echoed =
      std::min<hal::usize>(new_byte_count, echo_buffer.size());
    for (hal::usize i = 0; i < echoed; i++) {
      echo_buffer[i] = receive_buffer[cursor - echoed + i];
    }
    last_cursor = cursor;

    if (echoed > 0) {
      co_await hal::write(
        p_ctx, *console, { std::span<hal::byte const>(echo_buffer.data(), echoed) });
    }

    co_await 1s;
  }
}
