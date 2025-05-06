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
#include <memory_resource>

#include <awaitable.hpp>
#include <resource_list.hpp>
#include <task.hpp>

int global_counter = 0;

hal::v5::task<int> bar(std::pmr::polymorphic_allocator<>)
{
  co_return global_counter++;
}

hal::v5::task<int> foo(std::pmr::polymorphic_allocator<> p_allocator)
{
  co_return co_await bar(p_allocator);
}

void application()
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto clock = resources::uptime_clock();
  auto console = resources::console();

  while (true) {
    using namespace std::chrono_literals;
    using namespace std::string_view_literals;

#if 0
    std::string_view message = "Hello, World! %d\n";
    hal::print<32>(*console,
                   message.data(),  // NOLINT
                   hal::v5::sync_wait(foo(resources::coroutine_allocator())));
#endif
    auto count_value =
      hal::v5::sync_wait(foo(resources::coroutine_allocator()));
    hal::print<32>(*console, "Hello, World! __[ %d ]__\n", count_value);

    // Echo anything received
    std::array<hal::byte, 64> read_buffer;
    console->write(console->read(read_buffer).data);

    hal::delay(*clock, 1s);
  }
}
