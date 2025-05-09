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

u8 global_counter = 0;

// Awaiter for when this task is awaited
template<typename T>
struct awaiter
{
  std::coroutine_handle<hal::v5::hal_promise_type<T>> m_handle;

  explicit awaiter() noexcept
  {
  }

  [[nodiscard]] bool await_ready() const noexcept
  {
    return !m_handle || m_handle.done();
  }

  // Generic await_suspend for any promise type
  template<typename Promise>
  void await_suspend(std::coroutine_handle<Promise> continuation) noexcept
  {
    m_handle.promise().set_continuation(continuation);
  }

  // Specialized await_suspend for our promise type - propagates allocator
  template<typename U>
  void await_suspend(
    std::coroutine_handle<hal::v5::hal_promise_type<U>> continuation) noexcept
  {
    // Get allocator from parent coroutine
    auto* parent_allocator = continuation.promise().get_allocator();

    // Propagate allocator to child coroutine
    if (parent_allocator) {
      m_handle.promise().set_allocator(parent_allocator);
    }

    // Store continuation for resumption
    m_handle.promise().set_continuation(continuation);
  }

  T await_resume()
  {
    if (!m_handle) {
      throw std::runtime_error("Awaiting a null task");
    }

    if constexpr (std::is_void_v<T>) {
      m_handle.promise().result();
    } else {
      return m_handle.promise().result();
    }
  }
};

hal::v5::task<hal::u8> bar(hal::v5::coro_context& ctx)
{
  co_return global_counter++;
}

awaiter<int> bar()
{
  // What should I put here?
  awaiter<int> bar_runner;
  return bar_runner;
}

// Add coro_context parameter to the top-level coroutine
hal::v5::task<int> foo(hal::v5::coro_context& ctx)
{
  auto const value = co_await bar();
  co_return value + 1;
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

    // Create the context with your memory resource
    hal::v5::coro_context ctx(resources::coroutine_allocator());

    // Call foo with the context
    auto count_value = hal::v5::sync_wait(foo(ctx));
    hal::print<32>(*console, "Hello, World! __[ %d ]__\n", count_value);

    // Echo anything received
    std::array<hal::byte, 64> read_buffer;
    console->write(console->read(read_buffer).data);

    hal::delay(*clock, 1s);
  }
}
