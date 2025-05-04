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

#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/steady_clock.hpp>

#include <pointers.hpp>
#include <resource_list.hpp>

[[noreturn]] void terminate_handler() noexcept
{
  if (resources::opt_console) {
    hal::print(*resources::opt_console, "☠️ APPLICATION TERMINATED ☠️\n\n");
  }

  if (resources::opt_status_led && resources::opt_uptime_clock) {

    while (true) {
      using namespace std::chrono_literals;
      resources::opt_status_led->level(false);
      hal::delay(*resources::opt_uptime_clock, 100ms);
      resources::opt_status_led->level(true);
      hal::delay(*resources::opt_uptime_clock, 100ms);
      resources::opt_status_led->level(false);
      hal::delay(*resources::opt_uptime_clock, 100ms);
      resources::opt_status_led->level(true);
      hal::delay(*resources::opt_uptime_clock, 1000ms);
    }
  }

  // spin here forever
  while (true) {
    continue;
  }
}

int main()
{
  hal::set_terminate(terminate_handler);

  initialize_platform();

  // Acquire resources for terminate
  resources::opt_uptime_clock = resources::uptime_clock();
  resources::opt_status_led = resources::status_led();
  resources::opt_console = resources::console();

  application();
  std::terminate();
}

extern "C"
{
  // This gets rid of an issue with libhal-exceptions in Debug mode.
  void __assert_func()  // NOLINT
  {
  }
}

// Override global new operator
void* operator new(std::size_t)
{
  std::terminate();
}

// Override global new[] operator
void* operator new[](std::size_t)
{
  std::terminate();
}

void* operator new(unsigned int, std::align_val_t)
{
  std::terminate();
}

// Override global delete operator
void operator delete(void*) noexcept
{
}

// Override global delete[] operator
void operator delete[](void*) noexcept
{
}

// Optional: Override sized delete operators (C++14 and later)
void operator delete(void*, std::size_t) noexcept
{
}

void operator delete[](void*, std::size_t) noexcept
{
}

void operator delete[](void*, std::align_val_t) noexcept
{
}

void operator delete(void*, std::align_val_t) noexcept
{
}
