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
#include <libhal/error.hpp>

#include "resource_list.hpp"

[[noreturn]] void terminate_handler() noexcept
{

  // spin here forever
  while (true) {
    continue;
  }
}

int main()
{
  hal::set_terminate(terminate_handler);

  try {
    application();
  } catch (hal::bad_optional_ptr_access const& e) {
    auto console = resources::console();
    hal::print(*console,
               "A resource required by the application was not"
               "available!\n"
               "Calling terminate!\n");

  }  // Allow any other exceptions to terminate the application

  std::terminate();
}

extern "C"
{
  // This gets rid of an issue with libhal-exceptions in Debug mode.
  void __assert_func()  // NOLINT
  {
  }
}
